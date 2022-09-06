/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

/* Peripheral drivers and board includes*/
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_pint.h"
#include "fsl_dsp.h"
#if configUSE_TICKLESS_IDLE == 2
#include "fsl_rtc.h"
#endif
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/* Application includes */
#include "dsp_support.h"
#include "dsp_ipc.h"
#include "srtm_config.h"
#include "app_config.h"
#include "app_power.h"
#include "app_audio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Buttons */
#define SW2_INT_INPUTMUX_SEL kINPUTMUX_GpioPort0Pin10ToPintsel
#define SW1_INT_INPUTMUX_SEL kINPUTMUX_GpioPort0Pin25ToPintsel

/* The software timer period. */
#define SW_TIMER_PERIOD_MS (TIMEOUT_SECS * 1000 / portTICK_PERIOD_MS)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef enum _app_state
{
    USER_ACTION = 0,
    HWVAD,
    SWVAD_OR_PROCESSING,
    MAX_APP_STATE,
} app_state_t;

typedef enum _app_event
{
  gAppEvt_Btn1Press_c       = 0x00000001U,
  gAppEvt_Btn2Press_c       = 0x00000002U,
  gAppEvt_HWVAD_c           = 0x00000004U,
  gAppEvt_DMIC_Data_c       = 0x00000008U,
  gAppEvt_Timeout_c         = 0x00000010U,
  gAppEvt_EventsAll_c       = 0x0000001FU
}app_event_t;

extern void vPortRtcIsr(void);
extern void HardFault_Handler(void);
static void SwTimerCallback(TimerHandle_t xTimer);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static app_state_t s_AppState = USER_ACTION;
static state_power_config_t s_PowerConfig[MAX_APP_STATE];
static EventGroupHandle_t s_AppEvent;
static TimerHandle_t SwTimerHandle = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/
#if configUSE_TICKLESS_IDLE == 2
/*!
 * @brief Interrupt service function of LPT timer.
 *
 * This function to call vPortRtcIsr
 */
void RTC_IRQHandler(void)
{
    vPortRtcIsr();
}

/*!
 * @brief Function of LPT timer.
 *
 * This function to return LPT timer base address
 */
RTC_Type *vPortGetRtcBase(void)
{
    return RTC;
}

/*!
 * @brief Fuction of LPT timer.
 *
 * This function to return LPT timer interrupt number
 */
IRQn_Type vPortGetRtcIrqn(void)
{
    return RTC_IRQn;
}
#endif

static void BUTTON_PinIntHandler(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    if (pintr == kPINT_PinInt0)
    {
        /* Send event to App task that the button was pressed */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR( s_AppEvent, gAppEvt_Btn1Press_c, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
    else if (pintr == kPINT_PinInt1)
    {
        /* Send event to App task that the button was pressed */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR( s_AppEvent, gAppEvt_Btn2Press_c, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

static void HWVAD_Callback(void)
{
    /* Send event to App task that HWVAD interrupt was detected */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR( s_AppEvent, gAppEvt_HWVAD_c, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void DMIC_DMA_Callback(void)
{
    /* Send event to App task that DMIC data is available */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR( s_AppEvent, gAppEvt_DMIC_Data_c, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void LED_Init(void)
{
    GPIO_PortInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT);
    GPIO_PortInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT);
    GPIO_PortInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT);
    LED_RED_INIT(0);
    LED_GREEN_INIT(0);
    LED_BLUE_INIT(0);
}

static void BUTTON_Init(void)
{
    /* Configure the Input Mux block and connect the trigger source to PinInt channel. */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, SW2_INT_INPUTMUX_SEL); /* Using channel 0. */
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt1, SW1_INT_INPUTMUX_SEL); /* Using channel 1. */
    INPUTMUX_Deinit(INPUTMUX); /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */

    /* Configure the interrupt for SW pin. */
    PINT_Init(PINT);
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableRiseEdge, BUTTON_PinIntHandler);
    PINT_PinInterruptConfig(PINT, kPINT_PinInt1, kPINT_PinIntEnableRiseEdge, BUTTON_PinIntHandler);
    PINT_EnableCallback(PINT); /* Enable callbacks for PINT */

    /* Button press can wake up from deep sleep */
    EnableDeepSleepIRQ(PIN_INT0_IRQn);
    EnableDeepSleepIRQ(PIN_INT1_IRQn);
}

static void APP_StatesPowerConfig()
{
    for (uint8_t i = 0; i < MAX_APP_STATE; i++)
    {
        PWR_GetDefaultPowerConfig(&s_PowerConfig[i]);
    }

    /* Power off 1 MHz LPOSC during user action state */
    s_PowerConfig[USER_ACTION].pd_sleep_config[0] &= ~SYSCTL0_PDSLEEPCFG0_LPOSC_PD_MASK;
#if DMIC_USE_FRO
    /* Keep FRO powered during SWVAD and Processing state */
    s_PowerConfig[SWVAD_OR_PROCESSING].pd_sleep_config[0] |= SYSCTL0_PDSLEEPCFG0_FFRO_PD_MASK;
#endif
}

static void APP_DSP_IPC_Task(void *param)
{
    srtm_message msg;

    while (1)
    {
        /* Block for IPC message from DSP */
        dsp_ipc_recv_sync(&msg);

        /* Process message */
        if (msg.error != SRTM_Status_Success)
        {
            PRINTF("error: %d from cmd: %d, \r\n", msg.param[0], msg.head.command);
        }

        switch (msg.head.command)
        {
        case SRTM_Command_VIT_Init:
            PWR_AllowDeepSleep();
            break;
        case SRTM_Command_VIT_Process:
            PWR_AllowDeepSleep();
            if (msg.param[1] == VIT_WW_DETECTED)
            {
#if USER_ACTION_ENABLE || HWVAD_ENABLE
                xTimerReset(SwTimerHandle, 0);
#endif
                LED_GREEN_OFF();
                LED_BLUE_ON();
                PRINTF(" - WakeWord detected %d", msg.param[2]);

                // Retrieve WakeWord Name
                if ((char*) msg.param[3] != NULL)
                {
                    PRINTF(" %s\r\n", (char*) msg.param[3]);
                }
#if VOICECMD_ENABLE
                PRINTF("\r\nWaiting for voice command...\n");
#endif
            }
            else if (msg.param[1] == VIT_VC_DETECTED)
            {
#if USER_ACTION_ENABLE || HWVAD_ENABLE
                xTimerReset(SwTimerHandle, 0);
#endif
                LED_GREEN_ON();
                LED_BLUE_OFF();
                PRINTF(" - Voice Command detected %d", msg.param[4]);

                // Retrieve CMD Name
                if ((char*) msg.param[5] != NULL)
                {
                    PRINTF(" %s\r\n", (char*) msg.param[5]);
                }
                else
                {
                    PRINTF("\r\n");
                }
                PRINTF("\r\nWaiting for \"Hey NXP!\" wake word...\n");
            }
            break;
        case SRTM_Command_VIT_Reset:
            PWR_AllowDeepSleep();
            break;
        }
    }
}

/*!
 * @brief Software timer callback.
 */
static void SwTimerCallback(TimerHandle_t xTimer)
{
    /* Send timeout event to App task */
    xEventGroupSetBits( s_AppEvent, gAppEvt_Timeout_c );
}


/* Application Task */
static void APP_Task(void *pvParameters)
{
    /* Create application event */
    s_AppEvent = xEventGroupCreate();

    /* Init VIT */
    AUDIO_InitVIT();

    /* Create the software timer. */
    SwTimerHandle = xTimerCreate("SwTimer",          /* Text name. */
                                 SW_TIMER_PERIOD_MS, /* Timer period. */
                                 pdFALSE,            /* Disable auto reload. */
                                 0,                  /* ID is not used. */
                                 SwTimerCallback);   /* The callback function. */

#if USER_ACTION_ENABLE
    PRINTF("Waiting for SW2 button press...\r\n");
#else
    xEventGroupSetBits(s_AppEvent, gAppEvt_Btn1Press_c);
#endif

    while (1)
    {
        uint32_t app_event = xEventGroupWaitBits(s_AppEvent, gAppEvt_EventsAll_c, pdTRUE, pdFALSE, portMAX_DELAY);

        if ( (app_event & gAppEvt_Btn1Press_c) && (s_AppState == USER_ACTION) )
        {
            s_AppState = HWVAD;
            LED_RED_ON();
            AUDIO_StartDmic();
#if HWVAD_ENABLE
            PRINTF("\r\nStarting HWVAD\n");
            AUDIO_StartHWVAD(HWVAD_Callback);
#if HWVAD_BUFFERING
            /* Start DMA to keep audio buffering during HWVAD state */
            AUDIO_StartDmicDma(DMIC_DMA_Callback);
#endif
#else
            xEventGroupSetBits(s_AppEvent, gAppEvt_HWVAD_c);
#endif

#if USER_ACTION_ENABLE
            /* Start timer. */
            xTimerStart(SwTimerHandle, 0);
#endif
        }

        if ( (app_event & gAppEvt_HWVAD_c) && (s_AppState == HWVAD) )
        {
            s_AppState = SWVAD_OR_PROCESSING;
            LED_RED_OFF();
            LED_GREEN_ON();
            /* Change DMIC clock if needed */
            AUDIO_StartProcessing();
#if !(HWVAD_ENABLE && HWVAD_BUFFERING)
            AUDIO_StartDmicDma(DMIC_DMA_Callback);
#endif

#if DMIC_USE_FRO
            BOARD_SetPmicCoreVoltage(kPCA9420_Mode3, kPCA9420_Sw1OutVolt0V700);
#endif
            PRINTF("\r\nStarting Processing");
            PRINTF("\r\nWaiting for \"Hey NXP!\" wake word...\n");

#if HWVAD_ENABLE
            /* Restart timer. */
            xTimerReset(SwTimerHandle, 0);
#endif
        }

        if( app_event & gAppEvt_DMIC_Data_c && s_AppState == SWVAD_OR_PROCESSING )
        {
            int16_t *sampleData;
            while (AUDIO_GetNextFrame(&sampleData))
            {
                AUDIO_SendDataToDSP(&sampleData);
            }
        }

        if( app_event & gAppEvt_Timeout_c )
        {
            s_AppState = USER_ACTION;
            LED_RED_OFF();
            LED_GREEN_OFF();
            LED_BLUE_OFF();
#if USER_ACTION_ENABLE
            PRINTF("Waiting for SW2 button press...\r\n");
#else
            xEventGroupSetBits(s_AppEvent, gAppEvt_Btn1Press_c);
#endif

            AUDIO_Deinit();
            AUDIO_ResetVIT();
#if DMIC_USE_FRO
            BOARD_SetPmicCoreVoltage(kPCA9420_Mode3, kPCA9420_Sw1OutVolt0V600);
#endif
        }

        if ( (app_event & gAppEvt_Btn2Press_c) )
        {
            static bool toggle = false;
            toggle = !toggle;
            if (toggle)
            {
                PWR_DisallowDeepSleep();
                PRINTF("Deep Sleep disallowed\r\n");
            }
            else
            {
                PWR_AllowDeepSleep();
                PRINTF("Deep Sleep allowed\r\n");
            }
        }
    }
}

void APP_Idle(uint32_t xModifiableIdleTime)
{
    /* Enter Deep Sleep mode if allowed (DSP is idle) */
    if (PWR_DeepSleepAllowed())
    {
        POWER_EnterDeepSleep(&s_PowerConfig[s_AppState].pd_sleep_config[0]);

    }
    else
    {
        BOARD_DisablePeripheralsDuringSleep();
        POWER_EnterSleep();
        BOARD_RestorePeripheralsAfterSleep();
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware.*/
    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Copy DSP image to RAM */
    BOARD_DSP_CopyImage();

#if defined(ENABLE_RAM_VECTOR_TABLE)
    /* Install one IRQ handler to move vector table to RAM */
    InstallIRQHandler(HardFault_IRQn, (uint32_t)HardFault_Handler);
#endif

    /* Disable FlexSPI and other peripherals */
    BOARD_DisableUnusedPeripherals();

    /* Configure PMIC and power configuration for the different app states */
    BOARD_PmicConfig();
    APP_StatesPowerConfig();

    /* Init IO */
    GPIO_PortInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT);
    GPIO_PortInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT);
    GPIO_PortInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT);
    BOARD_InitLEDsPins();
    BOARD_InitBUTTONSPins();
    LED_Init();
    BUTTON_Init();

#if configUSE_TICKLESS_IDLE == 2
    /* Enable 32KHz Oscillator clock */
    CLOCK_EnableOsc32K(true);
    CLOCK_EnableClock(kCLOCK_Rtc);
    RTC->CTRL |= RTC_CTRL_SWRESET_MASK;
    /* Initialize RTC timer */
    RTC_Init(RTC);

    /* Enable RTC interrupt */
    RTC_EnableInterrupts(RTC, RTC_CTRL_WAKE1KHZ_MASK);
    EnableDeepSleepIRQ(RTC_IRQn);
#endif

    /* Clear MUA reset before run DSP core */
    RESET_PeripheralReset(kMU_RST_SHIFT_RSTn);

    /* Initialize RPMsg IPC interface between ARM and DSP cores. */
    BOARD_DSP_IPC_Init();

    /* Start DSP core */
    BOARD_DSP_Init();

    /* Create IPC processing task */
    if ( xTaskCreate( APP_DSP_IPC_Task, "DSP Msg Task", configMINIMAL_STACK_SIZE + 100,
                    NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS )
    {
        PRINTF("\r\nFailed to create IPC task\r\n");
        while (1);
    }

    /* Print the initial banner */
    PRINTF("Power optimized Voice UI application software pack\r\n\r\n");
    PRINTF("M33 clock: %d MHz\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk)/1000000);
    PRINTF("DSP clock: %d MHz\r\n", CLOCK_GetFreq(kCLOCK_DspCpuClk)/1000000);

    if (xTaskCreate(APP_Task, "APP_Task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 1, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1) ;
    }

    /* Run scheduler */
    vTaskStartScheduler();
    for (;;) ;
}
