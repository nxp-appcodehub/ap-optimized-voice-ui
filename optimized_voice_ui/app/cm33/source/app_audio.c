/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "fsl_dmic.h"
#include "fsl_dma.h"
#include "fsl_dmic_dma.h"
#include "fsl_power.h"
#include "fsl_inputmux.h"

#include "dsp_ipc.h"
#include "app_config.h"
#include "app_audio.h"
#include "app_power.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Audio */
#define DEMO_DMIC_RX_CHANNEL    16U
#define FIFO_DEPTH              12U
#define FRAME_SIZE              (320)   /* VIT frame size is 160 samples (16 bit). 10ms @16kHz */
#define NUM_FRAMES              (5)     /* 5 VIT frames = 50ms = 1,600B */
#define RECORD_BUFFER_SIZE      (NUM_FRAMES * FRAME_SIZE)
#define BUFFER_NUM              (10+1)     /* History buffer 10 * 50 ms = 500 ms of audio */
#define HISTORY_BUFFER_SIZE     (BUFFER_NUM * RECORD_BUFFER_SIZE)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* DMIC and DMA */
static dmic_dma_handle_t s_dmicDmaHandle;
static dma_handle_t s_dmicRxDmaHandle;
static dmic_transfer_t s_dmicRxTransfer[BUFFER_NUM];
SDK_ALIGN(static dma_descriptor_t s_dmaDescriptorPingpong[BUFFER_NUM], 16);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_buffer[RECORD_BUFFER_SIZE * BUFFER_NUM], 4);
static uint32_t volatile s_writeIndex = 0U;
static uint32_t volatile s_readIndex  = 0U;

/* App callbacks */
static audio_callback_t s_dmicHwvadCallback;
static audio_callback_t s_dmicDMACallback;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void initMessage(srtm_message *msg)
{
    msg->head.type = SRTM_MessageTypeRequest;
    msg->head.majorVersion = SRTM_VERSION_MAJOR;
    msg->head.minorVersion = SRTM_VERSION_MINOR;
}

/*!
 * @brief Call back for DMIC0 HWVAD Interrupt
 */
static void DMIC0_HWVAD_Callback(void)
{
    volatile int i;

    /* reset hwvad internal interrupt */
    DMIC_CtrlClrIntrHwvad(DMIC0, true);
    /* wait for HWVAD to settle */
    for (i = 0; i <= 500U; i++)
    {
    }
    /*HWVAD Normal operation */
    DMIC_CtrlClrIntrHwvad(DMIC0, false);

    if (s_dmicHwvadCallback != NULL)
    {
        s_dmicHwvadCallback();
    }
}

static void DMIC_DMA_callback(DMIC_Type *base, dmic_dma_handle_t *handle, status_t status, void *userData)
{
    // Update write index and check for wrap-around
    s_writeIndex += RECORD_BUFFER_SIZE;
    if (s_writeIndex >= HISTORY_BUFFER_SIZE)
        s_writeIndex -= HISTORY_BUFFER_SIZE;

    // Drop oldest if full
    if (s_writeIndex == s_readIndex)
    {
        /* Update Mic read index and check for wrap-around */
        s_readIndex += RECORD_BUFFER_SIZE;
        if (s_readIndex >= HISTORY_BUFFER_SIZE)
            s_readIndex -= HISTORY_BUFFER_SIZE;
    }

    if (s_dmicDMACallback != NULL)
    {
        s_dmicDMACallback();
    }
}

bool AUDIO_GetNextFrame(int16_t** buffer)
{
    bool ret = false;
    __disable_irq();

    int32_t read_index = s_readIndex;
    int32_t n_avail_Mic_buffer = s_writeIndex - read_index;

    if (n_avail_Mic_buffer < 0)
        n_avail_Mic_buffer += HISTORY_BUFFER_SIZE;

    if (n_avail_Mic_buffer >= RECORD_BUFFER_SIZE)
    {
        *buffer = (int16_t*) &s_buffer[read_index];

        /* Update Mic read index and check for wrap-around */
        s_readIndex += RECORD_BUFFER_SIZE;
        if (s_readIndex >= HISTORY_BUFFER_SIZE)
            s_readIndex -= HISTORY_BUFFER_SIZE;

        ret = true;
    }
    __enable_irq();
    return ret;
}

void AUDIO_StartDmic(void)
{
    dmic_channel_config_t dmic_channel_cfg;

    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Dmic0Ch0ToDmac0Ch16RequestEna, true);
    INPUTMUX_Deinit(INPUTMUX);

    /* DMIC clock at 1MHz using LPOSC*/
    /* Sample rate = 1MHz/(2*OSR) = 15.625KHz*/
    CLOCK_AttachClk(kLPOSC_to_DMIC);
    CLOCK_SetClkDiv(kCLOCK_DivDmicClk, 1);

    CLOCK_EnableClock(kCLOCK_Dmic0);
    RESET_ClearPeripheralReset(kDMIC_RST_SHIFT_RSTn);

    dmic_channel_cfg.divhfclk = kDMIC_PdmDiv1;
    dmic_channel_cfg.osr = 32U;
    dmic_channel_cfg.gainshft = 6U;
    dmic_channel_cfg.preac2coef = kDMIC_CompValueZero;
    dmic_channel_cfg.preac4coef = kDMIC_CompValueZero;
    dmic_channel_cfg.dc_cut_level = kDMIC_DcCut155;
    dmic_channel_cfg.post_dc_gain_reduce = 8U;
    dmic_channel_cfg.saturate16bit = 1U;
    dmic_channel_cfg.sample_rate = kDMIC_PhyFullSpeed;
#if defined(FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND) && (FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND)
    dmic_channel_cfg.enableSignExtend = false;
#endif

    DMIC_Init(DMIC0);
    DMIC_Use2fs(DMIC0, true);
    DMIC_EnableChannelInterrupt(DMIC0, kDMIC_Channel0, true);
    DMIC_ConfigChannel(DMIC0, kDMIC_Channel0, kDMIC_Left, &dmic_channel_cfg);
    DMIC_EnableChannnel(DMIC0, (DMIC_CHANEN_EN_CH0(1)));
}

void AUDIO_StartHWVAD(audio_callback_t callback)
{
    s_dmicHwvadCallback = callback;

    /* Gain of the noise estimator */
    DMIC_SetGainNoiseEstHwvad(DMIC0, 0x06U);

    /* Gain of the signal estimator */
    DMIC_SetGainSignalEstHwvad(DMIC0, 0x00U);

    /* 00 = first filter by-pass, 01 = hpf_shifter=1, 10 = hpf_shifter=4 */
    DMIC_SetFilterCtrlHwvad(DMIC0, 0x01U);

    /*input right-shift of (GAIN x 2 -10) bits (from -10bits (0000) to +14bits (1100)) */
    DMIC_SetInputGainHwvad(DMIC0, 0x05U);

    /* Reset the filters */
    DMIC_FilterResetHwvad(DMIC0, true);
    DMIC_FilterResetHwvad(DMIC0, false);
    /* Clear interrupt flag and start HWVAD */
    DMIC_CtrlClrIntrHwvad(DMIC0, true);
    /* Delay to clear first spurious interrupt and let the filter converge */
    vTaskDelay( pdMS_TO_TICKS(20) );
    /*HWVAD Normal operation */
    DMIC_CtrlClrIntrHwvad(DMIC0, false);
    DMIC_HwvadEnableIntCallback(DMIC0, DMIC0_HWVAD_Callback);
    EnableDeepSleepIRQ(HWVAD0_IRQn);
}

void AUDIO_StartDmicDma(audio_callback_t callback)
{
    s_dmicDMACallback = callback;

    DMIC_FifoChannel(DMIC0, kDMIC_Channel0, FIFO_DEPTH, true, true);

    /* configurations for DMIC channel */
    DMA_Init(DMA0);
    DMA_EnableChannel(DMA0, DEMO_DMIC_RX_CHANNEL);
    DMA_SetChannelPriority(DMA0, DEMO_DMIC_RX_CHANNEL, kDMA_ChannelPriority1);
    DMA_CreateHandle(&s_dmicRxDmaHandle, DMA0, DEMO_DMIC_RX_CHANNEL);

    /* Transfer configurations for channel0 */
    for (int i = 0; i < BUFFER_NUM; i++)
    {
        s_dmicRxTransfer[i].data                   = s_buffer + i * RECORD_BUFFER_SIZE;
        s_dmicRxTransfer[i].dataWidth              = sizeof(uint16_t);
        s_dmicRxTransfer[i].dataSize               = RECORD_BUFFER_SIZE;
        s_dmicRxTransfer[i].dataAddrInterleaveSize = kDMA_AddressInterleave1xWidth;
        s_dmicRxTransfer[i].linkTransfer           = &s_dmicRxTransfer[i + 1];
    }
    s_dmicRxTransfer[BUFFER_NUM - 1].linkTransfer = &s_dmicRxTransfer[0];

    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle, DMIC_DMA_callback, NULL, &s_dmicRxDmaHandle);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle, s_dmaDescriptorPingpong, BUFFER_NUM);

    SYSCTL0->HWWAKE = SYSCTL0_HWWAKE_DMICWAKE_MASK | SYSCTL0_HWWAKE_DMAC0WAKE_MASK;

    EnableDeepSleepIRQ(DMA0_IRQn);
    /* Clear old samples from FIFOs */
    DMIC_DoFifoReset(DMIC0, kDMIC_Channel0);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle, s_dmicRxTransfer, kDMIC_Channel0);
}

void AUDIO_StartProcessing(void)
{
#if HWVAD_ENABLE
    DisableDeepSleepIRQ(HWVAD0_IRQn);
    DMIC_HwvadDisableIntCallback(DMIC0, DMIC0_HWVAD_Callback);
#endif

#if DMIC_USE_FRO
    dmic_channel_config_t dmic_channel_cfg;

    dmic_channel_cfg.divhfclk            = kDMIC_PdmDiv1;
    dmic_channel_cfg.osr                 = 75U;
    dmic_channel_cfg.gainshft            = 6U;
    dmic_channel_cfg.preac2coef          = kDMIC_CompValueZero;
    dmic_channel_cfg.preac4coef          = kDMIC_CompValueZero;
    dmic_channel_cfg.dc_cut_level        = kDMIC_DcCut155;
    dmic_channel_cfg.post_dc_gain_reduce = 8U;
    dmic_channel_cfg.saturate16bit       = 1U;
    dmic_channel_cfg.sample_rate         = kDMIC_PhyFullSpeed;
#if defined(FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND) && (FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND)
    dmic_channel_cfg.enableSignExtend = false;
#endif

    CLOCK_AttachClk(kNONE_to_DMIC);
    DMIC_ResetChannelDecimator(DMIC0, kDMIC_EnableChannel0, true);

    DMIC_ConfigChannel(DMIC0, kDMIC_Channel0, kDMIC_Left, &dmic_channel_cfg);
    /* DMIC clock at 2.4MHz using FRO96 DIV4*/
    /* Sample rate = 2.4MHz/(2*75) = 16000KHz*/
    CLOCK_AttachClk(kFRO_DIV4_to_DMIC);
    CLOCK_SetClkDiv(kCLOCK_DivDmicClk, 10);

    DMIC_ResetChannelDecimator(DMIC0, kDMIC_EnableChannel0, false);
#endif
}

void AUDIO_Deinit(void)
{
    DMIC_HwvadDisableIntCallback(DMIC0, DMIC0_HWVAD_Callback);
    DisableDeepSleepIRQ(HWVAD0_IRQn);
    DMIC_TransferAbortReceiveDMA(DMIC0, &s_dmicDmaHandle);
    DMIC_DeInit(DMIC0);
    CLOCK_AttachClk(kNONE_to_DMIC);
    DMA_Deinit(DMA0);
    s_writeIndex = 0U;
    s_readIndex  = 0U;
}

void AUDIO_InitVIT(void)
{
    srtm_message msg = {0};
    initMessage(&msg);

    msg.head.command  = SRTM_Command_VIT_Init;
    msg.param[0] = VIT_WAKEWORD_ENABLE;
#if SWVAD_ENABLE
    msg.param[0] |= VIT_LPVAD_ENABLE;
#endif
#if VOICECMD_ENABLE
    msg.param[0] |= VIT_VOICECMD_ENABLE;
#endif

    /* Disallow Deep Sleep until response from DSP is received */
    PWR_DisallowDeepSleep();
    /* Send message to the DSP */
    dsp_ipc_send_sync(&msg);
}

void AUDIO_ResetVIT(void)
{
    srtm_message msg = {0};
    initMessage(&msg);

    msg.head.command  = SRTM_Command_VIT_Reset;

    /* Disallow Deep Sleep until response from DSP is received */
    PWR_DisallowDeepSleep();
    /* Send message to the DSP */
    dsp_ipc_send_sync(&msg);
}

void AUDIO_SendDataToDSP(int16_t** data)
{
    srtm_message msg = {0};
    initMessage(&msg);

    msg.head.command  = SRTM_Command_VIT_Process;
    msg.param[0] = NUM_FRAMES;
    msg.param[1] = (uint32_t)(*data);

    /* Disallow Deep Sleep until DSP finishes processing */
    PWR_DisallowDeepSleep();
    /* Send message to the DSP */
    dsp_ipc_send_sync(&msg);
}
