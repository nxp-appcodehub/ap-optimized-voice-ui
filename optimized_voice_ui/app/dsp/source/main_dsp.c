/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019, 2022-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <xtensa/config/core.h>
#include <xtensa/xos.h>
#include "fsl_debug_console.h"
#include "fsl_memory.h"
#include "fsl_sema42.h"

#include "dsp_config.h"
#include "board_fusionf1.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "srtm_config.h"

#include "vit_app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DSP_THREAD_STACK_SIZE (8 * 1024)
#define DSP_THREAD_PRIORITY   (XOS_MAX_PRIORITY - 3)

#define APP_SEMA42 SEMA42
#define SEMA_PRINTF_NUM	  0
#define SEMA_STARTUP_NUM  1
#define SEMA_CORE_ID_DSP  3

#if defined(PRINTF)
#undef PRINTF
#endif
#define PRINTF
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

typedef struct _dsp_handle_t
{
    struct rpmsg_lite_instance *rpmsg;
    struct rpmsg_lite_endpoint *ept;
    XosMsgQueue *rpmsg_queue;
    XosEvent event;
} dsp_handle_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
dsp_handle_t dsp;
static uint8_t dsp_thread_stack[DSP_THREAD_STACK_SIZE];

static int16_t audio_buffer[VIT_SAMPLES_PER_10MS_FRAME]; /* 10ms audio buffer for VIT */

/*******************************************************************************
 * Code
 ******************************************************************************/

static void XOS_INIT(void)
{
    xos_set_clock_freq(XOS_CLOCK_FREQ);
    xos_start_system_timer(-1, 0);
}

static int rpmsg_callback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    XosMsgQueue *queue = (XosMsgQueue *)priv;

    if (payload_len == sizeof(srtm_message))
    {
        xos_msgq_put(queue, payload);
    }
    else
    {
        /* Error / invalid message received. */
    }

    return RL_RELEASE;
}

static int DSP_MSG_Process(dsp_handle_t *dsp, srtm_message *msg)
{
    srtm_message_type_t input_type = msg->head.type;
    VIT_ReturnStatus_en VIT_Status;
    VIT_DetectionStatus_en VIT_DetectionResults = {0};	// VIT detection result
    VIT_WakeWord_st WakeWord = {0};                     // Wakeword info
    VIT_VoiceCommand_st VoiceCommand = {0};             // Voice Command info

    int nb_frames;
    uint32_t data;

    /* Sanity check */
    if ((msg->head.majorVersion != SRTM_VERSION_MAJOR) || (msg->head.minorVersion != SRTM_VERSION_MINOR))
    {
        PRINTF("SRTM version doesn't match!\r\n");
        return -1;
    }


    switch (msg->head.command)
    {
        /* Return SDK key component versions to host*/
        case SRTM_Command_VIT_Init:
        	VIT_Status = VIT_Initialize(msg->param[0]);
        	msg->param[0] = VIT_Status;
            if (VIT_Status != VIT_SUCCESS)
            {
                msg->error = SRTM_Status_Error;
            }
            break;
        case SRTM_Command_VIT_Process:
            nb_frames = msg->param[0];
            data = msg->param[1];
            msg->param[0] = VIT_SUCCESS;

            for (int i = 0; i < nb_frames; i++)
            {
            	/* M33-based address converted to DSP */
				memcpy(&audio_buffer, (int16_t *) (data - FSL_MEM_M33_ALIAS_OFFSET + i*VIT_SAMPLES_PER_10MS_FRAME*sizeof(int16_t)) , sizeof(audio_buffer) );
            	VIT_Status = VIT_Execute((int16_t *)&audio_buffer, &VIT_DetectionResults, &WakeWord, &VoiceCommand);

            	if (VIT_Status != VIT_SUCCESS)
            	{
            		msg->param[0] = VIT_Status;
                    msg->error = SRTM_Status_Error;
            	}

            	if ((VIT_DetectionResults == VIT_WW_DETECTED) || (VIT_DetectionResults == VIT_VC_DETECTED))
            	{
                    msg->param[1] = VIT_DetectionResults;
                    msg->param[2] = WakeWord.Id;
                    msg->param[3] = (uint32_t) WakeWord.pName + FSL_MEM_M33_ALIAS_OFFSET;
                    msg->param[4] = VoiceCommand.Id;
                    msg->param[5] = (uint32_t) VoiceCommand.pName + FSL_MEM_M33_ALIAS_OFFSET;
            	}
            }
            /* Set command back to process */
    		msg->head.command = SRTM_Command_VIT_Process;
        	break;
        case SRTM_Command_VIT_Reset:
        	VIT_Status = VIT_Reset();
        	msg->param[0] = VIT_Status;
            if (VIT_Status != VIT_SUCCESS)
            {
                msg->error = SRTM_Status_Error;
            }
            break;
        /* Unknown message. */
        default:
            msg->head.type = SRTM_MessageTypeNotification;
            msg->error     = SRTM_Status_InvalidMessage;
            break;
    }

    /* Send response message, if request was received */
    if (input_type == SRTM_MessageTypeRequest)
    {
        /* Prepare response msg */
        msg->head.type = SRTM_MessageTypeResponse;

        /* Send response */
        rpmsg_lite_send(dsp->rpmsg, dsp->ept, MCU_EPT_ADDR, (char *)msg, sizeof(srtm_message), RL_DONT_BLOCK);
    }

    return 0;
}

/*!
 * @brief Main function
 */
int DSP_Main(void *arg, int wake_value)
{
    dsp_handle_t *dsp = (dsp_handle_t *)arg;
    void *rpmsg_shmem_base;
    srtm_message msg;
    int status;

    SEMA42_Lock(APP_SEMA42, SEMA_STARTUP_NUM, SEMA_CORE_ID_DSP);

    PRINTF("[DSP] start\r\n");

#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    rpmsg_shmem_base = (void *)MEMORY_ConvertMemoryMapAddress((uint32_t)RPMSG_LITE_SHMEM_BASE, kMEMORY_Local2DMA);
#else
    rpmsg_shmem_base = RPMSG_LITE_SHMEM_BASE;
#endif

    dsp->rpmsg       = rpmsg_lite_remote_init(rpmsg_shmem_base, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    dsp->rpmsg_queue = rpmsg_queue_create(dsp->rpmsg);

    rpmsg_lite_wait_for_link_up(dsp->rpmsg, RL_BLOCK);

    PRINTF("[D] established RPMsg link\r\n");

    SEMA42_Unlock(APP_SEMA42, SEMA_STARTUP_NUM);

    dsp->ept = rpmsg_lite_create_ept(dsp->rpmsg, DSP_EPT_ADDR, rpmsg_queue_rx_cb, (void *)dsp->rpmsg_queue);

    while (1)
    {
        /* Block until receive message from ARM core */
        status =
            rpmsg_queue_recv(dsp->rpmsg, dsp->rpmsg_queue, NULL, (char *)&msg, sizeof(srtm_message), NULL, RL_BLOCK);
        if (status != RL_SUCCESS)
        {
            xos_fatal_error(status, "Failed to get item from RPMsg queue.\r\n");
        }

        /* Process request */
        status = DSP_MSG_Process(dsp, &msg);
    }

    rpmsg_lite_destroy_ept(dsp->rpmsg, dsp->ept);
    rpmsg_lite_deinit(dsp->rpmsg);

    return 0;
}

/*!
 * @brief Main function
 */
int main(void)
{
    XosThread thread_main;

    XOS_INIT();

//    BOARD_InitDebugConsole();

    /* SEMA42 init */
    SEMA42_Init(SEMA42);

    xos_event_create(&dsp.event, 0x3, XOS_EVENT_AUTO_CLEAR);

    xos_thread_create(&thread_main, NULL, DSP_Main, &dsp, "DSP Main", dsp_thread_stack, DSP_THREAD_STACK_SIZE,
                      DSP_THREAD_PRIORITY, 0, 0);

    /* Start XOS scheduler - does not return */
    xos_start(0);

    /* Should not reach this statement. */
    return 0;
}
