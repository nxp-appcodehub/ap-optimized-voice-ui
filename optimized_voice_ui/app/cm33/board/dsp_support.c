/*
 * Copyright 2018-2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "board.h"
#include "fsl_dsp.h"
#include "fsl_sema42.h"
#include "dsp_support.h"
#include "pmic_support.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
#if DSP_IMAGE_COPY_TO_RAM
void BOARD_DSP_CopyImage(void)
{
    dsp_copy_image_t reset_image;
    dsp_copy_image_t text_image;
    dsp_copy_image_t data_image;

    reset_image.destAddr = DSP_RESET_ADDRESS;
    text_image.destAddr  = DSP_TEXT_ADDRESS;
    data_image.destAddr  = DSP_SRAM_ADDRESS;

#if defined(__ICCARM__)
#pragma section = "__dsp_reset_section"
    reset_image.srcAddr = DSP_IMAGE_RESET_START;
    reset_image.size    = DSP_IMAGE_RESET_SIZE;
#pragma section = "__dsp_text_section"
    text_image.srcAddr = DSP_IMAGE_TEXT_START;
    text_image.size    = DSP_IMAGE_TEXT_SIZE;
#pragma section = "__dsp_data_section"
    data_image.srcAddr = DSP_IMAGE_DATA_START;
    data_image.size    = DSP_IMAGE_DATA_SIZE;
#elif defined(__GNUC__)
    reset_image.srcAddr = DSP_IMAGE_RESET_START;
    reset_image.size    = DSP_IMAGE_RESET_SIZE;
    text_image.srcAddr  = DSP_IMAGE_TEXT_START;
    text_image.size     = DSP_IMAGE_TEXT_SIZE;
    data_image.srcAddr  = DSP_IMAGE_DATA_START;
    data_image.size     = DSP_IMAGE_DATA_SIZE;
#endif

    /* Copy application from RAM to DSP_TCM */
    DSP_CopyImage(&reset_image);
    DSP_CopyImage(&text_image);
    DSP_CopyImage(&data_image);
}
#endif

void BOARD_DSP_Init(void)
{
    /* Clear SEMA42 reset */
    RESET_PeripheralReset(kSEMA_RST_SHIFT_RSTn);

    /* SEMA42 init */
    SEMA42_Init(SEMA42);
    /* Reset the sema42 gate */
    SEMA42_ResetAllGates(SEMA42);

    CLOCK_AttachClk(kFRO_DIV1_to_DSP_MAIN_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivDspCpuClk, 1);

    /* Set DSP to use primary static vector base(0x0000_0000) and remap to 0x80000. */
    DSP_SetVecRemap(kDSP_StatVecSelPrimary, 0x200U);

    /* Initializing DSP core */
    DSP_Init();

    /* Run DSP core */
    DSP_Start();

    /* Wait for the DSP to lock the semaphore */
    while (kSEMA42_LockedByProc3 != SEMA42_GetGateStatus(SEMA42, 1))
    {
    }

    /* Wait for the DSP to unlock the semaphore 1 */
    while (SEMA42_GetGateStatus(SEMA42, 1))
    {
    }
}
