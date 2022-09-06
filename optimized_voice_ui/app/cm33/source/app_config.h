/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/* App configuration */
#define USER_ACTION_ENABLE  1
#define HWVAD_ENABLE        1
#define SWVAD_ENABLE        0   /* Enable VIT Low-power VAD */
#define DMIC_USE_FRO        0   /* Uses more power and requires higher voltage during DeepSleep */
#define HWVAD_BUFFERING     1   /* Save history buffer to have past data before the HWVAD detection */
#define TIMEOUT_SECS        15  /* Timeout to return to the User Action state */
#define VOICECMD_ENABLE     1   /* Enable VIT voice command detection */

/* SRAM Partition configurations */
#define USED_SRAM_MASK          0x0007180F      /* All used SRAM partitions: CM33 + DMA + DSP */
#define AHB_SRAM_ACCESS_MASK    USED_SRAM_MASK  /* AHB partition access: CM33 + DMA */
#define AXI_SRAM_ACCESS_MASK    0xFFFFFFFF      /* No AXI masters (graphics) */
#define DSP_SRAM_ACCESS_MASK    USED_SRAM_MASK

#endif /* APP_CONFIG_H_ */
