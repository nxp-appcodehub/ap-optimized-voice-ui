/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VIT_APP_H_
#define _VIT_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "PL_platformTypes_HIFI4_FUSIONF1.h"
#include "VIT.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

VIT_ReturnStatus_en VIT_Initialize(VIT_OperatingMode_en vit_operating_mode);
VIT_ReturnStatus_en VIT_Execute(int16_t *pIn, VIT_DetectionStatus_en *pVIT_DetectionResults, VIT_WakeWord_st *pWakeWord, VIT_VoiceCommand_st *pVoiceCommand);
VIT_ReturnStatus_en VIT_Reset(void);
#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _VIT_APP_H_*/
