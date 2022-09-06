/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pmic_support.h"

#ifndef APP_POWER_H_
#define APP_POWER_H_

typedef struct _state_power_config
{
    uint32_t pd_sleep_config[4];
} state_power_config_t;

void PWR_GetDefaultPowerConfig(state_power_config_t * config);
void PWR_AllowDeepSleep(void);
void PWR_DisallowDeepSleep(void);
bool PWR_DeepSleepAllowed(void);
void BOARD_PmicConfig(void);
void BOARD_SetPmicCoreVoltage(pca9420_mode_t modeBase, pca9420_sw1_out_t volt);
void BOARD_DisableUnusedPeripherals();
void BOARD_DisablePeripheralsDuringSleep(void);
void BOARD_RestorePeripheralsAfterSleep(void);


#endif /* APP_POWER_H_ */
