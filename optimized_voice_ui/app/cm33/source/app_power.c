/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_config.h"
#include "app_power.h"
#include "board.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_DeepSleepDisallowCnt = 0;
static pca9420_modecfg_t pca9420ModeCfg[4];

/*******************************************************************************
 * Code
 ******************************************************************************/
void PWR_GetDefaultPowerConfig(state_power_config_t * config)
{
    /* Common deep sleep configuration for all states */
    config->pd_sleep_config[0] = SYSCTL0_PDSLEEPCFG0_RBB_PD_MASK | \
                                 SYSCTL0_PDSLEEPCFG0_RBBSRAM_PD_MASK | \
                                 SYSCTL0_PDSLEEPCFG0_LPOSC_PD_MASK;
    /* Use PMIC MODE 3 for Deep Sleep */
    config->pd_sleep_config[0] |= SYSCTL0_PDSLEEPCFG0_PMIC_MODE1(1) | SYSCTL0_PDSLEEPCFG0_PMIC_MODE0(1);
    config->pd_sleep_config[1] = 0x00000000;
    config->pd_sleep_config[1] |= SYSCTL0_PDSLEEPCFG1_DSP_PD_MASK;
    config->pd_sleep_config[2] = USED_SRAM_MASK;    /* Keep array power for all RAM partitions in use. */
    config->pd_sleep_config[3] = 0x00000000;        /* Turn off all periphery power. */
}

void PWR_AllowDeepSleep(void)
{
    if(s_DeepSleepDisallowCnt != 0)
    {
        s_DeepSleepDisallowCnt--;
    }
}

void PWR_DisallowDeepSleep(void)
{
    if(s_DeepSleepDisallowCnt != 0xFF)
    {
        s_DeepSleepDisallowCnt++;
    }
}

bool PWR_DeepSleepAllowed(void)
{
    bool returnValue;

    returnValue = s_DeepSleepDisallowCnt == 0 ? true : false;

    return returnValue;
}

void BOARD_PmicConfig(void)
{
    pca9420_config_t pca9420Config;
    power_pad_vrange_t vrange;

    POWER_SetLvdFallingTripVoltage(kLvdFallingTripVol_720);

    /* Initialize PMIC PCA9420 */
    BOARD_InitPmic();

    /* Configure I2C to communicate with PMIC.*/
    CLOCK_AttachClk(kFRO_DIV4_to_FLEXCOMM15);
    BOARD_PMIC_I2C_Init();

    /* Initialize the PMIC. */
    PCA9420_GetDefaultConfig(&pca9420Config);
    pca9420Config.I2C_SendFunc    = BOARD_PMIC_I2C_Send;
    pca9420Config.I2C_ReceiveFunc = BOARD_PMIC_I2C_Receive;
    PCA9420_Init(&pca9420Handle, &pca9420Config);

    /* Inform power lib that an external PMIC is used. */
    POWER_UpdatePmicRecoveryTime(1);

    /* Override default PMIC configurations */
    /* Mode 0: Default for max performance */
    PCA9420_GetDefaultModeConfig(&pca9420ModeCfg[0]);

    /* Mode 1: Active w/FRO96 VDDCORE 0.75V.
     * See Table 6. from datasheet Rev. 2 */
    PCA9420_GetDefaultModeConfig(&pca9420ModeCfg[1]);
    pca9420ModeCfg[1].sw1OutVolt = kPCA9420_Sw1OutVolt0V750;

    /* Mode 2: Deep Sleep Wake up VDDCORE 0.75V.
     * Used in cases where Active voltage is lower than LVD rising level 0 (0.74V) */
    PCA9420_GetDefaultModeConfig(&pca9420ModeCfg[2]);
    pca9420ModeCfg[2].sw1OutVolt = kPCA9420_Sw1OutVolt0V750;

    /* Mode 3: Deep Sleep VDDCORE 0.6V. */
    PCA9420_GetDefaultModeConfig(&pca9420ModeCfg[3]);
    pca9420ModeCfg[3].sw1OutVolt = kPCA9420_Sw1OutVolt0V600;

    /* Send new configuration to PMIC. */
    PCA9420_WriteModeConfigs(&pca9420Handle, kPCA9420_Mode0, &pca9420ModeCfg[0], 4);

    POWER_PmicPowerModeSelectControl(0xFF);

    POWER_DisableLVD();

    /* Switch to Mode 1 Active */
    PCA9420_SwitchMode(&pca9420Handle, kPCA9420_Mode1);

    /* Configure pads voltage ranges according to PMIC settings. */
    vrange.Vdde0Range = kPadVol_171_198;
    vrange.Vdde1Range = kPadVol_171_198;
    vrange.Vdde2Range = kPadVol_171_198;
    vrange.Vdde3Range = kPadVol_300_360;
    vrange.Vdde4Range = kPadVol_171_198;

    POWER_SetPadVolRange(&vrange);

    CLOCK_DisableClock(kCLOCK_Flexcomm15);
    CLOCK_AttachClk(kNONE_to_FLEXCOMM15);
}

void BOARD_SetPmicCoreVoltage(pca9420_mode_t modeBase, pca9420_sw1_out_t volt)
{
    CLOCK_AttachClk(kFRO_DIV4_to_FLEXCOMM15);
    CLOCK_EnableClock(kCLOCK_Flexcomm15);

    pca9420_modecfg_t pca9420ModeCfg;
    PCA9420_GetDefaultModeConfig(&pca9420ModeCfg);
    pca9420ModeCfg.sw1OutVolt = volt;
    PCA9420_WriteModeConfigs(&pca9420Handle, modeBase, &pca9420ModeCfg, 1);

    CLOCK_DisableClock(kCLOCK_Flexcomm15);
    CLOCK_AttachClk(kNONE_to_FLEXCOMM15);
}

void BOARD_DisableUnusedPeripherals()
{
    /* Disable FlexSPI0 as flash is no longer used */
    BOARD_DeinitFlash(FLEXSPI0);
    /* Optimize unused ISP/FlexSPI pins for low power */
    BOARD_SetDeepSleepPinConfig();

    /* Power down unused SRAM partitions */
    SYSCTL0->PDRUNCFG2_SET = ~USED_SRAM_MASK;
    SYSCTL0->PDRUNCFG3_SET = ~USED_SRAM_MASK;
    /* Limit access to used partitions*/
    SYSCTL0->AHB_SRAM_ACCESS_DISABLE = ~AHB_SRAM_ACCESS_MASK;
    SYSCTL0->AXI_SRAM_ACCESS_DISABLE = ~AXI_SRAM_ACCESS_MASK;
    SYSCTL0->DSP_SRAM_ACCESS_DISABLE = ~DSP_SRAM_ACCESS_MASK;

    /* Disable system PLL */
    CLOCK_DeinitSysPfd(kCLOCK_Pfd0);
    CLOCK_DeinitSysPfd(kCLOCK_Pfd1);
    CLOCK_DeinitSysPfd(kCLOCK_Pfd2);
    CLOCK_DeinitSysPfd(kCLOCK_Pfd3);
    CLOCK_DeinitSysPll();
    CLOCK_AttachClk(kNONE_to_SYS_PLL);

    /* Disable unused peripherals. */
    POWER_EnablePD(kPDRUNCFG_PD_AUDPLL_LDO);
    POWER_EnablePD(kPDRUNCFG_PD_AUDPLL_ANA);
    POWER_EnablePD(kPDRUNCFG_PD_ADC);
    POWER_EnablePD(kPDRUNCFG_LP_ADC);
    POWER_EnablePD(kPDRUNCFG_PD_ADC_TEMPSNS);
    POWER_EnablePD(kPDRUNCFG_PD_PMC_TEMPSNS);
    POWER_EnablePD(kPDRUNCFG_PD_ACMP);

    /* PowerDown clock, RAM, and access to unused peripherals */
    CLOCK_AttachClk(kNONE_to_SDIO0_CLK);
//    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_SDIO0_VDET);
//    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_SDIO0_REF);
    POWER_EnablePD(kPDRUNCFG_APD_USDHC0_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_USDHC0_SRAM);
    /* SDIO1 */
    CLOCK_AttachClk(kNONE_to_SDIO1_CLK);
//    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_SDIO1_VDET);
//    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_SDIO1_REF);
    POWER_EnablePD(kPDRUNCFG_APD_USDHC1_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_USDHC1_SRAM);
    /* CASPER */
    CLOCK_DisableClock(kCLOCK_Casper);
    POWER_EnablePD(kPDRUNCFG_PPD_CASPER_SRAM);
    /* ROM */
    CLOCK_DisableClock(kCLOCK_RomCtrlr);
    POWER_EnablePD(kPDRUNCFG_PD_ROM);
    /* OTP */
    CLOCK_DisableClock(kCLOCK_OtpCtrl);
    POWER_EnablePD(kPDRUNCFG_PD_OTP);
    /* FLEXSPI1 */
    CLOCK_AttachClk(kNONE_to_FLEXSPI1_CLK);
//    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_FSPI1_VDET);
//    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_FSPI1_REF);
    POWER_EnablePD(kPDRUNCFG_APD_FLEXSPI1_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_FLEXSPI1_SRAM);

    /* PMC */
    POWER_EnablePD(kPDRUNCFG_LP_VDD_COREREG);
    POWER_EnablePD(kPDRUNCFG_LP_PMCREF);
    POWER_EnablePD(kPDRUNCFG_PD_HVD1V8);
    POWER_EnablePD(kPDRUNCFG_LP_LVDCORE);
    POWER_EnablePD(kPDRUNCFG_PD_HVDCORE);

    /* FLEXSPI0 */
    CLOCK_DisableClock(kCLOCK_Flexspi0);
    CLOCK_AttachClk(kNONE_to_FLEXSPI0_CLK);
//    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_FSPI0_VDET);
//    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_FSPI0_REF);
    POWER_EnablePD(kPDRUNCFG_APD_FLEXSPI0_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_FLEXSPI0_SRAM);

    CLOCK_DisableClock(kCLOCK_Rng);
    CLOCK_DisableClock(kCLOCK_Puf);
    CLOCK_DisableClock(kCLOCK_HashCrypt);
    CLOCK_DisableClock(kCLOCK_Flexcomm14);
    CLOCK_DisableClock(kCLOCK_Flexcomm2);
    CLOCK_DisableClock(kCLOCK_Crc);
    CLOCK_DisableClock(kCLOCK_InputMux);

    CLOCK_AttachClk(kNONE_to_WDT0_CLK);
    CLOCK_AttachClk(kNONE_to_WDT1_CLK);
    CLOCK_AttachClk(kNONE_to_OSTIMER_CLK);
    CLOCK_AttachClk(kNONE_to_I3C_TC_CLK);
    CLOCK_AttachClk(kNONE_to_32KHZWAKE_CLK);
    CLKCTL1->I3C01FCLKSTSTCLKSEL = 7U;

    CLOCK_EnableSysOscClk(false, true, 0U);
    CLKCTL0->SYSOSCBYPASS = 7U;

    /* Apply power setting. */
    POWER_ApplyPD();
}

void BOARD_DisablePeripheralsDuringSleep(void)
{
    CLOCK_DisableClock(kCLOCK_Flexcomm0);
    CLOCK_AttachClk(kNONE_to_FLEXCOMM0);

    CLOCK_DisableClock(kCLOCK_HsGpio0);
    CLOCK_DisableClock(kCLOCK_HsGpio1);
    CLOCK_DisableClock(kCLOCK_HsGpio3);
    CLOCK_DisableClock(kCLOCK_HsGpio4);
}

void BOARD_RestorePeripheralsAfterSleep(void)
{
    CLOCK_EnableClock(kCLOCK_HsGpio0);
    CLOCK_EnableClock(kCLOCK_HsGpio1);
    CLOCK_EnableClock(kCLOCK_HsGpio3);
    CLOCK_EnableClock(kCLOCK_HsGpio4);

    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_EnableClock(kCLOCK_Flexcomm0);
}
