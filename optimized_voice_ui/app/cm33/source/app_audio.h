/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef APP_AUDIO_H_
#define APP_AUDIO_H_

/* VIT detection results */
#define VIT_WW_DETECTED 1
#define VIT_VC_DETECTED 2

typedef enum
{
    VIT_ALL_MODULE_DISABLE        = 0,     // None module activated
    VIT_SPECIFIC_MODE_ENABLE      = 1,     // Reserved for specific VIT validation : not to be used
    VIT_LPVAD_ENABLE              = 2,     // Low power VAD module activated
    VIT_WAKEWORD_ENABLE           = 4,     // Wake Word module activated
    VIT_VOICECMD_ENABLE           = 8,     // Voice Commands module activated
    VIT_ALL_MODULE_ENABLE         = VIT_LPVAD_ENABLE | VIT_WAKEWORD_ENABLE | VIT_VOICECMD_ENABLE, // LPVAD, Wake word and Voice commands activated
}VIT_OperatingMode_en;

typedef void (*audio_callback_t)(void);

bool AUDIO_GetNextFrame(int16_t** buffer);
void AUDIO_StartDmic(void);
void AUDIO_StartHWVAD(audio_callback_t callback);
void AUDIO_StartDmicDma(audio_callback_t callback);
void AUDIO_StartProcessing(void);
void AUDIO_Deinit(void);
void AUDIO_InitVIT(void);
void AUDIO_ResetVIT(void);
void AUDIO_SendDataToDSP(int16_t** data);

#endif /* APP_AUDIO_H_ */
