/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <xtensa/config/core.h>
#include <xtensa/xos.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#include "board_fusionf1.h"

#include "vit_app.h"
#include "VIT_Model_en.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEVICE_ID				VIT_IMXRT500
#define VIT_CMD_TIME_SPAN		3.0

/*
*   Useful Definitions : not to be changed
*/
// MEMORY
#define MEMORY_ALIGNMENT             8     // in bytes

// Current VIT lib is supporting only one 1 input channel
#define NUMBER_OF_CHANNELS          _1CHAN

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static VIT_Handle_t              VITHandle = PL_NULL;                      // VIT handle pointer
static VIT_InstanceParams_st     VITInstParams;                            // VIT instance parameters structure
static VIT_ControlParams_st      VITControlParams;                         // VIT control parameters structure
static PL_MemoryTable_st         VITMemoryTable;                           // VIT memory table descriptor
static PL_BOOL                   InitPhase_Error = PL_FALSE;
static PL_INT16                  *VIT_InputData;
static PL_INT8                   *pMemory[PL_NR_MEMORY_REGIONS];
/*******************************************************************************
 * Code
 ******************************************************************************/

VIT_ReturnStatus_en VIT_Initialize(VIT_OperatingMode_en vit_operating_mode)
{
    VIT_ReturnStatus_en     VIT_Status;                             /* Function call status */

    VIT_Status = VIT_SetModel(VIT_Model_en, VIT_MODEL_IN_FAST_MEM);

    if (VIT_Status != VIT_SUCCESS)
    {
        return VIT_Status;
    }

    /*
     *   Configure VIT Instance Parameters
     */
    VITInstParams.SampleRate_Hz            = VIT_SAMPLE_RATE;
    VITInstParams.SamplesPerFrame          = VIT_SAMPLES_PER_10MS_FRAME;
    VITInstParams.NumberOfChannel          = NUMBER_OF_CHANNELS;
    VITInstParams.DeviceId                 = DEVICE_ID;
    VITInstParams.APIVersion               = VIT_API_VERSION;

    /*
     *   VIT get memory table : Get size info per memory type
     */
    VIT_Status = VIT_GetMemoryTable( PL_NULL,                // VITHandle param should be NULL
                                 &VITMemoryTable,
                                 &VITInstParams);
    if (VIT_Status != VIT_SUCCESS)
    {
//        PRINTF("VIT_GetMemoryTable error : %d\n", VIT_Status);
        return VIT_Status;
    }

    /*
     *   Reserve memory space : Malloc for each memory type
     */
    for (int i = 0; i < PL_NR_MEMORY_REGIONS; i++)
    {
        /* Log the memory size */
        // DSP_PRINTF("Memory region %d, size %d in Bytes\n", (int)i, (int)VITMemoryTable.Region[i].Size);
        if (VITMemoryTable.Region[i].Size != 0)
        {
            // reserve memory space
            // NB : VITMemoryTable.Region[PL_MEMREGION_PERSISTENT_FAST_DATA] should be alloacted
            //      in the fastest memory of the platform (when possible) - this is not the case in this example.
            pMemory[i] = malloc(VITMemoryTable.Region[i].Size + MEMORY_ALIGNMENT);
            VITMemoryTable.Region[i].pBaseAddress = (void *)pMemory[i];

            // DSP_PRINTF(" Memory region address %p\n", VITMemoryTable.Region[i].pBaseAddress);
        }
    }

    /*
    *    Create VIT Instance
    */
    VITHandle = PL_NULL;                            // force to null address for correct memory initialization
    VIT_Status = VIT_GetInstanceHandle( &VITHandle,
                                        &VITMemoryTable,
                                        &VITInstParams);
    if (VIT_Status != VIT_SUCCESS)
    {
        InitPhase_Error = PL_TRUE;
//        PRINTF("VIT_GetInstanceHandle error : %d\n", VIT_Status);
    }

    /*
    *    Test the reset (OPTIONAL)
    */
    if (!InitPhase_Error)
    {
    	VIT_Status = VIT_ResetInstance(VITHandle);
        if (VIT_Status != VIT_SUCCESS)
        {
            InitPhase_Error = PL_TRUE;
//            PRINTF("VIT_ResetInstance error : %d\n", VIT_Status);
        }
    }

    /*
    *   Set and Apply VIT control parameters
    */
    VITControlParams.OperatingMode = vit_operating_mode;
    VITControlParams.Command_Time_Span = VIT_CMD_TIME_SPAN;

    if (!InitPhase_Error)
    {
        VIT_Status = VIT_SetControlParameters(  VITHandle,
                                            &VITControlParams);
        if (VIT_Status != VIT_SUCCESS)
        {
            InitPhase_Error = PL_TRUE;
//            PRINTF("VIT_SetControlParameters error : %d\n", VIT_Status);
        }
    }

    return VIT_Status;
}

VIT_ReturnStatus_en VIT_Execute(int16_t *pIn, VIT_DetectionStatus_en *pVIT_DetectionResults, VIT_WakeWord_st *pWakeWord, VIT_VoiceCommand_st *pVoiceCommand)
{

    VIT_ReturnStatus_en VIT_Status;
    *pVIT_DetectionResults = VIT_NO_DETECTION;  // VIT detection result

    VIT_InputData = (PL_INT16*)pIn;                      // PCM buffer : 16-bit - 16kHz - mono

    VIT_Status = VIT_Process ( VITHandle,
    		                   (void*)VIT_InputData,                               // temporal audio input data
                               pVIT_DetectionResults
                              );

    if (VIT_Status != VIT_SUCCESS)
    {
        // DSP_PRINTF("VIT_Process error : %d\n", Status);
        return VIT_Status;
    }

    if (*pVIT_DetectionResults == VIT_WW_DETECTED)
    {
    	// Retrieve id of the pWakeWord detected
		// String of the Command can also be retrieved (when WW and CMDs strings are integrated in Model)
    	VIT_Status = VIT_GetWakeWordFound( VITHandle, pWakeWord);
		if (VIT_Status != VIT_SUCCESS)
		{
//			DSP_PRINTF("VIT_GetpWakeWordFound error : %d\r\n", VIT_Status);
			return VIT_Status;
		}
    }
    else if (*pVIT_DetectionResults == VIT_VC_DETECTED)
    {
        // Retrieve id of the Voice Command detected
        // String of the Command can also be retrieved (when WW and CMDs strings are integrated in Model)
    	VIT_Status = VIT_GetVoiceCommandFound(VITHandle,
                                              pVoiceCommand
                                              );
        if (VIT_Status != VIT_SUCCESS)
        {
//            DSP_PRINTF("VIT_GetpVoiceCommandFound error : %d\r\n", VIT_Status);
            return VIT_Status;
        }
    }

    return VIT_Status;
}

VIT_ReturnStatus_en VIT_Reset(void)
{
    VIT_ReturnStatus_en VIT_Status;

	VIT_Status = VIT_ResetInstance(VITHandle);
    if (VIT_Status != VIT_SUCCESS)
    {
        return VIT_Status;
    }

    VIT_Status = VIT_SetControlParameters(VITHandle, &VITControlParams);
    if (VIT_Status != VIT_SUCCESS)
    {
        return VIT_Status;
    }
    return VIT_Status;
}
