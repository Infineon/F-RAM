/*******************************************************************************
* \file cy_smif_memconfig.h
* \version 1.0
*
* \brief
* Provides declarations of the SMIF-driver memory configuration.
*
* Note: This is an auto generated file. Do not modify it.
*
********************************************************************************
* \copyright
* Copyright 2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#ifndef CY_SMIF_MEMCONFIG_H
#define CY_SMIF_MEMCONFIG_H
#include "smif/cy_smif_memslot.h"

#define CY_SMIF_DEVICE_NUM 1

extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_readCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_writeEnCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_writeDisCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_eraseCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_chipEraseCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_programCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_readStsRegQeCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_readStsRegWipCmd;
extern const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_writeStsRegQeCmd;

extern const cy_stc_smif_mem_device_cfg_t Excelon_Ultra_SlaveSlot_2_DeviceCfg;

extern const cy_stc_smif_mem_config_t Excelon_Ultra_SlaveSlot_2;

extern const cy_stc_smif_mem_config_t* const smifMemConfigs[CY_SMIF_DEVICE_NUM];

extern const cy_stc_smif_block_config_t smifBlockConfig;


#endif /*CY_SMIF_MEMCONFIG_H*/
