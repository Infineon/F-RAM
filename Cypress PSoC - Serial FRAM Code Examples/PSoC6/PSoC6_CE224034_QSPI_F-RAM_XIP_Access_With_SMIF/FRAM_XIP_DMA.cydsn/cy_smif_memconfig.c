/*******************************************************************************
* \file cy_smif_memconfig.c
* \version 1.0
*
* \brief
* Provides definitions of the SMIF-driver memory configuration.
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

#include "cy_smif_memconfig.h"

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_readCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x03U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_QUAD,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_QUAD,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_QUAD,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 8U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_QUAD
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_writeEnCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_QUAD,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_QUAD,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_QUAD
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_writeDisCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_eraseCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_chipEraseCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_programCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x02U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_QUAD,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_QUAD,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_QUAD,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_QUAD
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_readStsRegQeCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_readStsRegWipCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

const cy_stc_smif_mem_cmd_t Excelon_Ultra_SlaveSlot_2_writeStsRegQeCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x00U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

const cy_stc_smif_mem_device_cfg_t deviceCfg_Excelon_Ultra_SlaveSlot_2 =
{
    /* Specifies the number of address bytes used by the memory slave device. */
    .numOfAddrBytes = 0x03U,
    /* The size of the memory. */
    .memSize = 0x0080000U,
    /* Specifies the Read command. */
    .readCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_readCmd,
    /* Specifies the Write Enable command. */
    .writeEnCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_writeEnCmd,
    /* Specifies the Write Disable command. */
    .writeDisCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_writeDisCmd,
    /* Specifies the Erase command. */
    .eraseCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_eraseCmd,
    /* Specifies the sector size of each erase. */
    .eraseSize = 0x0001000U,
    /* Specifies the Chip Erase command. */
    .chipEraseCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_chipEraseCmd,
    /* Specifies the Program command. */
    .programCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_programCmd,
    /* Specifies the page size for programming. */
    .programSize = 0x0000100U,
    /* Specifies the command to read the QE-containing status register. */
    .readStsRegQeCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_readStsRegQeCmd,
    /* Specifies the command to read the WIP-containing status register. */
    .readStsRegWipCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_readStsRegWipCmd,
    /* Specifies the command to write into the QE-containing status register. */
    .writeStsRegQeCmd = (cy_stc_smif_mem_cmd_t*)&Excelon_Ultra_SlaveSlot_2_writeStsRegQeCmd,
    /* The mask for the status register. */
    .stsRegBusyMask = 0x00U,
    /* The mask for the status register. */
    .stsRegQuadEnableMask = 0x00U,
    /* The max time for the erase type-1 cycle-time in ms. */
    .eraseTime = 1U,
    /* The max time for the chip-erase cycle-time in ms. */
    .chipEraseTime = 16U,
    /* The max time for the page-program cycle-time in us. */
    .programTime = 8U
};

const cy_stc_smif_mem_config_t Excelon_Ultra_SlaveSlot_2 =
{
    /* Determines the slot number where the memory device is placed. */
    .slaveSelect = CY_SMIF_SLAVE_SELECT_2,
    /* Flags. */
    .flags = CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN,
    /* The data-line selection options for a slave device. */
    .dataSelect = CY_SMIF_DATA_SEL0,
    /* The base address the memory slave is mapped to in the PSoC memory map.
    Valid when the memory-mapped mode is enabled. */
    .baseAddress = 0x18000000U,
    /* The size allocated in the PSoC memory map, for the memory slave device.
    The size is allocated from the base address. Valid when the memory mapped mode is enabled. */
    .memMappedSize = 0x80000U,
    /* If this memory device is one of the devices in the dual quad SPI configuration.
    Valid when the memory mapped mode is enabled. */
    .dualQuadSlots = 0,
    /* The configuration of the device. */
    .deviceCfg = (cy_stc_smif_mem_device_cfg_t*)&deviceCfg_Excelon_Ultra_SlaveSlot_2
};

const cy_stc_smif_mem_config_t* const smifMemConfigs[] = {
   &Excelon_Ultra_SlaveSlot_2
};

const cy_stc_smif_block_config_t smifBlockConfig =
{
    /* The number of SMIF memories defined. */
    .memCount = CY_SMIF_DEVICE_NUM,
    /* The pointer to the array of memory config structures of size memCount. */
    .memConfig = (cy_stc_smif_mem_config_t**)smifMemConfigs,
    /* The version of the SMIF driver. */
    .majorVersion = CY_SMIF_DRV_VERSION_MAJOR,
    /* The version of the SMIF driver. */
    .minorVersion = CY_SMIF_DRV_VERSION_MINOR
};


