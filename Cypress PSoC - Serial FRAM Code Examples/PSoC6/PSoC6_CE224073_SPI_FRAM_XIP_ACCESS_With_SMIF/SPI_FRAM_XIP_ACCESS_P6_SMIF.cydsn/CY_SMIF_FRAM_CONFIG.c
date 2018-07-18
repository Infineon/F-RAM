/****************************************************************************
*File Name: CY_SMIF_FRAM_CONFIG.c
*
* Version: 1.0
*
* Description: 
* This file contains all the initialization for SMIF to access the SPI F-RAM 
*
* Related Document: CE224073.pdf
*
* Hardware Dependency PSoC 6 Pioneer Kit (CY8CKIT-062-BLE) WITH SERIAL F-RAM 
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
**
If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "CY_SMIF_FRAM_CONFIG.h"

cy_stc_smif_mem_cmd_t  SPI_FRAM_SlaveSlot_2_readCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x0BU,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 8U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};


cy_stc_smif_mem_cmd_t  SPI_FRAM_SlaveSlot_2_writeCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x02U,
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


cy_stc_smif_mem_device_cfg_t deviceCfg_SPI_FRAM_SlaveSlot_2 =
{
    /* Specifies the number of address bytes used by the memory slave device. */
    .numOfAddrBytes = 0x03U,
    /* The size of the memory. */
    .memSize = 0x0080000U,
    /* Specifies the Read command. */
    .readCmd = &SPI_FRAM_SlaveSlot_2_readCmd,
    /* Specifies the Write command. */
    .programCmd = &SPI_FRAM_SlaveSlot_2_writeCmd,
    /* Specifies the page size for programming. */
    .programSize = 0x0080000U,
   
};

 cy_stc_smif_mem_config_t SPI_FRAM_SlaveSlot_2 =
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
    .deviceCfg = &deviceCfg_SPI_FRAM_SlaveSlot_2
};

 cy_stc_smif_mem_config_t* smifMemConfigs[] = {
   &SPI_FRAM_SlaveSlot_2
};

 cy_stc_smif_block_config_t smifBlockConfig =
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


