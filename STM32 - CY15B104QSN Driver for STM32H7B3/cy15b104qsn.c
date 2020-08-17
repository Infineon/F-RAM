/**
 ******************************************************************************
 * @file    cy15b104qsn.c
 * @brief   This file provides the CY15B104QSN QSPI drivers.
 ******************************************************************************
 * CY15B104QSN action :
 *   Quad IO protocol (QPI) bit of Configuration Register 2 :
 *     QPI = 1: Operates in Quad IO protocol (accepts 4-4-4 commands)
 *     QPI = 0: Operates in Single IO protocol (accepts 1-1-1 commands)
 *
 *   Memory commands support STR(Single Transfer Rate) & DTR(Double Transfer Rate) modes in QPI
 *   Memory commands support STR(Single Transfer Rate) mode in SPI
 *
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics / 2020 Cypress.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "cy15b104qsn.h"

/**
  * @brief  Get Fram information
  * @param  pInfo pointer to information structure
  * @retval error status
  */
int32_t CY15B104QSN_GetFramInfo(CY15B104QSN_Info_t *pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FramSize = CY15B104QSN_FRAM_SIZE;

  return CY15B104QSN_OK;
};

/* Read/Write Array Commands (3 Byte Address Command Set) *********************/
/**
  * @brief  Reads an amount of data from the QSPI memory in STR mode.
  *         SPI/QPI; 1-1-1/4-4-4
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to read
  * @retval QSPI memory status
  */
int32_t CY15B104QSN_ReadSTR(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the read command */
  s_command.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId               = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode       = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction           = CY15B104QSN_FAST_READ_CMD;
  s_command.AddressMode           = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address               = ReadAddr;
  s_command.AlternateBytesMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ALTERNATE_BYTES_1_LINE : HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  s_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  s_command.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  s_command.AlternateBytes        = 0U;
  s_command.DataMode              = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles           = CY15B104QSN_DUMMY_CYCLES_READ;
  s_command.NbData                = Size;
  s_command.DQSMode               = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory in DTR mode.
  *         QPI
  * @param  Ctx Component object pointer
  * @param  AddressSize Address size
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start addressS
  * @param  Size Size of data to read
  * @note   Only QPI mode support DTR transfer rate
  * @retval QSPI memory status
  */
int32_t CY15B104QSN_ReadDTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the read command */
  s_command.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId               = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode       = HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction           = CY15B104QSN_QPI_READ_DTR_CMD;
  s_command.AddressMode           = HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_ENABLE;
  s_command.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address               = ReadAddr;
  s_command.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  s_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_ENABLE;
  s_command.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  s_command.AlternateBytes        = 0U;
  s_command.DataMode              = HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode           = HAL_OSPI_DATA_DTR_ENABLE;
  s_command.DummyCycles           = CY15B104QSN_DUMMY_CYCLES_READ;
  s_command.NbData                = Size;
  s_command.DQSMode               = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory in STR mode.
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ CY15B104QSN_PAGE_SIZE
  * @note   Address size is forced to 3 Bytes when the 4 Bytes address size
  *         command is not available for the specified interface mode
  * @retval QSPI memory status
  */
int32_t CY15B104QSN_WriteSTR(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the program command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_WRITE_CMD;
  s_command.AddressMode        = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = WriteAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = Size;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Transmission of the data */
  if (HAL_OSPI_Transmit(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory in DTR mode.
  *         QPI
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ CY15B104QSN_PAGE_SIZE
  * @note   Only QPI mode support DTR transfer rate
  * @retval QSPI memory status
  */
int32_t CY15B104QSN_WriteDTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the program command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_QPI_WRITE_DTR_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = WriteAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = Size;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Transmission of the data */
  if (HAL_OSPI_Transmit(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Enable memory mapped mode for the QSPI memory on STR mode.
  *         SPI/QPI; 1-1-1/4-4-4
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @retval QSPI memory status
  */
int32_t CY15B104QSN_EnableSTRMemoryMappedMode(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode)
{
  OSPI_RegularCmdTypeDef   s_command = {0};
  OSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

  /* Initialize the read command */
  s_command.OperationType         = HAL_OSPI_OPTYPE_READ_CFG;
  s_command.FlashId               = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode       = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction           = CY15B104QSN_FAST_READ_CMD;
  s_command.AddressMode           = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  s_command.AlternateBytesMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ALTERNATE_BYTES_1_LINE : HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  s_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  s_command.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  s_command.AlternateBytes        = 0U;
  s_command.DataMode              = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles           = CY15B104QSN_DUMMY_CYCLES_READ;
  s_command.DQSMode               = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD; 
  
  /* Send the read command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Initialize the program command */
  s_command.OperationType         = HAL_OSPI_OPTYPE_WRITE_CFG;
  s_command.Instruction           = CY15B104QSN_WRITE_CMD;
  s_command.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles           = 0U;
  s_command.DQSMode               = HAL_OSPI_DQS_ENABLE;  /* not needed, but the system gives a HardFault if disabled */
  
  /* Send the write command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_OSPI_MemoryMapped(Ctx, &s_mem_mapped_cfg) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Enable memory mapped mode for the QSPI memory on DTR mode.
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @note   Only QPI mode support DTR transfer rate
  * @retval QSPI memory status
  */
int32_t CY15B104QSN_EnableDTRMemoryMappedMode(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Mode);

  OSPI_RegularCmdTypeDef   s_command = {0};
  OSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

  /* Initialize the read command */
  s_command.OperationType         = HAL_OSPI_OPTYPE_READ_CFG;
  s_command.FlashId               = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode       = HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction           = CY15B104QSN_QPI_READ_DTR_CMD;
  s_command.AddressMode           = HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_ENABLE;
  s_command.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  s_command.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  s_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_ENABLE;
  s_command.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  s_command.AlternateBytes        = 0U;
  s_command.DataMode              = HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode           = HAL_OSPI_DATA_DTR_ENABLE;
  s_command.DummyCycles           = CY15B104QSN_DUMMY_CYCLES_READ;
  s_command.DQSMode               = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Initialize the program command */
  s_command.OperationType         = HAL_OSPI_OPTYPE_WRITE_CFG;
  s_command.Instruction           = CY15B104QSN_QPI_WRITE_DTR_CMD;
  s_command.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles           = 0U;
  s_command.DQSMode               = HAL_OSPI_DQS_ENABLE;  /* not needed, but the system gives a HardFault if disabled */

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }
  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_OSPI_MemoryMapped(Ctx, &s_mem_mapped_cfg) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/* Register/Setting Commands **************************************************/
/**
  * @brief  This function send a Write Enable and wait it is effective.
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Rate Transfer rate STR or DTR
  * @retval error status
  */
int32_t CY15B104QSN_WriteEnable(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode)
{
  OSPI_RegularCmdTypeDef     s_command = {0};

  /* Initialize the write enable command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_WRITE_ENABLE_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  This function reset the (WEN) Write Enable Latch bit.
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Rate Transfer rate STR or DTR
  * @retval error status
  */
int32_t CY15B104QSN_WriteDisable(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the write disable command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_WRITE_DISABLE_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Read Fram register value
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Value Status register value pointer
  * @retval error status
  */
int32_t CY15B104QSN_ReadAnyReg(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint32_t RegAddr, uint8_t *Value)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the reading of status register */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_READ_ANY_REG_CMD;
  s_command.AddressMode        = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = RegAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = 1U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, Value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/**
  * @brief  Write Fram register
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Rate Transfer rate STR or DTR
  * @param  Value Value to write to Status register
  * @retval error status
  */
int32_t CY15B104QSN_WriteAnyReg(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint32_t RegAddr, uint8_t Value)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the writing of status register */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_WRITE_ANY_REG_CMD;
  s_command.AddressMode        = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = RegAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = 1U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  if (HAL_OSPI_Transmit(Ctx, &Value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/* ID Commands ****************************************************************/
/**
  * @brief  Read F-RAM 3 Byte IDs.
  *         Manufacturer ID, Memory type, Memory density
  *         SPI/QPI; 1-0-1/1-0-8
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  ID 3 bytes IDs pointer
  * @param  DualFram Dual F-RAM mode state
  * @retval error status
  */
int32_t CY15B104QSN_ReadID(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint8_t *ID)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the read ID command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_INSTRUCTION_1_LINE : HAL_OSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = CY15B104QSN_READ_ID_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = (Mode == CY15B104QSN_SPI_MODE) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = 3U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, ID, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return CY15B104QSN_ERROR;
  }

  return CY15B104QSN_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics / Cypress *****END OF FILE****/
