/**
  ******************************************************************************
  * @file    cy15b104qsn.h
  * @brief   This file contains all the description of the
  *          CY15B104QSN QSPI memory.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CY15B104QSN_H
#define CY15B104QSN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

/**
  * @brief  CY15B104QSN Size configuration
  */

#define CY15B104QSN_FRAM_SIZE                   (uint32_t)(4*1024*1024/8)  /* 4 Mbit => 512 kBytes */
#define CY15B104QSN_DUMMY_CYCLES_READ           8U     /* fine for up to 108 MHz STR or 54 MHz DTR */

/**
  * @brief  CY15B104QSN Error codes
  */
#define CY15B104QSN_OK                           (0)
#define CY15B104QSN_ERROR                        (-1)

/**
  * @brief re-definition of legacy memory mapped functions
  */
#define CY15B104QSN_EnableMemoryMappedModeDTR CY15B104QSN_EnableDTRMemoryMappedMode
#define CY15B104QSN_EnableMemoryMappedModeSTR CY15B104QSN_EnableSTRMemoryMappedMode


/******************************************************************************
  * @brief  CY15B104QSN Commands
  ****************************************************************************/

/***** READ/WRITE MEMORY *****************************************************/
#define CY15B104QSN_FAST_READ_CMD                        0x0BU   /*!< Fast Read */
#define CY15B104QSN_WRITE_CMD                            0x02U   /*!< Write */
#define CY15B104QSN_QPI_READ_DTR_CMD                     0xEDU   /*!< QIO Read DTR */
#define CY15B104QSN_QPI_WRITE_DTR_CMD                    0xDEU   /*!< QIO Write DTR */

/***** Setting commands ******************************************************/
#define CY15B104QSN_WRITE_ENABLE_CMD                     0x06U   /*!< Write Enable */
#define CY15B104QSN_WRITE_DISABLE_CMD                    0x04U   /*!< Write Disable */
#define CY15B104QSN_ENTER_DEEP_POWER_DOWN_CMD            0xB9U   /*!< Enter deep power down */

/***** Register Commands *****************************************************/
#define CY15B104QSN_READ_ID_CMD                          0x9FU   /*!< Read IDentification */
#define CY15B104QSN_READ_ANY_REG_CMD                     0x65U   /*!< Read Any Register */
#define CY15B104QSN_WRITE_ANY_REG_CMD                    0x71U   /*!< Write Any Register */


/******************************************************************************
  * @brief  CY15B104QSN Register Addresses
  ****************************************************************************/
#define CY15B104QSN_AR_SR1V                              0x00070000U
#define CY15B104QSN_AR_SR2V                              0x00070001U
#define CY15B104QSN_AR_CR1V                              0x00070002U
#define CY15B104QSN_AR_CR2V                              0x00070003U
#define CY15B104QSN_AR_CR3V                              0x00070004U
#define CY15B104QSN_AR_CR4V                              0x00070005U
#define CY15B104QSN_AR_CR5V                              0x00070006U
  

/** @defgroup CY15B104QSN_Exported_Types CY15B104QSN Exported Types
  * @{
  */
typedef struct {
  uint32_t FramSize;                        /*!< Size of the F-RAM                  */
} CY15B104QSN_Info_t;

typedef enum {
  CY15B104QSN_SPI_MODE = 0,                 /*!< 1-1-1, STR, power-on H/W default   */
  CY15B104QSN_QPI_MODE                      /*!< 4-4-4, STR or DTR                  */
} CY15B104QSN_Interface_t;

typedef enum {
  CY15B104QSN_STR_TRANSFER = 0,             /*!< Single Transfer Rate               */
  CY15B104QSN_DTR_TRANSFER                  /*!< Double Transfer Rate               */
} CY15B104QSN_Transfer_t;


/** @defgroup CY15B104QSN_Exported_Functions CY15B104QSN Exported Functions
  * @{
  */
/* Function by commands combined */
int32_t CY15B104QSN_GetFramInfo(CY15B104QSN_Info_t *pInfo);

/* Read/Write Array Commands **************************************************/
int32_t CY15B104QSN_ReadSTR(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t CY15B104QSN_ReadDTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t CY15B104QSN_WriteSTR(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
int32_t CY15B104QSN_WriteDTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
int32_t CY15B104QSN_EnableMemoryMappedModeSTR(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode);
int32_t CY15B104QSN_EnableMemoryMappedModeDTR(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode);

/* Register/Setting Commands **************************************************/
int32_t CY15B104QSN_WriteEnable(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode);
int32_t CY15B104QSN_WriteDisable(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode);
int32_t CY15B104QSN_ReadAnyReg(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint32_t RegAddr, uint8_t *Value);
int32_t CY15B104QSN_WriteAnyReg(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint32_t RegAddr, uint8_t Value);
int32_t CY15B104QSN_ReadID(OSPI_HandleTypeDef *Ctx, CY15B104QSN_Interface_t Mode, uint8_t *ID);


#ifdef __cplusplus
}
#endif

#endif /* CY15B104QSN_H */

/************************ (C) COPYRIGHT STMicroelectronics / Cypress *****END OF FILE****/
