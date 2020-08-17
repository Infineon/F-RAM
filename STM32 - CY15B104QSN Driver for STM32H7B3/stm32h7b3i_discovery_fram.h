/**
  ******************************************************************************
  * @file    stm32h7b3i_discovery_fram.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32h7b3i_discovery_fram.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics / 2020 Cypress.
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
#ifndef STM32H7B3I_DK_FRAM_H
#define STM32H7B3I_DK_FRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7b3i_discovery_conf.h"
#include "stm32h7b3i_discovery_errno.h"
#include "../Components/cy15b104qsn/cy15b104qsn.h"

/* Exported types ------------------------------------------------------------*/
/** @defgroup STM32H7B3I_DK_OSPI_Exported_Types OSPI_FRAM Exported Types
  * @{
  */
typedef enum {
  OSPI_ACCESS_NONE = 0,          /*!<  Instance not initialized,          */
  OSPI_ACCESS_INDIRECT,          /*!<  Instance use indirect mode access  */
  OSPI_ACCESS_MMP                /*!<  Instance use Memory Mapped Mode    */
} OSPI_Access_t;

#define BSP_OSPI_FRAM_Info_t                CY15B104QSN_Info_t
#define BSP_OSPI_FRAM_Interface_t           CY15B104QSN_Interface_t
#define BSP_OSPI_FRAM_Transfer_t            CY15B104QSN_Transfer_t

typedef struct
{
  OSPI_Access_t              IsInitialized;  /*!<  Instance access F-RAM method     */
  BSP_OSPI_FRAM_Interface_t  InterfaceMode;  /*!<  F-RAM Interface mode of Instance */
  BSP_OSPI_FRAM_Transfer_t   TransferRate;   /*!<  F-RAM Transfer mode of Instance  */
} OSPI_FRAM_Ctx_t;

typedef struct
{
  BSP_OSPI_FRAM_Interface_t   InterfaceMode;      /*!<  Current F-RAM Interface mode */
  BSP_OSPI_FRAM_Transfer_t    TransferRate;       /*!<  Current F-RAM Transfer rate  */
} BSP_OSPI_FRAM_Init_t;

/* Exported constants --------------------------------------------------------*/
/** @defgroup STM32H7B3I_DK_OSPI_Exported_Constants OSPI_FRAM Exported Constants
  * @{
  */
#define OSPI_FRAM_INSTANCES_NUMBER         1U

/* Definition for OSPI modes */
#define BSP_OSPI_FRAM_SPI_MODE            (BSP_OSPI_FRAM_Interface_t)CY15B104QSN_SPI_MODE      /* 1 Cmd Line, 1 Address Line and 1 Data Line    */
#define BSP_OSPI_FRAM_QPI_MODE            (BSP_OSPI_FRAM_Interface_t)CY15B104QSN_QPI_MODE      /* 4 Cmd Lines, 4 Address Lines and 4 Data Lines */

/* Definition for OSPI transfer rates */
#define BSP_OSPI_FRAM_STR_TRANSFER         (BSP_OSPI_FRAM_Transfer_t)CY15B104QSN_STR_TRANSFER   /* Single Transfer Rate */
#define BSP_OSPI_FRAM_DTR_TRANSFER         (BSP_OSPI_FRAM_Transfer_t)CY15B104QSN_DTR_TRANSFER   /* Double Transfer Rate */

/* Definition for OSPI clock resources */
#define OSPI_CLK_ENABLE()                 __HAL_RCC_OSPI1_CLK_ENABLE()
#define OSPI_CLK_DISABLE()                __HAL_RCC_OSPI1_CLK_DISABLE()

#define OSPI_CS_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_CLK_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define OSPI_D0_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D1_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_D2_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_D3_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_D4_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define OSPI_D5_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOH_CLK_ENABLE()
#define OSPI_D6_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_D7_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_DQS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

#define OSPI_FORCE_RESET()                __HAL_RCC_OSPI1_FORCE_RESET()
#define OSPI_RELEASE_RESET()              __HAL_RCC_OSPI1_RELEASE_RESET()

/* Definition for OSPI Pins */
#define OSPI_CS_PIN                 GPIO_PIN_6
#define OSPI_CS_GPIO_PORT           GPIOG
#define OSPI_CS_PIN_AF              GPIO_AF10_OCTOSPIM_P1
#define OSPI_CLK_PIN                GPIO_PIN_2
#define OSPI_CLK_GPIO_PORT          GPIOB
#define OSPI_CLK_PIN_AF             GPIO_AF9_OCTOSPIM_P1
#define OSPI_D0_PIN                 GPIO_PIN_11
#define OSPI_D0_GPIO_PORT           GPIOD
#define OSPI_D0_PIN_AF              GPIO_AF9_OCTOSPIM_P1
#define OSPI_D1_PIN                 GPIO_PIN_9
#define OSPI_D1_GPIO_PORT           GPIOF
#define OSPI_D1_PIN_AF              GPIO_AF10_OCTOSPIM_P1
#define OSPI_D2_PIN                 GPIO_PIN_7
#define OSPI_D2_GPIO_PORT           GPIOF
#define OSPI_D2_PIN_AF              GPIO_AF10_OCTOSPIM_P1
#define OSPI_D3_PIN                 GPIO_PIN_6
#define OSPI_D3_GPIO_PORT           GPIOF
#define OSPI_D3_PIN_AF              GPIO_AF10_OCTOSPIM_P1

/* Exported variables --------------------------------------------------------*/
/** @addtogroup STM32H7B3I_DK_OSPI_FRAM_Exported_Variables OSPI_FRAM Exported Variables
  * @{
  */
extern OSPI_HandleTypeDef hospi_fram[];
extern OSPI_FRAM_Ctx_t Ospi_Fram_Ctx[];

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32H7B3I_DK_OSPI_FRAM_Exported_FunctionsPrototypes OSPI_FRAM Exported Functions Prototypes
  * @{
  */
int32_t BSP_OSPI_FRAM_Init                    (uint32_t Instance, BSP_OSPI_FRAM_Init_t *Init);
int32_t BSP_OSPI_FRAM_DeInit                  (uint32_t Instance);
int32_t BSP_OSPI_FRAM_Read                    (uint32_t Instance, uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
int32_t BSP_OSPI_FRAM_Write                   (uint32_t Instance, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
int32_t BSP_OSPI_FRAM_GetInfo                 (uint32_t Instance, BSP_OSPI_FRAM_Info_t* pInfo);
int32_t BSP_OSPI_FRAM_EnableMemoryMappedMode  (uint32_t Instance);
int32_t BSP_OSPI_FRAM_DisableMemoryMappedMode (uint32_t Instance);
int32_t BSP_OSPI_FRAM_ReadID                  (uint32_t Instance, uint8_t *Id);
int32_t BSP_OSPI_FRAM_ConfigFram              (uint32_t Instance, BSP_OSPI_FRAM_Interface_t Mode, BSP_OSPI_FRAM_Transfer_t Rate);

#ifdef __cplusplus
}
#endif

#endif /* STM32H7B3I_DK_FRAM_H */

/************************ (C) COPYRIGHT STMicroelectronics / Cypress *****END OF FILE****/
