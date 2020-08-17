/**
  ******************************************************************************
  * @file    stm32h7b3i_discovery_ospi.c
  * @brief   This file includes a standard driver for the CY15B104QSN QSPI
  *          memory mounted on a special STM32H7B3I Discovery board.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) This driver is used to drive the CY15B104QSN QSPI F-RAM
       external memory mounted on a special STM32H7B3I Discovery board.

   (#) This driver need specific component driver (CY15B104QSN) to be included with.

   (#) CY15B104QSN Initialization steps:
       (++) Initialize the OPSI external memory using the BSP_OSPI_FRAM_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            OSPI interface with the external memory.

   (#) CY15B104QSN QSPI F-RAM memory operations
       (++) QSPI memory can be accessed with read/write operations once it is initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_OSPI_FRAM_Read()/BSP_OSPI_FRAM_Write().
       (++) The function BSP_OSPI_FRAM_GetInfo() returns the configuration of the QSPI memory.
            (see the FRAM memory data sheet)
       (++) The function BSP_OSPI_FRAM_GetStatus() returns the current status of the QSPI memory.
            (see the FRAM memory data sheet)
       (++) The memory access can be configured in memory-mapped mode with the call of
            function BSP_OSPI_FRAM_EnableMemoryMapped(). To go back in indirect mode, the
            function BSP_OSPI_FRAM_DisableMemoryMapped() should be used.
       (++) The function BSP_OSPI_FRAM_ReadID() returns the identifier of the memory
            (see the FRAM memory data sheet)
       (++) The configuration of the interface between peripheral and memory is done by
            the function BSP_OSPI_FRAM_ConfigFram(), three modes are possible :
            - SPI : instruction, address and data on one line
            - STR QPI : instruction, address and data on four lines with sampling on one edge of clock
            - DTR QPI : instruction, address and data on four lines with sampling on both edges of clock

  @endverbatim
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

#include "stm32h7b3i_discovery_fram.h"

/** @addtogroup STM32H7B3I_DK_OSPI_FRAM_Exported_Variables OSPI_FRAM Exported Variables
  * @{
  */
OSPI_HandleTypeDef hospi_fram[OSPI_FRAM_INSTANCES_NUMBER];
OSPI_FRAM_Ctx_t Ospi_Fram_Ctx[OSPI_FRAM_INSTANCES_NUMBER];

/** @defgroup STM32H7B3I_DK_OSPI_FRAM_Private_FunctionsPrototypes OSPI_FRAM Private Functions Prototypes
  * @{
  */
static void    OSPI_FRAM_MspInit      (OSPI_HandleTypeDef *hospi);
static void    OSPI_FRAM_MspDeInit    (OSPI_HandleTypeDef *hospi);

typedef struct
{
  uint32_t MemorySize;
  uint32_t ClockPrescaler;
  uint32_t SampleShifting;
  uint32_t TransferRate;
} CY_OSPI_Config;

HAL_StatusTypeDef CY_OSPI_FRAM_Init(OSPI_HandleTypeDef *hospi, CY_OSPI_Config *Config);

/** @defgroup STM32H7B3I_EVAL_OSPI_FRAM_Exported_Functions OSPI Exported Functions
  * @{
  */
/**
  * @brief  Initializes the OSPI interface.
  * @param  Instance   OSPI Instance
  * @param  Init       OSPI Init structure
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_Init(uint32_t Instance, BSP_OSPI_FRAM_Init_t *Init)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_OSPI_FRAM_Info_t pInfo;
  CY_OSPI_Config ospi_config;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (Ospi_Fram_Ctx[Instance].IsInitialized == OSPI_ACCESS_NONE)
    {
      /* Msp OSPI initialization */
      OSPI_FRAM_MspInit(&hospi_fram[Instance]);

      /* Get Fram informations of one memory */
      (void)CY15B104QSN_GetFramInfo(&pInfo);

      /* Fill config structure */
      ospi_config.ClockPrescaler = 6U;       /* 280 MHz / 6 = 46.7 MHz */
      ospi_config.MemorySize     = (uint32_t)POSITION_VAL((uint32_t)pInfo.FramSize);
      ospi_config.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
      ospi_config.TransferRate   = (uint32_t)Init->TransferRate;

      /* STM32 OSPI interface initialization */
      if (CY_OSPI_FRAM_Init(&hospi_fram[Instance], &ospi_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        /* Configure the memory */
        if (BSP_OSPI_FRAM_ConfigFram(Instance, Init->InterfaceMode, Init->TransferRate) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  De-Initializes the OSPI interface.
  * @param  Instance   OSPI Instance
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Disable Memory mapped mode */
    if(Ospi_Fram_Ctx[Instance].IsInitialized == OSPI_ACCESS_MMP)
    {
      if(BSP_OSPI_FRAM_DisableMemoryMappedMode(Instance) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }

    if(ret == BSP_ERROR_NONE)
    {
      /* Set default Ospi_Fram_Ctx values */
      Ospi_Fram_Ctx[Instance].IsInitialized = OSPI_ACCESS_NONE;
      Ospi_Fram_Ctx[Instance].InterfaceMode = BSP_OSPI_FRAM_SPI_MODE;
      Ospi_Fram_Ctx[Instance].TransferRate  = BSP_OSPI_FRAM_STR_TRANSFER;

      OSPI_FRAM_MspDeInit(&hospi_fram[Instance]);

      /* Call the DeInit function to reset the driver */
      if (HAL_OSPI_DeInit(&hospi_fram[Instance]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Initializes the OSPI interface.
  * @param  hospi          OSPI handle
  * @param  Config         OSPI config structure
  * @retval BSP status
  */
__weak HAL_StatusTypeDef CY_OSPI_FRAM_Init(OSPI_HandleTypeDef *hospi, CY_OSPI_Config *Config)
{
    /* OctoSPI initialization */
  hospi->Instance = OCTOSPI1;

  hospi->Init.FifoThreshold         = 4;
  hospi->Init.DualQuad              = HAL_OSPI_DUALQUAD_DISABLE;
  hospi->Init.DeviceSize            = Config->MemorySize;
  hospi->Init.ChipSelectHighTime    = 5;                      /* 6*21.4 ns > 110 ns @ 46.7 MHz */
  hospi->Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi->Init.ClockMode             = HAL_OSPI_CLOCK_MODE_0;
  hospi->Init.WrapSize              = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi->Init.ClockPrescaler        = Config->ClockPrescaler;
  hospi->Init.SampleShifting        = Config->SampleShifting;
  hospi->Init.ChipSelectBoundary    = 0;
  hospi->Init.ClkChipSelectHighTime = 0;
  hospi->Init.DelayBlockBypass      = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi->Init.MaxTran               = 0;
  hospi->Init.Refresh               = 0;

  if (Config->TransferRate == (uint32_t)BSP_OSPI_FRAM_DTR_TRANSFER)
  {
    hospi->Init.MemoryType            = HAL_OSPI_MEMTYPE_MICRON;
    hospi->Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  }
  else
  {
    hospi->Init.MemoryType            = HAL_OSPI_MEMTYPE_MICRON;
    hospi->Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  }

  return HAL_OSPI_Init(hospi);
}

/**
  * @brief  Reads an amount of data from the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pData     Pointer to data to be read
  * @param  ReadAddr  Read start address
  * @param  Size      Size of data to read
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_Read(uint32_t Instance, uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(Ospi_Fram_Ctx[Instance].TransferRate == BSP_OSPI_FRAM_STR_TRANSFER)
  {
    if(CY15B104QSN_ReadSTR(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode, pData, ReadAddr, Size) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    if(CY15B104QSN_ReadDTR(&hospi_fram[Instance], pData, ReadAddr, Size) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pData     Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size      Size of data to write
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_Write(uint32_t Instance, uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Enable write operations */
    if(CY15B104QSN_WriteEnable(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      if(Ospi_Fram_Ctx[Instance].TransferRate == BSP_OSPI_FRAM_STR_TRANSFER)
      {
        /* Issue STR write command */
        if(CY15B104QSN_WriteSTR(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode, pData, WriteAddr, Size) != CY15B104QSN_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
      }
      else
      {
        /* Issue DTR write command */
        if(CY15B104QSN_WriteDTR(&hospi_fram[Instance], pData, WriteAddr, Size) != CY15B104QSN_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Return the configuration of the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pInfo     pointer on the configuration structure
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_GetInfo(uint32_t Instance, BSP_OSPI_FRAM_Info_t* pInfo)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    (void)CY15B104QSN_GetFramInfo(pInfo);
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Configure the OSPI in memory-mapped mode
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_EnableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(Ospi_Fram_Ctx[Instance].TransferRate == BSP_OSPI_FRAM_STR_TRANSFER)
  {
    if(CY15B104QSN_EnableMemoryMappedModeSTR(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else /* Update OSPI context if all operations are well done */
    {
      Ospi_Fram_Ctx[Instance].IsInitialized = OSPI_ACCESS_MMP;
    }
  }
  else
  {
    if(CY15B104QSN_EnableMemoryMappedModeDTR(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else /* Update OSPI context if all operations are well done */
    {
      Ospi_Fram_Ctx[Instance].IsInitialized = OSPI_ACCESS_MMP;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Exit form memory-mapped mode
  *         Only 1 Instance can running MMP mode. And it will lock system at this mode.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_DisableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(Ospi_Fram_Ctx[Instance].IsInitialized != OSPI_ACCESS_MMP)
  {
    ret = BSP_ERROR_OSPI_MMP_UNLOCK_FAILURE;
  }/* Abort MMP back to indirect mode */
  else if(HAL_OSPI_Abort(&hospi_fram[Instance]) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    /* Update OSPI context if all operations are well done */
    Ospi_Fram_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Get fram ID 3 Bytes:
  *         Manufacturer ID, Memory type, Memory density
  * @param  Instance  OSPI instance
  * @param  Id Pointer to fram ID bytes
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if(Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(CY15B104QSN_ReadID(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode, Id) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Set Fram to desired Interface mode. And this instance becomes current instance.
  *         If current instance running at MMP mode then this function doesn't work.
  *         Indirect -> Indirect
  * @param  Instance  OSPI instance
  * @param  Mode      OSPI mode
  * @param  Rate      OSPI transfer rate
  * @retval BSP status
  */
int32_t BSP_OSPI_FRAM_ConfigFram(uint32_t Instance, BSP_OSPI_FRAM_Interface_t Mode, BSP_OSPI_FRAM_Transfer_t Rate)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_FRAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }     /* Check if MMP mode locked *******************************************/
  else if (Ospi_Fram_Ctx[Instance].IsInitialized == OSPI_ACCESS_MMP)
  {
    ret = BSP_ERROR_OSPI_MMP_LOCK_FAILURE;
  }
  else
  {
    /* Setup Fram interface **************************************************/
    switch(Mode)
    {
    case BSP_OSPI_FRAM_QPI_MODE:  /* 4-4-4 commands, enter QPI mode via CR2V */
      if (ret == BSP_ERROR_NONE && 
          (CY15B104QSN_WriteEnable(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK ||
           CY15B104QSN_WriteAnyReg(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode, CY15B104QSN_AR_CR2V, 0x40U) != CY15B104QSN_OK))
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;

    case BSP_OSPI_FRAM_SPI_MODE:  /* 1-1-1 commands, Power on H/W default setting, exit QPI mode via CR2V */
    default :
      if (Rate == BSP_OSPI_FRAM_DTR_TRANSFER)
      {
        /* not supported by device */
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }    
      else if (ret == BSP_ERROR_NONE && 
          (CY15B104QSN_WriteEnable(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK ||
           CY15B104QSN_WriteAnyReg(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode, CY15B104QSN_AR_CR2V, 0U) != CY15B104QSN_OK))
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
    }

    /* Update OSPI context if all operations are well done */
    if (ret == BSP_ERROR_NONE)
    {
      /* Update current status parameter *****************************************/
      Ospi_Fram_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;
      Ospi_Fram_Ctx[Instance].InterfaceMode = Mode;
      Ospi_Fram_Ctx[Instance].TransferRate  = Rate;
    }

    /* Set memory latency to 8 dummy cycles in CR1V */
    if (ret == BSP_ERROR_NONE &&
        (CY15B104QSN_WriteEnable(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK ||
         CY15B104QSN_WriteAnyReg(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode, CY15B104QSN_AR_CR1V, 0x80U) != CY15B104QSN_OK))
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    
    /* Set write enable latch */
    if (ret == BSP_ERROR_NONE &&
       CY15B104QSN_WriteEnable(&hospi_fram[Instance], Ospi_Fram_Ctx[Instance].InterfaceMode) != CY15B104QSN_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    } 
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Initializes the OSPI MSP.
  * @param  hospi OSPI handle
  * @retval None
  */
static void OSPI_FRAM_MspInit(OSPI_HandleTypeDef *hospi)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* hospi unused argument(s) compilation warning */
  UNUSED(hospi);

  /* Enable the OctoSPI memory interface clock */
  OSPI_CLK_ENABLE();

  /* Reset the OctoSPI memory interface */
  OSPI_FORCE_RESET();
  OSPI_RELEASE_RESET();

  /* Enable GPIO clocks */
  OSPI_CLK_GPIO_CLK_ENABLE();
  OSPI_CS_GPIO_CLK_ENABLE();
  OSPI_D0_GPIO_CLK_ENABLE();
  OSPI_D1_GPIO_CLK_ENABLE();
  OSPI_D2_GPIO_CLK_ENABLE();
  OSPI_D3_GPIO_CLK_ENABLE();

  /* OctoSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_CS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = OSPI_CS_PIN_AF;
  HAL_GPIO_Init(OSPI_CS_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI CLK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_CLK_PIN;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = OSPI_CLK_PIN_AF;
  HAL_GPIO_Init(OSPI_CLK_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D0_PIN;
  GPIO_InitStruct.Alternate = OSPI_D0_PIN_AF;
  HAL_GPIO_Init(OSPI_D0_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D1_PIN;
  GPIO_InitStruct.Alternate = OSPI_D1_PIN_AF;
  HAL_GPIO_Init(OSPI_D1_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D2 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D2_PIN;
  GPIO_InitStruct.Alternate = OSPI_D2_PIN_AF;
  HAL_GPIO_Init(OSPI_D2_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D3 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D3_PIN;
  GPIO_InitStruct.Alternate = OSPI_D3_PIN_AF;
  HAL_GPIO_Init(OSPI_D3_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  De-Initializes the OSPI MSP.
  * @param  hospi OSPI handle
  * @retval None
  */
static void OSPI_FRAM_MspDeInit(OSPI_HandleTypeDef *hospi)
{
  /* hospi unused argument(s) compilation warning */
  UNUSED(hospi);

  /* OctoSPI GPIO pins de-configuration  */
  HAL_GPIO_DeInit(OSPI_CLK_GPIO_PORT, OSPI_CLK_PIN);
  HAL_GPIO_DeInit(OSPI_CS_GPIO_PORT, OSPI_CS_PIN);
  HAL_GPIO_DeInit(OSPI_D0_GPIO_PORT, OSPI_D0_PIN);
  HAL_GPIO_DeInit(OSPI_D1_GPIO_PORT, OSPI_D1_PIN);
  HAL_GPIO_DeInit(OSPI_D2_GPIO_PORT, OSPI_D2_PIN);
  HAL_GPIO_DeInit(OSPI_D3_GPIO_PORT, OSPI_D3_PIN);

  /* Reset the OctoSPI memory interface */
  OSPI_FORCE_RESET();
  OSPI_RELEASE_RESET();

  /* Disable the OctoSPI memory interface clock */
  OSPI_CLK_DISABLE();
}

/************************ (C) COPYRIGHT STMicroelectronics / Cypress *****END OF FILE****/

