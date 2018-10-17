/****************************************************************************
*File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This is a code example for accessing the SPI F-RAM using PSoC6 SMIF
*
* Related Document: CE222967.pdf
*
* Hardware Dependency PSoC 6 Pioneer Kit (CY8CKIT-062-BLE) WITH SERIAL F-RAM 
*
*****************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation.
*****************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <project.h>
#include <inttypes.h>
#include "FRAM_ACCESS.h"
#include "SMIF_FRAM.h"
#include "cy_smif_memconfig.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/

/*Size of data buffer*/
#define BUFFER_SIZE            (255u)
#define BUFFER_SIZE_XIP         (25u)

/* External memory address */
#define MEMBASE                 (0x18000000)

/*******************************************************************************
* Global variables and functions
*******************************************************************************/
uint32_t CONFIG_REG2_ADDR =0x000003;     /* FRAM Device Configuration Register 2 (CR2) address */
uint32_t ACCESS_MODE = QPI_MODE;         /* Sets the device access mode */
 uint8_t MLC=0x08;                       /*Sets max latency for memory read at 100 MHz*/                   
 uint8_t RLC=0x03;                       /*Sets max latency for register read at 100 MHz*/
                                         /*Refer to QSPI F-RAM (CY15x104QSN)datasheet for details*/ 
 
 uint8_t regwBuffer[1]; 
 uint8_t DID_Reg[8]={0x50, 0x51, 0x82, 0x06, 0x00, 0x00, 0x00, 0x00}; /* 8-byte Device ID */
 uint8_t extMemAddress[ADDRESS_SIZE] = {0x00, 0x00, 0x00};            /* Memory address for write and read access */
 uint8_t extMemAddressWithMode[ADDRESS_PLUS_MODE_SIZE] = {0x00, 0x00, 0x00, 0x00};  /* Memory address for read access */ 
                                                                                    /* with mode byte */
uint32_t multBuf[BUFFER_SIZE];                                                    
cy_stc_smif_context_t smifContext;

void ExtMemInterrupt(void);
void HandleError(void);
void MultiplyXArray(volatile uint32_t * const XArray, uint32_t size);
void MultiplyXArray_8(volatile uint8_t * const XArray, uint8_t size);
void PowerUpMemoryDefaultSPI (void);
void PrintData(uint8_t *tst_rxBuffer,uint32_t size); /* Send the buffer data to the console */

/*******************************************************************************
* Function Name: main
********************************************************************************/
int main()
{
    /* Status variables */
    cy_en_sysint_status_t  intStatus;
    cy_en_smif_status_t    smifStatus;
    cy_en_dma_status_t     dmaStatus;
    cy_en_trigmux_status_t dmaTriggerStatus;
    
    uint32_t fillVal;                          /* Starting value of data to write to external F-RAM */
    uint32_t loopcount;                        /* Counter for For loops */
    uint8_t  write_fram_buffer[BUFFER_SIZE];   /* Data buffer for storing data array to be written in burst mode */    
    uint8_t  read_fram_buffer[BUFFER_SIZE];    /* Data buffer to read data array in burst mode */    
    uint32_t volatile * const Xip_buffer_32 = (uint32_t *) MEMBASE; /* 32 Bit Buffer in external memory */
    uint32_t volatile SRAM_buffer[BUFFER_SIZE];                     /* Local buffer for data transfer */
    uint32_t volatile source_buffer[BUFFER_SIZE];                   /* Local buffer for data transfer */
    
    cy_stc_dma_channel_config_t channelConfig;
    
    /* Enable global interrupts */
    __enable_irq();
    
    /* Start UART operation */
    UART_Start();    

    /* Initializes the SMIF FRAM interrupt. */
    intStatus = Cy_SysInt_Init(&SMIF_FRAM_SMIF_IRQ_cfg, ExtMemInterrupt);
	if(intStatus != CY_SYSINT_SUCCESS)
    {
        /* SMIF FRAM interrupt initialization failed, handle error */
        Cy_SCB_UART_PutString(UART_HW, "SMIF FRAM interrupt init failed\r\n");
        HandleError();
    }    
    
    /* SMIF block initialization */   
    smifStatus = Cy_SMIF_Init(SMIF0, &SMIF_FRAM_config, TIMEOUT_1_MS, &smifContext);  
    if(smifStatus != CY_SMIF_SUCCESS)
    {
        /* Smif initialization failed, handle error */
        Cy_SCB_UART_PutString(UART_HW, "SMIF initialization failed\r\n");        
        HandleError();
    }
    
    /* Enables the operation of the SMIF block. */
    Cy_SMIF_Enable(SMIF0, &smifContext);  
    
    /* Enable the SMIF interrupt */
    SMIF_EnableInt();
    
    /* initializes the XIP mmemory slot and maps memory slot devices into PSoC memory */
    smifStatus = Cy_SMIF_Memslot_Init(SMIF0, (cy_stc_smif_block_config_t*)&smifBlockConfig, &smifContext); 
    if(smifStatus != CY_SMIF_SUCCESS)
    {
        /* Smif memory slot initialization failed, handle error */
        Cy_SCB_UART_PutString(UART_HW, "SMIF memory slot initialization failed\r\n");           
        HandleError();
    }
    
/*****************************************************************************************
* DMA Init                                                                               *
* Set the channel descriptor configs (source, destination, xCount                        *
* Descriptor 1: SRAM to SRAM (256 words)                                                 *
* Descriptor 2: SRAM to FRAM (256 words)                                                 *
* Descriptor 3 is simply a starting descriptor. This is a single word dummy transaction. * 
* Trigger out of Descritor 3 is used to probe the start of Descriptor 1.                 *
*****************************************************************************************/
    DMA_1_Descriptor_1_config.srcAddress = (uint32_t *) source_buffer;
    DMA_1_Descriptor_1_config.dstAddress = (uint32_t *) Xip_buffer_32;
    DMA_1_Descriptor_2_config.srcAddress = (uint32_t *) Xip_buffer_32;
    DMA_1_Descriptor_2_config.dstAddress = (uint32_t *) SRAM_buffer;
    DMA_1_Descriptor_3_config.srcAddress = (uint32_t *) SRAM_buffer;
    DMA_1_Descriptor_3_config.dstAddress = (uint32_t *) Xip_buffer_32;    
    DMA_1_Descriptor_1_config.xCount = BUFFER_SIZE_XIP;
    DMA_1_Descriptor_2_config.xCount = BUFFER_SIZE_XIP;   
    DMA_1_Descriptor_3_config.xCount = BUFFER_SIZE_XIP;

    /* Initialize DMA descriptors */
    dmaStatus =  Cy_DMA_Descriptor_Init(&DMA_1_Descriptor_1, &DMA_1_Descriptor_1_config);
    dmaStatus += Cy_DMA_Descriptor_Init(&DMA_1_Descriptor_2, &DMA_1_Descriptor_2_config);
    dmaStatus += Cy_DMA_Descriptor_Init(&DMA_1_Descriptor_3, &DMA_1_Descriptor_3_config);
    if(dmaStatus != CY_DMA_SUCCESS)
    {
        /* DMA channel initialization failed, handle error */
        Cy_SCB_UART_PutString(UART_HW, "DMA descriptor initialization failed\r\n");           
        HandleError();
    }       
    
    /* Set DMA channel configurations */
    channelConfig.descriptor        = &DMA_1_Descriptor_1;
    channelConfig.preemptable       = DMA_1_PREEMPTABLE;
    channelConfig.priority          = DMA_1_PRIORITY;
    channelConfig.enable            = 0u;
    
    dmaStatus = Cy_DMA_Channel_Init(DMA_1_HW, DMA_1_DW_CHANNEL, &channelConfig);
    if(dmaStatus != CY_DMA_SUCCESS)
    {
        /* DMA channel initialization failed, handle error */
        Cy_SCB_UART_PutString(UART_HW, "DMA initialization failed\r\n");           
        HandleError();
    }    
    
    /* Enables the DMA channel interrupt mask */
    Cy_DMA_Channel_SetInterruptMask(DMA_1_HW, DMA_1_DW_CHANNEL, CY_DMA_INTR_MASK);
    
    /* Enables DMA channel */
    Cy_DMA_Channel_Enable(DMA_1_HW, DMA_1_DW_CHANNEL);
   
    /* Enable DMA hardware block */
    Cy_DMA_Enable(DMA_1_HW);
    
    /*DMA init complete*/
    
    printf("SMIF & DMA initialization finished\r\n");    
    printf("\r\n********QSPI F-RAM Access in XIP with PSoC 6 SMIF*********");
    printf("\r\n********Setting the device access mode to QSPI*********\r\n"); 
    
    /* Set config register 2 to indices in extMemAddress[] */
    extMemAddress [2]=CONFIG_REG2_ADDR;
    extMemAddress [1]=CONFIG_REG2_ADDR>>8;
    extMemAddress [0]=CONFIG_REG2_ADDR>>16;  
    
    /* Sets the device access mode to default SPI */     
    PowerUpMemoryDefaultSPI(); 
        
    ACCESS_MODE = SPI_MODE;   /*Set the SPI Access Mode*/ 
    
    printf("\r\n********Set the device access mode to QPI*********"); 
    
    extMemAddress [2]=CONFIG_REG2_ADDR;
    extMemAddress [1]=CONFIG_REG2_ADDR>>8;
    extMemAddress [0]=CONFIG_REG2_ADDR>>16;     
    regwBuffer[0] = 0x40;    
    
/******************************************************************
* Set SMIF into QPI mode and read the device ID and Serial Number *
******************************************************************/
    
    /* Write 0x40 to CR2 register to set the access mode mode to QPI (CR2[6] = 1)*/ 
    WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, &regwBuffer[0], 0x01, extMemAddress, ACCESS_MODE);
    
    ACCESS_MODE = QPI_MODE;     
    
    printf("\r\n\r\n 8-byte Device ID read in QPI - "); 
    
    /* Send RDID Cmd */
    WriteCmdRDID(SMIF0, &smifContext,&read_fram_buffer[0], ACCESS_MODE, RLC);
    PrintData(&read_fram_buffer[0], DID_REG_SIZE);
    
    /* Send read Serial Number */
    printf("\r\n 8-byte SN read in QPI        - "); 
    WriteCmdRDSN(SMIF0, &smifContext,&read_fram_buffer[0],DID_REG_SIZE, ACCESS_MODE,RLC);
    PrintData(&read_fram_buffer[0], SN_BUF_SIZE);     
    
    
/******************************************************************
* In normal mode:                                                 *  
* Send write enable and write 8 bit data buffer in QPI mode       *
* Read back written data into local buffer                        * 
******************************************************************/
    
    /* Send write enable */
    WriteCmdWREN(SMIF0, &smifContext, ACCESS_MODE); 
    
    /* Fill buffer with data to write to FRAM */
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
    {
        write_fram_buffer[loopcount] = loopcount + 0x55;
    }    
    
    /* Set write address to base of FRAM */
    extMemAddress [2]=CONFIG_REG2_ADDR;
    extMemAddress [1]=CONFIG_REG2_ADDR>>8;
    extMemAddress [0]=CONFIG_REG2_ADDR>>16; 
    
    /* Write the data into the FRAM and print it to UART */
    WriteCmdSPIWrite(SMIF0, &smifContext, &write_fram_buffer[0], 0xFF, extMemAddress, ACCESS_MODE);
    printf("\r\n\r\n::8-Bit Written data::\r\n"); 
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
    {
        if (loopcount % 21 == 0 && loopcount > 0)
        {
            printf("\n\r");
        }
        printf("0x%02X ", write_fram_buffer[loopcount]);
    }    
    
    /* Fill read buffer with zeros before read from FRAM */
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
    {
        read_fram_buffer[loopcount] = 0x00;    
    }
    
    /* Set write address to base of FRAM */
    extMemAddress [2]=CONFIG_REG2_ADDR;
    extMemAddress [1]=CONFIG_REG2_ADDR>>8;
    extMemAddress [0]=CONFIG_REG2_ADDR>>16;     
    
    /* Send read command with buffer to read data into */
    WriteCmdSPIRead(SMIF0, &smifContext, &read_fram_buffer[0], 0xFF, extMemAddress, ACCESS_MODE, MLC);
    printf("\r\n\r\n::8-Bit Read data in QPI normal mode:: \r\n"); 
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
    {
        if (loopcount % 21 == 0 && loopcount > 0)
        {
            printf("\n\r");
        }           
        printf("0x%02X ", read_fram_buffer[loopcount]);
    }  
   
/******************************************************************
* In XIP mode:                                                    *  
* Send write enable and write 32 bit data buffer in QPI mode      *
* Read back written data into local buffer                        * 
* Multiply data in external FRAM                                  *    
******************************************************************/    
    

    Cy_SMIF_CacheDisable(SMIF0, CY_SMIF_CACHE_BOTH);
    if(smifStatus != CY_SMIF_SUCCESS)
    {
        /* Smif memory slot initialization failed, handle error */
        Cy_SCB_UART_PutString(UART_HW, "SMIF cache disable failed\r\n");           
        HandleError();
    }    
    
    /* Set XIP mode */    
    Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY);
    
/**************************************
* Begin operations with DMA          *
**************************************/    
    
    printf("\r\n\r\n::DMA operations beginning::\r\n");   
    
    fillVal = 0x11111111; /* Set fillVal > 0x11111110 for UART formatting */
    
    /* Fill source buffer with incrementing values to 255 */
    /* Clear SRAM and XIP buffers                         */
    for(loopcount = 1; loopcount < BUFFER_SIZE_XIP + 1; loopcount++)
    {
        SRAM_buffer[loopcount-1]   = 0;
        Xip_buffer_32[loopcount-1] = 0;
        source_buffer[loopcount-1] = fillVal * loopcount;
    }    
    
    /* Trigger DMA transfer to load buffer in external memory    *
    *  and from external memory to system flash for manipulation */
    channelConfig.descriptor = &DMA_1_Descriptor_1;
    printf("\r\nDMA: Transferring Data\r\n");
    dmaTriggerStatus = DMA_1_Trigger(3);
    if(dmaTriggerStatus != CY_TRIGMUX_SUCCESS)
    {
        Cy_SCB_UART_PutString(UART_HW, "DMA trigger failed");
    }    
    else
    {
        printf("DMA: Done\r\n");      
    }
    
    /* Print SRAM buffer */
    printf("\r\n::SRAM BUFFER after DMA transfer::\r\n");
    for(loopcount = 0; loopcount < BUFFER_SIZE_XIP; loopcount++)
    {
        if (loopcount % 9  == 0 && loopcount > 0)/* New line every 9 points */
        {
            printf("\n\r");
        }
        printf("%#" PRIx32 " ", (uint32_t) SRAM_buffer[loopcount]);
    }
    printf("\r\n");   

    /* Print XIP buffer */
    printf("\n\r::XIP Buffer data in SRAM after DMA transfer::\n\r");
    for(loopcount = 0; loopcount < BUFFER_SIZE_XIP; loopcount++)
    {
        if (loopcount % 9  == 0 && loopcount > 0)/* New line every 9 points */
        {
            printf("\n\r");
        }
        printf("%#" PRIx32 " ", (uint32_t) Xip_buffer_32[loopcount]);
    } 
    printf("\r\n");    
    
    /* Multiply the SRAM buffer */
    MultiplyXArray(Xip_buffer_32, BUFFER_SIZE_XIP);    
    
    /* Print multiplied SRAM buffer */
    printf("\r\n::XIP buffer after multiply in external memory::\r\n");
    for(loopcount = 0; loopcount < BUFFER_SIZE_XIP; loopcount++)
    {
        if (loopcount % 9  == 0 && loopcount > 0)/* New line every 9 points */
        {
            printf("\n\r");
        }
        printf("%#" PRIx32 " ", (uint32_t) Xip_buffer_32[loopcount]);
    }    
    
    /* DMA the sram buffer back into FRAM */
    
    dmaTriggerStatus = DMA_1_Trigger(1);
    if(dmaTriggerStatus != CY_TRIGMUX_SUCCESS)
    {
        Cy_SCB_UART_PutString(UART_HW, "DMA trigger failed");
    }    
    else
    {
        printf("\r\n::DONE!::\r\n");    
    }    
    

    for(;;)
      {  
        /*Loops forever*/
        /* CM4 does nothing after SMIF operation is complete. */
      }    
}     

/*******************************************************************************
* Function Name: PowerUpMemoryDefaultSPI
****************************************************************************//**
*
* This functions sets the F-RAM to SPI mode.  
* Configures all user registers (status and configuration registers) to factory default.
*
*******************************************************************************/

void PowerUpMemoryDefaultSPI (void)
{                                
   /* Write CR2 for interface mode */
    {
        uint8_t CR2_Address[3] = {0x00, 0x00, 0x03}; /* CR2 register address 0x000003 */
        uint8_t CR2_Value[1] = {0x00};              /* CR2 value to set the SPI mode */
        
        /* Try in QPI mode*/
        WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, CR2_Value, sizeof CR2_Value, CR2_Address, QPI_MODE);

        /* Try in DPI mode*/
        WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, CR2_Value, sizeof CR2_Value, CR2_Address, DPI_MODE);
    }

    /* Write CR1 for memory latency setting */
    {
        uint8_t CR1_Address[3] = {0x00, 0x00, 0x02};  /* CR1 register address 0x000002 */
        uint8_t CR1_Value[1] = {(MLC<<4)|0x02};      /* CR1 value to set the memory access latency */
        
        WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, CR1_Value, sizeof CR1_Value, CR1_Address, SPI_MODE);
    }

     /* Write CR5 for register latency setting */
    {
        uint8_t CR5_Address[3] = {0x00, 0x00, 0x06};  /* CR5 register address 0x000006 */
        uint8_t CR5_Value[1] = {(RLC<<6)};           /* CR5 value to set the register access latency */

        WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, CR5_Value, sizeof CR5_Value, CR5_Address, SPI_MODE);
    }
}

/*******************************************************************************
* Function Name: ExtMemInterrupt
****************************************************************************//**
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external 
* memory are processed inside the SMIF ISR.
*  
*******************************************************************************/

void ExtMemInterrupt(void)
{
    Cy_SMIF_Interrupt(SMIF0, &smifContext);
}

/*******************************************************************************
* Function Name: MultiplyXArray
********************************************************************************
*   
* This function multiplies the contents of an input array by 2  
*
* \param XArray - The array to be multiplied  
* \param size - The size of the array being multiplied
********************************************************************************/
void MultiplyXArray(volatile uint32_t * const XArray, uint32_t size)
{
    uint32_t loopcount;
    
    for(loopcount = 0; loopcount < size; loopcount++)
    {
        XArray[loopcount] = 2 * XArray[loopcount];
    }    
}

/*******************************************************************************
* Function Name: MultiplyXArray_8
********************************************************************************
*   
* This function multiplies the contents of an input array by 2  
*
* \param XArray - The array to be multiplied  
* \param size - The size of the array being multiplied
********************************************************************************/
void MultiplyXArray_8(volatile uint8_t * const XArray, uint8_t size)
{
    uint8_t loopcount;
    
    for(loopcount = 0; loopcount < size; loopcount++)
    {
        XArray[loopcount] = 2 * XArray[loopcount];
    }    
}


/*******************************************************************************
* Function Name: HandleError
********************************************************************************
*
* Halts the CM4
*
*******************************************************************************/
void HandleError(void)
{
    Cy_SysLib_Halt(0x00); 
}

/*******************************************************************************
* Function Name: PrintData
****************************************************************************//**
*
* This function prints the content of the TX buffer to the UART console.
* 
* \param  tst_txBuffer - the buffer to output to the console
* 
* \param  size -  the size of the buffer to output to the console
*
*******************************************************************************/
void PrintData(uint8_t * tst_txBuffer, uint32_t size)
{
    uint32_t index;
   
    for(index = 0; index<size; index++)
    {
        printf("0x%02X ", (unsigned int) tst_txBuffer[index]);
    }
}

/*******************************************************************************
* Function Name: _write
*******************************************************************************/
#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library to allow
    the usage of the floating point conversion specifier. */
    asm (".global _printf_float");
    /* For GCC compiler revise _write() function for printf functionality */
    int _write(int file, char *ptr, int len)
    {
        int nChars = 0;

        /* Suppress the compiler warning about an unused variable. */
        if (0 != file)
        {
        }
        
        for (/* Empty */; len != 0; --len)
        {
            /* Block until there is space in the TX FIFO or buffer. */
            while (0UL == UART_Put(*ptr))
            {
            }
            
            ++nChars;
            ++ptr;
        }
        
        return (nChars);
    }
    #elif defined(__ARMCC_VERSION)
    /* For the MDK/RVDS compiler, revise the  fputc() function for the  printf functionality */
    struct __FILE
    {
        int handle;
    };

    enum
    {
        STDIN_HANDLE,
        STDOUT_HANDLE,
        STDERR_HANDLE
    };

    FILE __stdin = {STDIN_HANDLE};
    FILE __stdout = {STDOUT_HANDLE};
    FILE __stderr = {STDERR_HANDLE};

    int fputc(int ch, FILE *file)
    {
        int ret = EOF;
        switch(file->handle)
        {
            case STDOUT_HANDLE:
                /* Block until there is space in the TX FIFO or buffer. */
                while (0UL == UART0_Put(ch))
                {
                }
                
                ret = ch;
            break;

            case STDERR_HANDLE:
                ret = ch;
            break;

            default:
                file = file;
            break;
        }
        return(ret);
    }

    #elif defined (__ICCARM__)
    /* For the IAR compiler, revise the  __write() function for the  printf functionality */
    size_t __write(int handle, const unsigned char * buffer, size_t size)
    {
        size_t nChars = 0;

        for (/* Empty */; size != 0; --size)
        {
            /* Block until there is space in the TX FIFO or buffer. */
            while (0UL == UART0_Put(*buffer))
            {
            }
            
            ++nChars;
            ++buffer;
        }
    return (nChars);
}
#endif /* (__GNUC__) */

/* [] END OF FILE */
