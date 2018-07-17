/****************************************************************************
*File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This is a code example for accessing the SPI F-RAM using PSoC6 SMIF
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

#include <stdint.h>
#include <stdio.h>
#include <project.h>
#include "CY_SMIF_FRAM_CONFIG.h"
#include "FRAM_ACCESS.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/

/*Size of buffer and Id reg*/
#define BUFFER_SIZE            (256u)    
#define DID_REG_SIZE      	   (9u)
#define MEM_OFFSET             (0x100) /*Memory offset for new start address in
                                         memory mapped mode*/
#define DISP_PER_LINE          (0x10)  /*Number of hex display per line */                                    

/*******************************************************************************
* Global variables and functions
*******************************************************************************/

cy_stc_smif_context_t smifContext;
uint8_t extMemAddress[ADDRESS_SIZE] = {0x00, 0x00, 0x00}; /* Buffer to store start address for memory write and read operations */

void ExtMemInterrupt(void);
void PrintData(uint8_t *tst_rxBuffer,uint32_t size); /* Send the buffer data to the console */    							

/*************************************************************************************
* Function Name: main
**************************************************************************************
*
*Executes 9-byte device ID read using SMIF in memory mapped IO (MMIO)  
*
*Executes burst (256-Byte) memory write and read from a start address 
 using SMIF MMIO (Normal mode)
*
*Executes 256-Byte memory read using  SMIF memory mapped (or XIP, eXecute-In-Place). 
 Demonstrates both 4-byte (32-bit)  and 1-byte (8-bit) access per AHB transaction
*
*Executes 256-Byte memory write and read from a start address using SMIF memory mapped (or XIP)
 Demonstrates both 4-byte (32-bit) and 1-byte (8-bit) access per AHB transaction 
*
* Parameters:
*  None
*
* Return:
*  int
*
*************************************************************************************/
int main()
{
    uint8_t  write_fram_buffer_8[BUFFER_SIZE];     /* Data buffer for storing data array to be written in burst mode */
    uint32_t write_fram_buffer_32[BUFFER_SIZE];    /* Data buffer for storing data array to be written in burst mode */
    uint8_t  read_fram_buffer_8[BUFFER_SIZE];      /* Data buffer to read data array in burst mode */
    uint32_t read_fram_buffer_32[BUFFER_SIZE];     /* Data buffer to read data array in burst mode */
    uint32_t volatile *Xip_Buffer_32;              /* Xip  buffer for 32 bit data */  
    uint8_t  volatile *Xip_Buffer_8;                /* Xip  buffer for 8-bit data */  
    uint32 loopcount;                              /* Loop count for for 32_bit uint loops */ 
    uint8_t  tmpread_8;                            /* Temp variable to store intermediate data byte */

/* Initilaize the Xip buffer for memory mapped accesses */
    Xip_Buffer_32 =(uint32_t *)SPI_FRAM_SlaveSlot_2.baseAddress; /* Assign the SMIF base address to Xip buffer*/
    Xip_Buffer_8 =(uint8_t *)SPI_FRAM_SlaveSlot_2.baseAddress; /* Assign the SMIF base address to Xip buffer*/

    __enable_irq();  /* Enable global interrupts */                                                              
    
    UART_Start();    /* Start UART operation. */
    
    cy_stc_sysint_t smifIntConfig =
    {
        .intrSrc = (IRQn_Type)smif_interrupt_IRQn,      /* SMIF interrupt */
        .intrPriority = 0x01                            /* SMIF interrupt priority */
    };
    
    Cy_SysInt_Init (&smifIntConfig, ExtMemInterrupt);

/* SMIF initialization */
    Cy_SMIF_Init(SMIF0, &SMIF_FRAM_config, TIMEOUT_1_MS, &smifContext);    
    Cy_SMIF_Enable(SMIF0, &smifContext);    
    Cy_SMIF_Memslot_Init(SMIF0, (cy_stc_smif_block_config_t *)&smifBlockConfig, &smifContext); 
    
/* Initializes the XIP mode registers */  
    Cy_SMIF_SetMode(SMIF0, CY_SMIF_NORMAL); /* Sets the SMIF mode to access in MMIO mode*/
    
/* Enables the SMIF interrupt */
    SMIF_EnableInt();    
    
    printf("\r\n********SPI F-RAM Access using PSoC 6 SMIF - Code Example (CE224073)*********"); 
    
/******************************************/ 
/***********Read -9Byte ID ****************/
/******************************************/  
    
    WriteCmdRDID(SMIF0, &smifContext,&read_fram_buffer_8[0],DID_REG_SIZE);
    printf("\r\nRead Device ID (RDID 0x9F)\r\n ");
    PrintData(&read_fram_buffer_8[0], DID_REG_SIZE);        

/**********************************************************************/ 
/*******256 Byte Memory Write and Read using SMIF normal mode(MMIO)****/
/**********************************************************************/   
    
/* Write 256 bytes write_fram_buffer at extmemAddress in normal mode*/      
/* Set the write buffer and clear the read buffer */          
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	   {
	    read_fram_buffer_8[loopcount] = 0;
	    write_fram_buffer_8[loopcount] = loopcount+0x0A;
	   }  
    
    WriteCmdSPIWrite(SMIF0, &smifContext,&write_fram_buffer_8[0] ,BUFFER_SIZE, extMemAddress);
    printf("\r\n\r\nWrite data (256-Byte) in MMIO mode: "); 
 
/* Send the start address and write data bytes to UART for display */   
    printf("\r\nWrite Address: "); 
    PrintData(extMemAddress, 0x03);    
    printf("\r\nWrite Data:\r\n"); 
    PrintData(&write_fram_buffer_8[0], BUFFER_SIZE);   

/* Read 256 bytes in read_fram_buffer from extmemAddress */    
    WriteCmdSPIRead(SMIF0, &smifContext, &read_fram_buffer_8[0], BUFFER_SIZE, extMemAddress);
    
/* Send the start address and read data bytes to UART for display */   
    printf("\r\n\r\nRead Data (256-Byte) in MMIO mode: "); 
    printf("\r\nRead Address: "); 
    PrintData(extMemAddress, 0x03);
         
    printf("\r\nRead Data:\r\n"); 
    PrintData(&read_fram_buffer_8[0], BUFFER_SIZE);  
    
/*************************************************************************/ 
/*******256 Byte Memory Write and Read using SMIF memory mapped (XIP)*****/
/*************************************************************************/     
    
    printf("\r\n\r\nSet the SMIF memory mapped (XIP) mode \r\n");
    Cy_SMIF_CacheDisable(SMIF0, CY_SMIF_CACHE_BOTH);    
    Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY);   
    
    printf("\r\nRead in memory mapped (XIP) read - 32 bit AHB access \r\n"); 
    printf("Read sddress: (@base address = 0x18000000)\r\n"); 
    printf("Read data (32-bit per access):\r\n");  

    for(loopcount=0; loopcount<BUFFER_SIZE/4; loopcount++) /* Read 256 bytes (4-byte read per AHB access) */
       read_fram_buffer_32[loopcount] = Xip_Buffer_32[loopcount];      
     
    /* Disaply read data on the UART screen)*/    
    for(loopcount=0; loopcount<BUFFER_SIZE/4; loopcount++) 
     {
       tmpread_8 = read_fram_buffer_32[loopcount]>>24;        
       PrintData(&tmpread_8, 0x01);
       
       tmpread_8 = read_fram_buffer_32[loopcount]>>16;        
       PrintData(&tmpread_8, 0x01);
       
       tmpread_8 = read_fram_buffer_32[loopcount]>>8;        
       PrintData(&tmpread_8, 0x01);
       
       tmpread_8 = read_fram_buffer_32[loopcount];        
       PrintData(&tmpread_8, 0x01);
       
       printf("\r\n");     
     } 
    
    printf("\r\nRead in memory mapped (XIP) read - 8 bit AHB access \r\n"); 
    printf("Read Address: (@base address = 0x18000000)\r\n"); 
    printf("Read Data (8-bit per access):\r\n"); 
    for(loopcount=0; loopcount<BUFFER_SIZE; loopcount++)  /* Read 256 bytes (1-byte read per AHB access) */
    {
       tmpread_8 = Xip_Buffer_8[loopcount];
    
       PrintData(&tmpread_8, 0x01);
       
       if (!((loopcount+1)%DISP_PER_LINE))                     /* To start display at new line after every 8 bytes */ 
       printf("\r\n");   
    } 
    
    printf("\r\n\r\nSet SMIF mode to MMIO (normal) to set the WEL prior to memory write \r\n\r\n");

    Cy_SMIF_SetMode(SMIF0, CY_SMIF_NORMAL);   
    WriteCmdWREN(SMIF0, &smifContext);

   /* Set the write buffer prior to XIP write */

   for(loopcount=0; loopcount<BUFFER_SIZE/4; loopcount++) /* Fill the data buffer for 4-byte write per AHB access */
    {   
      write_fram_buffer_32[loopcount] =loopcount<<16|loopcount;     
    }           
    
   for(loopcount=0; loopcount<BUFFER_SIZE; loopcount++) /* Fill the data buffer for 1-byte write per AHB access */
    {        
      write_fram_buffer_8[loopcount] =loopcount+0x50;      
    }  
    
    printf("Set the SMIF memory mapped (XIP) mode for F-RAM write and read\r\n");         
    Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY);       
    
    printf("\r\nWrite using SMIF memory mapped (XIP) - 32 bit write per AHB access");
    printf("\r\nWrite Address: (@base address = 0x18000000)\r\n"); 
    
    for(loopcount=0; loopcount<BUFFER_SIZE/4; loopcount++) 
     {       
        Xip_Buffer_32[loopcount]= write_fram_buffer_32[loopcount]; /* Memory mapped (XIP) write of 256-byte data 4-byte per AHB access */
                                                                   /* Write at memory mapped address-0x18000000U */
      /* Standard SPI (Only) F-RAMs clear the WEL at the end of write cycle (CS high) */
      /* Hence, standard SPI (Only) F-RAMs will require WEL set to '1' prior 
         to executing every write using the memory mapped */
      /* Comment the following lines of code if accessing the QSPI F-RAM in SPI mode. 
         The QSPI F-RAMs don't clear the WEL at the end of memory write */
      
        Cy_SMIF_SetMode(SMIF0, CY_SMIF_NORMAL);   
        WriteCmdWREN(SMIF0, &smifContext);
        Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY); 
    }   
    
    printf("\r\nWrite using SMIF memory mapped (XIP) - 8 bit write per AHB access");  
    printf("\r\nWrite Address: (@@base address = 0x18000100)\r\n"); 
    
    for(loopcount=0; loopcount<BUFFER_SIZE; loopcount++) 
    {        
        Xip_Buffer_8[MEM_OFFSET+loopcount] = write_fram_buffer_8[loopcount];        /* Memory mapped (XIP) write of 256-byte data 1-byte per AHB access  */
                                                                                    /* Write at memory mapped address-0x18000100U (Base address + offset)*/
            
      /* Standard SPI (Only) F-RAMs clear the WEL at the end of write cycle (CS high) */
      /* Hence, standard SPI (Only) F-RAMs will require WEL set to '1' prior 
         to executing every write using the memory mapped */
      /* Comment the following lines of code if accessing the QSPI F-RAM in SPI mode. 
         The QSPI F-RAMs don't clear the WEL at the end of memory write */
        
        Cy_SMIF_SetMode(SMIF0, CY_SMIF_NORMAL);   
        WriteCmdWREN(SMIF0, &smifContext);
        Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY); 
    }   
    
    printf("\r\nRead using SMIF memory mapped (XIP) - 32 bit read per AHB access\r\n");
    printf("Read Address: (@base address = 0x18000000)\r\n"); 
    printf("Read Data (32-bit per access):\r\n");

    for(loopcount=0; loopcount<BUFFER_SIZE/4; loopcount++) 
        read_fram_buffer_32[loopcount] = Xip_Buffer_32[loopcount];  /* Memory mapped (XIP) read of 256-byte data 4-byte per AHB access */    
                                                                 /* Read from memory mapped address-0x18000000U */
    /* Disaply on the UART screen)*/    
    for(loopcount=0; loopcount<BUFFER_SIZE/4; loopcount++)
     {
        tmpread_8 = read_fram_buffer_32[loopcount]>>24;        
        PrintData(&tmpread_8, 0x01);
       
        tmpread_8 = read_fram_buffer_32[loopcount]>>16;        
        PrintData(&tmpread_8, 0x01);
       
        tmpread_8 = read_fram_buffer_32[loopcount]>>8;        
        PrintData(&tmpread_8, 0x01);
       
        tmpread_8 = read_fram_buffer_32[loopcount];        
        PrintData(&tmpread_8, 0x01);
       
        printf("\r\n");     
     } 
    
    printf("\r\nRead using SMIF memory mapped (XIP) - 8 bit read per AHB access"); 
    printf("\r\nRead Address: (@base address = 0x18000100)\r\n");
    printf("\r\nRead Data (8-bit per access):\r\n");
    
    for(loopcount=0; loopcount<BUFFER_SIZE; loopcount++)  
     {
        tmpread_8 = Xip_Buffer_8[MEM_OFFSET+loopcount];        /* Memory mapped (XIP) read of 256-byte data 1-byte per AHB access     */
        PrintData(&tmpread_8, 0x01);                           /* Read from memory mapped address-0x18000100U (Base address + offset) */        
       
        if (!((loopcount+1)%DISP_PER_LINE))                    /* To start display at new line after every 8 bytes */ 
        printf("\r\n"); 
     } 

    for(;;)
     {  
        /* Loops forever*/
        /* CM4 does nothing after SMIF operation is complete. */
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
   
    for(index=0; index<size; index++)
    {
        printf("0x%02X ", (unsigned int) tst_txBuffer[index]);
      
        if (!((index+1)%DISP_PER_LINE))                      /* To start display at new line after every 16 bytes */ 
        printf("\r\n"); 
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
                while (0UL == UART_Put(ch))
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
