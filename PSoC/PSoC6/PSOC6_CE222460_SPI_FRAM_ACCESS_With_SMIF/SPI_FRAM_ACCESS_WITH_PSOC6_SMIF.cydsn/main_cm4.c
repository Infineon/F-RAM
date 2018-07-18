/****************************************************************************
*File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This is a code example for accessing the SPI F-RAM using PSoC6 SMIF
*
* Related Document: CE222460.pdf
*
* Hardware Dependency PSoC 6 Pioneer Kit (CY8CKIT-062-BLE) WITH SERIAL F-RAM 
*
*****************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
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
#include "FRAM_ACCESS.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/

/*Size of data buffer*/
#define BUFFER_SIZE            (16u)

/*Test Status and LED control*/ 
#define TEST_PASS              (0x00)
#define TEST_FAIL              (0x01)
#define STATUS_LED_RED         (0x01)
#define STATUS_LED_GREEN       (0x02)
#define STATUS_LED_OFF         (0x03)
#define LED_ON_TIME            (0x3FF)   /* 1-sec delay */


/*Register Length*/ 
#define SR_SIZE      	       (1u)	     /* Status register1 size */
#define DID_REG_SIZE      	   (9u)	     /* Device ID status register size */
#define SN_BUF_SIZE      	   (8u)	     /* Serial Number register size */
#define USN_BUF_SIZE      	   (4u)	     /* Unique Serial Number register size  */

/*******************************************************************************
* Global variables and functions
*******************************************************************************/
uint32_t EXAMPLE_INITIAL_ADDR = 0x00;    /* Example F-RAM Initial Address */
uint32_t EXAMPLE_INITIAL_ADDR_SS = 0x00; /* Example F-RAM Initial Address for Special Sector */
uint32_t EXAMPLE_ANY_ADDR =0x123456;     /* Example F-RAM Any Address */
uint8_t  EXAMPLE_DATA_BYTE = 0xCA;       /* Example F-RAM Data */
uint8_t  SERIAL_NUMBER_WRITE_SIZE = 0x08;

cy_stc_smif_context_t smifContext;
cy_en_smif_slave_select_t ONBOARD_SSFRAM = CY_SMIF_SLAVE_SELECT_2;

void ExtMemInterrupt(void);
void PrintData(uint8_t *tst_rxBuffer,uint32_t size); /* Send the buffer data to the console */
    							

/*************************************************************************************
* Function Name: main
**************************************************************************************
********RGB LED turns ON (GREEN - pass, RED -fail) for 1 second after every
******** execution and then turns off to turn ON again for the next execution result**  

*Turns ON GREEN LED after sucessful SMIF Initialization
*
*Executes write and read access to the Status Register - 
*
*Executes random (1-Byte) memory write, read and verify from an address
*
*Executes burst (16-Byte) memory write, read and verify from a start address
*
*Executes burst (16-Byte) special sector write, burst read and verify from address 0x00
*
*Executes Serial Number write, read and verify
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
    uint32 loopcount;                         /* Loop count for For loops */
    uint8_t write_example_byte;               /* Data buffer for one byte write */
    uint8_t read_example_byte;                /* Data buffer for one byte read */   
    uint8_t write_fram_buffer[BUFFER_SIZE];   /* Data buffer for storing data array to be written in burst mode */
    uint8_t read_fram_buffer[BUFFER_SIZE];    /* Data buffer to read data array in burst mode */
    uint8_t extMemAddress[ADDRESS_SIZE];      /* Buffer to store start address for memory write and read operations */
    uint8_t testResult;                       /* Variable to hold the test result status after read and verify */
    
    Cy_WDT_Unlock();
    Cy_WDT_Disable();
    
    __enable_irq();  /* Enable global interrupts */    
    UART_Start();    /* Start UART operation. */

/* The SMIF interrupt setup */
    cy_stc_sysint_t smifIntConfig =
    {
        .intrSrc = (IRQn_Type)smif_interrupt_IRQn,      /* SMIF interrupt */
        .intrPriority = SMIF_PRIORITY       /* SMIF interrupt priority */
    };
    
    Cy_SysInt_Init(&smifIntConfig, ExtMemInterrupt);

/* SMIF configuration parameters */
    cy_stc_smif_config_t SPI_FRAMConfig =
    {
        .mode           = CY_SMIF_NORMAL,   /* The mode of operation */
        .deselectDelay  = DESELECT_DELAY,   /* The minimum duration of SPI deselection */
        .rxClockSel     = RX_CLOCK_SELECT,  /* The clock source for the receiver clock */
        .blockEvent     = AHB_BUS_ERROR,    /* What happens when there is a Read to an empty RX FIFO or Write to a full TX FIFO*/
    };
	
/* SMIF initialization */
    Cy_SMIF_Init(SMIF0, &SPI_FRAMConfig, TIMEOUT_1_MS, &smifContext);
    Cy_SMIF_SetDataSelect(SMIF0, ONBOARD_SSFRAM, CY_SMIF_DATA_SEL0);
    Cy_SMIF_Enable(SMIF0, &smifContext);
    
/* Enable the SMIF interrupt */
    SMIF_EnableInt();
   
/* Status LED starts with GREEN */     
    STATUS_LED_CNTRL_Write(STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
    CyDelay(LED_ON_TIME); /* STATUS LED ON TIME */
    
    printf("\r\n********SPI F-RAM Access with PSoC 6 SMIF - Code Example (CE222460)*********"); 
    
/*********SPI F-RAM Access Examples********/  
      
/******************************************/ 
/***********Read -9Byte ID ****************/
/******************************************/  
    
    WriteCmdRDID(SMIF0, &smifContext,&read_fram_buffer[0],DID_REG_SIZE);
    printf("\r\nRead Device ID (RDID 0x9F)\r\n ");
    PrintData(&read_fram_buffer[0], DID_REG_SIZE);    
    
/******************************************/ 
/***********Write Status Register**********/
/******************************************/  
/* Write Status Register - 1-Byte */ 
/* ||Bit7||Bit6||Bit5||Bit4||Bit3) ||Bit2  ||Bit1  ||Bit0|| */
/* ||WPEN||X(0)||X(0)||X(1)||BP1(0)||BP0(0)||WEL(0)||X(0)|| */
    
   printf("\r\n\r\nWrite Status Register (WRSR 0x01) ");
   write_example_byte = 0x4C; /* Byte to write into status register */
   WriteCmdWRSR(SMIF0, &smifContext,&write_example_byte,SR_SIZE);   
   printf("\r\nWrite Data - ");
   PrintData(&write_example_byte, SR_SIZE);
    
/******************************************/ 
/***********Read Status Register***********/
/******************************************/ 
   printf("\r\n\r\nRead Status Register (RDSR 0x05) ");      
   WriteCmdRDSR(SMIF0, &smifContext, &read_example_byte, SR_SIZE);   
   printf("\r\nStatus Reg Value - ");
   PrintData(&read_example_byte, SR_SIZE);
  
/*Clear the status register memory block protect setting to 
 *enable memory write in follow on sections***************/    
    
    printf("\r\n\r\nClear the Status Register (WRSR 0x01) ");
    write_example_byte = 0x00;  /* ||WPEN||X(0)||X(0)||X(1)||BP1(0)||BP0(0)||WEL(0)||X(0)|| */
    WriteCmdWRSR(SMIF0, &smifContext,&write_example_byte,0x01); 
    printf("\r\nWrite Data - ");
    PrintData(&write_example_byte, SR_SIZE);
    
/* Read Status Register - 1-Byte */      
    
    printf("\r\n\r\nRead Status Register (RDSR 0x05) "); 
    WriteCmdRDSR(SMIF0, &smifContext, &read_example_byte, SR_SIZE);
    printf("\r\nDefault Status Reg Value - ");
    PrintData(&read_example_byte, SR_SIZE);
    
/******************************************/ 
/***1-Byte Memory Write and Read Example***/
/******************************************/
    
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME);
    
/* Write 1 byte EXAMPLE_DATA_BYTE at EXAMPLE_ANY_ADDRESS */    

    write_example_byte = EXAMPLE_DATA_BYTE;
    printf("\r\n\nMemory Write (WRITE 0x02)1-Byte");    
    extMemAddress [2]=EXAMPLE_ANY_ADDR; 
    extMemAddress [1]=EXAMPLE_ANY_ADDR>>8;
    extMemAddress [0]=EXAMPLE_ANY_ADDR>>16;
    
    WriteCmdSPIWrite(SMIF0, &smifContext,&write_example_byte ,0x01, extMemAddress);
       
/* Send the start address and write data to UART for display */
    printf("\r\nWrite Address: "); 
    PrintData(extMemAddress, 0x03);    
    printf("\r\nWrite Data (1-Byte): "); 
    PrintData(&write_example_byte, 0x01);   
 
/* Read 1-byte from EXAMPLE_ANY_ADDRESS */
    
    printf("\r\n\nMemory Read (READ 0x03) 1-Byte");
    extMemAddress [2]=EXAMPLE_ANY_ADDR; 
    extMemAddress [1]=EXAMPLE_ANY_ADDR>>8;
    extMemAddress [0]=EXAMPLE_ANY_ADDR>>16;
    
    WriteCmdSPIRead(SMIF0, &smifContext, &read_example_byte, 0x01, extMemAddress);    
    
    if ( read_example_byte!=write_example_byte)
         {
             STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */   
             printf("\r\nRead 1-Byte Fail");  
             CyDelay(LED_ON_TIME);
         }   
    else
        {
         STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */  
/* Send the start address read data to UART for display */
         printf("\r\nRead Address: "); 
         PrintData(extMemAddress, 0x03);
         
         printf("\r\nRead Data (1-Byte): "); 
         PrintData(&read_example_byte, 0x01);  
         printf("\r\nRead 1-Byte Pass"); 
         CyDelay(LED_ON_TIME);
        }

/*******************************************/ 
/***16 Byte Memory Write and Read Example***/
/*******************************************/   
   
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME);  
  
/* Write 16 bytes write_fram_buffer at extmemAddress */  
    
/* Set the write buffer and clear the read buffer */          
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	   {
	    read_fram_buffer[loopcount] = 0;
	    write_fram_buffer[loopcount] = loopcount;
	   }  
        
    extMemAddress [2]=EXAMPLE_INITIAL_ADDR; 
    extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
    extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;
    WriteCmdSPIWrite(SMIF0, &smifContext,&write_fram_buffer[0] ,BUFFER_SIZE, extMemAddress);
    printf("\r\n\r\nWrite Data (16-Byte): "); 
 
/* Send the start address and write data bytes to UART for display */   
    printf("\r\nWrite Address: "); 
    PrintData(extMemAddress, 0x03);    
    printf("\r\nWrite Data: "); 
    PrintData(&write_fram_buffer[0], BUFFER_SIZE);   

/* Read 16 bytes in read_fram_buffer from extmemAddress */    
    WriteCmdSPIRead(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress);
    
/* Send the start address and read data bytes to UART for display */   
    printf("\r\n\r\nRead Data (16-Byte): "); 
    printf("\r\nRead Address: "); 
    PrintData(extMemAddress, 0x03);
         
    printf("\r\nRead Data: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
    
    testResult = 0x00;
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
          { 
            printf("\r\nRead Data Fail: "); 
            STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */
            CyDelay(LED_ON_TIME);
            testResult = TEST_FAIL;
            break; 
         }
     }  
      if(!testResult)
         {
            printf("\r\nRead Data Pass "); 
            STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
            CyDelay(LED_ON_TIME);
        }
       
    
    /* Read 16 bytes in read_fram_buffer from extmemAddress */    
    WriteCmdSPIFastRead(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress);
    
/* Send the start address and read data bytes to UART for display */   
    printf("\r\n\r\nFast Read Data (16-Byte): "); 
    printf("\r\nRead Address: "); 
    PrintData(extMemAddress, 0x03);
    printf("\r\nRead Data: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
    
/********************************************/ 
/***16-Byte Special Sector Write and Read ***/
/********************************************/   
  
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME);
  
/* Set the write buffer and clear the read buffer */
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	 {
	   read_fram_buffer[loopcount] = 0;
	   write_fram_buffer[loopcount] = loopcount+0x0A;
	 }
    
/* Write 16 bytes write_fram_buffer at extmemAddress */         
        
    extMemAddress [2]=EXAMPLE_INITIAL_ADDR_SS; 
    extMemAddress [1]=EXAMPLE_INITIAL_ADDR_SS>>8;
    extMemAddress [0]=EXAMPLE_INITIAL_ADDR_SS>>16;
    WriteCmdSSWR(SMIF0, &smifContext,&write_fram_buffer[0] ,BUFFER_SIZE, extMemAddress);  
    printf("\r\n\r\nWrite Special Sector (SSWR 0x42) 16-Byte: "); 
    
/* Send the start address and write data bytes to UART for display */
    
    printf("\r\nSpecial Sector Write Address: "); 
    PrintData(extMemAddress, 0x03);
    printf("\r\nSpecial Sector Write Data: "); 
    PrintData(&write_fram_buffer[0], BUFFER_SIZE);   

/* Read 16 bytes in read_fram_buffer from extmemAddress */
    
    WriteCmdSSRD(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress);
    
/* Send the start address and read data bytes to UART for display */
    
    printf("\r\n\r\nSpecial Sector Read (SSRD 0x4B) 16-Byte: "); 
    printf("\r\nSpecia Sector Read Address: "); 
    PrintData(extMemAddress, 0x03);         
    printf("\r\nRead Special Sector: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
     
    testResult = 0x00;
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
         { 
           printf("\r\nRead Special Sector Fail: "); 
           STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */
           CyDelay(LED_ON_TIME);
           testResult = TEST_FAIL;
           break; 
         }
     }  
      if(!testResult)
         {
          printf("\r\nRead Special Sector Pass "); 
          STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
          CyDelay(LED_ON_TIME);
        }
        
   
/******************************************************/ 
/***********Write and Read 8-Byte Serial Number********/
/******************************************************/
    
   STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
   CyDelay(LED_ON_TIME); 

/* Set the write buffer and clear the read buffer */          
 
    for(loopcount = 0; loopcount < SERIAL_NUMBER_WRITE_SIZE; loopcount++)
	 {
        read_fram_buffer[loopcount] = 0;
        write_fram_buffer[loopcount] = loopcount+0xC0;	
     } 
/* Write 8 bytes write_fram_buffer into serial number registers */         
  
    printf("\r\n\r\nWriteSerial Number (WRSN 0xC2)  8-Byte : ");   
    WriteCmdWRSN(SMIF0, &smifContext,&write_fram_buffer[0], SERIAL_NUMBER_WRITE_SIZE);
    printf("\r\nSerial Number: ");    
    PrintData(&write_fram_buffer[0], SERIAL_NUMBER_WRITE_SIZE);  

/* Read Device Serial No - 8-Byte */    
    printf("\r\n\r\nRead Serial Number (RDSN 0xC3) 8-Byte : "); 
    printf("\r\nSerial Number: ");
    WriteCmdRDSN(SMIF0, &smifContext,&read_fram_buffer[0],SERIAL_NUMBER_WRITE_SIZE);
    PrintData(&read_fram_buffer[0], SERIAL_NUMBER_WRITE_SIZE);

    for(loopcount=0;loopcount<SERIAL_NUMBER_WRITE_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
         { 
           printf("\r\nRead Serial Number Fail: "); 
           STATUS_LED_CNTRL_Write (STATUS_LED_RED); /*Turns RED LED ON*/
           CyDelay(LED_ON_TIME);   
          testResult = TEST_FAIL;
           break; 
         }
     }  
      if(!testResult)
         {
          printf("\r\nRead Serial Number Pass "); 
          STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /*Turns GREEN LED ON*/ 
          CyDelay(LED_ON_TIME);
        }
        
    
    printf("\r\nEnd of Example Project: ");
        
    for(;;)
      {  
        /*Loops forever*/
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
