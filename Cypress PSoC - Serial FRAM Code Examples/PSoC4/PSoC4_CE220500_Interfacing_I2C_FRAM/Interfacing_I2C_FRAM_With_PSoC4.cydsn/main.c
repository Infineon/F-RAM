/****************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description: This is the source code for the Serial NVRAM (I2C interface) code example
*
* Related Document: CE220500.pdf
*
* Hardware Dependency PSoCs: CY8CKIT-042, CY8CKIT-042 BLE
*
* Hardware Dependency SPI NVRAM: CY15FRAMKIT-001
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

#include "project.h"
#include <stdio.h>

/************************** Macro Definitions *********************************/

/*Size of data buffer*/
#define BUFFER_SIZE            (16u)

/*Size of uart print buffer*/
#define STRING_SIZE            (40u)

/*Example F-RAM Initial Address*/
#define EXAMPLE_INITIAL_ADDR   (0UL)

/*Example F-RAM Any Address*/
#define EXAMPLE_ANY_ADDR       (0x123456)

/*Example F-RAM Data*/
#define EXAMPLE_DATA_BYTE      (0x55)

/*I2C Slave ID for the DUT*/
#define I2C_SLAVE_ID           (0x50) 

/*Status LED control*/ 
#define STATUS_LED_RED         (0x01)
#define STATUS_LED_GREEN       (0x02)
#define STATUS_LED_OFF         (0x03)

/******************************************************************************
* Function Name: main
*******************************************************************************
* Summary: This is the system entrance point for Cortex-M4.
* This function executes the following
*  1. Write 1-Byte to memory at an address
*  2. Read 1-Byte to memory from an address
*  3. Write 16 bytes of data into memory starting at an address 
*  4. Read 16 bytes of data from memory at an address

* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None  
********************************************************************************/

int main(void)
{
    uint32 loopcount;                         /*Loop count for For loops*/
    uint8 write_example_byte;                 /*Register for one byte write*/
    uint8 read_example_byte;                  /*Register for one byte read*/   
    uint8 write_fram_buffer[BUFFER_SIZE];     /*Buffer for storing data array to be written in burst mode*/
    uint8 read_fram_buffer[BUFFER_SIZE];      /*Buffer to read data array in burst mode*/
    char  buf[STRING_SIZE];
      
    CyGlobalIntEnable;      /* Enable global interrupts */
    UART_Start();           /*Invoke UART_Init();*/
    SerialNVRAM_I2C_Start();/*Invoke SerialNVRAM_Init();*/
 
  /*Status LED starts with GREEN*/     
    STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /*Turns GREEN LED ON*/ 

    UART_UartPutString("\r\n********I2C NVRAM - Code Example (CE220500)*********"); 
 
  /*Initialization of variables*/    
    write_example_byte = 0x00;
    read_example_byte = 0x00;
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	{
	   read_fram_buffer[loopcount] = 0;
	   write_fram_buffer[loopcount] = loopcount;
	}      
 
  /******************************************/ 
  /***1 Byte Memory Write and Read Example***/
  /******************************************/ 

  /*Write 1 byte EXAMPLE_DATA_BYTE at EXAMPLE_ANY_ADDRESS*/    
    write_example_byte = EXAMPLE_DATA_BYTE;
    SerialNVRAM_I2C_MemoryWrite(I2C_SLAVE_ID, EXAMPLE_ANY_ADDR, &write_example_byte, 1);  
   
  /*Send the start address and write data to UART for display*/
    UART_UartPutString("\r\n>>Write 1-byte:  ");   
    sprintf(buf, "Address = 0x%08X    Data = 0x%02X", (uint8) EXAMPLE_ANY_ADDR, write_example_byte);   
    UART_UartPutString (buf);    
    
  /*Read 1 byte from EXAMPLE_ANY_ADDRESS*/      
    SerialNVRAM_I2C_MemoryRead(I2C_SLAVE_ID, EXAMPLE_ANY_ADDR, &read_example_byte, 1);    
   
    if ( read_example_byte!=write_example_byte)
         {
            STATUS_LED_CNTRL_Write (STATUS_LED_RED); /*Turns RED LED ON*/    
            UART_UartPutString("\r\n>>Read 1-byte failed");  
         }
    else 
         {
            STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /*Turns GREEN LED ON*/
  /*Send the start address read data to UART for display*/
            UART_UartPutString("\r\n>>Read 1-byte:  ");   
            sprintf(buf, "Address = 0x%08X    Data = 0x%02X", (uint8) EXAMPLE_ANY_ADDR, read_example_byte);   
            UART_UartPutString (buf);    
         }
    
  /*******************************************/ 
  /***16 Byte Memory Write and Read Example***/
  /*******************************************/   
    
  /*Write 16 bytes write_fram_buffer at memAddress*/    
    SerialNVRAM_I2C_MemoryWrite(I2C_SLAVE_ID, EXAMPLE_INITIAL_ADDR, &write_fram_buffer[0],BUFFER_SIZE);
    
  /*Send the start address and write data bytes to UART for display*/    
    UART_UartPutString("\r\n>>Burst Write Data at address: 0x");   
    sprintf(buf, "%08X \r\n", (uint8) EXAMPLE_INITIAL_ADDR);   
    UART_UartPutString (buf);  /*Displays address*/
   
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
        sprintf(buf, "0x%02X ", write_fram_buffer[loopcount]);
        UART_UartPutString(buf); /*Displays data*/        
     }
  
  /*******************************************/ 
  /***16 Byte Memory Current Read Example***/
  /*******************************************/   
    
  /*Set the internal address counter at EXAMPLE_INITIAL_ADDR*/    
    SerialNVRAM_I2C_MemoryWrite(I2C_SLAVE_ID, EXAMPLE_INITIAL_ADDR, &write_fram_buffer[0],0x00);
    
  /*Current Read 16 bytes from EXAMPLE_INITIAL_ADDR*/
    SerialNVRAM_I2C_CurrentMemoryRead (I2C_SLAVE_ID,&read_fram_buffer[0],BUFFER_SIZE);     
        
  /*Send the start address and read data bytes to UART for display*/   
    UART_UartPutString("\r\n>>Burst Current Read Data from Address: 0x");   
    sprintf(buf, "%08X\r\n", (uint8) EXAMPLE_INITIAL_ADDR);   
    UART_UartPutString (buf); /*Displays address*/  

    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
        sprintf(buf, "0x%02X ", read_fram_buffer[loopcount]);
        UART_UartPutString(buf); /*Displays data*/     
     }   
    
  /*Read 16 bytes in read_fram_buffer from memAddress*/    
     SerialNVRAM_I2C_MemoryRead(I2C_SLAVE_ID, EXAMPLE_INITIAL_ADDR, &read_fram_buffer[0],BUFFER_SIZE);
            
  /*Send the start address and read data bytes to UART for display*/   
    UART_UartPutString("\r\n>>Burst Read Data from Address: 0x");   
    sprintf(buf, "%08X\r\n", (uint8) EXAMPLE_INITIAL_ADDR);   
    UART_UartPutString (buf); /*Displays address*/  

    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
        sprintf(buf, "0x%02X ", read_fram_buffer[loopcount]);
        UART_UartPutString(buf); /*Displays data*/  
        
        if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
         {
            STATUS_LED_CNTRL_Write (STATUS_LED_RED); /*Turns RED LED ON*/    
            break;
         }
        
        else
           STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /*Turns GREEN LED ON*/  
     }   

    for(;;)
     {  
        /*Loops forever*/
     }
}
