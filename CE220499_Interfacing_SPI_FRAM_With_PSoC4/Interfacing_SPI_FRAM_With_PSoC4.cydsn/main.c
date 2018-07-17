/********************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description: This is the source code for the Serial NVRAM (SPI interface) code example
*
* Related Document: CE220499.pdf
*
* Hardware Dependency PSoCs: CY8CKIT-042, CY8CKIT-042 BLE
*
* Hardware Dependency SPI NVRAM: CY15FRAMKIT-001
*
********************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
********************************************************************************
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
*********************************************************************************/

#include "project.h"
#include <stdio.h>

/************************** Macro Definitions ***********************************/

/*Size of data buffer*/
#define BUFFER_SIZE            (16u)

/*Size of uart print buffer*/
#define STRING_SIZE            (40u)

/*Example F-RAM Initial Address*/
#define EXAMPLE_INITIAL_ADDR   (00UL)

/*Example F-RAM Any Address*/
#define EXAMPLE_ANY_ADDR   (0x123456)

/*Example F-RAM Data*/
#define EXAMPLE_DATA_BYTE      (0xa5)

/*Status LED control*/ 
#define STATUS_LED_OFF         (0x03)
#define STATUS_LED_RED         (0x01)
#define STATUS_LED_GREEN       (0x02)

/*********************************************************************************
* Function Name: main
**********************************************************************************
* Summary: This is the system entrance point for Cortex-M4.
* This function executes the following

*  The main function calls F-RAM Read/Write APIs
*  1. Write to Status Register
*  2. Read from Status Register
*  3. Write 1-Byte to memory at an address
*  4. Read 1-Byte to memory from an address
*  5. Write 16 bytes of data into memory starting at an address 
*  6. Read 16 bytes of data from memory at an address

* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
int main(void)
{
    uint32 loopcount;                         /*Loop count for For loops*/
    uint8 write_example_byte;                 /*Register for one byte write*/
    uint8 read_example_byte;                  /*Register for one byte read*/   
    uint8 write_fram_buffer[BUFFER_SIZE];     /*Buffer for storing data array to be written in burst mode*/
    uint8 read_fram_buffer[BUFFER_SIZE];      /*Buffer to read data array in burst mode*/
    uint8  dev_id_buffer[SerialNVRAM_SPI_9_BYTE_DEVICE_ID]; /*Buffer for storing Device ID after read*/
    char  buf[STRING_SIZE];
      
    CyGlobalIntEnable;      /* Enable global interrupts */
    UART_Start();           /*Invoke UART_Init();*/
    SerialNVRAM_SPI_Start();/*Invoke SerialNVRAM_Init();*/
 
  /*Status LED starts with GREEN*/     
    STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /*Turns GREEN LED ON*/ 

    UART_UartPutString("\r\n********SPI NVRAM - Code Example (CE220499)*********"); 
 
  /*Initialization of variables*/    
    write_example_byte = 0x00;
    read_example_byte = 0x00;
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	{
	   read_fram_buffer[loopcount] = 0;
	   write_fram_buffer[loopcount] = loopcount;
	}
      
  /******************************************/ 
  /***********Read Device ID Register********/
  /******************************************/  
  /*Read Device ID - 9-Byte*/      
    SerialNVRAM_SPI_DevIdRead(SerialNVRAM_SPI_CS0, &dev_id_buffer[0],SerialNVRAM_SPI_9_BYTE_DEVICE_ID);
    UART_UartPutString("\r\n>>Read Device ID =  ");
        
    for(loopcount=0;loopcount<SerialNVRAM_SPI_9_BYTE_DEVICE_ID;loopcount++)
     {
        sprintf(buf, "0x%02X ", dev_id_buffer[loopcount]);
        UART_UartPutString(buf); /*Displays data*/        
     }
    
  /******************************************/ 
  /***********Write Status Register**********/
  /******************************************/ 
       
  /*Write Status Register - 1-Byte     ||Bit7||Bit6||Bit5||Bit4||Bit3) ||Bit2  ||Bit1  ||Bit0||*/
    write_example_byte = 0x4C;      /* ||WPEN||X(0)||X(0)||X(1)||BP1(0)||BP0(0)||WEL(0)||X(0)||*/
    SerialNVRAM_SPI_StatusRegWrite(SerialNVRAM_SPI_CS0, write_example_byte);    
   
  /*Send the Status Register write data to UART for display*/
    UART_UartPutString("\r\n>>Write Status Register with data =  ");   
    sprintf(buf, "0x%02X",write_example_byte);   
    UART_UartPutString (buf);     
 
    
  /******************************************/ 
  /***********Read Status Register***********/
  /******************************************/ 
       
  /*Read Status Register - 1-Byte*/      
    SerialNVRAM_SPI_StatusRegRead(SerialNVRAM_SPI_CS0, &read_example_byte);    
   
  /*Send the start address read data to UART for display*/
    UART_UartPutString("\r\n>>Read Status Register =  ");   
    sprintf(buf, "0x%02X",read_example_byte);   
    UART_UartPutString (buf);  
  
  /*Clear the status register memory block protect setting to 
   *enable memory write in follow on sections***************/    
    write_example_byte = 0x00;  /*||WPEN||X(0)||X(0)||X(1)||BP1(0)||BP0(0)||WEL(0)||X(0)||*/
    SerialNVRAM_SPI_StatusRegWrite(SerialNVRAM_SPI_CS0, write_example_byte); 
   
  /*Read Status Register - 1-Byte*/  
    SerialNVRAM_SPI_StatusRegRead(SerialNVRAM_SPI_CS0, &read_example_byte);   
    
  /*Send the start address read data to UART for display*/
    UART_UartPutString("\r\n>>Read Status Register after block protect bits are cleared =  ");   
    sprintf(buf, "0x%02X",read_example_byte);   
    UART_UartPutString (buf);  
   
    
  /******************************************/ 
  /***1 Byte Memory Write and Read Example***/
  /******************************************/ 
    
  /*Write 1 byte EXAMPLE_DATA_BYTE at EXAMPLE_ANY_ADDRESS*/    
    write_example_byte = EXAMPLE_DATA_BYTE;
    SerialNVRAM_SPI_MemoryWrite(SerialNVRAM_SPI_CS0, EXAMPLE_ANY_ADDR, &write_example_byte, 1);  
   
  /*Send the start address and write data to UART for display*/
    UART_UartPutString("\r\n>>Write 1-byte:  ");   
    sprintf(buf, "Address = 0x%08X    Data = 0x%02X", (uint8) EXAMPLE_ANY_ADDR, write_example_byte);   
    UART_UartPutString (buf);    
    
  /*Read 1 byte from EXAMPLE_ANY_ADDRESS*/      
    SerialNVRAM_SPI_MemoryRead(SerialNVRAM_SPI_CS0, EXAMPLE_ANY_ADDR, &read_example_byte, 1);   
    
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
    SerialNVRAM_SPI_MemoryWrite(SerialNVRAM_SPI_CS0, EXAMPLE_INITIAL_ADDR, &write_fram_buffer[0],BUFFER_SIZE);
    
  /*Send the start address and write data bytes to UART for display*/    
    UART_UartPutString("\r\n>>Burst Write Data at address: 0x");   
    sprintf(buf, "%08X \r\n", (uint8) EXAMPLE_INITIAL_ADDR);   
    UART_UartPutString (buf);  /*Displays address*/
   
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
        sprintf(buf, "0x%02X ", write_fram_buffer[loopcount]);
        UART_UartPutString(buf); /*Displays data*/        
     }

  /*Read 16 bytes in read_fram_buffer from memAddress*/    
    SerialNVRAM_SPI_MemoryRead(SerialNVRAM_SPI_CS0, EXAMPLE_INITIAL_ADDR, &read_fram_buffer[0],BUFFER_SIZE);
    
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
