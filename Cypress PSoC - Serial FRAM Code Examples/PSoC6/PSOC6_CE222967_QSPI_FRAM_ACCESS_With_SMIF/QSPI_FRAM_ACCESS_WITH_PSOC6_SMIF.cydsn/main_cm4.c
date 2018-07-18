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
#include "SMIF_FRAM.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/

/*Size of data buffer*/
#define BUFFER_SIZE            (256u)

/*Status LED control*/ 
#define STATUS_LED_OFF         (0x03)
#define STATUS_LED_RED         (0x01)
#define STATUS_LED_GREEN       (0x02)
#define LED_ON_TIME            (0x3FF)   /* 1-sec delay */
#define TEST_PASS              (0x00)
#define TEST_FAIL              (0x01)
#define MODE_BYTE              (0x00)   /*non XIP*/

/*******************************************************************************
* Global variables and functions
*******************************************************************************/
uint32_t EXAMPLE_INITIAL_ADDR = 0x00;    /* Example F-RAM Initial Address */
uint32_t EXAMPLE_INITIAL_ADDR_SS = 0x00; /* Example F-RAM Initial Address for Special Sector */
uint32_t EXAMPLE_ANY_ADDR =0x123456;     /* Example F-RAM Any Address */
uint32_t CONFIG_REG2_ADDR =0x000003;     /* Configuration Register 2 (CR2) address */
uint32_t ACCESS_MODE = SPI_MODE;         /* Sets the device access mode */
 uint8_t EXAMPLE_DATA_BYTE = 0xCA;       /* Example F-RAM Data */ 
 uint8_t MLC=0x08;                       /*Sets max latency for memory read at 100 MHz*/                   
 uint8_t RLC=0x03;                       /*Sets max latency for register read at 100 MHz*/
                                         /*Refer to QSPI F-RAM (CY15x104QSN)datasheet for details*/  
 
 uint8_t regwBuffer[1]; 
 uint8_t DID_Reg[8]={0x50, 0x51, 0x82, 0x06, 0x00, 0x00, 0x00, 0x00}; /* 8-byte Device ID */
 uint8_t extMemAddress[ADDRESS_SIZE] = {0x00, 0x00, 0x00};            /* Memory address for write and read access */
 uint8_t extMemAddressWithMode[ADDRESS_PLUS_MODE_SIZE] = {0x00, 0x00, 0x00, 0x00}; /* Memory address for read access */ 
                                                                                   /* with mode bye */
cy_stc_smif_context_t smifContext;
cy_en_smif_slave_select_t ONBOARD_SSFRAM = CY_SMIF_SLAVE_SELECT_2;

void ExtMemInterrupt(void);
void PrintData(uint8_t *tst_rxBuffer,uint32_t size); /* Send the buffer data to the console */
void PowerUpMemoryDefaultSPI (void);

/*********************************************************************************************
*This code example tests the following features of QSPI F-RAM using PSoC6 SMIF
*********************************************************************************************
*
*Set the device access mode to default SPI mode. This ensure part starts with a known SPI mode
*
*Read two Status and four Configuration registers (SR1, SR2, CR1, CR2, CR4, CR5)
*
*Read 8-byte device ID read
*
*Write 256 bytes into F-RAM at a given address, in SPI mode 
*
*Read 256 bytes from F-RAM at a given address, using READ (0x02) opcode in SPI mode 
*
*Read 256 bytes from F-RAM at a given address, using READ (0x02) opcode in DPI mode 
*
*Read 256 bytes from F-RAM at a given address, using READ (0x02) opcode in QPI mode
*
*Read 256 bytes from F-RAM at a given address, using FastRead (0x0B) opcode in QPI mode
*
*Write and read 256 bytes special sector using SSWR and SSRD commands in QPI mode
**********************************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************/

int main()
{
   
    uint32 loopcount;                         /* Loop count for For loops */
    uint8_t write_fram_buffer[BUFFER_SIZE];   /* Data buffer for storing data array to be written in burst mode */
    uint8_t read_fram_buffer[BUFFER_SIZE];    /* Data buffer to read data array in burst mode */
    uint8_t testResult;                       /* Variable to hold the test result status after read and verify */ 
  
    /* Enable global interrupts */
    __enable_irq();
    
    /* Start UART operation */
    UART_Start();    
    
    /* Unlock watchdog timer configuration register */
    Cy_WDT_Unlock();    
    
    /*Disable watchdog*/
    Cy_WDT_Disable();    
    
    /* The SMIF interrupt setup */
    cy_stc_sysint_t smifIntConfig =
    {
        .intrSrc = (IRQn_Type)smif_interrupt_IRQn,     /* SMIF interrupt */
        .intrPriority = SMIF_PRIORITY       /* SMIF interrupt priority */
    };
    
    /* Initializes the referenced interrupt. */
    Cy_SysInt_Init(&smifIntConfig, ExtMemInterrupt);

	/* SMIF configuration parameters */
    cy_stc_smif_config_t SPI_FRAMConfig =
    {
        .mode           = CY_SMIF_NORMAL,   /* The mode of operation; non XIP */
        .deselectDelay  = DESELECT_DELAY,   /* The minimum duration of SPI deselection */
        .rxClockSel     = RX_CLOCK_SELECT,  /* The clock source for the receiver clock */
        .blockEvent     = AHB_BUS_ERROR,    /* What happens when there is a Read to an empty RX FIFO or Write to a full TX FIFO*/
    };
	
  
    Cy_SMIF_Init(SMIF0, &SPI_FRAMConfig, TIMEOUT_1_MS, &smifContext);  /* SMIF initialization */ 
    
    Cy_SMIF_SetDataSelect(SMIF0, ONBOARD_SSFRAM, CY_SMIF_DATA_SEL0);/* SMIF data select for FRAM slave */
    
    Cy_SMIF_Enable(SMIF0, &smifContext);  /* Enables the operation of the SMIF block. */
    
    SMIF_EnableInt(); /* Enable the SMIF interrupt */
    
    printf("SMIF initialization finished\r\n");    
       
    /* Status LED; starts with GREEN */     
    STATUS_LED_CNTRL_Write(STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
    CyDelay(LED_ON_TIME); /* STATUS LED ON TIME */
    
    printf("\r\n********QSPI F-RAM Access with PSoC 6 SMIF - Code Example (CE222967)*********"); 
    
    /* Sets the device access mode to default SPI */     
    PowerUpMemoryDefaultSPI (); 
        
    ACCESS_MODE =SPI_MODE;   /*Set the SPI Access Mode*/ 
        
/******************************************/ 
/*******SPI/QSPI F-RAM Access Examples****/  
/******************************************/ 
    
/******************************************/ 
/*Read-Status and Congiguration Registers**/
/******************************************/  
    
    printf("\r\n Read SR1 - "); 
    WriteCmdReadSRx(SMIF0, &smifContext,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDSR1, RLC);
    PrintData(&read_fram_buffer[0], 1);
    
    printf("\r\n Read SR2 - "); 
    WriteCmdReadSRx(SMIF0, &smifContext,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDSR2, RLC);
    PrintData(&read_fram_buffer[0], 1);
    
    printf("\r\n Read CR1 - "); 
    WriteCmdReadCRx(SMIF0, &smifContext,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR1, RLC);
    PrintData(&read_fram_buffer[0], 1);
    
    printf("\r\n Read CR2 - "); 
    WriteCmdReadCRx(SMIF0, &smifContext,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR2, RLC);
    PrintData(&read_fram_buffer[0], 1);
    
    printf("\r\n Read CR4 - "); 
    WriteCmdReadCRx(SMIF0, &smifContext,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR4, RLC);
    PrintData(&read_fram_buffer[0], 1);
    
    printf("\r\n Read CR5 - "); 
    WriteCmdReadCRx(SMIF0, &smifContext,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR5, RLC);
    PrintData(&read_fram_buffer[0], 1);
    printf("\r\n============================================\r\n");    
      
/******************************************/ 
/***********Read - 8Byte ID ***************/
/******************************************/  
    
    printf("\r\n 8-byte Device ID read - "); 

    WriteCmdRDID(SMIF0, &smifContext,&read_fram_buffer[0], ACCESS_MODE, RLC);
    PrintData(&read_fram_buffer[0], DID_REG_SIZE);
    
    printf("\r\n 8-byte SN read        - "); 
    WriteCmdRDSN(SMIF0, &smifContext,&read_fram_buffer[0],DID_REG_SIZE, ACCESS_MODE,RLC);
    PrintData(&read_fram_buffer[0], SN_BUF_SIZE);
    
    printf("\r\n 8-byte UNIQUE ID read - "); 
    WriteCmdRDUID(SMIF0, &smifContext,&read_fram_buffer[0],ACCESS_MODE, RLC);
    PrintData(&read_fram_buffer[0], UID_BUF_SIZE);
    printf("\r\n============================================\r\n");          
   
/******************************************************/ 
/***********Write and Read 8-Byte Serial Number********/
/******************************************************/
    
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME); 

    /* Set the write buffer and clear the read buffer */          
 
    for(loopcount = 0; loopcount < SN_BUF_SIZE; loopcount++)
	 {
        read_fram_buffer[loopcount] = 0;
        write_fram_buffer[loopcount] = loopcount+0xC0;	
     } 
    /* Write 8 bytes write_fram_buffer in SN register */         
  
    printf("\r\n\r\nWriteSerial Number (WRSN 0xC2)  8-Byte : ");   
    WriteCmdWRSN(SMIF0, &smifContext,&write_fram_buffer[0], SN_BUF_SIZE,ACCESS_MODE);
    printf("\r\nSerial Number: ");    
    PrintData(&write_fram_buffer[0], SN_BUF_SIZE);  

    /* Read 8-Byte Serial No - */    
    printf("\r\n\r\nRead Serial Number (RDSN 0xC3) 8-Byte : "); 
    printf("\r\nSerial Number: ");
    WriteCmdRDSN(SMIF0, &smifContext,&read_fram_buffer[0],SN_BUF_SIZE, ACCESS_MODE,RLC);
    PrintData(&read_fram_buffer[0], SN_BUF_SIZE);
    
    testResult = 0x00;
    for(loopcount=0;loopcount<SN_BUF_SIZE;loopcount++)
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
    
/********************************************/ 
/***256-Byte memory Write and Read in SPI***/
/********************************************/   
  
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME);
  
    /* Fill the write buffer and clear the read buffer */
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	 {
	   write_fram_buffer[loopcount] = loopcount;	   
       read_fram_buffer[loopcount] = 0x00;
	 }
    
    /* Set the memAddress with start address for write */         
        
    extMemAddress [2]=EXAMPLE_INITIAL_ADDR; 
    extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
    extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;
    
    /* Write 256 bytes from write_fram_buffer at extmemAddress */
    
    WriteCmdSPIWrite(SMIF0, &smifContext,&write_fram_buffer[0],BUFFER_SIZE, extMemAddress,ACCESS_MODE);  
    printf("\r\n\r\nWrite memery (WRITE 0x02) 256-Byte in SPI: "); 
    
    /* Dispaly the start address on UART */
    
    printf("\r\nMemory Write Address: "); 
    PrintData(extMemAddress, 0x03);
    printf("\r\nMemory Write Data: "); 
    PrintData(&write_fram_buffer[0], BUFFER_SIZE);   

    /* Read 256 bytes in read_fram_buffer from extmemAddress */
    
    WriteCmdSPIRead(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress, ACCESS_MODE, MLC);
    
    /* Send the start address and read data bytes to UART for display */
    
    printf("\r\n\r\nMemory Read (READ 0x03) in SPI 256-Byte: "); 
    printf("\r\nMemory Read Address: "); 
    PrintData(extMemAddress, 0x03);         
    printf("\r\nRead Memory in SPI: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
    
    testResult = 0x00;
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
         { 
           printf("\r\nRead Memory in SPI Fail: "); 
           STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */
           CyDelay(LED_ON_TIME);
           testResult = TEST_FAIL;
           break; 
         }
       } 
    
     if(!testResult)
        {
          printf("\r\nRead Memory in SPI Pass "); 
          STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
          CyDelay(LED_ON_TIME);          
        }       
  
/********************************************/ 
/***256-Byte memory Read in DPI***/
/********************************************/         
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next test */ 
    CyDelay(LED_ON_TIME);
    
    extMemAddress [2]=CONFIG_REG2_ADDR;
    extMemAddress [1]=CONFIG_REG2_ADDR>>8;
    extMemAddress [0]=CONFIG_REG2_ADDR>>16;        
    regwBuffer[0] = 0x10;   
    
    /* Write 0x10 to CR2 register to set the access mode mode to DPI (CR2[4] = 1)*/ 
    WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, &regwBuffer[0],0x01, extMemAddress, ACCESS_MODE);
    
    /* Set the access mode for SMIF transfer*/
    ACCESS_MODE = DPI_MODE;
    
    extMemAddress [2]=EXAMPLE_INITIAL_ADDR; 
    extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
    extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;
    
    testResult = 0x00;
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	 {
       read_fram_buffer[loopcount] = 0x00;
	 }
        
    WriteCmdSPIRead(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress, ACCESS_MODE, MLC);
    
/* Send the start address and read data bytes to UART for display */
    
    printf("\r\n\r\n Memory Read (READ 0x03) in DPI 256-Byte: "); 
    printf("\r\n Memory Read Address: "); 
    PrintData(extMemAddress, 0x03);         
    printf("\r\nRead Memory in DPI: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
  
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
        { 
           printf("\r\nRead Memory in DPI Fail: "); 
           STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */
           CyDelay(LED_ON_TIME);
           testResult = TEST_FAIL;
           break; 
        }
       } 
      
     if(!testResult)
        
        {
          printf("\r\nRead Memory in DPI Pass "); 
          STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
          CyDelay(LED_ON_TIME);
        }      
  
/******************************************/ 
/*******256-Byte memory Read in QPI********/
/******************************************/ 
        
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME);
    
    extMemAddress [2]=CONFIG_REG2_ADDR;
    extMemAddress [1]=CONFIG_REG2_ADDR>>8;
    extMemAddress [0]=CONFIG_REG2_ADDR>>16;     
    regwBuffer[0] = 0x40;    
    
    /* Write 0x40 to CR2 register to set the access mode mode to QPI (CR2[6] = 1)*/ 
    WriteCmdSPIWriteAnyReg(SMIF0, &smifContext, &regwBuffer[0],0x01, extMemAddress, ACCESS_MODE);
    
    /* Set the access mode for SMIF transfer*/
    ACCESS_MODE = QPI_MODE;
    
    extMemAddress [2]=EXAMPLE_INITIAL_ADDR; 
    extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
    extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;    
  
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	 {
       read_fram_buffer[loopcount] = 0x00;
	 }
    
    WriteCmdSPIRead(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress, ACCESS_MODE, MLC);
    
    /* Send the start address and read data bytes to UART for display */
    
    printf("\r\n\r\n Memory Read (READ 0x03) in QPI 256-Byte: "); 
    printf("\r\n Memory Read Address: "); 
    PrintData(extMemAddress, 0x03);         
    printf("\r\nRead Memory in QPI: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
   
    testResult = 0x00;
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
        {
           printf("\r\nRead Memory in QPI Fail: "); 
           STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */
           CyDelay(LED_ON_TIME);
           testResult = TEST_FAIL;
           break; 
        }
       } 
      
      if(!testResult)
        {
          printf("\r\nRead Memory in QPI Pass "); 
          STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
          CyDelay(LED_ON_TIME);          
        }       

/********************************************/ 
/*****256-Byte memory FastRead in QPI********/
/********************************************/ 
    
    extMemAddressWithMode [3]=MODE_BYTE;     
    extMemAddressWithMode [2]=EXAMPLE_INITIAL_ADDR; 
    extMemAddressWithMode [1]=EXAMPLE_INITIAL_ADDR>>8;
    extMemAddressWithMode [0]=EXAMPLE_INITIAL_ADDR>>16;    
  
   for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	 {
       read_fram_buffer[loopcount] = 0x00;
	 }
    
    WriteCmdSPIFastRead(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddressWithMode, ACCESS_MODE, MLC);
    
    /* Send the start address and read data bytes to UART for display */
    
    printf("\r\n\r\n Memory FastRead (FAST_READ 0x0B) 256-Byte in QPI : "); 
    printf("\r\n Memory FastRead Address: "); 
    PrintData(extMemAddress, 0x03);         
    printf("\r\nFastRead Memory in QPI: "); 
    PrintData(&read_fram_buffer[0], BUFFER_SIZE);  
   
    testResult = 0x00;
    for(loopcount=0;loopcount<BUFFER_SIZE;loopcount++)
     {
      if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
         { 
           printf("\r\nFastRead Memory Fail in QPI: "); 
           STATUS_LED_CNTRL_Write (STATUS_LED_RED); /* Turns RED LED ON */
           CyDelay(LED_ON_TIME);
           testResult = TEST_FAIL;
           break; 
         }
     } 
      if(!testResult)
        {
          printf("\r\nFastRead Memory Pass in QPI "); 
          STATUS_LED_CNTRL_Write (STATUS_LED_GREEN); /* Turns GREEN LED ON */ 
          CyDelay(LED_ON_TIME);          
        }   
    
/********************************************************/ 
/***256-Byte Special Sector Write and Read in QPI********/
/********************************************************/   
  
    STATUS_LED_CNTRL_Write (STATUS_LED_OFF); /* Turn off the Status LED before the next text */ 
    CyDelay(LED_ON_TIME);
  
    /* Set the write buffer and clear the read buffer */
    
    for(loopcount = 0; loopcount < BUFFER_SIZE; loopcount++)
	 {
	   write_fram_buffer[loopcount] = loopcount+0x0A;	   
       read_fram_buffer[loopcount] = 0x00;
	 }
    
    /* Write 256 bytes write_fram_buffer at memAddress */         
        
    extMemAddress [2]=EXAMPLE_INITIAL_ADDR_SS; 
    extMemAddress [1]=EXAMPLE_INITIAL_ADDR_SS>>8;
    extMemAddress [0]=EXAMPLE_INITIAL_ADDR_SS>>16;
    WriteCmdSSWR(SMIF0, &smifContext,&write_fram_buffer[0],BUFFER_SIZE, extMemAddress,ACCESS_MODE);  
    printf("\r\n\r\nWrite Special Sector (SSWR 0x42) 256-Byte in QPI: "); 
    
    /* Send the start address and write data bytes to UART for display */
    
    printf("\r\nSpecial Sector Write Address: "); 
    PrintData(extMemAddress, 0x03);
    printf("\r\nSpecial Sector Write Data: "); 
    PrintData(&write_fram_buffer[0], BUFFER_SIZE);   

    /* Read 256 bytes in read_fram_buffer from memAddress */
    
    WriteCmdSSRD(SMIF0, &smifContext, &read_fram_buffer[0], BUFFER_SIZE, extMemAddress, ACCESS_MODE, MLC);
    
    /* Send the start address and read data bytes to UART for display */
    
    printf("\r\n\r\nSpecial Sector Read (SSRD 0x4B) 256-Byte in QPI: "); 
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
       
   printf("\r\n========================================================================= ");   

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
     uint8_t SR_CR_RegDefault[5] = {0x00, ((MLC<<4)|0x02), 0x00, 0x08, (RLC<<6)}; /*{ SR1, CR1, CR2, CR4, CR5 }*/
      
    /* Burst write configuration registers in SPI mode using WRR */   
    WriteCmdWRSR (SMIF0, &smifContext,&SR_CR_RegDefault[0], 0x05, SPI_MODE);
        
    /* Burst write configuration registers in DPI mode using WRR */
    WriteCmdWRSR (SMIF0, &smifContext,&SR_CR_RegDefault[0], 0x05, DPI_MODE);
    
    /* Burst write configuration registers in QPI mode using WRR */
    WriteCmdWRSR (SMIF0, &smifContext,&SR_CR_RegDefault[0], 0x05, QPI_MODE); 
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
