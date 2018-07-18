/****************************************************************************
*File Name: FRAM_ACCESS.c
*
* Version: 1.0
*
* Description: 
* This file contains high-level functions for the SPI F-RAM access. 
* SMIF driver APIs are called from cy_smif.c file (file location: Genereted_Source/PSoC6/pdl/smf)  
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

#include "FRAM_ACCESS.h"
#include "stdio.h"
#include "project.h"

cy_smif_event_cb_t RxCmpltCallback;
cy_en_smif_slave_select_t ONBOARD_MEM_SSFRAM = CY_SMIF_SLAVE_SELECT_2;

/*******************************************************************************
* Function Name: WriteCmdWREN
****************************************************************************//**
*
* This function enables the Write bit in the status register. 
* The function sends the WREN 0x06 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdWREN(SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t spimode)
{    
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;     
    
    /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WREN,				  
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_LAST_BYTE, 
                            smifContext);     
        
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdWRDI
****************************************************************************//**
*
* This function disables the Write enable bit in the status register. 
* The function sends the WREN 0x04 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdWRDI(SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t spimode)
{    
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE; 
    
     /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRDI,				  
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_LAST_BYTE, 
                            smifContext); 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}
/*******************************************************************************
* Function Name: WriteCmdWRSR
****************************************************************************//**
*
* This function writes data to the external memory status register. 
* The function sends the WRSR 0x01 command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data. 
*
* \param cmdParam
* Data for the status register
* 
* \param cmdSize
* The size of data for the status register. Can be 1 to 5.
* 
* \param spimode
* Determines SMIF width single,dual,or quads
*
*******************************************************************************/
void WriteCmdWRSR(  SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 
                    uint32_t cmdSize,
                    uint8_t spimode)
{
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
    else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;   
    
    /* Write Enable */
    WriteCmdWREN(baseaddr, smifContext,spimode);
    
     /* Transmit command and data byte(s) */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSR,				  
                            CY_SMIF_WIDTH,
                            cmdParam, 
                            cmdSize, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_LAST_BYTE, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
    
}

/*******************************************************************************
* Function Name: WriteCmdRDSR
****************************************************************************//**
*
* This function reads data from the external memory status register and prints it
* to the UART console. The function sends the RDSR 0x05 command to the external
* memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
* 
*******************************************************************************/
void WriteCmdReadSRx(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize,
                    uint8_t spimode,
                    uint8_t cmdtype,
                    uint8_t latency)
{
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    uint8_t CMD_TYPE =0x00;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;  
    
    if (cmdtype==MEM_CMD_RDSR1)
    CMD_TYPE = MEM_CMD_RDSR1;
   
    else if (cmdtype==MEM_CMD_RDSR2)
    CMD_TYPE = MEM_CMD_RDSR2;
    
     /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            CMD_TYPE,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
      
    /* Receive data */
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIWriteAnyReg
****************************************************************************//**
*
* This function writes one byte (no burst write) to the status or configuration register. 
* The function sends WRAR, 0x71 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdSPIWriteAnyReg(SMIF_Type *baseaddr, 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)

{         
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext, spimode);
    
    /* Transmit command */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRAR,		
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    /* Transmit data */
    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIReadAnyReg
****************************************************************************//**
*
* This function reads one byte (no burst read) from the status or configuration register. 
* The function sends RDAR, 0x65 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIReadAnyReg(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDAR,				
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);

    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive command */
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdReadCRx
****************************************************************************//**
*
* This function reads data from the external memory configuration register and prints it
* to the UART console. The function sends the CRx command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param cmdtype
* Determines cmd type to tranmit in the command cycle
*
* \param latency
* Memory latency cycle during read.
* 
*******************************************************************************/
void WriteCmdReadCRx(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize,
                    uint8_t spimode,
                    uint8_t crtype,
                    uint8_t latency)
{
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    uint8_t CMD_TYPE = 1U;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;  
    
    if (crtype==MEM_CMD_RDCR1)
        CMD_TYPE = MEM_CMD_RDCR1;
   
     else if (crtype==MEM_CMD_RDCR2)
              CMD_TYPE = MEM_CMD_RDCR2;
  
     else if (crtype==MEM_CMD_RDCR4)
              CMD_TYPE = MEM_CMD_RDCR4;
  
    else if (crtype==MEM_CMD_RDCR5)
             CMD_TYPE = MEM_CMD_RDCR5;

   else
    CMD_TYPE = MEM_CMD_RDCR1; 
    
    /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            CMD_TYPE,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }   
}

/*******************************************************************************
* Function Name: WriteCmdSPIWrite
****************************************************************************//**
*
* This function writes data to the external memory in the single mode. 
* The function sends the Write, 0x02 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdSPIWrite(SMIF_Type *baseaddr, 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)
{       
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext, spimode);
	
    /* Transmit command and address */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRITE,		
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
     /* Transmit data */
    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIRead
****************************************************************************//**
*
* This function reads data from the external memory.  
* The function sends the READ 0x03 command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIRead(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command and address */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_READ,				
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext); 
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback,
                            smifContext);

    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIFastWrite
****************************************************************************//**
*
* This function writes data to the external memory. 
* The function sends the Fast Write 0xDA command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to. It combines the mode byte, following address byte  
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdSPIFastWrite(SMIF_Type *baseaddr, 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)
{       
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext, spimode);	
    
    /* Transmit command and address */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_FASTWRITE,		
                            CY_SMIF_WIDTH,
                            address, 
                            (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/  
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    /* Transmit data */
    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIFastRead
****************************************************************************//**
*
* This function reads data from the external memory in the single mode. 
* The function sends the READ 0x0B command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIFastRead(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command and address */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_FAST_READ,				
                            CY_SMIF_WIDTH,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext); 
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */ 
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback,
                            smifContext);
   
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSSWR
****************************************************************************//**
*
* This function writes data to the special sector (256-byte max). 
* The function sends the Page Program 0x42 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdSSWR(SMIF_Type *baseaddr, 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)
{  
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;  
    
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext,spimode);
	
    /* Transmit command and address */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSWR,		
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
     /* Transmit data */
    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdWRSN
****************************************************************************//**
*
* This function writes 8-byte serial number the external memory. 
* The function sends the Page Program 0xC2 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
*******************************************************************************/
void WriteCmdWRSN(  SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t spimode)
{
    
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE; 
    
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext, spimode);
    
    /* Transmit command */    
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSN,				  
                            CY_SMIF_WIDTH,
                            tst_txBuffer, 
                            txSize, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_LAST_BYTE, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
   
}

/*******************************************************************************
* Function Name: WriteCmdSPIWrite_DIOW_QIOW
****************************************************************************//**
*
* This function writes data to the external memory in the extended SPI mode in DIO or QIO. 
* The function sends the respective command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIWrite_DIOW_QIOW(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype)

{
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext, SPI_MODE);
    
    /* Transmit command and address in DIO or QIO */
      if( CMDtype==MEM_CMD_DIOW)
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DIOW ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_DUAL, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);         
    
      if( CMDtype==MEM_CMD_QIOW)
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIOW,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_QUAD, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);     
    
    /* Transmit data byte(s) in DIO or QIO */ 
    if( CMDtype==MEM_CMD_DIOW)    
    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback, 
                            smifContext);
       
    if( CMDtype==MEM_CMD_QIOW)
    Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer, 
                            txSize, 
                            CY_SMIF_WIDTH_QUAD, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIRead_DIOR_QIOR
****************************************************************************//**
*
* This function reads data from the external memory in extended SPI mode DIO or QIO. 
* The function sends the respective command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIRead_DIOR_QIOR(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency,
                        uint8_t CMDtype)
{       
   /* Transmit command and address in DIO or QIO */
    if (spimode==SPI_MODE)      
     { 
      if( CMDtype==MEM_CMD_DIOR)
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DIOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_DUAL, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);         
    
      if( CMDtype==MEM_CMD_QIOR)
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_QUAD, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);  
    
      }
    
    if (spimode==QPI_MODE)                                      
     { 	
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIOR,				
                            CY_SMIF_WIDTH_QUAD,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_QUAD, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
      } 
 
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
   
    /* Transmit data in DIO or QIO */
    if( CMDtype==MEM_CMD_DIOR)
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback,
                            smifContext);
    
    if( CMDtype==MEM_CMD_QIOR)    
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_QUAD, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIWrite_DIW_QIW
****************************************************************************//**
*
* This function writes data to the external memory in extended SPI mode DIW, QIW. 
* The function sends the READ 0x03 command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIWrite_DIW_QIW(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype)

{
    /* Set the write enable (WEL) bit in SR1 */
    WriteCmdWREN(baseaddr, smifContext, SPI_MODE);
    
    /* Transmit command and address in DIW or QIW */
    if( CMDtype==MEM_CMD_DIW)
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DIW ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);         
    
    if( CMDtype==MEM_CMD_QIW)
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIW,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);     
    
   /* Transmit data byte(s) in DIW or QIW */ 
   if( CMDtype==MEM_CMD_DIW)    
   Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback, 
                            smifContext);
       
   if( CMDtype==MEM_CMD_QIW)    
   Cy_SMIF_TransmitData(  baseaddr,
                            tst_txBuffer, 
                            txSize, 
                            CY_SMIF_WIDTH_QUAD, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/* Function Name: WriteCmdSPIRead_DOR_QOR
****************************************************************************//**
*
* This function reads data from the external memory in extended SPI DOR, QOR mode. 
* The function sends the respective command to the external memory. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
* \IOMode
* Determines extended SPI mode dual or quad
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSPIRead_DOR_QOR(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t CMDtype,
                        uint8_t latency)
{   
    
    /* Transmit command and data in DO */  
    if (CMDtype==MEM_CMD_DOR)      
     { 	
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
       /* Sends extra dummy clocks to adde clock cycle latency */
       Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);    
    
       Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback,
                            smifContext);
      }
    
    if (CMDtype==MEM_CMD_QOR)      
     { 	
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);    
    
    /* Transmit data in DO */
    if( CMDtype==MEM_CMD_DOR)
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback,
                            smifContext);
    
    if( CMDtype==MEM_CMD_QOR)    
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_QUAD, 
                            RxCmpltCallback,
                            smifContext);
      } 
 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdRDID
****************************************************************************//**
*
* This function reads 8-byte device ID from the external memory
* The function sends the RDID, 0x9F command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during register read.
*
*******************************************************************************/
void WriteCmdRDID(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                     uint8_t tst_rxBuffer[],
                    uint8_t spimode,
                    uint8_t latency)
{
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE; 
    
    /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDID,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        DID_REG_SIZE, 
                        CY_SMIF_WIDTH, 
                        RxCmpltCallback,
                        smifContext);       
  
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdRDUID
****************************************************************************//**
*
* This function reads 4-byte unique device ID of the external FRAM
* The function sends the RUID, 0x4C command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during register read.
*
*******************************************************************************/
void WriteCmdRDUID(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[],
                    uint8_t spimode,
                    uint8_t latency)
{
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RUID,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        UID_BUF_SIZE, 
                        CY_SMIF_WIDTH, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdRDSN
****************************************************************************//**
*
* This function reads 8-byte serial number from the external FRAM
* The function sends the RDSN, 0xC3 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during read.
*
*******************************************************************************/
void WriteCmdRDSN(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[],
                    uint32_t txSize, 
                    uint8_t spimode,
                    uint8_t latency)
{
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
     
    /* Transmit command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDSN,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        txSize, 
                        CY_SMIF_WIDTH, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}
/*******************************************************************************
* Function Name: WriteCmdSSRD
****************************************************************************//**
*
* This function reads special sector (max 256 bytes) of the external FRAM
* The function sends the 0x4B command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read special sector data from. 
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
void WriteCmdSSRD(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;   
    
    /* Transmit command */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSRD,				
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
     
    /* Receive data */
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
                            RxCmpltCallback,
                            smifContext);
 
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }        
}

/*******************************************************************************
* Function Name: WriteCmdEnterLPMode
****************************************************************************//**
*
* This function enters the device into low power DPD or hibernate mode. 
* The function sends the respective low power mode command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param cmdtype
* Determines cmd type to send in the command cycle
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
void WriteCmdEnterLPMode(SMIF_Type *baseaddr,	
                    cy_stc_smif_context_t *smifContext,
                    uint8_t cmdtype, 
                    uint8_t spimode)
{    
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    uint8_t CMD_TYPE=0x00;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;      
    
    if (cmdtype==MEM_CMD_ENTDPD)
      CMD_TYPE = MEM_CMD_ENTDPD;
   
     else if (cmdtype==MEM_CMD_ENTHBN)
           CMD_TYPE = MEM_CMD_ENTHBN;  
    else
    CMD_TYPE = MEM_CMD_RDSR1; 
    
    /* Transmit command */
    Cy_SMIF_TransmitCommand(baseaddr,
                            CMD_TYPE,				  
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_LAST_BYTE, 
                            smifContext); 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}  

/* [] END OF FILE */
