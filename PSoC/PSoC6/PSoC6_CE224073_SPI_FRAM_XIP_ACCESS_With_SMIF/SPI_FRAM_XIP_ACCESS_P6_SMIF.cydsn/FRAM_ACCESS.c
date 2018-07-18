/****************************************************************************
*File Name: FRAM_ACCESS.c
*
* Version: 1.0
*
* Description: 
* This file contains high-level functions for the SPI F-RAM access. 
* SMIF driver APIs are called from cy_smif.c file (file location: Genereted_Source/PSoC6/pdl/smif)  
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

#include "FRAM_ACCESS.h"
#include "stdio.h"
#include "project.h"

cy_smif_event_cb_t RxCmpltCallback;
cy_en_smif_slave_select_t ONBOARD_MEM_SSFRAM = CY_SMIF_SLAVE_SELECT_2;

/*******************************************************************************
* Function Name: WriteCmdWRSR (0x01)
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
* The size of data for the status register. Can be 1 or 2.
*
*******************************************************************************/
void WriteCmdWRSR(  SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 
                    uint32_t cmdSize)
{
    
    /* Write Enable */
    WriteCmdWREN(baseaddr, smifContext);
    
    /* Write Status */ 
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSR,				  
                            CY_SMIF_WIDTH_SINGLE,
                            cmdParam, 
                            cmdSize, 
                            CY_SMIF_WIDTH_SINGLE, 
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
* Function Name: WriteCmdRDSR (0x05)
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
* The buffer for read status register data.
* 
* \param rxSize 
* The size of data to read.
* 
*******************************************************************************/
void WriteCmdRDSR(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize)
{
      
    /* The Status Register Read command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDSR,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }

}

/*******************************************************************************
* Function Name: WriteCmdSPIWrite (0x02)
****************************************************************************//**
*
* This function writes data to the external memory in the single mode. 
* The function sends the WRITE 0x02 command to the external memory.
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
void WriteCmdSPIWrite(SMIF_Type *baseaddr, 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address)
{     
    
    /* Write Enable */
    WriteCmdWREN(baseaddr, smifContext);
	
    /* The memory write command */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRITE,		
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);

    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSSWR (0x42)
****************************************************************************//**
*
* This function writes data to the external memory in the single mode. 
* The function sends the SSWR 0x42 command to the external memory.
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
void WriteCmdSSWR(SMIF_Type *baseaddr, 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address)
{  
    /* Write Enable */
    WriteCmdWREN(baseaddr, smifContext);
	
    /* The memory write command */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSWR,		
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);

    Cy_SMIF_TransmitData(baseaddr, 
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}


/*******************************************************************************
* Function Name: WriteCmdWRSN (0xC2)
****************************************************************************//**
*
* This function writes data to the external memory in the single mode. 
* The function sends the WRSN 0xC2 command to the external memory.
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
                    uint8_t cmdParam[], 
                    uint32_t cmdSize)
{
    
    /* Write Enable */
    WriteCmdWREN(baseaddr, smifContext);
    
    /* Write Status */    
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSN,				  
                            CY_SMIF_WIDTH_SINGLE,
                            cmdParam, 
                            cmdSize, 
                            CY_SMIF_WIDTH_SINGLE, 
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
* Function Name: WriteCmdSPIRead (0x03)
****************************************************************************//**
*
* This function reads data from the external memory in the single mode. 
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
*******************************************************************************/
void WriteCmdSPIRead(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address)
{   
    /* Read memory data */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_READ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
 
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSPIFastRead (0x0B)
****************************************************************************//**
*
* This function reads data from the external memory in the single mode. 
* The function sends the FAST_READ 0x0B command to the external memory. 
* The fast read command requires 8-clock cycle latency before valida is driven on the output pin
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
*******************************************************************************/
void WriteCmdSPIFastRead(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address)
{   
    /* Read memory data */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_FAST_READ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);    

   Cy_SMIF_SendDummyCycles (baseaddr, 0x08);
    
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
}

/*******************************************************************************
* Function Name: WriteCmdWREN (0x06)
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
*******************************************************************************/
void WriteCmdWREN(SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext)
{    
    /* Memory Write Enable */
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WREN,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
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
* Function Name: WriteCmdWRDI (0x04)
****************************************************************************//**
*
* This function disables the Write enable bit in the status register. 
* The function sends the WRDI 0x04 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
void WriteCmdWRDI(SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext)
{    
    /* Memory Write Enable */
       Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRDI,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
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
* Function Name: WriteCmdRDID (0x9F)
****************************************************************************//**
*
* This function reads device ID from the external FRAM
* The function sends the RDID 0x9F command to the external memory.
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
*******************************************************************************/
void WriteCmdRDID(SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[], 
                  uint32_t rxSize)
{
  
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDID,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
 }

/*******************************************************************************
* Function Name: WriteCmdRDUID (0x4C)
****************************************************************************//**
*
* This function reads device ID from the external FRAM
* The function sends the RUID 0x4C command to the external memory.
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
*******************************************************************************/
void WriteCmdRDUID(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize)
{
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RUID,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);    
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdRDSN (0xC3)
****************************************************************************//**
*
* This function reads device ID from the external FRAM
* The function sends the RDSN 0xC3 command to the external memory.
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
*******************************************************************************/
void WriteCmdRDSN(SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,   
                  uint8_t tst_rxBuffer[], 
                   uint32_t rxSize)
{
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDSN,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
}

/*******************************************************************************
* Function Name: WriteCmdSSRD (0x4B)
****************************************************************************//**
*
* This function reads device ID from the external FRAM
* The function sends the SSRD 0x4B command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
void WriteCmdSSRD(SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address)
{   
    /* Read memory data */
	Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSRD,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
                            ONBOARD_MEM_SSFRAM, 
                            TX_NOT_LAST_BYTE, 
                            smifContext);    
    Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }        
}

/********************************************************
* Power Mode Commands************************************
*********************************************************/

/*****************************************************************************
* Function Name: WriteENTHBN (0xB9)
***************************************************************************//*
*
* This function disables the Write enable bit in the status register. 
* The function sends the HBN 0xB9 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
void WriteCmdENTHBN(SMIF_Type *baseaddr,	
                    cy_stc_smif_context_t *smifContext)
{    
    /* Memory Write Enable */ 
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_ENTHBN,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
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
* Function Name: WriteENTDPD (0xBA)
****************************************************************************//**
*
* This function disables the Write enable bit in the status register. 
* The function sends the DPD 0xBA command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
void WriteCmdENTDPD(SMIF_Type *baseaddr,	
                    cy_stc_smif_context_t *smifContext)
{    
    /* Memory Write Enable */  
    Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_ENTDPD,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
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
