/****************************************************************************
*File Name: FRAM_ACCESS.h
*
* Version: 1.0
*
* Description: 
* This header file contains all the defines for the SPI F-RAM access 
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
#include <stdbool.h>
#include "smif/cy_smif.h"
#include "SMIF_FRAM.h"

/***************************************
*       SMIF Function Prototypes 
***************************************/

#define SMIF_EnableInt()   NVIC_EnableIRQ((IRQn_Type)smif_interrupt_IRQn)

/***************************************
*       SMIF API Constants
***************************************/

/* SMIF configuration parameters */
#define DESELECT_DELAY      (1u)	/* The minimal duration of SPI de-selection */ 
#define RX_CLOCK_SELECT     (1u)	/* The clock source for the receiver clock */
#define AHB_BUS_ERROR       (0u)	/* What happens when there is a Read 
									* to an empty RX FIFO or Write to a full TX FIFO
                                    */ 
                                    
/* SMIF Interrupt Configuration*/
#define SMIF_INT_MAP        (NvicMux10)     /* 
                                             * Mapping the SMIF M4 core interrupt to 
                                             * the 10th M0+ core interrupt
                                             */
#define SMIF_PRIORITY       (1u)            /* SMIF interrupt priority */

/***************************************
*       SPIM specific constants
***************************************/

/* Memory Read*/
#define MEM_CMD_READ        (0x03)  /* Read in the single mode */
#define MEM_CMD_FAST_READ   (0x0B)  /* Read in the single mode */
#define MEM_CMD_SSRD        (0x4B) 	/* Special sector -256 byte read */

/* Memory Write*/    
#define MEM_CMD_WRITE       (0x02) 	/* The memory write in the single mode */
#define MEM_CMD_WREN        (0x06) 	/* Write Enable */
#define MEM_CMD_SSWR        (0x42) 	/* Special sector -256 byte write */

/* Status*/
#define MEM_CMD_RDSR        (0x05) 	/* Read the status register */
#define MEM_CMD_WRSR        (0x01) 	/* Write the status register */
#define MEM_CMD_WRDI        (0x04) 	/* Write disable */

/* Device ID Write*/
#define MEM_CMD_RDID        (0x9F) 	/* Read device ID */
#define MEM_CMD_RDSN        (0xC3) 	/* Read serial number */
#define MEM_CMD_RUID        (0x4C) 	/* Read unique device ID  */
#define MEM_CMD_WRSN        (0xC2) 	/* Write tserial number */   
    
/* Power Mode Commands*/
#define MEM_CMD_ENTDPD      (0xBA) 	/* Enter DPD - Ramtron mode*/
#define MEM_CMD_ENTHBN      (0xB9) 	/* Enter Hibernate - Ramtron Mode */

/* Constants */
#define CMD_WITHOUT_PARAM   (0u)    
#define TX_LAST_BYTE       	(1u) 	/* The last byte in command transmission 
									* (SS is set to high after transmission) 
									*/
#define TX_NOT_LAST_BYTE    (0u) 	/* Not the last byte in command transmission 
									* (SS remains low after transmission) 
									*/
#define ADDRESS_SIZE      	(3u)	/* Memory address size */
#define NO_ADDR_CMD_ADDRESS_SIZE  (0u)	/* Memory address size */ 

#define WIP_BIT             (1u)    /* The memory Write in the Progress bit */ 
#define PACKET_SIZE         (256u)  /* The emory Read/Write packet */  
    
#define TX_RX_EQUAL       	(1u) 	/* The transmitted and received arrays are equal */
#define TX_RX_NOT_EQUAL     (0u) 	/* The transmitted and received arrays are not equal */
#define TIMEOUT_1_MS        (1000ul)/* 1 ms timeout for all blocking functions */

void WriteCmdWRSR(SMIF_Type *baseaddr,                      /* Change the Status Register */
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 	
                    uint32_t cmdSize);		    		

void WriteCmdRDSR(SMIF_Type *baseaddr,	                    /* Read the status register */
                    cy_stc_smif_context_t *smifContextuint8_t,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize);	

void WriteCmdSPIWrite(SMIF_Type *baseaddr, 	                /* Write to F-RAM memory */
                    cy_stc_smif_context_t *smifContext, 	
                    uint8_t tst_txBuffer[], 	
                    uint32_t txSize, 	
                    uint8_t *address);	    			

void WriteCmdSPIRead(SMIF_Type *baseaddr,	                /* Read data from F-RAM */
                    cy_stc_smif_context_t *smifContext, 	
                    uint8_t tst_rxBuffer[], 	
                    uint32_t rxSize, 	
                    uint8_t *address);   	

void WriteCmdSPIFastRead(SMIF_Type *baseaddr,	            /*Fast Read data from F-RAM with Fast Read opcode*/
                    cy_stc_smif_context_t *smifContext, 	
                    uint8_t tst_rxBuffer[], 	
                    uint32_t rxSize, 	
                    uint8_t *address); 

void WriteCmdWREN(SMIF_Type *baseaddr,	                    /* Memory Write Enable */
                    cy_stc_smif_context_t *smifContext);	

void WriteCmdWRDI(SMIF_Type *baseaddr,	                    /* Memory Write Disable */
                    cy_stc_smif_context_t *smifContext);				   

bool TxRxEqualCheck(uint8_t tst_txBuffer[], 	            /* Check if the transmitted and received arrays are equal */
					uint8_t tst_rxBuffer[]);	     	    

void WriteCmdSSWR(SMIF_Type *baseaddr,                      /* Wtite 256-byte Special Sector */ 
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address);

void WriteCmdSSRD(SMIF_Type *baseaddr,                      /* Read 256-byte Special Sector */
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address);

void WriteCmdWRSN(SMIF_Type *baseaddr,                      /* Wrire 8-byte Serial Number */   
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 	
                    uint32_t cmdSize);

void WriteCmdRDSN(SMIF_Type *baseaddr,                      /* Read 8-byte Serial Number */  
                  cy_stc_smif_context_t *smifContext,   
                  uint8_t tst_rxBuffer[], 
                   uint32_t rxSize);    

void WriteCmdRDID(SMIF_Type *baseaddr,                      /* Read Device ID */
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize);

void WriteCmdRDUID(SMIF_Type *baseaddr,                     /* Read Unique Device ID */
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize);

void WriteCmdENTHBN(SMIF_Type *baseaddr,	                /* Enter DPD HIBERNATE */
                    cy_stc_smif_context_t *smifContext);	

void WriteCmdENTDPD(SMIF_Type *baseaddr,	                /* Enter DPD Mode */
                    cy_stc_smif_context_t *smifContext);			 	

    
/* [] END OF FILE */
