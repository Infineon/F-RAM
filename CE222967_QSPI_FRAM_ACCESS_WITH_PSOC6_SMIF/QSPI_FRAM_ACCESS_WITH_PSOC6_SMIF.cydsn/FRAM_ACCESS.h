/****************************************************************************
*File Name: FRAM_ACCESS.h
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
#define DESELECT_DELAY      (7u)	        /* The minimal duration of SPI de-selection */ 
#define RX_CLOCK_SELECT     (1u)	        /* The clock source for the receiver clock */
#define AHB_BUS_ERROR       (0u)	        /* What happens when there is a Read 
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
#define CMD_WITHOUT_PARAM         (0u)      /* Opcode only commands */
#define TX_LAST_BYTE       	      (1u) 	    /* The last byte in command transmission 
									         * (SS is set to high after transmission)*/
#define TX_NOT_LAST_BYTE          (0u)   	/* Not the last byte in command transmission 
									         * (SS remains low after transmission) */
#define ADDRESS_SIZE      	      (3u)	    /* F-RAM memory address size */
#define ADDRESS_PLUS_MODE_SIZE    (4u)	    /* F-RAM memory address size with mode byte combined*/    
#define TIMEOUT_1_MS              (1000ul)  /* 1 ms timeout for all blocking functions */

/* SPIM Access Modes */
#define SPI_MODE                  (0u)      /* Transmits in SPI SDR mode */
#define DPI_MODE                  (1u)      /* Transmits in DPI SDR mode */ 
#define QPI_MODE                  (2u)      /* Transmits in QPI SDR mode */ 

/*Register and buffer Length*/ 
#define PACKET_SIZE               (256u)     /* The emory Read/Write packet */ 
#define SR_SIZE      	          (1u)	     /* Status register1 size */
#define DID_REG_SIZE      	      (8u)	     /* Device ID status register size */
#define SN_BUF_SIZE      	      (8u)	     /* Serial Number register size */
#define UID_BUF_SIZE      	      (8u)	     /* Unique Serial Number register size */

/***************************************
*     QSPI F-RAM specific Opcodes
***************************************/
/* QSPI F-RAM Read Opcodes*/
#define MEM_CMD_READ              (0x03)    /* Memory READ opcode */
#define MEM_CMD_FAST_READ         (0x0B)    /* Memory FastRead opcode */
#define MEM_CMD_DIOR              (0xBB)    /* Memory read opcode - Dual IO Read (DIOR) */
#define MEM_CMD_QIOR              (0xEB)    /* Memory read opcode - Quad IO Read (QIOR) */
#define MEM_CMD_DOR               (0x3B)    /* Memory read opcode - Dual output Read (DOR) */
#define MEM_CMD_QOR               (0x6B)    /* Memory read opcode - Quad output Read (QOR) */
#define MEM_CMD_SSRD              (0x4B) 	/* Special sector -256 byte read */

/* QSPI F-RAM Write Opcodes*/    
#define MEM_CMD_WRITE             (0x02) 	/* Memory WRITE opcode */
#define MEM_CMD_WREN              (0x06) 	/* Write Enable */
#define MEM_CMD_SSWR              (0x42) 	/* Special sector -256 byte write */
#define MEM_CMD_FASTWRITE         (0xDA)    /* Fast Write opcode */
#define MEM_CMD_DIW               (0xA2)    /* Dual input write - data on dual line */
#define MEM_CMD_DIOW              (0xA1)    /* Dual input input write - address and data on dual line */
#define MEM_CMD_QIW               (0x32)    /* Quad input write - data on quad line */
#define MEM_CMD_QIOW              (0xD2)    /* Quad input input write - address and data on quad line */

/* Status and Configuration Register Opcodes */
#define MEM_CMD_WRSR              (0x01) 	/* Write the status and configuration registers (SR1, CR1, CR2, CR4, CR5) */
#define MEM_CMD_WRDI              (0x04) 	/* Write disable the write enable latch */
#define MEM_CMD_RDSR1             (0x05) 	/* Read Status Register 1 (SR1) */
#define MEM_CMD_RDSR2             (0x07) 	/* Read Status Register 2 (SR2) */
#define MEM_CMD_RDCR1             (0x35) 	/* Read Configuration Register 1 (CR1) */
#define MEM_CMD_RDCR2             (0x3F) 	/* Read Configuration Register 2 (CR2) */
#define MEM_CMD_RDCR4             (0x45) 	/* Read Configuration Register 4 (CR4) */
#define MEM_CMD_RDCR5             (0x5E) 	/* Read Configuration Register 5 (CR5) */
#define MEM_CMD_WRAR              (0x71) 	/* Write to any one register (SR1, CR1, CR2, CR4, CR5), one byte at a time */
#define MEM_CMD_RDAR              (0x65) 	/* Read from any one register (SR1, CR1, CR2, CR4, CR5), one byte at a time */

/* Device ID Access Opcodes*/
#define MEM_CMD_RDID              (0x9F) 	/* Command to read 8-byte device ID */
#define MEM_CMD_RDSN              (0xC3) 	/* Command to read 8-byte Serial Numer (SN) */
#define MEM_CMD_RUID              (0x4C) 	/* Command to read 8-byte unique ID */
#define MEM_CMD_WRSN              (0xC2) 	/* Command to write 8-byte Serial Numer (SN) */   
    
/* Low Power Mode Commands*/
#define MEM_CMD_ENTDPD      (0xB9) 	/* Enter DPD*/
#define MEM_CMD_ENTHBN      (0xBA) 	/* Enter Hibernate*/

/***************************************/
/*QSPI F-RAM Fiunction Protype         */
/***************************************/

void WriteCmdWRSR(SMIF_Type *baseaddr,                   /* Change the Status and Config Register */
                  cy_stc_smif_context_t *smifContext, 
                  uint8_t cmdParam[], 	
                  uint32_t cmdSize,
                  uint8_t spimode);		    		

void WriteCmdReadSRx(SMIF_Type *baseaddr,                 /* Read from Status Register (SR1 and SR2) */
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[], 
                  uint32_t rxSize,
                  uint8_t spimode,
                  uint8_t cmdtype,
                  uint8_t latency);	

void WriteCmdSPIReadAnyReg(SMIF_Type *baseaddr,         /* Read Any Register (Status and Config) from register address, one byte */
                           cy_stc_smif_context_t *smifContext, 
                           uint8_t tst_rxBuffer[], 
                           uint32_t rxSize, 
                           uint8_t *address,
                           uint8_t spimode,
                           uint8_t latency);

void WriteCmdSPIWriteAnyReg(SMIF_Type *baseaddr,        /* Write Any Register (Status and Config) at register address, one byte */ 
                            cy_stc_smif_context_t *smifContext, 
                            uint8_t tst_txBuffer[], 
                            uint32_t txSize, 
                            uint8_t *address,
                            uint8_t spimode);

void WriteCmdReadCRx(SMIF_Type *baseaddr,               /* Read Config Register (CR1, CR2, CR4, CR5)*/
                   cy_stc_smif_context_t *smifContext,
                   uint8_t tst_rxBuffer[], 
                   uint32_t rxSize,
                   uint8_t spimode,
                   uint8_t crtype,
                   uint8_t latency);

void WriteCmdSPIWrite(SMIF_Type *baseaddr, 	            /* Write to memory */
                      cy_stc_smif_context_t *smifContext, 	
                      uint8_t tst_txBuffer[], 	
                      uint32_t txSize, 	
                      uint8_t *address,
                      uint8_t spimode);	    			

void WriteCmdSPIRead(SMIF_Type *baseaddr,	            /* Read from memory using Read command */
                     cy_stc_smif_context_t *smifContext, 	
                     uint8_t tst_rxBuffer[], 	
                     uint32_t rxSize, 	
                     uint8_t *address,
                     uint8_t spimode,
                     uint8_t latency);   

void WriteCmdSPIFastWrite(SMIF_Type *baseaddr,         /* Fast Write to memory */
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode);

void WriteCmdSPIFastRead(SMIF_Type *baseaddr,	        /* Read from memory using FastRead command */
                         cy_stc_smif_context_t *smifContext, 	
                         uint8_t tst_rxBuffer[], 	
                         uint32_t rxSize, 	
                         uint8_t *address,
                         uint8_t spimode,
                         uint8_t latency);

void WriteCmdSPIWrite_DIOW_QIOW(SMIF_Type *baseaddr,    /* Write to memory using DIOW, QIOW in extended SPI dual IO, quad IO mode */ 
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype);

void WriteCmdSPIRead_DIOR_QIOR(SMIF_Type *baseaddr,     /* Read from memory using DIOR, QIOR in extended SPI dual IO, quad IO mode */ 
                               cy_stc_smif_context_t *smifContext, 
                               uint8_t tst_rxBuffer[], 
                               uint32_t rxSize, 
                               uint8_t *address,
                               uint8_t spimode,
                               uint8_t latency,
                               uint8_t CMDtype);

void WriteCmdSPIWrite_DIW_QIW(SMIF_Type *baseaddr,      /* Write to memory using DIW, QIW in extended SPI dual line data input, quad line input */ 
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype);

void WriteCmdSPIRead_DOR_QOR(SMIF_Type *baseaddr,       /* Read from memory using DOR, QOR in extended SPI dual line output, quad line output  */ 
                             cy_stc_smif_context_t *smifContext, 
                             uint8_t tst_rxBuffer[], 
                             uint32_t rxSize, 
                             uint8_t *address,
                             uint8_t CMDtype,
                             uint8_t latency);

void WriteCmdWREN(SMIF_Type *baseaddr,	                /* Memory Write Enable */
                  cy_stc_smif_context_t *smifContext,
                  uint8_t spimode);	

void WriteCmdWRDI(SMIF_Type *baseaddr,	                /* Memory Write Disable */
                  cy_stc_smif_context_t *smifContext,
                  uint8_t spimode);	   

void WriteCmdSSWR(SMIF_Type *baseaddr,                  /* Wtite 256-byte Special Sector */ 
                  cy_stc_smif_context_t *smifContext, 
                  uint8_t tst_txBuffer[], 
                  uint32_t txSize, 
                  uint8_t *address,
                  uint8_t spimode);

void WriteCmdSSRD(SMIF_Type *baseaddr,                  /* Read 256-byte Special Sector */
                   cy_stc_smif_context_t *smifContext, 
                   uint8_t tst_rxBuffer[], 
                   uint32_t rxSize, 
                   uint8_t *address,
                   uint8_t spimode,
                   uint8_t latency);

void WriteCmdWRSN(SMIF_Type *baseaddr,                  /* Wrire 8-byte Serial Number */   
                  cy_stc_smif_context_t *smifContext, 
                  uint8_t cmdParam[], 	
                  uint32_t cmdSize,
                  uint8_t spimode);

void WriteCmdRDSN(SMIF_Type *baseaddr,                  /* Read 8-byte Serial Number */  
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[],
                  uint32_t txSize,
                  uint8_t spimode,
                  uint8_t latency);    

void WriteCmdRDID(SMIF_Type *baseaddr,                  /* Read 8-byte Device ID */
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[], 
                  uint8_t spimode,
                  uint8_t latency);

void WriteCmdRDUID(SMIF_Type *baseaddr,                 /* Read 8-byte Unique Device ID */
                   cy_stc_smif_context_t *smifContext,
                   uint8_t tst_rxBuffer[], 
                   uint8_t spimode,
                   uint8_t latency);

void WriteCmdEnterLPMode(SMIF_Type *baseaddr,	        /* Enter low power modes (DPD and HIBNET) */ 
                  cy_stc_smif_context_t *smifContext,
                  uint8_t powermode, 
                  uint8_t spimode);	
    
/* [] END OF FILE */
