/*
 * spifi_fram.h
 *
 *  Created on: Feb 5, 2018
 *      Author: vini
 */

#include "spifilib_dev.h"
#include "private/spifilib_chiphw.h"

#ifndef INC_SPIFI_FRAM_H_
#define INC_SPIFI_FRAM_H_

#define FRAMDeviceID		0x0000000006825150
#define opcode(A)			(A << 24)
#define memSize 			512*1024
#define SPIFI_MAX_PAGESIZE	(16*1024)

#define STAT_CMD	0x02
#define IN			0	// IN from FRAM
#define OUT			1	// OUT to FRAM

#define WREN		0x06
#define WRDSB		0x04
#define WRR			0x01
#define RDSR1		0x05
#define RDSR2		0x07
#define RDCR1		0x35
#define RDCR2		0x3F
#define RDCR4		0x45
#define RDCR5		0x5E
#define READ		0x03
#define WRITE		0x02
#define FAST_RD		0x0B
#define RDID		0x9F
#define WRAR		0x71
#define WRSN		0xC2
#define RDSN		0xC3
#define DIW			0xA2
#define DIOW		0xA3
#define QIW			0x32
#define QIOW		0xD2
#define DOR			0x3B
#define DIOR		0xBB
#define QOR			0x6B
#define QIOR		0xEB

#define SRWD		7
#define TBPROT		5
#define set(A)		(0x01 << A)
#define reset(A)	~(0x01 << A)

typedef enum
{
	FRAM_ERR_QPI_ENABLED = 0,
	FRAM_DONE
}FRAM_ERR_T;

#define Bool_T	_Bool

typedef struct FRAM_REGISTER
{
	uint8_t statusREGISTER1;
	uint8_t statusREGISTER2;
	uint8_t configREGISTER1;
	uint8_t configREGISTER2;
	uint8_t configREGISTER4;
	uint8_t configREGISTER5;
}FRAM_REGISTER_T;

typedef enum
{
	Ready = 0,
	Busy
}WIP;

typedef enum
{
	None = 0,
	one_64thMemory,
	one_32ndMemory,
	one_16thMemory,
	one_8thMemory,
	one_4thMemory,
	halfMemory,
	fullMemory
}BLOCK_PROTECT_T;

typedef enum
{
	Protected = 0,
	Writable
}WRITE_PROTECTION_T;

typedef struct spifiFRAM
{
	uint32_t pSpifi;
	uint8_t fieldform;
	uint8_t frameform;
	uint8_t intdlen;
}spifiFRAM_T;

void FRAM_Write(spifiFRAM_T *,uint32_t Addr,uint32_t *writeBuff,uint32_t bytes);
void FRAM_Read(spifiFRAM_T	*,uint32_t Addr,uint32_t *writeBuff,uint32_t bytes);
void FRAM_WriteEnable(spifiFRAM_T *fram);
void FRAM_WriteDisable(spifiFRAM_T *fram);
void FRAM_StatusRegisterWriteDisable(spifiFRAM_T *fram,Bool_T);
void FRAM_TBPROT(spifiFRAM_T *fram,Bool_T);
void FRAM_BlockProtect(spifiFRAM_T *fram,BLOCK_PROTECT_T BP);
WIP FRAM_WIP(spifiFRAM_T *fram);
void FRAM_SetLatency(spifiFRAM_T *fram,uint8_t );
uint8_t FRAM_GetLatency(spifiFRAM_T *fram);
void FRAM_SetQuadMode(spifiFRAM_T *fram,Bool_T);
uint8_t FRAM_GetQuadMode(spifiFRAM_T *fram);
FRAM_ERR_T FRAM_SetQPI(spifiFRAM_T *fram,Bool_T);
void FRAM_SetIO3R(spifiFRAM_T *fram,Bool_T);
FRAM_ERR_T FRAM_SetDPI(spifiFRAM_T *fram,Bool_T);
void FRAM_SetOuputImpedance(spifiFRAM_T *fram,uint8_t OI);
void FRAM_SetDPDPOR(spifiFRAM_T *fram,Bool_T);
void FRAM_SetRegisterLatency(spifiFRAM_T *fram,uint8_t);
void FRAM_WriteSerialNumber(spifiFRAM_T *,uint32_t *);
void FRAM_ReadSerialNumber(spifiFRAM_T *,uint32_t *);
uint8_t SpifiFram_config(spifiFRAM_T *fram);

#endif /* INC_SPIFI_FRAM_H_ */
