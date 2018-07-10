/*
 * spifi_fram.c
 *
 *  Created on: Feb 5, 2018
 *      Author: vini
 */


#include "spifi_fram.h"

static FRAM_REGISTER_T framRegister;
LPC_SPIFI_CHIPHW_T *pSpifi;

static SPIFI_ERR_T spifi_write(spifiFRAM_T *fram, uint32_t Addr, uint32_t *writeBuff, uint32_t bytes)
{
	/*~~~~~~~~~~~~~~~~~~~~*/
	uint8_t		*writeBuff8;
	uint32_t	dwords;
	uint32_t	cmd;
	uint8_t 	opcode;
	/*~~~~~~~~~~~~~~~~~~~~*/

	pSpifi = (LPC_SPIFI_CHIPHW_T *) fram->pSpifi;

	if(fram->fieldform == SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS)
	{
		opcode = (pSpifi->CTRL & (0x01 << 28))?DIW:QIW;
	}
	else if(fram->fieldform = SPIFI_FIELDFORM_SERIAL_OPCODE)
	{
		opcode = (pSpifi->CTRL & (0x01 << 28))?DIOW:QIOW;
	}
	else
	{
		opcode = WRITE;
	}

	cmd =
		(
			SPIFI_CMD_OPCODE(opcode) |
			SPIFI_CMD_DOUT(1) |
			SPIFI_CMD_FIELDFORM(fram->fieldform) |
			SPIFI_CMD_FRAMEFORM(fram->frameform) |
			SPIFI_CMD_DATALEN(bytes)
		);

	if(bytes <= memSize)
	{
		dwords = bytes >> 2;

		if(bytes & 0x03)
		{
			writeBuff8 = (uint8_t *) writeBuff;

			spifi_HW_WaitCMD(pSpifi);
			spifi_HW_SetAddr(pSpifi, Addr);
			spifi_HW_SetCmd(pSpifi, cmd);

			/* Write data */
			while(bytes)
			{
				spifi_HW_SetData8(pSpifi, *writeBuff8);
				++writeBuff8;
				--bytes;
			}

			spifi_HW_WaitCMD(pSpifi);
		}
		else if(dwords)
		{

			/* Set address and increment for any remaining */
			spifi_HW_SetAddr(pSpifi, Addr);

			/* Finally send command and write the data */
			spifi_HW_SetCmd(pSpifi, cmd);
			while(dwords)
			{
				spifi_HW_SetData32(pSpifi, *writeBuff);
				++writeBuff;
				--dwords;
			}

			spifi_HW_WaitCMD(pSpifi);
		}
	}

	return SPIFI_ERR_NONE;
}

/*
 =======================================================================================================================
 =======================================================================================================================
 */
static SPIFI_ERR_T spifi_read(spifiFRAM_T *fram, uint32_t Addr, uint32_t *readBuff, uint32_t bytes)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	uint8_t		*readBuff8;
	uint32_t	dwords;
	uint32_t	cmd;
	uint8_t 	opcode;
#if FAST_READ
	uint32_t	READ_CMD = FAST_RD;
	uint32_t	dummy = 8;
#else
	uint32_t	READ_CMD = READ;
	uint32_t	dummy = 0;
#endif
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	pSpifi = (LPC_SPIFI_CHIPHW_T *) fram->pSpifi;

	if(fram->fieldform == SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS)
	{
		opcode = (pSpifi->CTRL & (0x01 << 28))?DOR:QOR;
	}
	else if(fram->fieldform == SPIFI_FIELDFORM_SERIAL_OPCODE)
	{
		opcode = (pSpifi->CTRL & (0x01 << 28))?DIOR:QIOR;
	}
	else
	{
		opcode = READ_CMD;
	}

	cmd =
		(
			SPIFI_CMD_OPCODE(opcode) |
			SPIFI_CMD_DOUT(0) |
			SPIFI_CMD_FIELDFORM(fram->fieldform) |
			SPIFI_CMD_FRAMEFORM(fram->frameform)
		);

	if(bytes <= memSize)
	{
		dwords = bytes >> 2;

		if(bytes & 0x03)
		{
			readBuff8 = (uint8_t *) readBuff;

			spifi_HW_SetAddr(pSpifi, Addr);
			spifi_HW_SetCmd(pSpifi, cmd | SPIFI_CMD_DATALEN(bytes) | SPIFI_CMD_INTER(dummy));

			/* read data */
			while(bytes)
			{
				*readBuff8 = spifi_HW_GetData8(pSpifi);
				++readBuff8;
				--bytes;
			}

			spifi_HW_WaitCMD(pSpifi);
		}
		else if(dwords)
		{

			/* Set address and increment for any remaining */
			spifi_HW_SetAddr(pSpifi, Addr);

			/* Finally send command and write the data */
			spifi_HW_SetCmd(pSpifi, cmd | SPIFI_CMD_DATALEN(bytes));
			while(dwords)
			{
				*readBuff = spifi_HW_GetData32(pSpifi);
				++readBuff;
				--dwords;
			}

			spifi_HW_WaitCMD(pSpifi);
		}
	}

	return SPIFI_ERR_NONE;
}

void FRAM_Write(spifiFRAM_T	*fram,uint32_t Addr,uint32_t *writeBuff,uint32_t bytes)
{
	/*~~~~~~~~~~~~~~~~~~~~~~*/
	uint32_t	numberOfpages;
	uint32_t	lastPageBytes;
	uint32_t	pageStartAddr;
	uint32_t	pageCount;
	/*~~~~~~~~~~~~~~~~~~~~~~*/

	numberOfpages = bytes / SPIFI_MAX_PAGESIZE;
	lastPageBytes = bytes % SPIFI_MAX_PAGESIZE;
	pageStartAddr = Addr;

	for(pageCount = 0; pageCount < numberOfpages; pageCount++)
	{
		spifi_write(fram, pageStartAddr, writeBuff, SPIFI_MAX_PAGESIZE);
		pageStartAddr += SPIFI_MAX_PAGESIZE;
		writeBuff += SPIFI_MAX_PAGESIZE / 4;
	}

	spifi_write(fram, pageStartAddr, writeBuff, lastPageBytes);
}

void FRAM_Read(spifiFRAM_T	*fram,uint32_t Addr,uint32_t *readBuff,uint32_t bytes)
{
	/*~~~~~~~~~~~~~~~~~~~~~~*/
	uint32_t	numberOfpages;
	uint32_t	lastPageBytes;
	uint32_t	pageStartAddr;
	uint32_t	pageCount;
	/*~~~~~~~~~~~~~~~~~~~~~~*/

	numberOfpages = bytes / SPIFI_MAX_PAGESIZE;
	lastPageBytes = bytes % SPIFI_MAX_PAGESIZE;
	pageStartAddr = Addr;

	for(pageCount = 0; pageCount < numberOfpages; pageCount++)
	{
		spifi_read(fram, pageStartAddr, readBuff, SPIFI_MAX_PAGESIZE);
		pageStartAddr += SPIFI_MAX_PAGESIZE;
		readBuff += SPIFI_MAX_PAGESIZE / 4;
	}

	spifi_read(fram, pageStartAddr, readBuff, lastPageBytes);
}

void FRAM_WriteEnable(spifiFRAM_T *fram)
{
	pSpifi = (LPC_SPIFI_CHIPHW_T *)fram->pSpifi;
	pSpifi->CMD = SPIFI_CMD_OPCODE(WREN) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP);
}

void FRAM_WriteDisable(spifiFRAM_T *fram)
{
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRDSB) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP);
}

void FRAM_StatusRegisterWriteDisable(spifiFRAM_T *fram,Bool_T A)
{
	if(A)
	{
		framRegister.statusREGISTER1 |= set(SRWD);
	}
	else
	{
		framRegister.statusREGISTER1 &= reset(SRWD);
	}

	while(pSpifi->STAT & STAT_CMD); //spifi will wait to finish previous operation.
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 1 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT8 = framRegister.statusREGISTER1;
}

void FRAM_TBPROT(spifiFRAM_T *fram,Bool_T A)
{
	if(A)
	{
		framRegister.statusREGISTER1 &= set(TBPROT);
	}
	else
	{
		framRegister.statusREGISTER1 &= reset(TBPROT);
	}

	while(pSpifi->STAT & STAT_CMD); //spifi will wait to finish previous operation.
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 1 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT8 = framRegister.statusREGISTER1;
}

void FRAM_BlockProtect(spifiFRAM_T *fram,BLOCK_PROTECT_T BP)
{
	framRegister.statusREGISTER1 |= BP << 3;

	while(pSpifi->STAT & STAT_CMD); //spifi will wait to finish previous operation.
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 1 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT8 = framRegister.statusREGISTER1;
}

WIP FRAM_WIP(spifiFRAM_T *fram)
{
	while(pSpifi->STAT & STAT_CMD); //spifi will wait to finish previous operation.
	pSpifi->CMD = SPIFI_CMD_OPCODE(RDSR1) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 1 | SPIFI_CMD_DOUT(IN);

	if(pSpifi->DAT8 & 0x01)
	{
		return Busy;
	}
	else
	{
		return Ready;
	}

}

void FRAM_SetLatency(spifiFRAM_T *fram,uint8_t Latency)
{
	framRegister.configREGISTER1 |= Latency << 4;

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 2 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;

}

uint8_t FRAM_GetLatency(spifiFRAM_T *fram)
{
	while(pSpifi->STAT & STAT_CMD); //spifi will wait to finish previous operation.
	pSpifi->CMD = SPIFI_CMD_OPCODE(RDCR1) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 1 | SPIFI_CMD_DOUT(IN);

	return ((pSpifi->DAT8 & 0xF0) >> 4);
}

void FRAM_SetQuadMode(spifiFRAM_T *fram, Bool_T A)
{
	framRegister.configREGISTER1 |= (0x02 & A);

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 2 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
}

FRAM_ERR_T FRAM_SetQPI(spifiFRAM_T *fram,Bool_T A)
{
	framRegister.configREGISTER2 |= (0x40 & A);

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 3 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
	pSpifi->DAT8 = framRegister.configREGISTER2;

	fram->fieldform = SPIFI_FIELDFORM_NO_SERIAL;
	pSpifi->CTRL &= ~(0x01 << 28);
	return FRAM_DONE;
}

void FRAM_SetIO3R(spifiFRAM_T *fram,Bool_T A)
{
	framRegister.configREGISTER2 |= (0x20 & A);

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 3 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
	pSpifi->DAT8 = framRegister.configREGISTER2;

}

FRAM_ERR_T FRAM_SetDPI(spifiFRAM_T *fram,Bool_T A)
{
	framRegister.configREGISTER2 |= (0x10 & A);

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 3 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
	pSpifi->DAT8 = framRegister.configREGISTER2;

	if(framRegister.configREGISTER2 & 0x40)
	{
		return FRAM_ERR_QPI_ENABLED;
	}
	fram->fieldform = SPIFI_FIELDFORM_NO_SERIAL;
	pSpifi->CTRL |= 0x01 << 28;
	return FRAM_DONE;
}

void FRAM_SetOuputImpedance(spifiFRAM_T *fram,uint8_t OI)
{
	framRegister.configREGISTER4 |= OI << 5;

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 4 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
	pSpifi->DAT16 = framRegister.configREGISTER2 | framRegister.configREGISTER4 << 8;

}

void FRAM_SetDPDPOR(spifiFRAM_T *fram,Bool_T A)
{
	framRegister.configREGISTER4 |= A?0x04:0x00;

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 4 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
	pSpifi->DAT16 = framRegister.configREGISTER2 | framRegister.configREGISTER4 << 8;

}

void FRAM_SetRegisterLatency(spifiFRAM_T *fram,uint8_t Latency)
{
	framRegister.configREGISTER5 |= Latency << 6;

	while(pSpifi->STAT & STAT_CMD);
	pSpifi->CMD = SPIFI_CMD_OPCODE(WRR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | 5 | SPIFI_CMD_DOUT(OUT);
	pSpifi->DAT16 = framRegister.statusREGISTER1 | framRegister.configREGISTER1 << 8;
	pSpifi->DAT16 = framRegister.configREGISTER2 | framRegister.configREGISTER4 << 8;
	pSpifi->DAT8 = framRegister.configREGISTER5;

}


void FRAM_WriteSerialNumber(spifiFRAM_T *fram,uint32_t *SN)
{
	uint32_t cmd;

	pSpifi = (LPC_SPIFI_CHIPHW_T *)fram->pSpifi;

	while(pSpifi->STAT & 0x02);
	cmd = SPIFI_CMD_OPCODE(WRSN) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | SPIFI_CMD_DOUT(OUT) | 8;
	pSpifi->CMD = cmd;
	pSpifi->DAT32 = SN;
	pSpifi->DAT32 = (SN++);
}

void FRAM_ReadSerialNumber(spifiFRAM_T *fram,uint32_t *SN)
{
	uint32_t cmd;

	pSpifi = (LPC_SPIFI_CHIPHW_T *)fram->pSpifi;

	while(pSpifi->STAT & 0x02);
	cmd = SPIFI_CMD_OPCODE(RDSN) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) | SPIFI_CMD_DOUT(IN) | 8;
	pSpifi->CMD = cmd;
	*SN = spifi_HW_GetData32(pSpifi);
	SN++;
	*SN = spifi_HW_GetData32(pSpifi);
}

uint8_t SpifiFram_config(spifiFRAM_T *fram)
{
	pSpifi = (LPC_SPIFI_CHIPHW_T *)fram->pSpifi;
	uint8_t configMatch;
	int cnt;
	fram->fieldform = SPIFI_FIELDFORM_ALL_SERIAL;
	fram->frameform = SPIFI_FRAMEFORM_OP_3ADDRESS;
	fram->intdlen = 0;
	configMatch = 0;
	cnt = 0;


	while(cnt < 3)
	{
		FRAM_WriteEnable(fram);
		while(pSpifi->STAT & STAT_CMD);
		pSpifi->CMD = SPIFI_CMD_OPCODE(WRAR) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(fram->frameform) | SPIFI_CMD_INTER(0) | SPIFI_CMD_DOUT(OUT) | 1;
		pSpifi->ADDR = 0x00000003;
		pSpifi->DAT8 = 0x00;

		while(pSpifi->STAT & STAT_CMD);
		fram->fieldform = SPIFI_FIELDFORM_NO_SERIAL;
		cnt++;

		if(cnt == 1)
		{
			pSpifi->CTRL |= 0x10000000;
		}
		else if(cnt == 2)
		{
			pSpifi->CTRL &= 0xEFFFFFFF;
		}
	}

	fram->intdlen = 0;
	fram->fieldform = SPIFI_FIELDFORM_ALL_SERIAL;
	fram->frameform = SPIFI_FRAMEFORM_OP;

	pSpifi->CMD = SPIFI_CMD_OPCODE(RDSR1) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(fram->frameform) | SPIFI_CMD_INTER(fram->intdlen) | SPIFI_CMD_DOUT(IN) | 1;
	framRegister.statusREGISTER1 = pSpifi->DAT8;
	while(pSpifi->STAT & STAT_CMD);

	pSpifi->CMD = SPIFI_CMD_OPCODE(RDCR1) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(fram->frameform) | SPIFI_CMD_INTER(fram->intdlen) | SPIFI_CMD_DOUT(IN) | 1;
	framRegister.statusREGISTER1 = pSpifi->DAT8;
	while(pSpifi->STAT & STAT_CMD);

	pSpifi->CMD = SPIFI_CMD_OPCODE(RDCR2) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(fram->frameform) | SPIFI_CMD_INTER(fram->intdlen) | SPIFI_CMD_DOUT(IN) | 1;
	framRegister.statusREGISTER1 = pSpifi->DAT8;
	while(pSpifi->STAT & STAT_CMD);

	pSpifi->CMD = SPIFI_CMD_OPCODE(RDCR4) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(fram->frameform) | SPIFI_CMD_INTER(fram->intdlen) | SPIFI_CMD_DOUT(IN) | 1;
	framRegister.statusREGISTER1 = pSpifi->DAT8;
	while(pSpifi->STAT & STAT_CMD);

	pSpifi->CMD = SPIFI_CMD_OPCODE(RDCR5) | SPIFI_CMD_FIELDFORM(fram->fieldform) | SPIFI_CMD_FRAMEFORM(fram->frameform) | SPIFI_CMD_INTER(fram->intdlen) | SPIFI_CMD_DOUT(IN) | 1;
	framRegister.statusREGISTER1 = pSpifi->DAT8;
	while(pSpifi->STAT & STAT_CMD);

	return 1;
}
