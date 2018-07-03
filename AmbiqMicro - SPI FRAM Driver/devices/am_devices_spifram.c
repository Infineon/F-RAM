//*****************************************************************************
//
//! @file am_devices_spifram.c
//!
//! @brief Generic spifram driver.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2017, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//
//*****************************************************************************

#include <string.h>
#include "am_mcu_apollo.h"
#include "am_devices_spifram.h"

//! Maximum supported by IOM device in one transfer.
#define MAX_TRANSFERSIZE (4095)
//! Adjusted 4091 max for IOM 4.
#define MAX_TRANSFERSIZE_IOM4 (4091)
// Support max 4 queued transactions
#define MAX_QUEUED_WRITES   4

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
am_devices_spifram_t g_sSpiIOMSettings;

//! Write buffer when single write must be used. Otherwise use caller's buffer.
am_hal_iom_buffer(MAX_TRANSFERSIZE) g_xferBuffer[MAX_QUEUED_WRITES];
uint32_t                            g_WrBufIdx = 0;
am_hal_iom_buffer(4)                g_tempBuffer[MAX_QUEUED_WRITES];
uint32_t                            g_TempBufIdx = 0;


//*****************************************************************************
//
//! @brief Initialize the spifram driver.
//!
//! @param psIOMSettings - IOM device structure describing the target spifram.
//! @param pfnWriteFunc - Function to use for spi writes.
//! @param pfnReadFunc - Function to use for spi reads.
//!
//! This function should be called before any other am_devices_spifram
//! functions. It is used to set tell the other functions how to communicate
//! with the external spifram hardware.
//!
//! The \e pfnWriteFunc and \e pfnReadFunc variables may be used to provide
//! alternate implementations of SPI write and read functions respectively. If
//! they are left set to 0, the default functions am_hal_iom_spi_write() and
//! am_hal_iom_spi_read() will be used.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_spifram_init(am_devices_spifram_t *psIOMSettings)
{
    //
    // Initialize the IOM settings from the application.
    //
    g_sSpiIOMSettings.ui32IOMModule = psIOMSettings->ui32IOMModule;
    g_sSpiIOMSettings.ui32ChipSelect = psIOMSettings->ui32ChipSelect;

}

//*****************************************************************************
//
//! @brief Reads the ID register for the external fram
//!
//! @param ui32DeviceNumber - Device number of the external fram
//!
//! This function reads the ID register of the external fram.
//! The processor will block during the data transfer process, but will return
//! as soon as the ID register had been read. The ID is a 8 byte for
//! Excelon F-RAM and 9 byte for legacy F-RAMs
//!
//! @return None
//
//*****************************************************************************
am_hal_iom_buffer(4) g_spiCmdResponse;

void
am_devices_spifram_id(uint8_t *devID)
{

    //
    // Send a command to read the ID register in the external fram.
    //
    am_hal_iom_spi_read(g_sSpiIOMSettings.ui32IOMModule,
                        g_sSpiIOMSettings.ui32ChipSelect,
                        (uint32_t *)devID, DEVID_LEN,
                        AM_HAL_IOM_OFFSET(AM_DEVICES_SPIFRAM_RDID));
}

//*****************************************************************************
//
//! @brief Reads the current status of the external fram
//!
//! @param ui32DeviceNumber - Device number of the external fram
//!
//! This function reads the status register of the external fram, and returns
//! the result as an 8-bit unsigned integer value. The processor will block
//! during the data transfer process, but will return as soon as the status
//! register had been read.
//!
//! Macro definitions for interpreting the contents of the status register are
//! included in the header file.
//!
//! @return 8-bit status register contents
//
//*****************************************************************************
uint8_t
am_devices_spifram_status(void)
{

    //
    // Send the command and read the response.
    //
    am_hal_iom_spi_read(g_sSpiIOMSettings.ui32IOMModule,
                        g_sSpiIOMSettings.ui32ChipSelect,
                        g_spiCmdResponse.words, 1,
                        AM_HAL_IOM_OFFSET(AM_DEVICES_SPIFRAM_RDSR));

    //
    // Return the status read from the external fram.
    //
    return g_spiCmdResponse.bytes[0];
}

//*****************************************************************************
//
//! @brief Writes to the status register of the external fram
//!
//! @param ui32DeviceNumber - Device number of the external fram
//!
//! This function writes the status register of the external fram.
//! The processor will block during the data transfer process, but will return
//! as soon as the status register had been read.
//!
//! Macro definitions for interpreting the contents of the status register are
//! included in the header file.
//!
//! @return 8-bit status register contents
//
//*****************************************************************************
void
am_devices_spifram_status_write(uint32_t ui32StatusValue)
{
    g_tempBuffer[g_TempBufIdx].bytes[0] = AM_DEVICES_SPIFRAM_WREN;
    g_tempBuffer[g_TempBufIdx].bytes[1] = 0;
    g_tempBuffer[g_TempBufIdx].bytes[2] = 0;
    g_tempBuffer[g_TempBufIdx].bytes[3] = 0;

    //
    // Send the 1 byte write enable command to the IOM.
    //
    am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                         g_sSpiIOMSettings.ui32ChipSelect,
                         g_tempBuffer[g_TempBufIdx].words,
                         1, AM_HAL_IOM_RAW);

    //
    // Send the 2 byte Write Status Register command
    //
    g_tempBuffer[g_TempBufIdx].bytes[0] = AM_DEVICES_SPIFRAM_WRSR;
    g_tempBuffer[g_TempBufIdx].bytes[1] = (uint8_t)ui32StatusValue;

    //
    // Send the 2 byte write command to the IOM.
    //
    am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                         g_sSpiIOMSettings.ui32ChipSelect,
                         g_tempBuffer[g_TempBufIdx].words,
                         2, AM_HAL_IOM_RAW);

    g_tempBuffer[g_TempBufIdx].bytes[0] = AM_DEVICES_SPIFRAM_WRDI;
    g_tempBuffer[g_TempBufIdx].bytes[1] = 0;
    g_tempBuffer[g_TempBufIdx].bytes[2] = 0;
    g_tempBuffer[g_TempBufIdx].bytes[3] = 0;

    //
    // Issue the WRDI command to reset WEL (Write Enable Latch).
    // The WREN command sets WEL.
    //
    am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                         g_sSpiIOMSettings.ui32ChipSelect,
                         g_tempBuffer[g_TempBufIdx].words,
                         1, AM_HAL_IOM_RAW);
}

//! Return max transfersize or bytes Remaining.
uint32_t am_devices_spifram_max_transfersize(
         uint8_t rwcmd, uint32_t iomModule, uint32_t bytesRemaining)
{
    uint32_t max_transfersize =
        (iomModule == 4 ? MAX_TRANSFERSIZE_IOM4 : MAX_TRANSFERSIZE);

    if (rwcmd == AM_DEVICES_SPIFRAM_WRITE)
    {

#if (THREE_BYTE_ADDR == 1)			
        max_transfersize -= 4;
#else
        max_transfersize -= 3;			
#endif	
			
    }

    return (bytesRemaining > max_transfersize ? max_transfersize : bytesRemaining);
}

//*****************************************************************************
//
//! @brief Reads the contents of the external fram into a buffer.
//!
//! @param ui32DeviceNumber - Device number of the external fram
//! @param pui8RxBuffer - Buffer to store the received data from the fram
//! @param ui32ReadAddress - Address of desired data in external fram
//! @param ui32NumBytes - Number of bytes to read from external fram
//!
//! This function reads the external fram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//*****************************************************************************
void
am_devices_spifram_read(uint8_t *pui8RxBuffer, uint32_t ui32ReadAddress,
                        uint32_t ui32NumBytes)
{
    uint32_t ui32BytesRemaining, ui32CurrentReadAddress;
    uint8_t *pui8Dest;

    //
    // Set the total number of bytes,and the starting transfer destination.
    //
    ui32BytesRemaining = ui32NumBytes;
    pui8Dest = pui8RxBuffer;
    ui32CurrentReadAddress = ui32ReadAddress;

    while ( ui32BytesRemaining > 0)
    {
        //
        // Set the transfer size to either TRANSFERSIZE,
        // or the number of remaining bytes, whichever is smaller.
        //
        uint32_t ui32TransferSize = am_devices_spifram_max_transfersize(
            AM_DEVICES_SPIFRAM_WRITE, g_sSpiIOMSettings.ui32IOMModule,
            ui32BytesRemaining);

#if (THREE_BYTE_ADDR == 1)
        //
        // READ is a 4-byte command (opcode + 24-bit address)
        //
        g_tempBuffer[g_TempBufIdx].bytes[0] = AM_DEVICES_SPIFRAM_READ;
        g_tempBuffer[g_TempBufIdx].bytes[1] = (ui32CurrentReadAddress & 0x00FF0000) >> 16;
        g_tempBuffer[g_TempBufIdx].bytes[2] = (ui32CurrentReadAddress & 0x0000FF00) >> 8;
        g_tempBuffer[g_TempBufIdx].bytes[3] = (ui32CurrentReadAddress & 0x000000FF);

        //
        // Send Read Command to SPIFRAM, Hold CS Low, then SPI Read to recieve data.
        //
        am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                             g_sSpiIOMSettings.ui32ChipSelect,
                             g_tempBuffer[g_TempBufIdx].words, 4,
                             AM_HAL_IOM_CS_LOW | AM_HAL_IOM_RAW);
#else
        //
        // READ is a 3-byte command (opcode + 16-bit address)
        //
        g_tempBuffer[g_TempBufIdx].bytes[0] = AM_DEVICES_SPIFRAM_READ;
        g_tempBuffer[g_TempBufIdx].bytes[1] = (ui32CurrentReadAddress & 0x0000FF00) >> 8;
        g_tempBuffer[g_TempBufIdx].bytes[2] = (ui32CurrentReadAddress & 0x000000FF);

        //
        // Send Read Command to SPIFRAM, Hold CS Low, then SPI Read to recieve data.
        //
        am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                             g_sSpiIOMSettings.ui32ChipSelect,
                             g_tempBuffer[g_TempBufIdx].words, 3,
                             AM_HAL_IOM_CS_LOW | AM_HAL_IOM_RAW);
#endif

        am_hal_iom_spi_read(g_sSpiIOMSettings.ui32IOMModule,
                            g_sSpiIOMSettings.ui32ChipSelect, (uint32_t *)pui8Dest,
                            ui32TransferSize, AM_HAL_IOM_RAW);

        //
        // Update the number of bytes remaining and the destination.
        //
        ui32BytesRemaining -= ui32TransferSize;
        pui8Dest += ui32TransferSize;
        ui32CurrentReadAddress += ui32TransferSize;
    }
}

//*****************************************************************************
//
//! @brief Programs the given range of fram addresses.
//!
//! @param ui32DeviceNumber - Device number of the external fram
//! @param pui8TxBuffer - Buffer to write the external fram data from
//! @param ui32WriteAddress - Address to write to in the external fram
//! @param ui32NumBytes - Number of bytes to write to the external fram
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external fram at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target fram
//! memory or underflow the pui8TxBuffer array
//
//*****************************************************************************
void
am_devices_spifram_write(uint8_t *pui8TxBuffer, uint32_t ui32WriteAddress,
                         uint32_t ui32NumBytes)
{
    uint32_t ui32DestAddress;
    uint32_t ui32BytesRemaining;
    uint8_t *pui8Source;

    //
    // Prepare the command for write-enable.
    //
    uint32_t am_devices_spifram_wren = AM_DEVICES_SPIFRAM_WREN;

    //
    // Set the total number of bytes, and the starting transfer destination.
    //
    ui32BytesRemaining = ui32NumBytes;
    pui8Source = pui8TxBuffer;
    ui32DestAddress = ui32WriteAddress;

    while ( ui32BytesRemaining )
    {
        //
        // Send the write-enable command to prepare the external fram for
        // program operations, and wait for the write-enable latch to be set in
        // the status register.
        //
        //
        // Send the 1 byte write command, to the spifram.
        //
        am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                             g_sSpiIOMSettings.ui32ChipSelect,
                             &am_devices_spifram_wren,
                             1, AM_HAL_IOM_RAW);

        //
        // Set the transfer size to either TRANSFERSIZE,
        // or the number of remaining bytes, whichever is smaller.
        //
        uint32_t ui32TransferSize = am_devices_spifram_max_transfersize(
            AM_DEVICES_SPIFRAM_WRITE, g_sSpiIOMSettings.ui32IOMModule,
            ui32BytesRemaining);

        //
        // Set the CMD and copy the data into the same buffer.
        //
        g_xferBuffer[g_WrBufIdx].bytes[0] = AM_DEVICES_SPIFRAM_WRITE;

#if (THREE_BYTE_ADDR == 1)
        g_xferBuffer[g_WrBufIdx].bytes[1] = (ui32DestAddress & 0x00FF0000) >> 16;
        g_xferBuffer[g_WrBufIdx].bytes[2] = (ui32DestAddress & 0x0000FF00) >> 8;
        g_xferBuffer[g_WrBufIdx].bytes[3] = (ui32DestAddress & 0x000000FF);

        // IAR doesn't have bcopy(pui8Source, &g_xferBuffer.bytes[3], ui32TransferSize);
        memcpy(&g_xferBuffer[g_WrBufIdx].bytes[4], pui8Source, ui32TransferSize);

        //
        // Send the 3 byte write command and the data to the IOM.
        //
        am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                             g_sSpiIOMSettings.ui32ChipSelect,
                             g_xferBuffer[g_WrBufIdx].words,
                             ui32TransferSize + 4, AM_HAL_IOM_RAW);
#else
        g_xferBuffer[g_WrBufIdx].bytes[1] = (ui32DestAddress & 0x0000FF00) >> 8;
        g_xferBuffer[g_WrBufIdx].bytes[2] = (ui32DestAddress & 0x000000FF);

        // IAR doesn't have bcopy(pui8Source, &g_xferBuffer.bytes[3], ui32TransferSize);
        memcpy(&g_xferBuffer[g_WrBufIdx].bytes[3], pui8Source, ui32TransferSize);

        //
        // Send the 3 byte write command and the data to the IOM.
        //
        am_hal_iom_spi_write(g_sSpiIOMSettings.ui32IOMModule,
                             g_sSpiIOMSettings.ui32ChipSelect,
                             g_xferBuffer[g_WrBufIdx].words,
                             ui32TransferSize + 3, AM_HAL_IOM_RAW);
#endif



        //
        // Update the number of bytes remaining, as well as the source and
        // destination pointers
        //
        ui32BytesRemaining -= ui32TransferSize;
        pui8Source += ui32TransferSize;
        ui32DestAddress += ui32TransferSize;
    }

    return;
}


