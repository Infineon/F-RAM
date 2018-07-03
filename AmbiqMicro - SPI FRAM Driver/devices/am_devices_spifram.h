//*****************************************************************************
//
//! @file am_devices_spifram.h
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
// This is part of revision 1.2.8 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_DEVICES_SPIFRAM_H
#define AM_DEVICES_SPIFRAM_H

#define EXCELON_FRAM 1
#define THREE_BYTE_ADDR 1

//*****************************************************************************
//
// Global definitions for fram commands
//
//*****************************************************************************

#define AM_DEVICES_SPIFRAM_WRSR        0x01      /* Write Status Register */
#define AM_DEVICES_SPIFRAM_WRITE       0x02      /* Write Memory */
#define AM_DEVICES_SPIFRAM_READ        0x03      /* Read Memory */
#define AM_DEVICES_SPIFRAM_WRDI        0x04      /* Reset Write Enable Latch */
#define AM_DEVICES_SPIFRAM_RDSR        0x05      /* Read Status Register */
#define AM_DEVICES_SPIFRAM_WREN        0x06      /* Write Enable Latch */
#define AM_DEVICES_SPIFRAM_RDID        0x9F      /* Read Device ID */

//*****************************************************************************
//
// Global definitions for the fram status register
//
//*****************************************************************************
#define AM_DEVICES_SPIFRAM_WEL         0x02        // Write enable latch
#define AM_DEVICES_SPIFRAM_BP0         0x04        // Block Protect 0
#define AM_DEVICES_SPIFRAM_BP1         0x08        // Block Protect 0
#define AM_DEVICES_SPIFRAM_WPEN        0x80        // Status Register Write Protect

//*****************************************************************************
//
// FRAM status register mask bits.
//
//*****************************************************************************
#define AM_DEVICES_SPIFRAM_STAT_WPEN    0x80
#define AM_DEVICES_SPIFRAM_STAT_RSVD    0x70
#define AM_DEVICES_SPIFRAM_STAT_BP1     0x08
#define AM_DEVICES_SPIFRAM_STAT_BP0     0x04
#define AM_DEVICES_SPIFRAM_STAT_WEL     0x02
#define AM_DEVICES_SPIFRAM_STAT_ZERO    0x01

#ifdef  EXCELON_FRAM
#define DEVID_LEN                       0x08
#else
#define DEVID_LEN                       0x09
#endif

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Device structure used for communication.
//
//*****************************************************************************
typedef struct
{
    //*************************************************************************
    // Parameters supplied by application.
    //*************************************************************************

    //
    // Module number to use for IOM access.
    //
    uint32_t ui32IOMModule;

    //
    // Chip Select number to use for IOM access.
    //
    uint32_t ui32ChipSelect;

}
am_devices_spifram_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void am_devices_spifram_init(am_devices_spifram_t *psIOMSettings);
extern uint8_t am_devices_spifram_status(void);
extern void    am_devices_spifram_status_write(uint32_t ui32StatusValue);
extern void am_devices_spifram_id(uint8_t *devID);
extern uint32_t am_devices_spifram_max_transfersize(
    uint8_t rwcmd, uint32_t iomModule, uint32_t bytesRemaining);
extern void am_devices_spifram_read(uint8_t *pui8RxBuffer,
                                     uint32_t ui32ReadAddress,
                                     uint32_t ui32NumBytes);
extern void am_devices_spifram_write(uint8_t *ui8TxBuffer,
                                      uint32_t ui32WriteAddress,
                                      uint32_t ui32NumBytes);


#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_SPIFRAM_H

