//*****************************************************************************
//
//! @file spi_boot_host.c
//!
//! @brief An example to drive the IO Slave on a second board.
//!
//! This example acts as the boot host for spi_boot and multi_boot on Apollo
//! and Apollo2 MCUs. It will deliver a predefined firmware image to a boot
//! slave over a SPI protocol. The purpose of this demo is to show how a host
//! processor might store, load, and update the firmware on an Apollo or
//! Apollo2 device that is connected as a slave.
//!
//! Please see the multi_boot README.txt for more details on how to run the
//! examples.
//!
//! @verbatim
//! PIN fly lead connections assumed by multi_boot:
//!     HOST                                    SLAVE (multi_boot target)
//!     --------                                --------
//!     GPIO[2]  GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
//!     GPIO[4]  OVERRIDE pin   (host to slave) GPIO[18] Override pin or n/c
//!     GPIO[5]  IOM0 SPI CLK/I2C SCL           GPIO[0]  IOS SPI SCK/I2C SCL
//!     GPIO[6]  IOM0 SPI MISO/I2C SDA          GPIO[1]  IOS SPI MISO/I2C SDA
//!     GPIO[7]  IOM0 SPI MOSI                  GPIO[2]  IOS SPI MOSI
//!     GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
//!     GPIO[17] Slave reset (host to slave)    Reset Pin or n/c
//!     GND                                     GND
//! Reset and Override pin connections from Host are optional
//! Keeping Button1 pressed on target has same effect as host driving override
//! @endverbatim
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2018, Ambiq Micro
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
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
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
// This is part of revision 1.2.12 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_devices_spifram.h"
#include "common.h"

// Macro Definitions

// Uncomment for Debug info
#define DEBUG_SWO

#define FRAM_IOM         0
#define FRAM_CS          0

#define BUF_SIZE         16

uint8_t devID[9];
uint8_t status_reg;
uint8_t ui8WrBuf[BUF_SIZE];
uint8_t ui8RdBuf[BUF_SIZE];

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_24MHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 4,
    .ui8ReadThreshold = 60,
};

am_devices_spifram_t spifram_dev =
{
    .ui32IOMModule = FRAM_IOM,
    .ui32ChipSelect = FRAM_CS
};

//*****************************************************************************
//
// Configure GPIOs for this example
//
//*****************************************************************************
void
configure_pins(void)
{
    //
    // Configure I/O Master 0 as SPI
    //
    am_hal_gpio_pin_config(5, (AM_HAL_PIN_5_M0SCK| AM_HAL_GPIO_INPEN | AM_HAL_GPIO_HIGH_DRIVE));
    am_hal_gpio_pin_config(6,  AM_HAL_PIN_6_M0MISO);
    am_hal_gpio_pin_config(7,  AM_HAL_PIN_7_M0MOSI);
    am_hal_gpio_pin_config(42, AM_HAL_PIN_42_M0nCE0);
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
     uint32_t i;
     
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

   
    am_hal_iom_pwrctrl_enable(FRAM_IOM);
    
    //
    // Setup the pins for IO Master Example.
    //
    configure_pins();
    
    am_hal_iom_config(FRAM_IOM, &g_sIOMConfig);    

    am_devices_spifram_init(&spifram_dev);
    
    //
    // Turn on the IOM for this operation.
    //
    am_bsp_iom_enable(0);

#ifdef DEBUG_SWO
    // Initialize the printf interface for ITM/SWO output.
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_hal_itm_print);
    // Initialize the SWO GPIO pin
    am_bsp_pin_enable(ITM_SWO);
    // Enable the ITM.
    am_hal_itm_enable();
    // Enable debug printf messages using ITM on SWO pin
    am_bsp_debug_printf_enable();
    // Clear the terminal and print the banner.
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("----------------------------------------------------------------\r\n");    
    am_util_stdio_printf("Interfacing SPI F-RAM with Ambiq Apollo2\r\n");
    am_util_stdio_printf("----------------------------------------------------------------\r\n");    
#endif
    
// 1. Read the device ID
    am_devices_spifram_id(devID);

#ifdef DEBUG_SWO
       am_util_stdio_printf("Device ID = %02X %02X %02X %02X %02X %02X %02X %02X\r\n\n", 
                            devID[0], devID[1], devID[2], devID[3], devID[4], devID[5], devID[6], devID[7]);
#endif
       
// 2. Status Register Write/Read Check
    am_devices_spifram_status_write(0x08);
    status_reg = am_devices_spifram_status();
#ifdef DEBUG_SWO
       am_util_stdio_printf("Status register Write = 0x08 Read = 0x%02X\r\n\n", status_reg);
#endif
    am_devices_spifram_status_write(0x0);       


// 3. Memory Write/Read Check
    for(i=0;i<BUF_SIZE;i++)
      ui8WrBuf[i] = i+3;
    
    am_devices_spifram_write(ui8WrBuf, 0x00000, BUF_SIZE);
    
    am_devices_spifram_read(ui8RdBuf, 0x00000, BUF_SIZE);

#ifdef DEBUG_SWO
       am_util_stdio_printf("Memory Write = ");
       for(i=0;i<BUF_SIZE;i++)
           am_util_stdio_printf(" %02X", ui8WrBuf[i]);
       am_util_stdio_printf(" \r\n\n");       
       
       am_util_stdio_printf("Memory Read  = ");
       for(i=0;i<BUF_SIZE;i++)
           am_util_stdio_printf(" %02X", ui8RdBuf[i]);
       am_util_stdio_printf(" \r\n\n");                   		
#endif		

#ifdef DEBUG_SWO       
    am_util_stdio_printf("----------------------------------------------------------------\r\n");    
#endif
    
    //
    // Loop forever.
    //
    while (1)
    {
    }
}
