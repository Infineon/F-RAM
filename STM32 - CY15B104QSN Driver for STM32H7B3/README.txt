
             Excelon QSPI F-RAM Driver for STM32 OctoSPI platforms
             =====================================================
               
   This package includes a new driver for Cypress Excelon F-RAM memories on
   STM32 microcontrollers with an OctoSPI controller, i.e.
   
     STM32L4Rxxx, STM32L4Sxxx, STM32L4P5xx, STM32L4Q5xx,
     STM32L5xxxx,
     STM32H7A3xx, STM32H7B3xx
     
   and more. Although the driver originally has been developed for a 4 Mbit
   CY15B104QSN F-RAM on a STM32H7B3 Discovery Kit, it should be easy and
   straightforward to modify and adopt it for other combinations as well.
   
   STM32H7B3 Discovery Kits come with a Macronix Octal flash device on the 
   board and the corresponding BSP includes dedicated drivers for it. This 
   SPI driver stack has been used as a starting point and has been modified
   in order to support Cypress Excelon F-RAM as follows:
   
     stm32h7b3i_discovery_ospi.[h|c]  NEW =>  stm32h7b3i_discovery_fram.[h|c]
     mx25lm51245g.[h|c]               NEW =>  cy15b104qsn.[h|c]
     stm32h7xx_hal_ospi.[h|c]         SAME    stm32h7xx_hal_ospi.[h|c]
   
   Two new board and chip drivers are included in this package. The OctoSPI
   controller driver from the BSP (stm32h7xx_hal_ospi) can be used unchanged.

   Once the new drivers have been integratd (Makefile update), F-RAM specific
   initialization code has to be added to some higher layer. The following
   code shows a simple example. It initializes the system for SPI-Single
   Transfer Rate and enables direct memory mapped read and write access to
   F-RAM:
   
     BSP_OSPI_FRAM_Init_t ospiFramInit;
     ospiFramInit.InterfaceMode = BSP_OSPI_FRAM_SPI_MODE;
     ospiFramInit.TransferRate  = BSP_OSPI_FRAM_STR_TRANSFER;
     BSP_OSPI_FRAM_Init( 0, &ospiFramInit );
     BSP_OSPI_FRAM_EnableMemoryMappedMode( 0 );
  
   The new driver supports SPI-STR, QPI-STR and QPI-DTR modes. Fine tuning
   of the SPI clock can be done via ClockPrescaler in BSP_OSPI_FRAM_Init()
   [stm32h7b3i_discovery_fram.c]. By default, the SPI clock runs with a
   prescaler of 6, i.e. at 280/6 MHz = 46.7 MHz. This should work fine in
   all cases and might be increased for STR modes.

   Once all changes are in place, the F-RAM memory can be accessed from the
   application at address OCTOSPI1_BASE. It can be used to store application
   data or even be used as frame buffer (QPI-DTR mode recommended).
