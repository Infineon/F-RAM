This example showcases how to interface spi fram on Apollo and Apollo2 MCUs. 
The example code will read id, perform basic read/write to memory.
This requires an arduino compatible F-RAM board CY15FRAMKIT-001/CY15FRAMKIT-002.

Steps to run the project:

1. Download AmbiqSuite 1.2.12 from https://ambiqmicro.com/

2. Place spi_fram project folder (SPI_FRAM\boards\apollo2_evb\examples\)
   in Ambiq SDK under "AmbiqSuite-Rel1.2.12\boards\apollo2_evb\examples\"

3. Add the F-RAM driver files am_devices_spifram.c and am_devices_spifram.h
   under "AmbiqSuite-Rel1.2.12\devices"

4. Project can be compiled in Keil uVision5 or IAR Embedded Workbench

5. Download the binary into Apollo2 AMAPH1KK-KBR EVB board. Plug-in CY15FRAMKIT-001/CY15FRAMKIT-002

6. J-Link SWO Viewer to observe the project outputs. J-Link can be downloaded from 
   https://www.segger.com/downloads/jlink/

7. Press the System Reset button to reproduce the output
