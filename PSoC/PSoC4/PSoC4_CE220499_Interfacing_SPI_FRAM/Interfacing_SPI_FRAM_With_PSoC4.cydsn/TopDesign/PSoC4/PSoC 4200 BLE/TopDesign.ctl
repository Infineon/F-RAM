-- =============================================================================
-- The following directives assign pins to the locations specific for the
-- CY8CKIT-042-BLE kit.
-- =============================================================================

-- === RGB LED ===
attribute port_location of LED_GREEN(0) : label is "PORT(3,6)"; -- GREEN LED
attribute port_location of LED_RED(0) : label is "PORT(2,6)"; -- RED LED

-- === UART ===
attribute port_location of \UART:rx(0)\ : label is "PORT(3,0)";
attribute port_location of \UART:tx(0)\ : label is "PORT(3,1)";

-- === SPI Master ===
attribute port_location of \SerialNVRAM_SPI:CS_0(0)\ : label is "PORT(0,2)";
attribute port_location of \SPI_Master:miso_m(0)\ : label is "PORT(0,1)";
attribute port_location of \SPI_Master:mosi_m(0)\ : label is "PORT(0,0)";
attribute port_location of \SPI_Master:sclk_m(0)\ : label is "PORT(0,3)";