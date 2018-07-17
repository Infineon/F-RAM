-- =============================================================================
-- The following directives assign pins to the locations specific for the
-- CY8CKIT-042 kit.
-- =============================================================================

-- === RGB LED ===
attribute port_location of LED_GREEN(0) : label is "PORT(0,2)"; -- GREEN LED
attribute port_location of LED_RED(0) : label is "PORT(1,6)"; -- RED LED

-- === UART ===
attribute port_location of \UART:rx(0)\ : label is "PORT(4,0)";
attribute port_location of \UART:tx(0)\ : label is "PORT(4,1)";

-- === SPI Master ===
attribute port_location of \SerialNVRAM_SPI:CS_0(0)\ : label is "PORT(3,4)";
attribute port_location of \SPI_Master:miso_m(0)\ : label is "PORT(3,1)";
attribute port_location of \SPI_Master:mosi_m(0)\ : label is "PORT(3,0)";
attribute port_location of \SPI_Master:sclk_m(0)\ : label is "PORT(0,6)";
