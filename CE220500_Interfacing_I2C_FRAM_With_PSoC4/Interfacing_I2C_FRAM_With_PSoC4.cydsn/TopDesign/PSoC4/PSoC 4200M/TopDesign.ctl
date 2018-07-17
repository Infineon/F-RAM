-- =============================================================================
-- The following directives assign pins to the locations specific for the
-- CY8CKIT-044 kit.
-- =============================================================================

-- === RGB LED ===
attribute port_location of LED_GREEN(0) : label is "PORT(2,6)"; -- GREEN LED
attribute port_location of LED_RED(0) : label is "PORT(0,6)"; -- RED LED

-- === UART ===
attribute port_location of \UART:rx(0)\ : label is "PORT(7,0)";
attribute port_location of \UART:tx(0)\ : label is "PORT(7,1)";

-- === I2C Master ===
attribute port_location of \I2C_Master:scl(0)\ : label is "PORT(4,0)";
attribute port_location of \I2C_Master:sda(0)\ : label is "PORT(4,1)";