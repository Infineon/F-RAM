-- =============================================================================
-- The following directives assign pins to the locations specific for the
-- CY8CKIT-042 kit.
-- =============================================================================

-- === RGB LED ===
attribute port_location of LED_GREEN(0) : label is "PORT(0,2)"; -- GREEN LED
attribute port_location of LED_RED(0) : label is "PORT(1,6)"; -- RED LED

-- === UART ===
attribute port_location of \UART:rx(0)\ : label is "PORT(0,4)";
attribute port_location of \UART:tx(0)\ : label is "PORT(0,5)";

-- === I2C Master ===
attribute port_location of \I2C_Master:scl(0)\ : label is "PORT(4,0)";
attribute port_location of \I2C_Master:sda(0)\ : label is "PORT(4,1)";