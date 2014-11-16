CAN Testing with the MCP2515, MCP2551, and Arduino Due was complete on November 15, 2014. Testing began at the beginning of winter quarter. The Arduino is connected to the MCP2515 over the SPI port (see http://epic-cdn.epictinker.com/v/vspfiles/photos/SPI.pinout.on.Due.png). The Due is powered from the Native USB port. This allows it to provide 5V to the MCP2515 and the MCP2551. For Wiring instructions, refer to https://www.sparkfun.com/datasheets/DevTools/Arduino/canbus_shield-v12.pdf.
NOTE: It is important to include the 16MHz oscillator, as the 2515 will not work without it.

Two tests were complete. 

Test 1:

	The MCP2515 was put in loopback mode using commands over SPI. This internally connects the TX and RX pins, allowing diagnostic messages to be sent and received by the same chip. This test was complete successfully.

Test 2:
	Two MCP2515s were connected to two different Dues. 2551 tranceivers were connected to the TX/RX pins of the 2515s. A 47kOhm resistor was placed between the Rs pin and GND on the 2551. the CANH and CANL lines of the 2551s were connected. A 2-byte communication was successfull sent between the two boards.  

Alexander Martin, Nusolar, 2014