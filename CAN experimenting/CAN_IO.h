/*
 * CAN_IO.h
 * Contains class definition for CAN_IO library.
 */

#ifndef CAN_IO_h
#define CAN_IO_h

#include "DaveAK_can/MCP2515.h"
#include "DaveAK_can/MCP2515_defs.h"
#include "MCP2515_defs_extended.h"

/*
 * Class for handling CAN I/O operations using the
 * MCP2515 CAN controller.
 */
class CAN_IO {
public:
	/*
	 * Constructor. Creates a MCP2515 object.
	 */
	CAN_IO();

	/*
	 * Initializes the CAN controller to desired settings,
	 * including read masks/filters.
	 */
	void setup();

	/*
	 * Reads messages from the CAN controller.
	 */
	void readCAN();

	/*
	 * Sends messages to the CAN bus via the controller.
	 */
	void sendCAN();

	/*
	 * ISR for MCP2515 interrupt.
	 */
	void rx_int();
private:
	MCP2515 controller;

	/*
	 * Helper function for configuring the RX masks/filters.
	 * If RXB0 is true, sets the mask/filter for the first buffer;
	 * otherwise sets the second.
	 */
	void setup_rx_filter(unsigned int mask, unsigned int filter, bool RXB0);

	/*
	 * Given a 12-bit mask/filter, extracts the first 8 bits.
	 */
	byte first_byte(unsigned int value);

	/*
	 * Given a 12-bit mask/filter, extracts the last 4 bits and 
	 * appends zeros to complete the byte.
	 */
	byte second_byte(unsigned int value);
};

#endif