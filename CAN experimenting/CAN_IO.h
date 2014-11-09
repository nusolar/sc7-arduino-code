/*
 * CAN_IO.h
 * Contains class definition for CAN_IO library.
 */

#ifndef CAN_IO_h
#define CAN_IO_h

#include <cstdint>
#include "DaveAK_can/MCP2515.h"
#include "DaveAK_can/MCP2515_defs.h"

/*
 * Enum for state of the CAN bus.
 */
enum CAN_State { Normal, Error };

/*
 * Struct containing the filter info for the rx buffers.
 * Can specify mask and 2 filters for RXB0, and mask and
 * 5 filters for RXB1.
 */
struct FilterInfo {
	uint16_t RXM0; // mask 0 (RXB0)
	uint16_t RXM1; // mask 1 (RXB1)
	uint16_t RXF0; // filter 0 (RXB0)
	uint16_t RXF1; // filter 1 (RXB0)
	uint16_t RXF2; // filter 2 (RXB1)
	uint16_t RXF3; // filter 3 (RXB1)
	uint16_t RXF4; // filter 4 (RXB1)
	uint16_t RXF5; // filter 5 (RXB1)
};

/*
 * Class for handling CAN I/O operations using the
 * MCP2515 CAN controller.
 */
class CAN_IO {
public:
	const uint16_t BUFFER_SIZE = 8; // constant for the frame buffer size
	Frame buffer[BUFFER_SIZE]; // frame buffer
	uint16_t buffer_index; // location of first unfilled buffer
	Can_State status; // status of the CAN interface

	/*
	 * Constructor. Creates a MCP2515 object using
	 * the given pins.
	 */
	CAN_IO(byte CS_pin, byte INT_pin);

	/*
	 * Initializes the CAN controller to desired settings,
	 * including read masks/filters. All types of interrupt
	 * are enabled.
	 */
	void setup(FilterInfo& filters);

	/*
	 * Invoked when the interrupt pin is pulled low. Handles
	 * errors or reads messages, determined by the type of interrupt.
	 */
	void receive_CAN();

	/*
	 * Sends messages to the CAN bus via the controller.
	 */
	void send_CAN();
private:
	MCP2515 controller;

	/*
	 * Helper function for configuring the RX masks/filters.
	 * If first is true, sets the mask/filter for the first buffer;
	 * otherwise sets the second.
	 */
	void write_rx_filter(uint16_t mask, uint16_t filter, bool first);

	/*
	 * Given a 12-bit mask/filter with bits 0 1 2 ... 11,
	 * extracts bits 1-8.
	 */
	uint8_t first_byte(uint16_t value);

	/*
	 * Given a 12-bit mask/filter with bits 0 1 2 ... 11,
	 * extracts bits 9-11 and appends zeros to complete a byte.
	 */
	uint8_t second_byte(uint16_t value);
};

#endif