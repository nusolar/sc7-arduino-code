/*
 * CAN_IO.cpp
 * Implementation of CAN_IO class.
 */

#include "CAN_IO.h"

CAN_IO::CAN_IO(byte CS_pin, byte INT_pin): 
	controller(CS_pin, INT_pin), 
	status(Normal),
	buffer_index(0) {}

void CAN_IO::setup(FilterInfo& filters) {
	// init the controller
	int baudRate = controller.Init(25, 125); // not sure what this really does or what the args should be

	if (baudRate <= 0) { // error
		status = Error;
	}

	// return controller to config mode
	bool success;
	success = controller.Mode(MODE_CONFIG);
	if (!success) { // error
		status = Error;
	}

	// config RX masks/filters
	write_rx_filter(RXM0SIDH, filters.RXM0);
	write_rx_filter(RXM1SIDH, filters.RXM1);
	write_rx_filter(RXF0SIDH, filters.RXF0);
	write_rx_filter(RXF1SIDH, filters.RXF1);
	write_rx_filter(RXF2SIDH, filters.RXF2);
	write_rx_filter(RXF3SIDH, filters.RXF3);
	write_rx_filter(RXF4SIDH, filters.RXF4);
	write_rx_filter(RXF5SIDH, filters.RXF5);

	// return controller to normal mode
	success = controller.Mode(MODE_NORMAL);
	if (!success) { // error
		status = Error;
	}

	// how should errors be detected/handled?
	// what else do we need to do?
}

void CAN_IO::receive_CAN() {
	// read status of CANINTF register
	byte interrupt = controller.GetInterrupts();

	if (interrupt & MERRF) { // message error
		status = Error;
	}

	if (interrupt & WAKIF) { // wake-up interrupt

	}

	if (interrupt & ERRIF) { // error interrupt
		status = Error;
	}

	if (interrupt & TX2IF) { // transmit buffer 2 empty

	}

	if (interrupt & TX1IF) { // transmit buffer 1 empty

	}

	if (interrupt & TX0IF) { // transmit buffer 0 empty

	}

	if (interrupt & RX1IF) { // receive buffer 1 full
		if (buffer_index < BUFFER_SIZE) { // buffer space left
			buffer[buffer_index] = controller.ReadBuffer(RXB1);
			buffer_index++;
		}
	}

	if (interrupt & RX0IF) { // receive buffer 0 full
		if (buffer_index < BUFFER_SIZE) { // buffer space left
			buffer[buffer_index] = controller.ReadBuffer(RXB0);
			buffer_index++;
		}
	}

	// clear interrupt
	controller.ResetInterrupts(0xFF); // reset all interrupts
}

void CAN_IO::send_CAN(Layout& layout) {
	LoadBuffer(TXB0, layout.generate_frame());
	SendBuffer(TXB0);
}

void CAN_IO::write_rx_filter(uint8_t address, uint16_t data) {
	// write mask to registers
	uint8_t bytes[2] = {first_byte(data), second_byte(data)};
	controller.Write(address, bytes, 2);

	// how do we disable buffers that we don't want?
}

uint8_t CAN_IO::first_byte(uint16_t value) {
	value &= 0x07F0;
	value >>= 5;
	return value;
	//return (value - (value % 16)) / 16;
	// how does conversion to byte happen?
}

uint8_t CAN_IO::second_byte(uint16_t value) {
	value &= 0x000F;
	value <<= 4;
	return value;
	//return (value % 16) * 16;
	// how does conversion to byte happen?
}