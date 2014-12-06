/*
 * CAN_IO.cpp
 * Implementation of CAN_IO class.
 */

#include "CAN_IO.h"
#include <SPI.h>

CAN_IO::CAN_IO(byte CS_pin, byte INT_pin) :
controller(CS_pin, INT_pin) {}

void CAN_IO::setup(FilterInfo& filters, byte& errors) {
	// SPI setup
	SPI.setClockDivider(10);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();

	// init the controller
	int baudRate = controller.Init(1000, 20);
	if (baudRate <= 0) { // error
		errors |= 0x04;
	}

	// return controller to config mode
	if (!controller.Mode(MODE_CONFIG)) { // error
		errors |= 0x08;
	}

	// disable interrupts we don't care about
	controller.Write(CANINTE, 0xA3); // 10100011

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
	if (!controller.Mode(MODE_NORMAL)) { // error
	}
}

void CAN_IO::receive_CAN(uint8_t& errflags) {
	// read status of CANINTF register
	byte interrupt = controller.GetInterrupt();

	if (interrupt & MERRF) { // message error
		errflags |= 0x01; // this needs to be a real value!
	}

	if (interrupt & WAKIF) { // wake-up interrupt

	}

	if (interrupt & ERRIF) { // error interrupt
		errflags |= 0x02; // this needs to be a real value!
	}

	if (interrupt & TX2IF) { // transmit buffer 2 empty

	}

	if (interrupt & TX1IF) { // transmit buffer 1 empty

	}

	if (interrupt & TX0IF) { // transmit buffer 0 empty

	}

	if (interrupt & RX1IF) { // receive buffer 1 full
		buffer.enqueue(controller.ReadBuffer(RXB1));
	}

	if (interrupt & RX0IF) { // receive buffer 0 full
		buffer.enqueue(controller.ReadBuffer(RXB0));
	}

	// clear interrupt
	controller.ResetInterrupt(0xFF); // reset all interrupts
}

void CAN_IO::send_CAN(Layout& layout) {
	controller.LoadBuffer(TXB0, layout.generate_frame());
	controller.SendBuffer(TXB0);
}

void CAN_IO::write_rx_filter(uint8_t address, uint16_t data) {
	uint8_t bytes[2] = { first_byte(data), second_byte(data) };
	controller.Write(address, bytes, 2);
}

uint8_t CAN_IO::first_byte(uint16_t value) {
	return (value >> 3) & 0x00FF;
}

uint8_t CAN_IO::second_byte(uint16_t value) {
	return (value << 5) & 0x00E0;
}
