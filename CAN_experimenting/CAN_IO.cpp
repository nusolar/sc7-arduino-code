/*
 * CAN_IO.cpp
 * Implementation of CAN_IO class.
 */

#include "CAN_IO.h"
#include <SPI.h>

CAN_IO::CAN_IO(byte CS_pin, byte INT_pin): 
	controller(CS_pin, INT_pin), 
	buffer_index(0),
        messageavailable(false) {}

void CAN_IO::setup(FilterInfo& filters) {
	// SPI setup
	SPI.setClockDivider(10);
  	SPI.setDataMode(SPI_MODE0);
  	SPI.setBitOrder(MSBFIRST);
  	SPI.begin();
  	
	// init the controller
	int baudRate = controller.Init(125, 25);

	if (baudRate <= 0) { // error
	}

	// return controller to config mode
	bool success;
	success = controller.Mode(MODE_CONFIG);
	if (!success) { // error
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
	success = controller.Mode(MODE_NORMAL);
	if (!success) { // error
	}
}

void CAN_IO::receive_CAN(uint8_t& errflags) {
	// read status of CANINTF register
	byte interrupt = controller.GetInterrupt();

	if (interrupt & MERRF) { // message error
		errflags = 0x01; // this needs to be a real value!
	}

	if (interrupt & WAKIF) { // wake-up interrupt

	}

	if (interrupt & ERRIF) { // error interrupt
		errflags = 0x02; // this needs to be a real value!
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
			//buffer_index++;
                        messageavailable = true;
		}
	}

	if (interrupt & RX0IF) { // receive buffer 0 full
		if (buffer_index < BUFFER_SIZE) { // buffer space left
			buffer[buffer_index] = controller.ReadBuffer(RXB0);
			//buffer_index++;
                        messageavailable = true; 
		}
	}

	// clear interrupt
	controller.ResetInterrupt(0xFF); // reset all interrupts
}

void CAN_IO::send_CAN(Layout& layout) {
	controller.LoadBuffer(TXB0, layout.generate_frame());
	controller.SendBuffer(TXB0);
}

void CAN_IO::write_rx_filter(uint8_t address, uint16_t data) {
	uint8_t bytes[2] = {first_byte(data), second_byte(data)};
	controller.Write(address, bytes, 2);
}

uint8_t CAN_IO::first_byte(uint16_t value) {
	return (value >> 3) & 0x00FF;
}

uint8_t CAN_IO::second_byte(uint16_t value) {
	return (value << 5) & 0x00E0;
}
