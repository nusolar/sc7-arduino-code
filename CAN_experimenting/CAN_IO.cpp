/*
 * CAN_IO.cpp
 * Implementation of CAN_IO class.
 */

#include "CAN_IO.h"
#include <SPI.h>

CAN_IO::CAN_IO(byte CS_pin, byte INT_p) :
errptr(0), INT_pin(INT_p), controller(CS_pin, INT_p)  {}

/*
 * Define global interrupt function
 */
void CAN_ISR()
{
  mainCAN->receiveCAN();
}
// Make sure to initialize the mainCAN pointer to 0 here.
CAN_IO* mainCAN = 0;

/*
 * Setup function for CAN_IO. Arguments are a FilterInfo struct and a pointer to a place to raise error flags.
 */
void CAN_IO::setup(const FilterInfo& filters, uint16_t* errorflags, bool isMainCan) {
	// SPI setup
	SPI.setClockDivider(10);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();

        // Set as main can
        if (isMainCan){  
            mainCAN = this;
            Serial.println((int)mainCAN,HEX);
        }
        
        attachInterrupt(INT_pin,CAN_ISR,LOW);
        
        // Attach error flag pointer
        errptr = errorflags;

	// init the controller
	int baudRate = controller.Init(1000, 20);
	if (baudRate <= 0) { // error
		*errptr |= CANERR_SETUP_BAUDFAIL;
	}

	// return controller to config mode
	if (!controller.Mode(MODE_CONFIG)) { // error
		*errptr |= CANERR_SETUP_MODEFAIL;
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

void CAN_IO::receiveCAN() {
	// read status of CANINTF register
	byte interrupt = controller.GetInterrupt();

	if (interrupt & MERRF) { // message error
		*errptr |= 0x01; // this needs to be a real value!
	}

	if (interrupt & WAKIF) { // wake-up interrupt

	}

	if (interrupt & ERRIF) { // error interrupt
		*errptr |= 0x02; // this needs to be a real value!
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
	controller.ResetInterrupt(INTALL); // reset all interrupts
}

void CAN_IO::sendCAN(Layout& layout) {
	controller.LoadBuffer(TXB0, layout.generate_frame());
	controller.SendBuffer(TXB0);

        // Errors????
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
