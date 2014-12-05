/*
 * CAN_IO.h
 * Contains class definition for CAN_IO library.
 */

#ifndef CAN_IO_h
#define CAN_IO_h

#include <stdint.h>
#include "MCP2515.h"
#include "MCP2515_defs.h"
#include "Layouts.h"
#include "RX_Queue.h"

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

       /* FilterInfo() : RXM0(0), RXM1(0) {} // Initialize to no masking

        void setRB0(uint16_t m0, uint16_t f0, uint16_t f1)
        {
          RXM0 = m0;
          RXF0 = f0;
          RXF1 = f1;
        }
        
        void setRB1(uint16_t m1, uint16_t f2, uint16_t f3, uint16_t f4, uint16_t f5)
        {
          RXM1 = m1;
          RXF2 = f2;
          RXF3 = f3;
          RXF4 = f4;
          RXF5 = f5;
        }*/
};

/*
 * Define Extra can errors besides those defined in MCP2515_defs.
  #define RX0IF                  0x01
  #define RX1IF                  0x02
  #define TX0IF                  0x04
  #define TX1IF                  0x08
  #define TX2IF                  0x10
  #define ERRIF                  0x20
  #define WAKIF                  0x40
  #define MERRF                  0x80 
 */
  #define CANERR_SETUP_BAUDFAIL   0x0100 // Failed to set baud rate properly during setup
  #define CANERR_SETUP_MODEFAIL   0x0200 // Failed to switch modes
  #define CANERR_BUFFER_FULL      0x0400 // Local buffer is full
  #define CANERR_MCPBUF_FULL      0x0800 // MCP2515 is reporting buffer overflow errors

/*
 * Class for handling CAN I/O operations using the
 * MCP2515 CAN controller.
 */
class CAN_IO {
public:
	RX_Queue<8> buffer;

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
	void setup(const FilterInfo& filters, uint16_t* errorflags, bool isMainCan);

	/*
	 * Invoked when the interrupt pin is pulled low. Handles
	 * errors or reads messages, determined by the type of interrupt.
	 */
	void receiveCAN();

	/*
	 * Sends messages to the CAN bus via the controller.
	 */
	void sendCAN(Layout& layout);
        
        // Will be used to set up receive filters in a cleaner way.
        /*void set_RB1_filters(uint16_t mask,uint16_t filter,uint16_t filter);
        void set_RB0_filters(uint16_tmask, uint16_t f1, uint16_t filter, uint16_t filter, uint16_t filter);*/
      
	/*
	 * Returns true if the RX buffer is not empty.
	 */
	bool messageExists()
	{
		return !buffer.is_empty();
	}
        
    MCP2515 controller;
private:
        uint16_t* errptr;
        byte      INT_pin;
	/*
	 * Helper function for configuring the RX masks/filters.
	 * If first is true, sets the mask/filter for the first buffer;
	 * otherwise sets the second.
	 */
	void write_rx_filter(uint8_t address, uint16_t);

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

/*
 * Declare a pointer to the main can_io instance
 */
extern CAN_IO* mainCAN;

#endif
