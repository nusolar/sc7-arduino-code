/*
 * DriverControls.ino
 * Contains code to run the driver controls
 * board for sc7.
 */

#include <stdint.h>
#include <Metro.h>
#include "CAN_IO.h"
#include "WDT.h"

//------------------------------CONSTANTS----------------------------//
// pins
const byte BRAKE_PIN = 0;
const byte ACCEL_PIN = 0;
const byte REGEN_PIN = 0;
const byte INTERRUPT_PIN = 0;
const byte CS_PIN = 0;

// CAN parameters
const uint16_t BAUD_RATE = 1000;
const byte FREQ = 16;
const uint16_t RXM0 = MASK_NONE;
const uint16_t RXM1 = MASK_NONE;
const uint16_t RXF0 = MASK_NONE;
const uint16_t RXF1 = MASK_NONE;
const uint16_t RXF2 = MASK_NONE;
const uint16_t RXF3 = MASK_NONE;
const uint16_t RXF4 = MASK_NONE;
const uint16_t RXF5 = MASK_NONE;

// timer intervals
const uint16_t MC_HB_INTERVAL = 1000; // motor controller heartbeat
const uint16_t SW_HB_INTERVAL = 1000; // steering wheel heartbeat
const uint16_t BMS_HB_INTERVAL = 1000; // bms heartbeat
const uint16_t MC_SEND_INTERVAL = 1000; // motor controller send packet
const uint16_t BMS_SEND_INTERVAL = 1000; // bms send packet
const uint16_t SW_SEND_INTERVAL = 1000; // steering wheel send packet
const uint16_t DC_SEND_INTERVAL = 1000; // driver controls heartbeat

//----------------------------TYPE DEFINITIONS------------------------//
/*
 * Enum to represet the possible gear states.
 */
enum GearState { BRAKE, FORWARD, REVERSE, REGEN };

/*
 * Struct to hold informations about the car state.
 */
struct CarState {
  // pedals
  bool brakeEngaged;
  bool regenEngaged;
  uint16_t accelPedal;
  
  // motor info
  uint16_t motorVelocity;
  uint16_t busCurrent;
  float carVelocity;
};

//----------------------------DATA/VARIABLES---------------------------//
// CAN variables
CAN_IO canController(CS_PIN, INTERRUPT_PIN, BAUD_RATE, FREQ);

// car state
CarState state;

// timers
Metro mcHbTimer(MC_HB_INTERVAL); // motor controller heartbeat
Metro swHbtimer(SW_HB_INTERVAL); // steering wheel heartbeat
Metro bmsHbtimer(BMS_HB_INTERVAL); // bms heartbeat
Metro mcSendTimer(MC_SEND_INTERVAL); // motor controller send packet
Metro swSendtimer(SW_SEND_INTERVAL); // steering wheel send packet
Metro bmsSendtimer(BMS_SEND_INTERVAL); // bms send packet
Metro dcSendtimer(DC_SEND_INTERVAL); // driver controls heartbeat
WDT wdt; // watchdog timer

// errors
uint16_t errorFlags;

//--------------------------HELPER FUNCTIONS--------------------------//
/*
 * Reads general purpose input and updates car state.
 */
void readInputs();

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN();

/*
 * Sets general purpose output according to car state.
 */
void writeOutputs();

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN();

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers();

/*
 * Checks the CAN controller and any other components for errors.
 * If errors exist, updates the error state.
 */
void checkErrors();

//--------------------------MAIN FUNCTIONS---------------------------//
void setup() {
  // setup pin I/O
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(REGEN_PIN, INPUT_PULLUP);
  
  // setup CAN
  CanFilterOpt filters;
  filters.setRBO(RXM0, RXF0, RXF1);
  filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canController.setup(filters, &errorFlags);
  
  // reset timers
  mcHbTimer.reset();
  swHbTimer.reset();
  bmsHbTimer.reset();
  mcSendTimer.reset();
  swSendTimer.reset();
  bmsSendTimer.reset();
  dcSendTimer.reset();
}

void loop() {
}
