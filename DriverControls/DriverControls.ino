/*
 * DriverControls.ino
 * Contains code to run the driver controls
 * board for sc7.
 */

#include <stdint.h>
#include <Metro.h>
#include <SPI.h>
#include "sc7-can-libinclude.h"
//#include "WDT.h"

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

// acceleration parameters
const uint16_t MAX_ACCEL_VOLTAGE = 1024;
const float MAX_ACCEL_RATIO = 0.8;

// errors
const uint16_t MC_TIMEOUT = 0x0001;
const uint16_t BMS_TIMEOUT = 0x0002;
const uint16_t SW_TIMEOUT = 0x0004;

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
  uint16_t accelRaw; // raw voltage reading
  
  // gearing
  GearState gear;
  
  // motor info
  float motorVelocity;
  float busCurrent;
  float carVelocity;
};

//----------------------------DATA/VARIABLES---------------------------//
// CAN variables
CAN_IO canControl(CS_PIN, INTERRUPT_PIN, BAUD_RATE, FREQ);

// car state
CarState state;

// timers
Metro mcHbTimer(MC_HB_INTERVAL); // motor controller heartbeat
Metro swHbTimer(SW_HB_INTERVAL); // steering wheel heartbeat
Metro bmsHbTimer(BMS_HB_INTERVAL); // bms heartbeat
Metro mcSendTimer(MC_SEND_INTERVAL); // motor controller send packet
Metro swSendTimer(SW_SEND_INTERVAL); // steering wheel send packet
Metro bmsSendTimer(BMS_SEND_INTERVAL); // bms send packet
Metro dcSendTimer(DC_SEND_INTERVAL); // driver controls heartbeat
//WDT wdt; // watchdog timer

// errors
uint16_t errorFlags;

//--------------------------HELPER FUNCTIONS--------------------------//
/*
 * Reads general purpose input and updates car state.
 */
void readInputs() {
  // read brake, regen
  state.brakeEngaged = (digitalRead(BRAKE_PIN) == LOW);
  state.regenEngaged = (digitalRead(REGEN_PIN) == LOW);
  
  // read accel pedal
  state.accelRaw = analogRead(ACCEL_PIN);
}

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN() {
  while (canControl.Available()) { // there are messages
    Frame& f = canControl.Read(); // read one message
    
    // determine source and update heartbeat timers
    if ((f.id & 0xF00) == BMS_HEARTBEAT_ID) { // source is bms
      bmsHbTimer.reset();
    }
    else if ((f.id & 0xF00) == MC_HEARTBEAT_ID) { // source is mc
      mcHbTimer.reset();
    }
    else if ((f.id & 0xF00) == SW_HEARTBEAT_ID) { // source is sw
      swHbTimer.reset();
    }
    
    // check for specific packets
    if (f.id == MC_BUS_STATUS_ID) { // motor controller bus status
      MC_BusStatus packet(f);
      state.busCurrent = packet.bus_current;
    }
    else if (f.id == MC_VELOCITY_ID) { // motor controller velocity
      MC_Velocity packet(f);
      state.motorVelocity = packet.motor_velocity;
      state.carVelocity = packet.car_velocity;
    }   
  }
}

/*
 * Sets general purpose output according to car state.
 */
void writeOutputs() {
  // nothing to do for now
}

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN() {
  // see if motor controller packet needs to be sent
  if (mcSendTimer.check()) { // ready to send drive command
    // setup velocity
    float velocity;
    switch (state.gear) {
    case FORWARD:
      velocity = 100;
      break;
    case REVERSE:
      velocity = -100;
      break;
    case REGEN:
    case BRAKE:
      velocity = 0;
      break;
    }
    
    // setup current
    float currentRatio = float(state.accelRaw) / MAX_ACCEL_VOLTAGE;
    if (currentRatio > MAX_ACCEL_RATIO) {
      currentRatio = MAX_ACCEL_RATIO;
    }
    
    // create and send packet
    DC_Drive packet(velocity, currentRatio);
    canControl.Send(packet, TXB0);
    
    // reset timer
    mcSendTimer.reset();
  }
}

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers() {
  if (mcHbTimer.check()) { // motor controller timeout
    errorFlags &= MC_TIMEOUT;
  }
  if (bmsHbTimer.check()) {
    errorFlags &= BMS_TIMEOUT;
  }
  if (swHbTimer.check()) {
    errorFlags &= SW_TIMEOUT;
  }
}

/*
 * Checks the CAN controller and any other components for errors.
 * If errors exist, updates the error state.
 */
void checkErrors() {
  // nothing to do for now
}

//--------------------------MAIN FUNCTIONS---------------------------//
void setup() {
  // setup pin I/O
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(REGEN_PIN, INPUT_PULLUP);
  
  // setup CAN
  CANFilterOpt filters;
  filters.setRB0(RXM0, RXF0, RXF1);
  filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canControl.Setup(filters, &errorFlags);
  
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
  // clear watchdog timer
  
  // read GPIO
  readInputs();
  
  // read CAN
  readCAN();
  
  // write GPIO
  writeOutputs();
  
  // write CAN
  writeCAN();
  
  // check timers
  checkTimers();
  
  // check errors
  checkErrors();
}
