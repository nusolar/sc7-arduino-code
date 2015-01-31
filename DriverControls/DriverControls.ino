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
const byte ERROR_PIN = 13;

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

// timer intervals (all in ms)
const uint16_t MC_HB_INTERVAL = 1000; // motor controller heartbeat
const uint16_t SW_HB_INTERVAL = 1000; // steering wheel heartbeat
const uint16_t BMS_HB_INTERVAL = 1000; // bms heartbeat
const uint16_t MC_SEND_INTERVAL = 1000; // motor controller send packet
const uint16_t BMS_SEND_INTERVAL = 1000; // bms send packet
const uint16_t SW_SEND_INTERVAL = 1000; // steering wheel send packet
const uint16_t DC_SEND_INTERVAL = 1000; // driver controls heartbeat
const uint16_t WDT_INTERVAL = 5000; // watchdog timer

// drive parameters
const uint16_t MAX_ACCEL_VOLTAGE = 1024; // max possible accel voltage
const float MAX_ACCEL_RATIO = 0.8; // maximum safe accel ratio
const uint16_t MAX_REGEN_VOLTAGE = 1024; // max possible regen voltage
const float MAX_REGEN_RATIO = 1.0; // maximum safe regen ratio
const float MIN_PEDAL_TOLERANCE = 0.05; // anything less is basically zero
const float FORWARD_VELOCITY = 100.0f; // velocity to use if forward
const float REVERSE_VELOCITY = -100.0f; // velocity to use if reverse

// driver control errors
const uint16_t MC_TIMEOUT = 0x0001; // motor controller timed out
const uint16_t BMS_TIMEOUT = 0x0002; // bms timed out
const uint16_t SW_TIMEOUT = 0x0004; // sw timed out
const uint16_t SW_BAD_GEAR = 0x0008; // bad gearing from steering wheel

//----------------------------TYPE DEFINITIONS------------------------//
/*
 * Enum to represet the possible gear states.
 */
enum GearState { BRAKE, FORWARD, REVERSE, REGEN };

/*
 * Struct to hold informations about the car state.
 */
struct CarState {
  // RAW DATA
  // pedals
  bool brakeEngaged;
  uint16_t regenRaw; // raw voltage reading from regen input
  uint16_t accelRaw; // raw voltage reading from accel input
  
  // steering wheel info
  bool gearForward; // true if gear is forward, false if reverse
  bool hornEngaged; // true if horn is engaged by driver
  
  // motor info
  float motorVelocity;
  float carVelocity;
  float busCurrent;
  
  // bms info
  float bmsPercentSOC; // percent state of charge of bms
  
  // DERIVED DATA
  // pedals
  float accelRatio; // ratio of accel voltage to max voltage
                    // constrained for safety
  float regenRatio; // ratio of regen voltage to max voltage
                    // constrained for safety
                    
  // gearing
  GearState gear; // brake, foward, reverse, regen
  
  // errors
  uint16_t canErrorFlags; // keep track of errors with CAN bus
  uint16_t dcErrorFlags; // keep track of other errors
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

//--------------------------HELPER FUNCTIONS--------------------------//
/*
 * Reads general purpose input and updates car state.
 */
void readInputs() {
  // read brake
  state.brakeEngaged = (digitalRead(BRAKE_PIN) == LOW);
  
  // read accel and regen pedals pedal
  state.accelRaw = analogRead(ACCEL_PIN);
  state.regenRaw = analogRead(REGEN_PIN);
}

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN() {
  while (canControl.Available()) { // there are messages
    Frame& f = canControl.Read(); // read one message
    
    // determine source and update heartbeat timers
    if ((f.id & MASK_Sx00) == BMS_HEARTBEAT_ID) { // source is bms
      bmsHbTimer.reset();
    }
    else if ((f.id & MASK_Sx00) == MC_HEARTBEAT_ID) { // source is mc
      mcHbTimer.reset();
    }
    else if ((f.id & MASK_Sx00) == SW_HEARTBEAT_ID) { // source is sw
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
    else if (f.id == BMS_SOC_ID) { // bms state of charge
      BMS_SOC packet(f);
      state.bmsPercentSOC = packet.percent_SOC;
    }
    else if (f.id == SW_DATA_ID) { // steering wheel data
      /*
      SW_Data packet(f);
      
      // get gear info
      if (packet.gear_forward == packet.gear_reverse) { // bad gear
        state.dcErrorFlags |= SW_BAD_GEAR; // update error flags
      }
      else { // gear ok
        state.gearForward = packet.gear_forward;
      }
      
      // get other info
      state.hornEngaged = packet.horn;
      */
    }  
  }
}

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers() {
  if (mcHbTimer.check()) { // motor controller timeout
    state.dcErrorFlags |= MC_TIMEOUT;
  }
  if (bmsHbTimer.check()) {
    state.dcErrorFlags |= BMS_TIMEOUT;
  }
  if (swHbTimer.check()) {
    state.dcErrorFlags |= SW_TIMEOUT;
  }
}

/*
 * Checks the CAN controller and any other components for errors.
 * If errors exist, updates the error state.
 */
void checkErrors() {
  // nothing to do for now
}

/*
 * Process information read from GPIO and CAN and updates
 * the car state accordingly.
 */
void updateState() {
  // calculate accel, regen ratios
  state.accelRatio = constrain(float(state.accelRaw)/MAX_ACCEL_VOLTAGE,
                               0.0f,
                               MAX_ACCEL_RATIO);
  state.regenRatio = constrain(float(state.regenRaw)/MAX_REGEN_VOLTAGE,
                               0.0f,
                               MAX_REGEN_RATIO);
  
  // update gear state
  if (state.brakeEngaged) { // brake engaged, overrides all other gears
    state.gear = BRAKE;
  }
  else if (state.accelRatio > MIN_PEDAL_TOLERANCE) { // accel engaged
                                                     // overrides regen
    state.gear = (state.gearForward ? FORWARD : REVERSE);
  }
  else if (state.regenRatio > MIN_PEDAL_TOLERANCE) { // regen engaged
    state.gear = REGEN;
  }
  // if no pedal engaged, gear state unchanged
}

/*
 * Sets general purpose output according to car state.
 */
void writeOutputs() {
  // check for errors
  if (state.canErrorFlags || state.dcErrorFlags) { // error exists
    digitalWrite(ERROR_PIN, HIGH);
  }
  else { // no error
    digitalWrite(ERROR_PIN, LOW);
  }
}

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN() {
  // see if motor controller packet needs to be sent
  if (mcSendTimer.check()) { // ready to send drive command
    // determine velocity, current
    float velocity, current;
    switch (state.gear) {
    case FORWARD:
      velocity = FORWARD_VELOCITY;
      current = state.accelRatio;
      break;
    case REVERSE:
      velocity = REVERSE_VELOCITY;
      current = state.accelRatio;
      break;
    case REGEN:
      velocity = 0;
      current = state.regenRatio;
      break;
    case BRAKE:
      velocity = 0;
      current = 0;
      break;
    }
    
    // create and send packet
    DC_Drive packet(velocity, current);
    canControl.Send(packet, TXB0);
    
    // reset timer
    mcSendTimer.reset();
  }
  
  // check if driver controls heartbeat needs to be sent
  if (dcSendTimer.check()) {
    // create and send packet
    DC_Heartbeat packet(0,0);
    canControl.Send(packet, TXB0);
    
    // reset timer
    dcSendTimer.reset();
  }
}

//--------------------------MAIN FUNCTIONS---------------------------//
void setup() {
  // setup pin I/O
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(ERROR_PIN, OUTPUT);
  
  // setup CAN
  CANFilterOpt filters;
  filters.setRB0(RXM0, RXF0, RXF1);
  filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canControl.Setup(filters, &state.canErrorFlags);
  
  // init car state
  state = {}; // init all members to 0
  state.gear = BRAKE;
  
  // set the watchdog timer interval
  WDT_Enable(WDT, 0x2000 | WDT_INTERVAL| ( WDT_INTERVAL << 16 ));
  
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
  
  // check timers
  checkTimers();
  
  // check errors
  checkErrors();
  
  // process information that was read
  updateState();
  
  // write GPIO
  writeOutputs();
  
  // write CAN
  writeCAN();
}
