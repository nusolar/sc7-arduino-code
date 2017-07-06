/*
 * DriverControls.ino
 * Contains code to run the driver controls
 * board for sc7.
 */

#include <CAN_IO.h>
#include <stdint.h>
#include <Metro.h>
#include <SPI.h>
#include "steering-defs.h"
#include <OneWire.h>

//------------------------------CONSTANTS----------------------------//
// debugging
const bool DEBUG       = true;    // change to true to output debug info over serial
byte       debugStep   = 0;       // It's too slow to send out all the debug over serial at once, so we split it into 3 steps.
const int  SERIAL_BAUD = 115200;  // baudrate for serial (maximum)

// pins
const byte IGNITION_PIN   = 52;
const byte BRAKE_PIN      = 9;
const byte ACCEL_PIN      = A0;
const byte REGEN_PIN      = A1;
const byte INTERRUPT_PIN  = 7;
const byte CS_PIN         = 3;
const byte HORN_PIN       = 2;
const byte RIGHT_TURN_PIN = 11;
const byte LEFT_TURN_PIN  = 12;
const byte HEADLIGHT_PIN  = 10;
const byte BRAKELIGHT_PIN = 13;
const byte BOARDLED       = 13;
const byte BMS_STROBE_PIN = 48;

// CAN parameters
const uint16_t BAUD_RATE = 1000;
const byte     FREQ      = 16;

const uint16_t RXM0      = MASK_Sxxx;
const uint16_t RXF0      = 0; // Match any steering_wheel packet (because mask is Sx00)
const uint16_t RXF1      = BMS_VOLT_CURR_ID; // Can't put 0 here, otherwise it will match all packets that start with 0.

const uint16_t RXM1      = MASK_Sxxx;
const uint16_t RXF2      = SW_DATA_ID;
const uint16_t RXF3      = BMS_STATUS_EXT_ID;
const uint16_t RXF4      = MC_VELOCITY_ID;
const uint16_t RXF5      = MC_BUS_STATUS_ID; //Also kinda useless right now since we read BMS current.

// timer intervals (all in ms)
const uint16_t MC_HB_INTERVAL    = 1000;  // motor controller heartbeat
const uint16_t SW_HB_INTERVAL    = 5000;  // steering wheel heartbeat
const uint16_t BMS_HB_INTERVAL   = 1000;  // bms heartbeat
const uint16_t DC_DRIVE_INTERVAL = 150;   // drive command packet
const uint16_t DC_INFO_INTERVAL  = 150;   // driver controls info packet
const uint16_t DC_HB_INTERVAL    = 1000;  // driver controls heartbeat packet
const uint16_t DC_POWER_INTERVAL = 1000;  // driver controls power packet
const uint16_t WDT_INTERVAL      = 5000;  // watchdog timer
const uint16_t TOGGLE_INTERVAL   = 500;   // toggle interval for right/left turn signals, hazards
const uint16_t DEBUG_INTERVAL    = 333;   // interval for debug calls output
const uint16_t TEMP_CONV_INTERVAL = 800;   // interval for temp sense conversion
const uint16_t TEMP_READ_INTERVAL = 100;   // interval for temp sense reading

// drive parameters
const uint16_t MAX_ACCEL_VOLTAGE   = 1024;    // max possible accel voltage
const float    MAX_ACCEL_RATIO     = 0.8;     // maximum safe accel ratio
const uint16_t MAX_REGEN_VOLTAGE   = 1024;    // max possible regen voltage
const float    MAX_REGEN_RATIO     = 1.0;     // maximum safe regen ratio
const float    MIN_PEDAL_TOLERANCE = 0.07;    // anything less is basically zero
const uint8_t  MIN_BRAKE_COUNT     = 10;      // Minimum # of LOW reads on the brake pin it takes to enable the brake state (for de-noising)
const float    FORWARD_VELOCITY    = 100.0f;  // velocity to use if forward
const float    REVERSE_VELOCITY    = -100.0f; // velocity to use if reverse
const float    MAX_MOTOR_CURRENT   = 1.0;     // sent to the motor to set the maximum amount of current to draw
const float    GEAR_CHANGE_CUTOFF  = 5.0f;    // cannot change gear unless velocity is below this threshold
const float    M_PER_SEC_TO_MPH    = 2.237f;  // conversion factor from m/s to mph
const int      MAX_CAN_PACKETS_PER_LOOP = 10; // Maximum number of receivable CAN packets per loop
const bool     ENABLE_REGEN        = false;   // flag to enable/disable regen
const uint16_t DC_ID               = 0x00C7;  // For SC7
const uint16_t DC_SER_NO           = 0x0042;  // Don't panic!
const float    CHARGE_TEMP         = 45.0;    // battery temp threshold when current is positive
const float    DISCHARGE_TEMP      = 60.0;    // battery temp threshold when current is negative

// steering wheel parameters
const byte NEUTRAL_RAW = 0x03;
const byte FORWARD_RAW = 0x02;
const byte REVERSE_RAW = 0x01;
const byte SW_ON_BIT   = 0;        // value that corresponds to on for steering wheel data
const bool NO_STEERING = false;    // set to true to read light, horn, gear controls directly from board (also automatically enabled when comm with SW is lost).

// BMS parameters
const float MAX_CURRENT_THRESH    = 68000; // mA
const float CONTINUOUS_CURRENT_THRESH   = 50000; // current may exceed this value no more than 7 times in 50 ms
const float CHARGE_CURRENT_THRESH       = 36000; // current limit for charging (36.4 to be exact)
const int   CURRENT_BUFFER_SIZE         = 10;    // number of current values from BMS stored
const int   OVERCURRENTS_ALLOWED        = 7;     // max number of overcurrent values allowed before trip

// driver control errors
const uint16_t MC_TIMEOUT  = 0x01; // motor controller timed out
const uint16_t BMS_TIMEOUT = 0x02; // bms timed out
const uint16_t SW_TIMEOUT  = 0x04; // sw timed out
const uint16_t SW_BAD_GEAR = 0x08; // bad gearing from steering wheel
const uint16_t BMS_OVER_CURR = 0x10; // Detected BMS overcurrent, tripped car.
const uint16_t RESET_MCP2515 = 0x20; // Had to reset the MCP2515

// bms status flags
const uint32_t OVERVOLTAGE = 0x01;
const uint32_t UNDERVOLTAGE = 0x02;
const uint32_t DRIVERCONTROLSERROR = 0x20;

//temp sensor object
OneWire ds(50);

//----------------------------TYPE DEFINITIONS------------------------//
/*
 * Enum to represet the possible gear states.
 */
enum GearState { REVERSE = 0x01, FORWARD = 0x02, NEUTRAL = 0x03, BRAKE = 0x04, REGEN = 0x05 };

/*
 * Enum to represent   states
 */
enum IgnitionState { Ignition_Start = 0x0040, Ignition_Run = 0x0020, Ignition_Park = 0x0010 };

/*
 * Struct to hold informations about the car state.
 */
struct CarState {
  // RAW DATA
  // pedals
  bool brakeEngaged;
  unsigned long brakeCountRaw; // internal counter use for de-noising
  uint16_t regenRaw; // raw voltage reading from regen input
  uint16_t accelRaw; // raw voltage reading from accel input
  
  // steering wheel info (if we need to go to digital controls on the driver box)
  bool altSteeringEnable; // true if we are using alternate steering
  byte gearRaw;       // 00 = neutral, 01 = forward, 10 = reverse, 11 = undefined
  bool horn;          // true if driver wants horn on (no toggle)
  bool headlights;    // true if driver wants headlights on (no toggle)
  bool rightTurn;     // true if driver wants right turn on (no toggle)
  bool leftTurn;      // true if driver wants left turn on (no toggle)
  bool hazards;       // true if driver wants hazards on (no toggle)
  //bool cruiseCtrl;    // true if driver wants cruise control on
  
  // motor info
  float motorVelocity;  // rotational speed of motor (rpm)
  float carVelocity;    // velocity of car (mph)
  int16_t busCurrent;
  
  // bms info
  float bmsPercentSOC;                      // percent state of charge of bms
  float bmsCurrent;                         // Current reading from BMS (negative is out of the batteries)
  int currentBufferIndex;                   // points to index of next value to be written
  float currentBuffer[CURRENT_BUFFER_SIZE]; // buffer to hold most recent current values from BMS
  int numOvercurrents;                      // number of current values in buffer over threshold
  bool updateCurrentBufferRequested;        // set to true when a BMS current packet comes in.
  
  // ignition
  IgnitionState ignitionRaw;                // ingition requested by ignition switch
  
  // debugging
  bool wasReset;       // true on initilization, false otherwise
  byte canstat_reg;    // holds value of canstat register on the MCP2515
  int SW_timer_reset_by;
  
  // DERIVED DATA
  // pedals
  float accelRatio; // ratio of accel voltage to max voltage, constrained for safety
  float regenRatio; // ratio of regen voltage to max voltage, constrained for safety
                    
  // motor current
  float accelCurrent;   // current to use if in drive/reverse
  float regenCurrent;   // current to use if in regen
                    
  // cruise control
  //bool cruiseCtrlOn;     // true if cruise control should be active
  //bool cruiseCtrlPrev;   // prev value of cruiseCtrl
  //float cruiseCtrlRatio; // pedal ratio to use if cruise control active
                    
  // gearing and ignition
  GearState gear;         // brake, foward, reverse, regen, neutral
  IgnitionState ignition; // start, run, park
  bool tripped;           // flag for overcurrent  
  
  // outputs
  bool rightTurnOn;   // true if we should turn rt signal on
  bool leftTurnOn;    // true if we should turn lt signal on
  bool bmsStrobeOn;   // true if we should turn on the bms strobe (for tripping)

  // errors
  uint16_t canErrorFlags; // keep track of errors with CAN bus
  uint16_t dcErrorFlags;  // keep track of other errors
  uint32_t bmsErrorFlags; // keep track of error flags from BMS (for bms led strobe)

  //temperature
  float celsius[26];
  float fahrenheit[26];
  float maxTemp;
  float avgTemp;
  float previousValidTemp;
};

//----------------------------DATA/VARIABLES---------------------------//
// CAN variables
CAN_IO canControl(CS_PIN, INTERRUPT_PIN, BAUD_RATE, FREQ);

// car state
CarState state;

// timers
Metro mcHbTimer(MC_HB_INTERVAL);       // motor controller heartbeat
Metro swHbTimer(SW_HB_INTERVAL);       // steering wheel heartbeat
Metro bmsHbTimer(BMS_HB_INTERVAL);     // bms heartbeat
Metro dcDriveTimer(DC_DRIVE_INTERVAL); // motor controller send packet
Metro dcInfoTimer(DC_INFO_INTERVAL);   // steering wheel send packet
Metro dcHbTimer(DC_HB_INTERVAL);       // driver controls heartbeat
Metro hazardsTimer(TOGGLE_INTERVAL);   // timer for toggling hazards
Metro rightTurnTimer(TOGGLE_INTERVAL); // timer for toggling right turn signal
Metro leftTurnTimer(TOGGLE_INTERVAL);  // timer for toggling left turn signal
Metro debugTimer(DEBUG_INTERVAL);      // timer for debug output over serial
Metro dcPowerTimer(DC_POWER_INTERVAL);
Metro tempConvertTimer(TEMP_CONV_INTERVAL);   // timer for issuing convert command to temp sensors
Metro tempReadTimer(TEMP_READ_INTERVAL);      // timer for reading the values from temp sensorss

// debugging variables
long loopStartTime = 0;
long loopSumTime = 0;
int loopCount = 0;

//temp sense
byte tempCount;    // keeps track of how many times readTempSensor() has run
byte addr[8];

byte i;            // just used as a counter variable to print out ALL temperatures in the serial debug section
float j;         // used as a counter when calculating Avg and Max temperatures

//--------------------------HELPER FUNCTIONS--------------------------//
/*
 * Reads general purpose input and updates car state.
 */
void readInputs() {
  // read brake
  if (digitalRead(BRAKE_PIN) == LOW)
    state.brakeCountRaw = min(MIN_BRAKE_COUNT, state.brakeCountRaw + 1);
  else if (!state.brakeEngaged && state.brakeCountRaw >= 1)
    state.brakeCountRaw--;
  else 
    state.brakeCountRaw = 0;

  if (state.brakeCountRaw >= MIN_BRAKE_COUNT)
    state.brakeEngaged = true;
  else
    state.brakeEngaged = false;
  
  // read accel and regen pedals pedal
  state.accelRaw = analogRead(ACCEL_PIN);
  if (ENABLE_REGEN) { // will stay 0 if disabled
    state.regenRaw = analogRead(REGEN_PIN);
  }
  
  // read ignition switch
  state.ignitionRaw = digitalRead(IGNITION_PIN) == LOW ? Ignition_Start : Ignition_Park;
  digitalWrite(BOARDLED,digitalRead(IGNITION_PIN));
  
  // read steering wheel controls if steering wheel disconnected
  if (state.altSteeringEnable) {
    // read gear
    bool forward_on = (digitalRead(FORWARD_PIN) == LOW);
    bool neutral_on = (digitalRead(NEUTRAL_PIN) == LOW);
    if (neutral_on) {
      state.gearRaw = NEUTRAL_RAW;
    }
    else if (forward_on) {
      state.gearRaw = FORWARD_RAW;
    }
    else {
      state.gearRaw = REVERSE_RAW;
    }
    
    // read lights
    state.rightTurn = digitalRead(RIGHT_TURN_SW_PIN) == LOW;
    state.leftTurn = digitalRead(LEFT_TURN_SW_PIN) == LOW;
    state.headlights = digitalRead(HEADLIGHT_SW_PIN) == LOW;
    state.hazards = digitalRead(HAZARDS_SW_PIN) == LOW;
    
    // other
    state.horn = digitalRead(HORN_SW_PIN) == LOW;
  }
}

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN() {
  int safetyCount = 0;  
  while(canControl.Available() && safetyCount <= MAX_CAN_PACKETS_PER_LOOP) { // there are messages
    safetyCount++;                // Increment safety counter
    Frame& f = canControl.Read(); // read one message
    
    // determine source and update heartbeat timers
    if ((f.id & MASK_Sx00) == BMS_BASEADDRESS) { // source is bms
      bmsHbTimer.reset();
      state.dcErrorFlags &= ~BMS_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == MC_BASEADDRESS) { // source is mc
      mcHbTimer.reset();
      state.dcErrorFlags &= ~MC_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == SW_BASEADDRESS) { // source is sw
      swHbTimer.reset();
      state.dcErrorFlags &= ~SW_TIMEOUT; // clear flag
      state.SW_timer_reset_by = f.id;
    }
    
    // check for specific packets
    if (f.id == MC_BUS_STATUS_ID) { // motor controller bus status
      MC_BusStatus packet(f);
      state.busCurrent = packet.bus_current;
    }
    else if (f.id == MC_VELOCITY_ID) { // motor controller velocity
      MC_Velocity packet(f);
      state.motorVelocity = packet.motor_velocity;
      state.carVelocity = packet.car_velocity * M_PER_SEC_TO_MPH;
    }
    else if (f.id == BMS_SOC_ID) { // bms state of charge
      BMS_SOC packet(f);
      state.bmsPercentSOC = packet.percent_SOC;
    }
    else if (f.id == BMS_STATUS_EXT_ID) { //bms status
      BMS_Status_Ext packet(f);
      state.bmsErrorFlags = packet.flags;
    }
    else if (f.id == SW_DATA_ID) { // steering wheel data
      SW_Data packet(f);
      
      // read data
      state.gearRaw =    packet.gear;
      state.horn =      (packet.horn == SW_ON_BIT);
      state.rightTurn = (packet.rts == SW_ON_BIT);
      state.leftTurn =  (packet.lts == SW_ON_BIT);
      state.headlights =(packet.headlights == SW_ON_BIT);
      state.hazards =   (packet.hazards == SW_ON_BIT);
      
      // read cruise control
      //state.cruiseCtrlPrev = state.cruiseCtrl;
      //state.cruiseCtrl = (packet.cruisectrl == SW_ON_BIT);
    }
    else if (f.id == BMS_VOLT_CURR_ID) { // BMS Voltage Current Packet
      BMS_VoltageCurrent packet(f);
      state.bmsCurrent = packet.current;
      state.updateCurrentBufferRequested = true;
    }
    else if (DEBUG) {
      Serial.print("Unk. Packet: "); Serial.println(frameToString(f));
    }
  }
}

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers() {
  // check motor controller
  if (mcHbTimer.expired()) { // motor controller timeout
    state.dcErrorFlags |= MC_TIMEOUT; // set flag
  }
  
  // check bms
  if (bmsHbTimer.expired()) { // bms timeout
    state.dcErrorFlags |= BMS_TIMEOUT; // set flag
  }
  
  // check steering wheel
  if (swHbTimer.expired()) { // steering wheel timeout
    state.dcErrorFlags |= SW_TIMEOUT; // set flag
  }

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
  state.dcErrorFlags &= ~SW_BAD_GEAR; // clear bad gear flag
  if (state.brakeEngaged) { // brake engaged, overrides all other gears
    state.gear = BRAKE;
  }
  else if (state.regenRatio > MIN_PEDAL_TOLERANCE) {  // regen engaged
    state.gear = REGEN;
  }
  else { // accel or nothing engaged
    switch (state.gearRaw){
    case NEUTRAL_RAW:
      state.gear = NEUTRAL; // can always change to neutral
      break;
    case FORWARD_RAW:
      if (state.carVelocity > -GEAR_CHANGE_CUTOFF) { // going forward or velocity less than cutoff, gear switch ok
        state.gear = FORWARD;
      }
      break;
    case REVERSE_RAW:
      if (state.carVelocity < GEAR_CHANGE_CUTOFF) { // going backward or velocity less than cutoff, gear switch ok
        state.gear = REVERSE;
      }
      break;
    default: // unknown gear
      state.gear = NEUTRAL; // safe default gear?
      state.dcErrorFlags |= SW_BAD_GEAR; // flag bad gear
      break;
    }
  }
  
  // update lights state
  // check hazards
  if (state.hazards) { // hazards active
    if (hazardsTimer.check()) { // timer expired, toggle
      state.rightTurnOn = !state.rightTurnOn;
      state.leftTurnOn = state.rightTurnOn; // make sure they have same value
    }
  }
  else { // hazards inactive
    // check right turn signal
    if (state.rightTurn) { // right turn signal active
      if (rightTurnTimer.check()) { // timer expired, toggle
        state.rightTurnOn = !state.rightTurnOn;
      }
    }
    else { // right turn signal inactive
      state.rightTurnOn = false;
    }
    // check left turn signal
    if (state.leftTurn) { // left turn signal active
      if (leftTurnTimer.check()) { // timer expired, toggle
        state.leftTurnOn = !state.leftTurnOn;
      }
    }
    else { // left turn signal inactive
      state.leftTurnOn = false;
    }
  }

  // Trip if temp sensors are overtemp, temperature threshold depending on whether discharge or charge
  if(state.bmsCurrent < 0.0){   // negative current, discharge
    if(state.maxTemp >= DISCHARGE_TEMP){
      state.tripped = true;
      state.bmsStrobeOn = true; 
      Serial.print("Batteries are discharging, and the max temperature is ");
      Serial.println(state.maxTemp);
    }
  }      
  else{ // positive current, charge
    if(state.maxTemp >= CHARGE_TEMP){
      state.tripped = true;
      state.bmsStrobeOn = true;
      Serial.print("Batteries are charging, and the max temperature is ");
      Serial.println(state.maxTemp);
    }
  }

  // bms strobe light trip conditions
  if(state.bmsErrorFlags & BMS_Status_Ext::F_OVERVOLTAGE){
    state.tripped = true;
    state.bmsStrobeOn = true;
  }
  if(state.bmsErrorFlags & BMS_Status_Ext::F_UNDERVOLTAGE){
    state.tripped = true;
    state.bmsStrobeOn = true;
  }
  if(state.bmsErrorFlags & BMS_Status_Ext::F_DRVCTRLSLOST){
    state.tripped = true;
    state.bmsStrobeOn = true;
  }
  
  // update cruise control state
  //if (!state.cruiseCtrlPrev && state.cruiseCtrl) { // cruise control just switched on
  //  state.cruiseCtrlOn = true;
  //  state.cruiseCtrlRatio = state.accelRatio;
  //}
  //if (state.gear == BRAKE || state.gear == REGEN ||
  //         !state.cruiseCtrl) { // regen pedal pressed or cruise control switched off
  //  state.cruiseCtrlOn = false;
  //  state.cruiseCtrlPrev = true; // covers edge case where cruise control goes from off to on, 
  //                               // then brake is pressed and released before next packet comes in
  //                               // without this line, cruise control would come back on when brake is released
  //}
  
  // update current values to be sent to motor controller
  state.regenCurrent = state.regenRatio < MIN_PEDAL_TOLERANCE ? 
                       0 : 
                       state.regenRatio;
  state.accelCurrent = state.accelRatio < MIN_PEDAL_TOLERANCE ? 
                       0 : 
                       state.accelRatio; 
  
  //if (state.cruiseCtrlOn) { // cruise control on, make sure that current doesn't fall below cruise control ratio
  //  state.accelCurrent = state.accelRatio < state.cruiseCtrlRatio ? 
  //                       state.cruiseCtrlRatio : 
  //                       state.accelRatio;
  //}
  //else { // no cruise control, take pedal value (or 0 if below tolerance)
  //  state.accelCurrent = state.accelRatio < MIN_PEDAL_TOLERANCE ? 
  //                       0 : 
  //                       state.accelRatio;
  //}
  
  // check for trip current condition from BMS
  if (state.updateCurrentBufferRequested)
  {
    // charging current check
    if (state.bmsCurrent > 0.0 && state.bmsCurrent >= CHARGE_CURRENT_THRESH){
      state.tripped = true;
      state.bmsStrobeOn = true;
    }
    // This code compares the incoming value with the value in the array that it replaces. If one is overcurrent
    // and the other is undercurrent, it increments/decrements the counter accordingly.
    float absBMSCurrent = abs(state.bmsCurrent);
    if (absBMSCurrent >= CONTINUOUS_CURRENT_THRESH && 
        state.currentBuffer[state.currentBufferIndex] < CONTINUOUS_CURRENT_THRESH) { // increment overcurrent counter
      state.numOvercurrents++;
    }
    else if (absBMSCurrent < CONTINUOUS_CURRENT_THRESH && 
             state.currentBuffer[state.currentBufferIndex] >= CONTINUOUS_CURRENT_THRESH) { // decrement overcurrent counter
      state.numOvercurrents--;
    }

    //Store the incoming value in the array
    state.currentBuffer[state.currentBufferIndex] = absBMSCurrent; // store current in buffer
    state.currentBufferIndex = (state.currentBufferIndex+1) % CURRENT_BUFFER_SIZE; // increment buffer index
    
    //Check for a current trip condition
    if (absBMSCurrent >= MAX_CURRENT_THRESH ||
        state.numOvercurrents > OVERCURRENTS_ALLOWED) { // kill car
      state.tripped = true;
      state.bmsStrobeOn = true;
      state.dcErrorFlags |= BMS_OVER_CURR;
    }

    //Finally mark this update request handled
    state.updateCurrentBufferRequested = false;
  }
  
  // update ignition state
  if (state.tripped) { // kill car
    state.ignition = Ignition_Park;
  }
  else {
    state.ignition = state.ignitionRaw; 
  }
  //digitalWrite(BOARDLED,state.ignition == Ignition_Park ? HIGH : LOW);
  
  //Enable alternate steering if the SW is disconnected.
  if (NO_STEERING || state.dcErrorFlags & SW_TIMEOUT)
    state.altSteeringEnable = true;
  else
    state.altSteeringEnable = false;
}

/*
 * Sets general purpose output according to car state.
 */
void writeOutputs() {
  digitalWrite(HORN_PIN, state.horn ? HIGH : LOW);
  digitalWrite(HEADLIGHT_PIN, state.headlights ? HIGH : LOW);
  digitalWrite(BRAKELIGHT_PIN, state.brakeEngaged ? HIGH : LOW);
  digitalWrite(RIGHT_TURN_PIN, state.rightTurnOn ? HIGH : LOW);
  digitalWrite(LEFT_TURN_PIN, state.leftTurnOn ? HIGH : LOW);
  digitalWrite(BMS_STROBE_PIN, state.bmsStrobeOn ? HIGH : LOW); 
}

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN() {
  // see if motor controller packet needs to be sent
  if (dcDriveTimer.expired() && !state.tripped) { // ready to send drive command
    // determine velocity, current
    float MCvelocity, MCcurrent;
    switch (state.gear) {
    case FORWARD:
      MCvelocity = FORWARD_VELOCITY;
      MCcurrent = state.accelCurrent;
      break;
    case REVERSE:
      MCvelocity = REVERSE_VELOCITY;
      MCcurrent = state.accelCurrent;
      break;
    case REGEN: // drive current back through batteries to recharge them
      MCvelocity = 0;
      MCcurrent = state.regenCurrent; 
      break;
    case BRAKE: // do regen while braking
      MCvelocity = 0;
      MCcurrent = state.regenCurrent;
      break;
    case NEUTRAL: // coast
      MCvelocity = 0;
      MCcurrent = 0;
      break;
    }
    
    // create and send packet
    bool trysend = canControl.Send(DC_Drive(MCvelocity, MCcurrent), TXBANY);

    //Serial.println("Sending DC_Drive: ");
    //Serial.print(trysend);
    
    // reset timer
    if (trysend) 
      dcDriveTimer.reset();

  }
  
  // check if driver controls heartbeat needs to be sent
  if (dcHbTimer.expired()) {
    // create and send packet
    canControl.Send(DC_Heartbeat(DC_ID, DC_SER_NO), TXBANY);

    // reset timer
    dcHbTimer.reset(); 
  }

  // check if driver controls info packet needs to be sent
  if (dcInfoTimer.expired()) {
    // create and send packet
    /*DC_Info packet(state.accelRatio, state.regenRatio, state.brakeEngaged,
                            state.canErrorFlags, state.dcErrorFlags, state.wasReset, 
                            ((state.ignition != Ignition_Park) ? true : false), // fuel door, which we use to control the BMS since the ignition switch doesn't work.
                            state.gear, state.ignition);
                        
    bool trysend = canControl.Send(packet, TXBANY);*/

    bool trysend = canControl.SendVerified(DC_Info(state.accelRatio, state.regenRatio, state.brakeEngaged, state.canErrorFlags, state.dcErrorFlags, state.wasReset,
                                            ((state.ignition != Ignition_Park) ? true : false), state.gear, state.ignition, state.tripped),
                                    TXBANY);

    //Serial.println("Sending DC_Info: ");
    //Serial.print(trysend);

    // reset timer
    if (trysend) 
      dcInfoTimer.reset();
    
    state.wasReset = false; // clear reset    
  }
  
  if (dcPowerTimer.expired()) {
    bool trysend = canControl.Send(DC_Power(MAX_MOTOR_CURRENT), TXBANY);
    
    if (trysend) 
      dcPowerTimer.reset();
  }
}

/*
 * Checks the CAN controller and any other components for errors.
 * If errors exist, updates the error state.
 */
void checkErrors() {
  //Check the can bus for errors
  canControl.FetchErrors();
  
  // Reset the MCP if we are heading towards a bus_off condition
  if (canControl.tec > 200 || canControl.rec > 200) {
    if (DEBUG) {
      Serial.println("Reseting MCP2515");
      Serial.print("TEC/REC: ");
      Serial.print(canControl.tec); Serial.print(" / "); Serial.println(canControl.rec);
    }
    canControl.ResetController();
    if (DEBUG) {
      Serial.println("Reset MCP2515");
    }

    state.dcErrorFlags |= RESET_MCP2515;
  }
  
  // Check the mode of the MCP2515 (sometimes it is going to sleep randomly)
  canControl.FetchStatus();

  if ((canControl.canstat_register & 0b00100000) == 0b00100000) {
    canControl.ResetController();
    canControl.FetchStatus(); // check that everything worked
    if (DEBUG) {
       Serial.print("MCP2515 went to sleep. CANSTAT reset to: ");
       Serial.println(canControl.canstat_register);
    }
  }
}

void ReadTempSensor() {

  if (tempReadTimer.check()) { // temp sensor timeout
  
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    
    type_s=0;  //we're only using DS18B20 devices (as opposed to other OneWire devices)
  
  
     if ( !ds.search(addr)) {
      ds.reset_search();
      //Serial.print("no more addresses");
       }
  
    if (OneWire::crc8(addr, 7) != addr[7]) {
        //Serial.println("CRC is not valid!");
        return;
    }
  
  if (tempCount==0) 
    {
      ds.reset();                  
      ds.skip();                   // tell all sensors on bus
      ds.write(0x44,0);            // to convert temperature
      tempCount++;
      tempConvertTimer.reset();
    }
  else if ((tempCount > 0) && tempConvertTimer.expired())
    {
      present = ds.reset();
  
     ds.select(addr);    
     ds.write(0xBE);         // Read Scratchpad
  
     //Serial.print("  Data = ");
     //Serial.println(present, HEX);
     //Serial.print(" ");
     for ( i = 0; i < 9; i++) {           // we need 9 bytes
       data[i] = ds.read();
       //Serial.print(data[i], HEX);
       //Serial.print(" ");
     }
     //Serial.print(" CRC=");
     //Serial.print(OneWire::crc8(data, 8), HEX);
     //Serial.println();  
  
      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
     // even when compiled on a 32 bit processor.
     int16_t raw = (data[1] << 8) | data[0];
     /*Serial.print("RAW = ");
     Serial.print(raw, HEX);
     Serial.print(", ");
     Serial.print(raw);*/
     if (type_s) {
       raw = raw << 3; // 9 bit resolution default
       if (data[7] == 0x10) {
         // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
     } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
     }
      float rawtemp = (float)(raw >> 4);
      if(previousValidTemp == -1) //On first run
      {
        previousValidTemp = rawTemp;
      }
      //state.celsius[tempCount-1] = (float)(raw >> 4) + 5; // offset to account for inaccuracy in the BPS temp testing method used in scrutineering
      if(rawtemp > previousValidTemp + 5 || rawtemp < previousValidTemp - 5){ // Catch large random spikes in temperature sensor 
        state.celsius[tempCount-1] = 0.0;
      }
      else{
        state.celsius[tempCount-1] = rawtemp;
        previousValidTemp = rawtemp; 
      }
      /*int blah;
      blah = (int)(raw >> 4);
      Serial.print("Int value: ");
      Serial.println(blah);
      Serial.print("Float value: ");
      Serial.println(state.celsius[tempCount-1]);*/
      state.fahrenheit[tempCount-1] = state.celsius[tempCount-1] * 1.8 + 32.0;
      
      tempCount++;
      
      if (tempCount==27)
      {
          tempCount=0;      //reset count after tempCount has cycled thru all 26 sensors
            
          // Calculate Average Temperature  //
          j=0;                            // contains sum of temps
          for (i = 0; i < 26; i++)
          {
            j=j + state.celsius[i];
          }
          state.avgTemp=j/26;

          // Calculate Max Temperature //
          j=0;
          for (i = 0; i < 26; i++)
          {
            if (state.celsius[i] > j)
            {
              j=state.celsius[i];
            }
          }
          state.maxTemp=j;
      }
      //Serial.print("Max Temp = ");
      //Serial.println(state.maxTemp);    
  }
} 

  
}

//--------------------------MAIN FUNCTIONS---------------------------//
void setup() {
  // setup pin I/O
  pinMode(IGNITION_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);
  pinMode(BRAKELIGHT_PIN, OUTPUT);
  pinMode(RIGHT_TURN_PIN, OUTPUT);
  pinMode(LEFT_TURN_PIN, OUTPUT);
  pinMode(BOARDLED,OUTPUT);
  pinMode(BMS_STROBE_PIN, OUTPUT);
  
  // init steering wheel inputs for use if we lose the steering wheel
  pinMode(NEUTRAL_PIN, INPUT_PULLUP);
  pinMode(FORWARD_PIN, INPUT_PULLUP); 
  pinMode(LEFT_TURN_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_TURN_SW_PIN, INPUT_PULLUP);
  pinMode(HEADLIGHT_SW_PIN, INPUT_PULLUP);
  pinMode(HAZARDS_SW_PIN, INPUT_PULLUP);
  pinMode(HORN_SW_PIN, INPUT_PULLUP);

  digitalWrite(BOARDLED,HIGH); // Turn on durring initialization
  
  // debugging [ For some reason the board doesn't work unless I do this here instead of at the bottom ]
  if (DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Serial Initialized");
  }
  Serial.begin(SERIAL_BAUD);
  
  // init car state
  state = {}; // init all members to 0
  state.gear = FORWARD;
  state.gearRaw = FORWARD_RAW;
  state.wasReset = true;
  state.ignition = Ignition_Park;
  for (int i = 0; i < CURRENT_BUFFER_SIZE; i++) {
    state.currentBuffer[i] = 0;
  }
  state.tripped = false;
  state.bmsStrobeOn = false;

  // init valid temperature
  state.previousValidTemp = -1;
    
  // set the watchdog timer interval
  WDT_Enable(WDT, 0x2000 | WDT_INTERVAL| ( WDT_INTERVAL << 16 ));
  
  // reset timers
  mcHbTimer.reset();
  swHbTimer.reset();
  bmsHbTimer.reset();
  dcDriveTimer.reset();
  dcInfoTimer.reset();
  dcHbTimer.reset();
  debugTimer.reset();
  tempConvertTimer.reset();
  tempReadTimer.reset();

  
  // setup CAN
  canControl.filters.setRB0(RXM0, RXF0, RXF1);
  canControl.filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canControl.Setup(RX0IE | RX1IE | TX0IE | TX1IE | TX2IE);
  
  //Enabling One-Shot mode (debugging)
  //canControl.controller.Write(CANCTRL, 00001111);
  //Serial.println((canControl.controller.Read(CANCTRL)), BIN);
 
  digitalWrite(BOARDLED,LOW);   // Turn of led after initialization
  
  if (DEBUG && canControl.errors != 0) {
    Serial.print("Init CAN error: ");
    Serial.println(canControl.errors, HEX);
  }


}

void loop() {
  // Start timer
  if (DEBUG) {
    loopStartTime = micros();
  }
  

  //read temp sensors
  ReadTempSensor();
  
  // read GPIO
  readInputs();
  
  // clear watchdog timer
  WDT_Restart(WDT);

  // get any CAN messages that have come in
  canControl.Fetch();
    
  // read CAN
  readCAN();
  
  // clear watchdog timer
  WDT_Restart(WDT);
  
  // check timers
  checkTimers();
  
  // process information that was read
  updateState();

  // get any CAN messages that have come in 
  canControl.Fetch();
  
  // write GPIO
  writeOutputs();
  
  // write CAN
  writeCAN();

  // clear watchdog timer
  WDT_Restart(WDT);

  // check for errors and fix them
  checkErrors();
  
  // Add the loop time to the sum time
  if (DEBUG) { 
    loopSumTime += micros() - loopStartTime;
    loopCount += 1;
  }
  
  // debugging printout
  if (DEBUG && debugTimer.expired()) {
    
    /************************ TEMP CAN DEBUGGING VARIABLES (DELETE LATER)******/
    byte Txstatus[3] = {0,0,0};
    Txstatus[0] = canControl.controller.Read(TXB0CTRL);
    Txstatus[1] = canControl.controller.Read(TXB1CTRL);
    Txstatus[2] = canControl.controller.Read(TXB2CTRL);
    byte canintf = 0; canintf = canControl.last_interrupt;
    byte canctrl = 0; canctrl = canControl.controller.Read(CANCTRL);

    Serial.println("TXnCTRL: ");
    Serial.println(Txstatus[0], BIN);
    Serial.println(Txstatus[1], BIN);
    Serial.println(Txstatus[2], BIN);
    Serial.print("Last Interrupt: ");
    Serial.println(canintf, BIN);
    Serial.print("CANCTRL: ");
    Serial.println(canctrl, BIN);
    Serial.print("CANSTAT: ");
    Serial.println(state.canstat_reg, BIN);
    Serial.println("");

    Serial.print("Gear: ");
    switch (state.gear) {
    case BRAKE:
      Serial.println("BRAKE");
      break;
    case FORWARD:
      Serial.println("FORWARD");
      break;
    case REVERSE:
      Serial.println("REVERSE");
      break;
    case REGEN:
      Serial.println("REGEN");
      break;
    case NEUTRAL:
      Serial.println("NEUTRAL");
      break;
    }
    Serial.print("Gear Raw: ");
    Serial.println(state.gearRaw);    
    Serial.print("Average Loop time (us): ");
    Serial.println(loopSumTime / loopCount);
    Serial.print("System time: ");
    Serial.println(millis());
    Serial.println();

    switch (debugStep) {
      case 0:
        Serial.print("Brake pin: ");
        Serial.println(state.brakeEngaged ? "pressed" : "not pressed");
        Serial.print("Accel pedal raw: ");
        Serial.println(state.accelRaw);
        Serial.print("Accel ratio: ");
        Serial.println(state.accelRatio);
        Serial.print("Accel current: ");
        Serial.println(state.accelCurrent);
        Serial.print("Regen pedal raw: ");
        Serial.println(state.regenRaw);
        Serial.print("Regen ratio: ");
        Serial.println(state.regenRatio);
        Serial.print("Regen current: ");
        Serial.println(state.regenCurrent);
        Serial.print("Car tripped: ");
        Serial.println(state.tripped ? "YES" : "NO");
        break;
      case 1:
        Serial.print("Ignition: ");
        Serial.println(state.ignition,HEX);
        Serial.print("IgnitionRaw: ");
        Serial.println(state.ignitionRaw,HEX);
        Serial.print("Ignition digitalRead: ");
        Serial.println(digitalRead(52));
        Serial.print("Horn: ");
        Serial.println(state.horn ? "ON" : "OFF");
        Serial.print("Headlights: ");
        Serial.println(state.headlights ? "ON" : "OFF");
        Serial.print("Brakelights: ");
        Serial.println(state.brakeEngaged ? "ON" : "OFF");
        Serial.print("Right turn signal: ");
        Serial.println(state.rightTurn ? "ON" : "OFF");
        Serial.print("Right turn singal active: ");
        Serial.println(state.rightTurnOn ? "YES" : "NO");
        Serial.print("Left turn signal: ");
        Serial.println(state.leftTurn ? "ON" : "OFF");
        Serial.print("Left turn signal active: ");
        Serial.println(state.leftTurnOn ? "YES" : "NO");
        Serial.print("Hazards: ");
        Serial.println(state.hazards ? "YES" : "NO");
        Serial.print("BMS Strobe: ");
        Serial.println(state.bmsStrobeOn ? "YES" : "NO");
        break;
      case 2: 
        //Serial.print("Cruise control: ");
        //Serial.println(state.cruiseCtrl ? "ON" : "OFF");
        //Serial.print("Cruise control previous: ");
        //Serial.println(state.cruiseCtrlPrev ? "ON" : "OFF");
        //Serial.print("Cruise control active: ");
        //Serial.println(state.cruiseCtrlOn ? "YES" : "NO");
        //Serial.print("Cruise control ratio: ");
        //Serial.println(state.cruiseCtrlRatio);
        Serial.print("CAN error: ");
        Serial.println(canControl.errors, HEX);
        Serial.print("TEC/REC: ");
        Serial.print(canControl.tec); Serial.print(" / "); Serial.println(canControl.rec);
        Serial.print("Interrupt Counter: ");
        Serial.println(canControl.int_counter);
        Serial.print("RX buffer counter: ");
        Serial.println(canControl.RXbuffer.size());
        Serial.print("Board error: ");
        Serial.println(state.dcErrorFlags, HEX);
        Serial.print("BMS Current: ");
        Serial.println(state.bmsCurrent);
        if (state.SW_timer_reset_by != 0) {
          Serial.print("SW RESET BY: ");
          Serial.println(state.SW_timer_reset_by, HEX);
          state.SW_timer_reset_by = 0;
        }
        break;
        case 3:
            for (i = 0; i < 26; i++)
            {
               Serial.print("Temperature = ");
               Serial.print(state.celsius[i]);
               Serial.print(" Celsius, ");
               Serial.print(state.fahrenheit[i]);
               Serial.println(" Fahrenheit");
            }
            Serial.print("Average Temp: ");
            Serial.println(state.avgTemp);
            Serial.print("Max Temp: ");
            Serial.println(state.maxTemp);
         
         break;
    }
    
    Serial.print("System time: ");
    Serial.println(millis());
    Serial.println();
    
    debugStep = (debugStep+1) % 4;
    debugTimer.reset();
    
    // Reset loop timer variables
    loopSumTime = 0;
    loopCount = 0;
  }
  //Serial.print("Max Temp: ");
  //          Serial.println(state.maxTemp);
  // Reset canErrorFlags after each loop.
  state.canErrorFlags = 0;
}
