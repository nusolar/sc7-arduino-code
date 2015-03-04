/*
 * DriverControls.ino
 * Contains code to run the driver controls
 * board for sc7.
 */

#include <stdint.h>
#include <Metro.h>
#include <SPI.h>
#include "sc7-can-libinclude.h"

//------------------------------CONSTANTS----------------------------//
// debugging
const bool DEBUG = false; // change to true to output debug info over serial

// pins
const byte BRAKE_PIN      = 44;
const byte ACCEL_PIN      = 4;
const byte REGEN_PIN      = 3;
const byte INTERRUPT_PIN  = 5;
const byte CS_PIN         = 4;
const byte HORN_PIN       = 6;
const byte RIGHT_TURN_PIN = 7;
const byte LEFT_TURN_PIN  = 8;
const byte HEADLIGHT_PIN  = 9;
const byte BRAKELIGHT_PIN = 10;
const byte BOARDLED       = 13;

// CAN parameters
const uint16_t BAUD_RATE = 1000;
const byte     FREQ      = 16;
const uint16_t RXM0      = MASK_NONE;
const uint16_t RXM1      = MASK_NONE;
const uint16_t RXF0      = MASK_NONE;
const uint16_t RXF1      = MASK_NONE;
const uint16_t RXF2      = MASK_NONE;
const uint16_t RXF3      = MASK_NONE;
const uint16_t RXF4      = MASK_NONE;
const uint16_t RXF5      = MASK_NONE;

// timer intervals (all in ms)
const uint16_t MC_HB_INTERVAL    = 1000;  // motor controller heartbeat
const uint16_t SW_HB_INTERVAL    = 1000;  // steering wheel heartbeat
const uint16_t BMS_HB_INTERVAL   = 1000;  // bms heartbeat
const uint16_t DC_DRIVE_INTERVAL = 50;    // drive command packet
const uint16_t DC_INFO_INTERVAL  = 200;   // driver controls info packet
const uint16_t DC_HB_INTERVAL    = 200;   // driver controls heartbeat packet
const uint16_t WDT_INTERVAL      = 5000;  // watchdog timer
const uint16_t TOGGLE_INTERVAL   = 500;   // toggle interval for right/left turn signals, hazards
const uint16_t DEBUG_INTERVAL    = 1000;  // interval for debug output

// drive parameters
const uint16_t MAX_ACCEL_VOLTAGE  = 1024;    // max possible accel voltage
const float MAX_ACCEL_RATIO       = 0.8;     // maximum safe accel ratio
const uint16_t MAX_REGEN_VOLTAGE  = 1024;    // max possible regen voltage
const float MAX_REGEN_RATIO       = 1.0;     // maximum safe regen ratio
const float MIN_PEDAL_TOLERANCE   = 0.05;    // anything less is basically zero
const float FORWARD_VELOCITY      = 100.0f;  // velocity to use if forward
const float REVERSE_VELOCITY      = -100.0f; // velocity to use if reverse
const uint16_t DC_ID              = 0x00C7;  // For SC7
const uint16_t DC_SER_NO          = 0x0042;  // Don't panic!

// steering wheel parameters
const byte GEAR_NEUTRAL = 0x3;
const byte GEAR_FORWARD = 0x2;
const byte GEAR_REVERSE = 0x1;
const byte SW_ON_BIT    = 0;   // value that corresponds to on for steering wheel data

// driver control errors
const uint16_t MC_TIMEOUT  = 0x01; // motor controller timed out
const uint16_t BMS_TIMEOUT = 0x02; // bms timed out
const uint16_t SW_TIMEOUT  = 0x04; // sw timed out
const uint16_t SW_BAD_GEAR = 0x08; // bad gearing from steering wheel

//----------------------------TYPE DEFINITIONS------------------------//
/*
 * Enum to represet the possible gear states.
 */
enum GearState { BRAKE, FORWARD, REVERSE, REGEN, NEUTRAL };

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
  byte gearRaw;       // 00 = neutral, 01 = forward, 10 = reverse, 11 = undefined
  bool horn;          // true if driver wants horn on (no toggle)
  bool headlights;    // true if driver wants headlights on (no toggle)
  bool rightTurn;     // true if driver wants right turn on (no toggle)
  bool leftTurn;      // true if driver wants left turn on (no toggle)
  bool hazards;       // true if driver wants hazards on (no toggle)
  bool cruiseCtrl;    // true if driver wants cruise control on
  
  // motor info
  float motorVelocity;
  float carVelocity;
  float busCurrent;
  
  // bms info
  float bmsPercentSOC; // percent state of charge of bms
  
  // debugging
  bool wasReset;       // true on initilization, false otherwise
  
  // DERIVED DATA
  // pedals
  float accelRatio; // ratio of accel voltage to max voltage
                    // constrained for safety
  float regenRatio; // ratio of regen voltage to max voltage
                    // constrained for safety
                    
  // motor current
  float accelCurrent;   // current to use if in drive/reverse
  float regenCurrent;   // current to use if in regen
                    
  // cruise control
  bool cruiseCtrlOn;     // true if cruise control should be active
  bool cruiseCtrlPrev;   // prev value of cruiseCtrl
  float cruiseCtrlRatio; // pedal ratio to use if cruise control active
                    
  // gearing
  GearState gear; // brake, foward, reverse, regen, neutral
  
  // outputs
  bool rightTurnOn;   // true if we should turn rt signal on
  bool leftTurnOn;    // true if we should turn lt signal on
  
  // errors
  uint16_t canErrorFlags; // keep track of errors with CAN bus
  byte dcErrorFlags;      // keep track of other errors
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

// debugging
int debugStartTime = 0;
int debugEndTime = 0;

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
    // first three digits will be exactly equal to heartbeat ids
    if ((f.id & MASK_Sx00) == BMS_HEARTBEAT_ID) { // source is bms
      bmsHbTimer.reset();
      state.dcErrorFlags &= ~BMS_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == MC_HEARTBEAT_ID) { // source is mc
      mcHbTimer.reset();
      state.dcErrorFlags &= ~MC_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == SW_HEARTBEAT_ID) { // source is sw
      swHbTimer.reset();
      state.dcErrorFlags &= ~SW_TIMEOUT; // clear flag
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
      SW_Data packet(f);
      
      // read data
      state.gearRaw = packet.gear;
      state.horn = (packet.horn == SW_ON_BIT);
      state.rightTurn = (packet.rts == SW_ON_BIT);
      state.leftTurn = (packet.lts == SW_ON_BIT);
      state.headlights = (packet.headlights == SW_ON_BIT);
      state.hazards = (packet.hazards == SW_ON_BIT);
      
      // read cruise control
      state.cruiseCtrlPrev = state.cruiseCtrl;
      state.cruiseCtrl = (packet.cruisectrl == SW_ON_BIT);
    }  
  }
}

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers() {
  // check motor controller
  if (mcHbTimer.check()) { // motor controller timeout
    state.dcErrorFlags |= MC_TIMEOUT; // set flag
  }
  
  // check bms
  if (bmsHbTimer.check()) { // bms timeout
    state.dcErrorFlags |= BMS_TIMEOUT; // set flag
  }
  
  // check steering wheel
  if (swHbTimer.check()) { // steering wheel timeout
    state.dcErrorFlags |= SW_TIMEOUT; // set flag
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
  state.dcErrorFlags &= ~SW_BAD_GEAR; // clear bad gear flag
  if (state.brakeEngaged) { // brake engaged, overrides all other gears
    state.gear = BRAKE;
  }
  else if (state.regenRatio > MIN_PEDAL_TOLERANCE) {  // regen engaged
    state.gear = REGEN;
  }
  else { // accel or nothing engaged
    switch (state.gearRaw){
    case GEAR_NEUTRAL:
      state.gear = NEUTRAL;
      break;
    case GEAR_FORWARD:
      state.gear = FORWARD;
      break;
    case GEAR_REVERSE:
      state.gear = REVERSE;
      break;
    default: // unknown gear
      state.gear = NEUTRAL; // safe default gear?
      state.dcErrorFlags |= SW_BAD_GEAR; // flag bad gear
      break;
    }
  }
  
  // update top shell state
  // check hazards
  if (state.hazards) { // hazards active
    if (hazardsTimer.check()) { // timer expired, toggle
      state.rightTurnOn = !state.rightTurnOn;
      state.leftTurnOn = state.rightTurnOn; // make sure they have same value
      hazardsTimer.reset();
    }
  }
  else { // hazards inactive
    // check right turn signal
    if (state.rightTurn) { // right turn signal active
      if (rightTurnTimer.check()) { // timer expired, toggle
        state.rightTurnOn = !state.rightTurnOn;
        rightTurnTimer.reset();
      }
    }
    else { // right turn signal inactive
      state.rightTurnOn = false;
    }
    // check left turn signal
    if (state.leftTurn) { // left turn signal active
      if (leftTurnTimer.check()) { // timer expired, toggle
        state.leftTurnOn = !state.leftTurnOn;
        leftTurnTimer.reset();
      }
    }
    else { // left turn signal inactive
      state.leftTurnOn = false;
    }
  }
  
  // update cruise control state
  if (!state.cruiseCtrlPrev && state.cruiseCtrl) { // cruise control just switched on
    state.cruiseCtrlOn = true;
    state.cruiseCtrlRatio = state.accelRatio;
  }
  else if (state.gear == BRAKE || state.gear == REGEN ||
           !state.cruiseCtrl) { // regen pedal pressed or cruise control switched off
    state.cruiseCtrlOn = false;
  }
  
  // update current values to be sent to motor controller
  state.regenCurrent = state.regenRatio < MIN_PEDAL_TOLERANCE ? 
                       0 : 
                       state.regenRatio; 
  
  if (state.cruiseCtrlOn) { // cruise control on, make sure that current doesn't fall below cruise control ratio
    state.accelCurrent = state.accelRatio < state.cruiseCtrlRatio ? 
                         state.cruiseCtrlRatio : 
                         state.accelRatio;
  }
  else { // no cruise control, take pedal value (or 0 if below tolerance)
    state.accelCurrent = state.accelRatio < MIN_PEDAL_TOLERANCE ? 
                         0 : 
                         state.accelRatio;
  }
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
}

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN() {
  // see if motor controller packet needs to be sent
  if (dcDriveTimer.check()) { // ready to send drive command
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
      MCcurrent = MAX_REGEN_RATIO;
      break;
    case NEUTRAL: // coast
      MCvelocity = 0;
      MCcurrent = 0;
      break;
    }
    
    // create and send packet
    canControl.Send(DC_Drive(MCvelocity, MCcurrent), TXB0);
    
    // reset timer
    dcDriveTimer.reset();
    
    delay(10); // mcp2515 seems to require small delay
  }
  
  // check if driver controls heartbeat needs to be sent
  if (dcHbTimer.check()) {
    // create and send packet
    canControl.Send(DC_Heartbeat(DC_ID, DC_SER_NO), TXB1);
    
    // reset timer
    dcHbTimer.reset(); 
    
    delay(10);
  }
  
  // check if driver controls info packet needs to be sent
  if (dcInfoTimer.check()) {
    // create and send packet
    canControl.Send(DC_Info(state.accelRatio, state.regenRatio, state.brakeEngaged,
                            state.canErrorFlags, state.dcErrorFlags, state.wasReset), TXB2);
    
    // reset timer
    dcInfoTimer.reset();
    
    state.wasReset = false; // clear reset    
    delay(10); // mcp2515 seems to require small delay
  }
}

//--------------------------MAIN FUNCTIONS---------------------------//
void setup() {
  // setup pin I/O
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);
  pinMode(BRAKELIGHT_PIN, OUTPUT);
  pinMode(RIGHT_TURN_PIN, OUTPUT);
  pinMode(LEFT_TURN_PIN, OUTPUT);
  pinMode(BOARDLED,OUTPUT);
  
  digitalWrite(BOARDLED,HIGH); // Turn on durring initialization
  
  // debugging [ For some reason the board doesn't work unless I do this here instead of at the bottom ]
  if (DEBUG) {
    Serial.begin(9600);
    Serial.println("Serial Initialized");
  }
  
  // setup CAN
  CANFilterOpt filters;
  filters.setRB0(RXM0, RXF0, RXF1);
  filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canControl.Setup(filters, &state.canErrorFlags);

  // init car state
  state = {}; // init all members to 0
  state.gear = NEUTRAL;
  state.gearRaw = GEAR_NEUTRAL;
  state.wasReset = true;
    
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
 
  digitalWrite(BOARDLED,LOW);   // Turn of led after initialization
  
  if (DEBUG) {
    Serial.print("Init CAN error: ");
    Serial.println(state.canErrorFlags, HEX);
  }
}

void loop() {  
  // clear watchdog timer
  WDT_Restart(WDT);
  
  // read GPIO
  readInputs();
  
  // read CAN
  readCAN();
  
  // clear watchdog timer
  WDT_Restart(WDT);
  
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
  
  // clear watchdog timer
  WDT_Restart(WDT);
  
  // debugging
  if (DEBUG && debugTimer.check()) {
    debugStartTime = millis();

    Serial.print("Loop time: ");
    Serial.println(debugStartTime - debugEndTime);
    Serial.print("System time: ");
    Serial.println(millis());
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
    Serial.println(state.hazards ? "ON" : "OFF");
    Serial.print("Cruise control: ");
    Serial.println(state.cruiseCtrl ? "ON" : "OFF");
    Serial.print("Cruise control active: ");
    Serial.println(state.cruiseCtrlOn ? "YES" : "NO");
    Serial.print("Cruise control ratio: ");
    Serial.println(state.cruiseCtrlRatio);
    Serial.print("CAN error: ");
    Serial.println(state.canErrorFlags, HEX);
    Serial.print("Board error: ");
    Serial.println(state.dcErrorFlags, HEX);
    Serial.print("System time: ");
    Serial.println(millis());
    Serial.println();
    
    debugEndTime = millis();
    
    debugTimer.reset();
  }
  
  state.canErrorFlags = 0;
}
