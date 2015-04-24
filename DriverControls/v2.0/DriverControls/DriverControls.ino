/*
 * DriverControls.ino
 * Contains code to run the driver controls
 * board for sc7.
 */

#include <stdint.h>
#include <Metro.h>
#include <SPI.h>
#include <math.h>
#include "sc7-can-libinclude.h"

//------------------------------CONSTANTS----------------------------//
// debugging
const bool DEBUG = true; // change to true to output debug info over serial
byte       debugStep = 0; // It's too slow to send out all the debug over serial at once, so we split it into 3 steps.

// pins
const byte IGNITION_PIN   = 53;
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

// CAN parameters
const uint16_t BAUD_RATE = 1000;
const byte     FREQ      = 16;

const uint16_t RXM0      = MASK_Sx00;
const uint16_t RXF0      = SW_BASEADDRESS; // Match any steering_wheel packet (because mask is Sx00)
const uint16_t RXF1      = 0;

const uint16_t RXM1      = MASK_Sxxx;
const uint16_t RXF2      = BMS_VOLT_CURR_ID;
const uint16_t RXF3      = BMS_SOC_ID; // Most useless: replace first (soc not used by DC currently)
const uint16_t RXF4      = MC_VELOCITY_ID;
const uint16_t RXF5      = MC_BUS_STATUS_ID; //Also kinda useless right now since we read BMS current.

// timer intervals (all in ms)
const uint16_t MC_HB_INTERVAL    = 1000;  // motor controller heartbeat
const uint16_t SW_HB_INTERVAL    = 1000;  // steering wheel heartbeat
const uint16_t BMS_HB_INTERVAL   = 1000;  // bms heartbeat
const uint16_t DC_DRIVE_INTERVAL = 50;    // drive command packet
const uint16_t DC_INFO_INTERVAL  = 80;   // driver controls info packet
const uint16_t DC_HB_INTERVAL    = 200;   // driver controls heartbeat packet
const uint16_t WDT_INTERVAL      = 5000;  // watchdog timer
const uint16_t TOGGLE_INTERVAL   = 500;   // toggle interval for right/left turn signals, hazards
const uint16_t DEBUG_INTERVAL    = 333;  // interval for debug calls output
const int      SERIAL_BAUD       = 115200; // baudrate for serial (maximum)

// drive parameters
const uint16_t MAX_ACCEL_VOLTAGE   = 1024;    // max possible accel voltage
const float    MAX_ACCEL_RATIO     = 0.8;     // maximum safe accel ratio
const uint16_t MAX_REGEN_VOLTAGE   = 1024;    // max possible regen voltage
const float    MAX_REGEN_RATIO     = 1.0;     // maximum safe regen ratio
const float    MIN_PEDAL_TOLERANCE = 0.05;    // anything less is basically zero
const float    FORWARD_VELOCITY    = 100.0f;  // velocity to use if forward
const float    REVERSE_VELOCITY    = -100.0f; // velocity to use if reverse
const float    GEAR_CHANGE_CUTOFF  = 5.0f;    // cannot change gear unless velocity is below this threshold
const float    M_PER_SEC_TO_MPH    = 2.237f;  // conversion factor from m/s to mph
const int      MAX_CAN_PACKETS_PER_LOOP = 10;
const uint16_t DC_ID               = 0x00C7;  // For SC7
const uint16_t DC_SER_NO           = 0x0042;  // Don't panic!

// steering wheel parameters
const byte NEUTRAL_RAW = 0x3;
const byte FORWARD_RAW = 0x2;
const byte REVERSE_RAW = 0x1;
const byte SW_ON_BIT   = 0;   // value that corresponds to on for steering wheel data

// BMS parameters
const float TRIP_CURRENT_THRESH		= 5000;

// driver control errors
const uint16_t MC_TIMEOUT  = 0x01; // motor controller timed out
const uint16_t BMS_TIMEOUT = 0x02; // bms timed out
const uint16_t SW_TIMEOUT  = 0x04; // sw timed out
const uint16_t SW_BAD_GEAR = 0x08; // bad gearing from steering wheel
const uint16_t BMS_OVER_CURR = 0x10; // Detected BMS overcurrent, tripped car.
const uint16_t RESET_MCP2515 = 0x20; // Had to reset the MCP2515
//----------------------------TYPE DEFINITIONS------------------------//
/*
 * Enum to represet the possible gear states.
 */
enum GearState { BRAKE = 0x04, FORWARD = 0x08, REVERSE = 0x01, REGEN = 0x0C, NEUTRAL = 0x02 }; //3 bytes (bit values try to match tritium bit positions in the switches bit (see can.h)

/*
 * Enum to represent ignition states
 */
enum IgnitionState { Ignition_Start = 0x0040, Ignition_Run = 0x0020, Ignition_Park = 0x0010 };

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
  //bool cruiseCtrl;    // true if driver wants cruise control on
  
  // motor info
  float motorVelocity;  // rotational speed of motor (rpm)
  float carVelocity;    // velocity of car (mph)
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
  //bool cruiseCtrlOn;     // true if cruise control should be active
  //bool cruiseCtrlPrev;   // prev value of cruiseCtrl
  //float cruiseCtrlRatio; // pedal ratio to use if cruise control active
                    
  // gearing and ignition
  GearState gear;               // brake, foward, reverse, regen, neutral
  IgnitionState ignition;	// start, run, park
  
  // outputs
  bool rightTurnOn;   // true if we should turn rt signal on
  bool leftTurnOn;    // true if we should turn lt signal on


  // errors
  uint16_t canErrorFlags; // keep track of errors with CAN bus
  uint16_t dcErrorFlags;  // keep track of other errors
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

  /*
  // read ignition key here
  if (digitalRead(IGNITION_PIN) == LOW)
     state.ignition = Ignition_Start;
  else
     state.ignition = Ignition_Run;
  */
     
  if (digitalRead(49) == LOW)
    state.ignition = Ignition_Start;
  if (digitalRead(50) == LOW)
    state.ignition = Ignition_Run;
  if (digitalRead(51) == LOW)
    state.ignition = Ignition_Park;

}

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN() {
  nointerrupts(); // disable interrupts while 

  int i = 0;
  while(i <= MAX_CAN_PACKETS_PER_LOOP) { // there are messages
    i++;

    // disable interrupts while we deal with the queue, so we don't get
    // an interrupt trying to write to our queue while we read
    nointerrupts(); 

      // If there is nothing in the queue, break (this didn't go above because I needed nointerrupts first)
      if (!canControl.Available())
        break;

      Frame& f = canControl.Read(); // read one message

    // We are done with the queue, and this next part might take a while,
    // so we turn interrupts back on
    interrupts();
    
    // determine source and update heartbeat timers
    // first three digits will be exactly equal to heartbeat ids
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
      //state.cruiseCtrlPrev = state.cruiseCtrl;
      //state.cruiseCtrl = (packet.cruisectrl == SW_ON_BIT);
    }
    else if (f.id == BMS_VOLT_CURR_ID) { // BMS Voltage Current Packet
    	BMS_VoltageCurrent packet(f);

    	if (packet.current >= TRIP_CURRENT_THRESH) {
    		state.ignition = Ignition_Park; // KILL THE BATTERIES
    		state.dcErrorFlags |= BMS_OVER_CURR;
    	}
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
    case NEUTRAL_RAW:
      state.gear = NEUTRAL; // can always change to neutral
      break;
    case FORWARD_RAW:
      if (fabs(state.carVelocity) < GEAR_CHANGE_CUTOFF || 
          state.carVelocity > 0.0f) { // going forward or velocity less than cutoff, gear switch ok
        state.gear = FORWARD;
      }
      break;
    case REVERSE_RAW:
      if (fabs(state.carVelocity) < GEAR_CHANGE_CUTOFF ||
          state.carVelocity < 0.0f) { // going backward or velocity less than cutoff, gear switch ok
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
  if (dcDriveTimer.check() || state.dcErrorFlags & BMS_OVER_CURR ) { // ready to send drive command
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
    canControl.Send(DC_Drive(MCvelocity, MCcurrent), TXB0);

    delay(10);

    // Send BMS Ignition Packet
    //canControl.Send(DC_SwitchPos(state.ignition),TXB2);
    
    // reset timer
    dcDriveTimer.reset();
    
    // reset any BMS error that occured which caused us to send this packet immediately.
    state.dcErrorFlags &= ~BMS_OVER_CURR;
       
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
                            state.canErrorFlags, state.dcErrorFlags, state.wasReset, 
                            ((state.ignition == 0x0040) ? true : false), // fuel door, which we use to control the BMS since the ignition switch doesn't work.
                            state.gear, state.ignition), TXB0);
    
    // reset timer
    dcInfoTimer.reset();
    
    state.wasReset = false; // clear reset    
    delay(10); // mcp2515 seems to require small delay
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
  
  pinMode(49, INPUT_PULLUP);
  pinMode(50, INPUT_PULLUP);
  pinMode(51, INPUT_PULLUP);
  
  digitalWrite(BOARDLED,HIGH); // Turn on durring initialization
  
  // debugging [ For some reason the board doesn't work unless I do this here instead of at the bottom ]
  if (DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Serial Initialized");
  }
  
  // init car state
  state = {}; // init all members to 0
  state.gear = FORWARD;
  state.gearRaw = FORWARD_RAW;
  state.wasReset = true;
  state.ignition = Ignition_Run;
    
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
  
  
  // setup CAN
  CANFilterOpt filters;
  filters.setRB0(RXM0, RXF0, RXF1);
  filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canControl.Setup(filters, RX0IE | RX1IE); // Start can with only the two receive interupts enabled (to try and eliminate the system freezing on a continuous interrupt)
 
  digitalWrite(BOARDLED,LOW);   // Turn of led after initialization
  
  if (DEBUG) {
    Serial.print("Init CAN error: ");
    Serial.println(canControl.errors, HEX);
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

  // Get any CAN errors that have occured
  canControl.FetchErrors();
  
  // Reset the MCP if we are heading towards a bus_off condition
  if (canControl.errors & CANERR_HIGH_ERROR_COUNT)
  {
    canControl.ResetController();
    state.dcErrorFlags |= RESET_MCP2515;
    if (DEBUG)
      Serial.print("Reset MCP2515");
  }
  
  // debugging
  if (DEBUG && debugTimer.check()) {
    debugStartTime = millis();
    Serial.print("Loop time: ");
    Serial.println(debugStartTime - debugEndTime);
    Serial.print("System time: ");
    Serial.println(millis());
    switch (debugStep)
    {
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
      break;
      ///////////////////
      case 1:
        Serial.print("Ignition: ");
        Serial.println(state.ignition,HEX);
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
      break;
      ////////////////////
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
        Serial.print(canControl.tec); Serial.print(" \ "); Serial.println(canControl.rec);
        Serial.print("Interrupt Counter: ");
        Serial.println(canControl.int_counter);
        Serial.print("RX buffer counter: ");
        Serial.println(canControl.RXbuffer.size());
        Serial.print("Board error: ");
        Serial.println(state.dcErrorFlags, HEX);
     break;
    }
    
    Serial.print("System time: ");
    Serial.println(millis());
    Serial.println();
    
    debugStep = (debugStep+1) % 3;
    debugEndTime = millis();
    debugTimer.reset();
  }
  
  state.canErrorFlags = 0;
}


