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
#include <Metro.h>

//------------------------------CONSTANTS----------------------------//
// debugging
#define DEBUG                   // change to true to output debug info over serial
byte debugStep = 0;             // It's too slow to send out all the debug over serial at once, so we split it into 3 steps.
const int SERIAL_BAUD = 115200; // baudrate for serial (maximum)

// pins
const byte INTERRUPT_PIN = 7;
const byte CS_PIN = 3;
const byte RIGHT_TURN_PIN = A10;
const byte LEFT_TURN_PIN = A9;
const byte BRAKELIGHT_PIN = A6;
const byte BOARDLED = 13;
const byte BMS_STROBE_PIN = A2;
const byte IGNITION_PIN = 30;
const byte THROTTLE_PIN = A3;

// CAN parameters
const uint16_t BAUD_RATE = 125;
const byte FREQ = 16;

const uint32_t RXM0 = MASK_EID;
const uint32_t RXF0 = MTBA_FRAME0_REAR_LEFT_ID;  // Match any steering_wheel packet (because mask is Sx00)
const uint32_t RXF1 = MTBA_FRAME0_REAR_RIGHT_ID; // Can't put 0 here, otherwise it will match all packets that start with 0.

const uint16_t RXM1 = MASK_Sxxx;
const uint16_t RXF2 = BMS19_VCSOC_ID;
const uint16_t RXF3 = BMS19_TRIP_STAT_ID;
const uint16_t RXF4 = 0;
const uint16_t RXF5 = 0;

// timer intervals (all in ms)

const uint16_t MC_HB_INTERVAL = 1000;         // motor controller heartbeat
const uint16_t BMS_HB_INTERVAL = 1000;        // bms heartbeat
const uint16_t WDT_INTERVAL = 5000;           // watchdog timer
const uint16_t TOGGLE_INTERVAL = 500;         // toggle interval for right/left turn signals, hazards
const uint16_t DEBUG_INTERVAL = 333;          // interval for debug calls output
const uint16_t TEMP_CONV_INTERVAL = 800;      // interval for temp sense conversion
const uint16_t TEMP_READ_INTERVAL = 100;      // interval for temp sense reading
const uint16_t TEMP_SEND_INTERVAL = 500;      // interval for temp sense sending (over can)
const uint16_t TEMP_OVERHEAT_INTERVAL = 1000; // interval for overheat temp sense sending (over can)
const uint16_t MPPT_REQ_INTERVAL = 500;
const uint16_t MC_REQ_INTERVAL = 500;
const uint16_t MC_SEND_INTERVAL = 100;        // interval for sending throttle packets

// drive parameters
const float M_PER_SEC_TO_MPH = 2.237f;   // conversion factor from m/s to mph
const int MAX_CAN_PACKETS_PER_LOOP = 10; // Maximum number of receivable CAN packets per loop
const uint8_t NUM_TEMP_MODULES = 26;     // Number of temperature sensors
const uint8_t CHARGE_TEMP = 45;          // battery temp threshold when current is positive
const uint8_t DISCHARGE_TEMP = 60;       // battery temp threshold when current is negative
const float RPM_TO_MPH = 2.2369f;        // Change for correct values, diameter 19 inch
const uint8_t MAX_VELOCITY = 100;        // max velocity for motor velocity setpoint

// driver control errors
const uint16_t MC_TIMEOUT = 0x01;    // motor controller timed out
const uint16_t BMS_TIMEOUT = 0x02;   // bms timed out
const uint16_t BMS_OVER_CURR = 0x10; // Detected BMS overcurrent, tripped car.
const uint16_t RESET_MCP2515 = 0x20; // Had to reset the MCP2515

//temp sensor object
OneWire ds(50);

//----------------------------TYPE DEFINITIONS------------------------//
/*
 * Struct to hold informations about the car state.
 */
struct CarState
{
  // motor info
  float motorVelocity[2]; // rotational speed of motor (rpm): 0 is left, 1 is right
  float carVelocity[2];
  int16_t busCurrent;

  // bms info
  float bmsPercentSOC; // percent state of charge of bms
  float bmsCurrent;    // Current reading from BMS (negative is out of the batteries)

  // debugging
  bool wasReset;    // true on initilization, false otherwise
  byte canstat_reg; // holds value of canstat register on the MCP2515
  int SW_timer_reset_by;

  // DERIVED DATA

  // outputs
  bool bmsIgnition; // true if we should turn on the bms strobe (for tripping)
  bool bmsStrobeToggle;

  // errors
  uint16_t canErrorFlags; // keep track of errors with CAN bus
  uint16_t dcErrorFlags;  // keep track of other errors
  uint32_t bmsErrorFlags; // keep track of error flags from BMS (forif bms led strobe)

  //temperature
  uint8_t tempsCelsius[32];
  uint8_t tempsFahrenheit[32];
  uint8_t maxTemp = 0; // FOR DEBUGGING
  uint8_t avgTemp;

  bool validTemp;

  bool dischargeEnable;
  bool chargeEnable;
  bool MPOEnable;
};

//----------------------------DATA/VARIABLES---------------------------//
// CAN variables
CAN_IO canControl(CS_PIN, INTERRUPT_PIN, BAUD_RATE, FREQ);

// car state
CarState state;

// timers
Metro bmsHbTimer(BMS_HB_INTERVAL); // bms heartbeat
Metro mcHbTimer(MC_HB_INTERVAL);   // motor controller heartbeat
Metro mcReqTimer(MC_REQ_INTERVAL);
Metro strobeTimer(TOGGLE_INTERVAL);

//Metro dcPowerTimer(DC_POWER_INTERVAL);
Metro tempConvertTimer(TEMP_CONV_INTERVAL);      // timer for issuing convert command to temp sensors
Metro tempReadTimer(TEMP_READ_INTERVAL);         // timer for reading the values from temp sensorss
Metro tempSendTimer(TEMP_SEND_INTERVAL);         // timer for reading the values from temp sensorss
Metro tempOverheatTimer(TEMP_OVERHEAT_INTERVAL); // timer for indication of overheat from temp sensors
Metro mpptReqTimer(MPPT_REQ_INTERVAL);           //Timer for MPPT
Metro mcSendTimer(MC_SEND_INTERVAL);

// debugging variables
#ifdef DEBUG
Metro debugTimer(DEBUG_INTERVAL); // timer for debug output over serial
long loopStartTime = 0;
long loopSumTime = 0;
int loopCount = 0;
#endif

//temp sense
byte tempCount = 0;     // keeps track of how many times readTempSensor() has run
byte tempSendCount = 0; // keeps track of which temp packet to send out over can
byte addr[8];

byte i;  // just used as a counter variable to print out ALL temperatures in the serial debug section
float j; // used as a counter when calculating Avg and Max temperatures

//throttle variables
uint16_t readings[2];   // current and previous readings of throttle
uint8_t setpoint[2];    // current and previous throttle velocity setpoints


//--------------------------HELPER FUNCTIONS--------------------------//
/*
 * Reads analog throttle, converts to velocity setpoint, returns velocity setpoint
 */
 void readThrottle()
 {
  readings[1] = readings[0];
  readings[0] = analogRead(THROTTLE_PIN);
  setpoint[1] = setpoint[0];
  setpoint[0] = (int) map(readings[0],0,1023,0, MAX_VELOCITY);
 }

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN()
{
  int safetyCount = 0;
  while (canControl.Available() && safetyCount <= MAX_CAN_PACKETS_PER_LOOP)
  {                               // there are messages
    safetyCount++;                // Increment safety counter
    Frame &f = canControl.Read(); // read one message
#ifdef DEBUG
//Serial.println("\n\nGetting CAN Packet");
#endif

    // Needs to check if BMS packet
    if (f.id == BMS19_VCSOC_ID)
    {
      BMS19_VCSOC packet(f);
      bmsHbTimer.reset();                 // Updates here as well since there is only BMS packet we can read
      state.dcErrorFlags &= ~BMS_TIMEOUT; // clear flag
      //Serial.print("SOC Packet");
      //Serial.println(f.toString());
      state.bmsPercentSOC = packet.packSOC;
      state.bmsCurrent = packet.current;
    }
    else if (f.id == BMS19_TRIP_STAT_ID)
    {
      BMS19_Trip_Stat packet(f);
      bmsHbTimer.reset();
      state.dcErrorFlags &= ~BMS_TIMEOUT; // clear flag
      // For the following, default packet output is 0
      state.dischargeEnable = packet.dischargeRelay;
      state.chargeEnable = packet.chargeRelay;
      state.MPOEnable = packet.MPO;
    }

    // There are two main masks,
    else if (f.id == MTBA_FRAME0_REAR_LEFT_ID)
    { // source is mc, checks if bit at address
      MTBA_F0_RLeft packet(f);
      mcHbTimer.reset();
      state.dcErrorFlags &= ~MC_TIMEOUT; // clear flag
      state.motorVelocity[0] = packet.motor_rotating_speed;
      state.carVelocity[0] = packet.motor_rotating_speed * RPM_TO_MPH;
      //state.carVelocity[0] = packet.car_velocity * M_PER_SEC_TO_MPH; NO LONGER EXISTS
    }
    else if (f.id == MTBA_FRAME0_REAR_RIGHT_ID)
    { // source is mc, checks if bit at address
      MTBA_F0_RRight packet(f);
      mcHbTimer.reset();
      state.dcErrorFlags &= ~MC_TIMEOUT; // clear flag
      state.motorVelocity[1] = packet.motor_rotating_speed;
      state.carVelocity[1] = packet.motor_rotating_speed * RPM_TO_MPH;
      //state.carVelocity[0] = packet.car_velocity * M_PER_SEC_TO_MPH; NO LONGER EXISTS
    }
#ifdef DEBUG
    else
    {
      Serial.print("Unk. Packet: ");
      Serial.println(frameToString(f));
    }
#endif
  }
}

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers()
{
  // check motor controller
  if (mcHbTimer.expired())
  {                                   // motor controller timeout
    state.dcErrorFlags |= MC_TIMEOUT; // set flag
  }

  // check bms
  if (bmsHbTimer.expired())
  {                                    // bms timeout
    state.dcErrorFlags |= BMS_TIMEOUT; // set flag
  }
}

/*
 * Process information read from GPIO and CAN and updates
 * the car state accordingly.
 */
void updateState()
{
  if (strobeTimer.check())
  {
    if (~state.bmsIgnition)
    {
      state.bmsStrobeToggle = !state.bmsStrobeToggle;
    }
    else
    {
      state.bmsStrobeToggle = false;
    }
  }

  // Trip if temp sensors are overtemp, temperature threshold depending on whether discharge or charge
  if ((state.bmsCurrent < 0.0 && state.maxTemp >= DISCHARGE_TEMP) ||
      state.maxTemp >= CHARGE_TEMP)
  { // Uses short circuiting
    // negative current, discharge
    state.validTemp = false;
    state.bmsIgnition = true;
  }
  else
  {
    state.validTemp = true;
  }

  // BMS 19 Strobe light, if any conditions are false- trip strobe
  if (state.chargeEnable && state.dischargeEnable && state.MPOEnable)
  {
    state.bmsIgnition = true;
  }
  else
  {
    state.bmsIgnition = false;
  }
}

/*
 * Sets general purpose output according to car state.
 */
void writeOutputs()
{
  digitalWrite(BMS_STROBE_PIN, state.bmsStrobeToggle ? HIGH : LOW);
  // If Strobe light in on, cut ignition
  digitalWrite(IGNITION_PIN, state.bmsIgnition ? HIGH : LOW);
}

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN()
{
  if (tempSendTimer.check())
  {
    switch (tempSendCount)
    {
    case 0:
      canControl.Send(DC_Temp_0(state.maxTemp, state.avgTemp, state.tempsCelsius), TXBANY);
      break; // send first 6 temps + min and average
    case 1:
      canControl.Send(DC_Temp_1(state.tempsCelsius + 6), TXBANY);
      break; // temps 7 - 14
    case 2:
      canControl.Send(DC_Temp_2(state.tempsCelsius + 14), TXBANY);
      break; // temps 15 - 22
    case 3:
      canControl.Send(DC_Temp_3(state.tempsCelsius + 22), TXBANY);
      break; // temps 23 - 26
    }

    tempSendCount = (tempSendCount + 1) % 4;
    // Don't reset yet because then the temperature sensor code will never see the timer expired.
  }

  if (tempOverheatTimer.check())
  {
    canControl.Send(BMS19_Overheat_Precharge(state.validTemp, false), TXBANY);
  };

  // MPPT Packet Request, response handled by dashboard LCD
  if (mpptReqTimer.check())
  {
    canControl.Send(MPPT_Request(MPPT_LEFT_OFFSET), TXBANY);
    canControl.Send(MPPT_Request(MPPT_RIGHT_OFFSET), TXBANY);
    canControl.Send(MPPT_Request(MPPT_SUB_OFFSET), TXBANY);
  }

  if (mcReqTimer.check())
  {
    canControl.Send(MTBA_ReqCommRLeft(1u), TXBANY); //We only care about frame 0
    canControl.Send(MTBA_ReqCommRRight(1u), TXBANY);
  }

  //throttle control
  if(mcSendTimer.check())
  {
    canControl.Send(TRI88_Drive(setpoint[0], 100),TXBANY);  //Velocity control with 100% torque
  }
}

/*
 * Checks the CAN controller and any other components for errors.
 * If errors exist, updates the error state.
 */
void checkErrors()
{
  //Check the can bus for errors
  canControl.FetchErrors();

  // Reset the MCP if we are heading towards a bus_off condition
  if (canControl.tec > 200 || canControl.rec > 200)
  {
#ifdef DEBUG
    Serial.println("Reseting MCP2515");
    Serial.print("TEC/REC: ");
    Serial.print(canControl.tec);
    Serial.print(" / ");
    Serial.println(canControl.rec);
#endif
    canControl.ResetController();
#ifdef DEBUG
    Serial.println("Reset MCP2515");
#endif

    state.dcErrorFlags |= RESET_MCP2515;
  }

  // Check the mode of the MCP2515 (sometimes it is going to sleep randomly)
  canControl.FetchStatus();

  if ((canControl.canstat_register & 0b00100000) == 0b00100000)
  {
    canControl.ResetController();
    canControl.FetchStatus(); // check that everything worked
#ifdef DEBUG
    Serial.print("MCP2515 went to sleep. CANSTAT reset to: ");
    Serial.println(canControl.canstat_register);
#endif
  }
}

void ReadTempSensor()
{

  if (tempReadTimer.check())
  { // temp sensor timeout

    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];

    type_s = 0; //we're only using DS18B20 devices (as opposed to other OneWire devices)

    if (!ds.search(addr))
    {
      ds.reset_search();
      //Serial.print("no more addresses");
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
      //Serial.println("CRC is not valid!");
      return;
    }

    if (tempCount == 0)
    {
      ds.reset();
      ds.skip();         // tell all sensors on bus
      ds.write(0x44, 0); // to convert temperature
      tempCount++;
      tempConvertTimer.reset();
    }
    else if ((tempCount > 0) && tempConvertTimer.expired())
    {
      present = ds.reset();

      ds.select(addr);
      ds.write(0xBE); // Read Scratchpad

      //Serial.print("  Data = ");
      //Serial.println(present, HEX);
      //Serial.print(" ");
      for (i = 0; i < 9; i++)
      { // we need 9 bytes
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
      if (type_s)
      {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10)
        {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      }
      else
      {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
          raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
          raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
          raw = raw & ~1; // 11 bit res, 375 ms
                          //// default is 12 bit resolution, 750 ms conversion time
      }

      state.tempsCelsius[tempCount - 1] = (raw >> 4);
      if (state.tempsCelsius[tempCount - 1] >= 84)
      {
        state.tempsCelsius[tempCount - 1] = 0;
      }
      state.tempsFahrenheit[tempCount - 1] = (uint8_t)(state.tempsCelsius[tempCount - 1] * 1.8 + 32.0);

      tempCount++;

      if (tempCount >= NUM_TEMP_MODULES + 1)
      {
        tempCount = 0; //reset count after tempCount has cycled thru all 26 sensors

        // Calculate Average Temperature  //
        j = 0; // contains sum of temps
        for (i = 0; i < NUM_TEMP_MODULES; i++)
        {
          j = j + state.tempsCelsius[i];
        }
        state.avgTemp = j / NUM_TEMP_MODULES;

        // Calculate Max Temperature //
        j = 0;
        for (i = 0; i < NUM_TEMP_MODULES; i++)
        {
          if (state.tempsCelsius[i] > j)
          {
            j = state.tempsCelsius[i];
          }
        }
        state.maxTemp = j;
      }
    }
  }
}

//--------------------------MAIN FUNCTIONS---------------------------//
void setup()
{
  pinMode(BOARDLED, OUTPUT);
  pinMode(BMS_STROBE_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, INPUT);

  digitalWrite(BOARDLED, HIGH); // Turn on durring initialization

  // debugging [ For some reason the board doesn't work unless I do this here instead of at the bottom ]

  Serial.begin(SERIAL_BAUD);
  Serial.println("Serial Initialized");

  // init car state
  state = {}; // init all members to 0
  state.bmsIgnition = false;

  // set the watchdog timer interval
  WDT_Enable(WDT, 0x2000 | WDT_INTERVAL | (WDT_INTERVAL << 16));

  // reset timers
  bmsHbTimer.reset();
  tempConvertTimer.reset();
  tempReadTimer.reset();

#ifdef DEBUG
  debugTimer.reset();
#endif

  // setup CAN
  canControl.filters.setRB0(RXM0, RXF0, RXF1, true);
  canControl.filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5, false);
  canControl.Setup(RX0IE | RX1IE | TX0IE | TX1IE | TX2IE);

  //Enabling One-Shot mode (debugging)
  //canControl.controller.Write(CANCTRL, 00001111);
  //Serial.println((canControl.controller.Read(CANCTRL)), BIN);

  // Toggles BMS Precharge Signal after initialization
  // WARNING: DOES NOT CHECK FOR OVERHEAT AT THIS POINT
  Serial.print("Waiting for precharge...");
  canControl.Send(BMS19_Overheat_Precharge(false, false), TXBANY);
  delay(5000);
  Serial.println(" Ready");
  canControl.Send(BMS19_Overheat_Precharge(false, true), TXBANY);
  ;
  digitalWrite(BOARDLED, LOW); // Turn of led after initialization

  if (canControl.errors != 0)
  {
    Serial.print("Init CAN error: ");
    Serial.println(canControl.errors, HEX);
  }
}

void loop()
{
// Start timer
#ifdef DEBUG
  loopStartTime = micros();
#endif

  //read temp sensors
  ReadTempSensor();

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
#ifdef DEBUG
  loopSumTime += micros() - loopStartTime;
  loopCount += 1;
  // debugging printout

  if (debugTimer.expired())
  {

    /************************ TEMP CAN DEBUGGING VARIABLES (DELETE LATER)******/
    byte Txstatus[3] = {0, 0, 0};
    Txstatus[0] = canControl.controller.Read(TXB0CTRL);
    Txstatus[1] = canControl.controller.Read(TXB1CTRL);
    Txstatus[2] = canControl.controller.Read(TXB2CTRL);
    byte canintf = 0;
    canintf = canControl.last_interrupt;
    byte canctrl = 0;
    canctrl = canControl.controller.Read(CANCTRL);

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
    Serial.print("Average Loop time (us): ");
    Serial.println(loopSumTime / loopCount);
    Serial.print("System time: ");
    Serial.println(millis());
    Serial.println();

    switch (debugStep)
    {
    case 0:
      /*
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
        Serial.print("Trip Flag: "); */
      //Serial.println(state.tripFlag,HEX);
      break;
    case 1:
      Serial.print("Ignition: ");
      Serial.println(state.bmsIgnition, HEX);
      Serial.print("Strobe Toggle: ");
      Serial.println(state.bmsStrobeToggle, HEX);
      /*
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
        Serial.println(state.leftTurnOn ? "YES" : "NO"); */
      //Serial.print("Hazards: ");
      //Serial.println(state.hazards ? "YES" : "NO");
      Serial.print("BMS Strobe: ");
      Serial.println(state.bmsIgnition ? "YES" : "NO");
      Serial.print("BMS SOC: ");
      Serial.println(state.bmsPercentSOC);
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
      Serial.print(canControl.tec);
      Serial.print(" / ");
      Serial.println(canControl.rec);
      Serial.print("Interrupt Counter: ");
      Serial.println(canControl.int_counter);
      Serial.print("RX buffer counter: ");
      Serial.println(canControl.RXbuffer.size());
      Serial.print("Board error: ");
      Serial.println(state.dcErrorFlags, HEX);
      Serial.print("BMS Current: ");
      Serial.println(state.bmsCurrent);
      if (state.SW_timer_reset_by != 0)
      {
        Serial.print("SW RESET BY: ");
        Serial.println(state.SW_timer_reset_by, HEX);
        state.SW_timer_reset_by = 0;
      }
      break;
    case 3:
      for (i = 0; i < 26; i++)
      {
        Serial.print("Temperature = ");
        Serial.print(state.tempsCelsius[i]);
        Serial.print(" Celsius, ");
        Serial.print(state.tempsFahrenheit[i]);
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

    debugStep = (debugStep + 1) % 4;
    debugTimer.reset();

    // Reset loop timer variables
    loopSumTime = 0;
    loopCount = 0;
  }
#endif
  // Reset canErrorFlags after each loop.
  state.canErrorFlags = 0;
}
