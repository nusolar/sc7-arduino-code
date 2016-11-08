/*
 * TemperatureBoard.ino
 * Contains code to run temperature board for sc7.
 */
#include <stdint.h>
#include "sc7-can-libinclude.h"

// pins
const int MUXA_PIN = 51; //lsb
const int MUXB_PIN = 52;
const int MUXC_PIN = 53; //msb

// constants
const float MIN_RESISTANCE = 1455.4; // from NTCS0805E3103FLT datasheet corresponding to 85 C
const bool DEBUG = true;

// CAN parameters
const byte     CAN_CS        = 50;
const byte     CAN_INT       = 48;
const uint16_t CAN_BAUD_RATE = 1000;
const byte     CAN_FREQ      = 16;
const int      MAX_CAN_PACKETS_PER_LOOP = 10; // Maximum number of receivable CAN packets per loop

// Set up the can controller object.
CAN_IO CanControl(CAN_CS,CAN_INT,CAN_BAUD_RATE,CAN_FREQ);

// type definitions
/*
 * Structure to store DC_Info so that when we write DC_Info packet with shut down message,
 * the rest of the packet will be correct. Contains all data from DC_Info except fuel door
 */
struct DC_State {
  float accel_ratio;
  float regen_ratio;
  bool brake_engaged;
  uint16_t can_error_flags;
  byte dc_error_flags;
  bool was_reset;
  byte gear;
  uint16_t ignition_state;
};


// variables
unsigned long previous_send_time = 0;
DC_State state;


// functions
/*
 * Reads packets from CAN message queue. Stores actual DC_Info data to allows us to write an altered DC_Info packet to the 
 * BMS to shut down the car.
 */
void readCAN() {
  int safetyCount = 0;  
  while(CanControl.Available() && safetyCount <= MAX_CAN_PACKETS_PER_LOOP) { // there are messages
    safetyCount++;                // Increment safety counter
    
    Frame& f = CanControl.Read(); // read one message

    if (f.id == DC_INFO_ID)
    {
      DC_Info packet(f);

      state.accel_ratio = packet.accel_ratio;
      state.regen_ratio = packet.regen_ratio;
      state.brake_engaged = packet.brake_engaged;
      state.can_error_flags = packet.can_error_flags;
      state.dc_error_flags = packet.dc_error_flags;
      state.was_reset = packet.was_reset;
      state.gear = packet.gear;
      state.ignition_state = packet.ignition_state;
    }
  }
}

/*
 * Writes DC_INFO CAN packet with fuel door set to false, all other data unaltered. This directs BMS to shut down
 */
void shutDown() {
  if (millis() - previous_send_time > 500) //Checks time to see if packets need to be sent out over the CAN bus.
  {
    // create packet
    DC_Info packet(state.accel_ratio, state.regen_ratio, state.brake_engaged,
                            state.can_error_flags, state.dc_error_flags, state.was_reset, 
                            false, // fuel door, which we use to control the BMS since the ignition switch doesn't work.
                            state.gear, state.ignition_state);

    //attempt to send packet, reset timer if succesful
    Serial.println("Tried it");
    //bool trysend = CanControl.SendVerified(packet, TXBANY);
    bool trysend = CanControl.Send(Telm_Heartbeat(false,false),TXB0);
    if (trysend)                        
      previous_send_time = millis();
      Serial.println("Sent it");
  }
}

void checkErrors() {
  //Check the can bus for errors
  CanControl.FetchErrors();
  
  // Reset the MCP if we are heading towards a bus_off condition
  if (CanControl.tec > 200 || CanControl.rec > 200) {
    if (DEBUG) {
      Serial.println("Reseting MCP2515");
      Serial.print("TEC/REC: ");
      Serial.print(CanControl.tec); Serial.print(" / "); Serial.println(CanControl.rec);
    }
    CanControl.ResetController();
    if (DEBUG) {
      Serial.println("Reset MCP2515");
    }

  }
  
  // Check the mode of the MCP2515 (sometimes it is going to sleep randomly)
  CanControl.FetchStatus();

  if ((CanControl.canstat_register & 0b00100000) == 0b00100000) {
    CanControl.ResetController();
    CanControl.FetchStatus(); // check that everything worked
    if (DEBUG) {
       Serial.print("MCP2515 went to sleep. CANSTAT reset to: ");
       Serial.println(CanControl.canstat_register);
    }
  }
}

const uint16_t RXM0      = MASK_Sxxx;
const uint16_t RXF0      = 0; // Match any steering_wheel packet (because mask is Sx00)
const uint16_t RXF1      = BMS_VOLT_CURR_ID; // Can't put 0 here, otherwise it will match all packets that start with 0.

const uint16_t RXM1      = MASK_Sxxx;
const uint16_t RXF2      = SW_DATA_ID;
const uint16_t RXF3      = 0; // Most useless: replace first (soc not used by DC currently)
const uint16_t RXF4      = MC_VELOCITY_ID;
const uint16_t RXF5      = MC_BUS_STATUS_ID;

void setup() {
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  Serial.begin(9600);
  Serial.println("Connected");

  //CanControl.Setup();
  CanControl.filters.setRB0(MASK_Sxxx,DC_INFO_ID, 0); 
  CanControl.filters.setRB1(MASK_Sxxx,0,0,0,0);

  CanControl.Setup(RX0IE|RX1IE);
  
//CanControl.filters.setRB0(RXM0, RXF0, RXF1);
//  CanControl.filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
//  CanControl.Setup();
  if (DEBUG && CanControl.errors != 0) {
    Serial.print("Init CAN error: ");
    Serial.println(CanControl.errors, HEX);
  }
}


void loop() {
  // get any CAN messages that have come in to keep DC_INFO up to date
  CanControl.Fetch();
  readCAN();
  checkErrors();
  
  // loop through binary 0-7 for mux selectors
  // and check for out of bounds temperatures
  for (int bit1 = 0; bit1 < 2; bit1++) {
    for (int bit2 = 0; bit2 < 2; bit2++) {
      for (int bit3 = 0; bit3 < 2; bit3++) {
        
        // write to selectors
        int decimal = 0;
        if (bit1 == 1) {
          digitalWrite(MUXC_PIN, HIGH);
          decimal = decimal + 4;
        }
        else {
          digitalWrite(MUXC_PIN, LOW);
        }
        
        if (bit2 == 1) {
          digitalWrite(MUXB_PIN, HIGH);
          decimal = decimal + 2;
        }
        else {
          digitalWrite(MUXB_PIN, LOW);
        }
        
        if (bit3 == 1) {
          digitalWrite(MUXA_PIN, HIGH);
          decimal = decimal + 1;  
        }
        else {
          digitalWrite(MUXA_PIN, LOW);
        }

        // read 4 analog signals
        for (int n = 0; n < 4; n++) {
          int analogSignal = analogRead(n);
          float voltage = analogSignal * (3.3 / 1023.0);
          float current = voltage / 10000.0;
          float resistance = (3.3 - voltage) / current;
          Serial.print("Mux num: ");
          Serial.print(n);
          Serial.print(" Mux input: ");
          Serial.print(decimal);
          Serial.print(" Analog read: ");
          Serial.print(analogSignal);
          Serial.print(" Resistance: ");
          Serial.println(resistance);
          delay(200);

          state.accel_ratio = 0;
          state.regen_ratio = 0;
          state.brake_engaged = false;
          state.can_error_flags = 0;
          state.dc_error_flags = 0;
          state.was_reset = false;
          state.gear = 0;
          state.ignition_state = 0;
          shutDown();
          
          if (resistance < MIN_RESISTANCE) {
            Serial.println("****TOO HOT STOPPPPP****");
            shutDown();
          }
        }
        
      }
    }
  }
}
