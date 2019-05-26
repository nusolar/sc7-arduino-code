#include <CAN_IO.h>
#include <Metro.h>
#include <SPI.h>
#include <RA8875.h>
#include "src/sc7Dashboard_UI.h"

#define RA8875_INT 4
#define RA8875_CS 10
#define RA8875_RESET 9

#define DEBUG

/*Defining the bitwise functions (bitwise operators)
  We're using bits to store data because there are only 8 bytes available for use in a CAN packet.
  We can store all of the necessary data in a single byte and save space. This frees up space to
  send other data that we might need in the future between the steering wheel and the driver controls*/
#define BIT(n) (1 << (n))
#define BIT_SET(y, mask) (y |= (mask))                               //Set the value of BIT = 1
#define BIT_CLEAR(y, mask) (y &= ~(mask))                            //Set the value of BIT = 0
#define BIT_FLIP(y, mask) (y ^= (mask))                              //Flip the value of BIT
#define BIT_DIFFERENT(y, x, mask) (y & mask == x & mask)             //See if two bits are different values
#define BIT_CHECK(y, bit, mask) (BIT_DIFFERENT(y, (bit)*mask, mask)) //check function utilizing the previous defined different function

//Defining the bit masks
#define FWD_GEAR BIT(0) // might be diff with the new motors
#define REV_GEAR BIT(1) // might be diff with the new motors
#define HEADLIGHT BIT(2)
#define HAZARDLIGHT BIT(3)
//#define LAP_TIMER BIT(4)
#define HORN BIT(5)
#define LEFT_TURN BIT(6)
#define RIGHT_TURN BIT(7)

/*These are the bytes containing the above-mentioned bits. The two bytes will be used
  to compare the switch states and determine whether there has been a change or not.*/
char young = 0xFF; //young is continuously assigned to the new switch states 255
char old;          //old is the previous switch states

//Conversion data for displayed measurements
#define RPM_TO_MPH 2.2369f //Change for correct values, diameter 19 inch

//BMS Error Strings
#define BMS_12VERR_STR String("TRIP: 12V ERR")
#define BMS_DRIVECONTROLSERR_STR String("TRIP: DCTRLS ERR")
#define BMS_CONTACTERR_STR String("TRIP: CNTCR ERR")
#define BMS_CURRENTERR_STR String("TRIP: OVERCURRENT") // This currently doesn't do anything since the BMS doesn't report current trips
#define BMS_OVVOLTAGE_STR String("TRIP: OVER V")
#define BMS_UNVOLTAGE_STR String("TRIP: UNDER V")
#define BMS_UNKERR_STR String("TRIP: UNK. ERR")
#define GENERIC_TRIP_STR String("SOMETHING IS WRONG WITH YOUR CAR")

//set up pins that connect to switch terminals
//const int fgp =   6  //forward gear --> don't have these rn for sc7
//const int rgp =   7;  //reverse gear
#define hdp A4           //headlights
#define hzp A5           //hazardlights
#define brkp A6          //brakes
#define strbp A7         //strobe light
#define hrnp A8          //horn
#define ltp A9           //left turn
#define rtp A10          //right turn

// miso, mosi, and sck as well

//set up metro timer, these may be diff, check later
//1st: switch state reading timer - frequency at which switches are read
Metro switch_timer = Metro(100);
//2nd: CAN Transmission timer - frequency at which CAN packets are sent if switch states have not changed
Metro CAN_TX = Metro(1000);
//3rd: CAN Reception timer - duration between CAN packets received (will trigger error if it expires)
Metro CAN_RX = Metro(1000);
//4th: Notification Timer - duration for which notification is displayed
Metro notif_timer = Metro(500);
//5th: Display Timer - frequency at which display will refresh if nothing changes
Metro display_timer = Metro(500);
//6th: Turn signal blinking timer
Metro blinking_timer = Metro(500);
//7th: Debug Timer
Metro debug_timer = Metro(200);
//8th: Telemetry HB timer
Metro telmetry_timer = Metro(2500);

//CAN parameters --> check these, may be diff
const byte     CAN_CS    = 52; // CAN computer system
const byte     CAN_INT   = 14; //10 CAN Interrupt 
const uint16_t CAN_BAUD_RATE = 250; // CAN Baud Rate
const byte     CAN_FREQ      = 16; //CAN Frequency 
uint16_t errors;

CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ); 

//Indicators:
float VELOC;       // Velocity (from CAN)
float BAT_CURRENT; // Battery Current (from CAN)
float MIN_BAT;
float ARRAY_CURRENT;   // Array Current (from CAN)
float MAX_TEMPERATURE; // Max Pack Temperature (from CAN)
float avgTemp;

String HAZARD_LIGHT; // Hazard Lights
boolean LEFT_LIGHT;  // Left Turn Signal
boolean RIGHT_LIGHT; // Right Turn Signal
boolean BRAKES;      // Indicate if brakes are deployed
String Err;          // Error message
boolean TRIPPED;
uint16_t tx, ty;

RA8875 lcd(RA8875_CS, RA8875_RESET);
sc7Dashboard_UI tft(lcd);

void setup()
{
  // put your setup code here, to run once:

  pinMode(hdp, INPUT_PULLUP);           // headlights
  pinMode(hzp, INPUT_PULLUP);           // hazardlights
  pinMode(brkp, INPUT_PULLUP);          // brakes
  pinMode(strbp, INPUT_PULLUP);         // strobe lights
  pinMode(hrnp, INPUT_PULLUP);          // horn
  pinMode(ltp, INPUT_PULLUP);           // left turn
  pinMode(rtp, INPUT_PULLUP);           // right turn
  //pinMode(caninterruptp, INPUT_PULLUP); // CAN interrupt
  //pinMode(canchipp, INPUT_PULLUP);      // CAN chip select

  Serial.begin(115200);
  delay(3000);
  
  tft.begin();

 // initializePins(); // Part of SWControls
 const uint16_t RXM0      = MASK_Sxxx;
 const uint16_t RXF0      = 0; // Match any steering_wheel packet (because mask is Sx00)
 const uint16_t RXF1      = BMS19_VCSOC_ID; // Can't put 0 here, otherwise it will match all packets that start with 0.

 const uint16_t RXM1      = MASK_Sxxx;
 const uint16_t RXF2      = SW_DATA_ID;
 const uint16_t RXF3      = 0; // No longer relevant, but keeping here to have a value
 const uint16_t RXF4      = (MTBA_FRAME0_REAR_LEFT_ID & MASK_Sxxx); // Not sure if necessary, but MTBA IDs are 29 bits
 const uint16_t RXF5      = (MTBA_FRAME0_REAR_RIGHT_ID & MASK_Sxxx); 

  CanControl.filters.setRB0(MASK_Sxxx, RXF0, RXF1);
  CanControl.filters.setRB1(MASK_Sxxx, RXF2, RXF3, RXF4, RXF5); //**MC_VELOCITY_ID, **MC_PHASE_ID
  CanControl.Setup(RX0IE | RX1IE | TX1IE | TX2IE | TX0IE);

  // Insert DEBUG and LOOPBACK steps
  #ifdef DEBUG
    if (CanControl.errors != 0) {
      Serial.print("Init CAN error: ");
      Serial.println(CanControl.errors, HEX);

      byte Txstatus[3] = {0,0,0};
      Txstatus[0] = CanControl.controller.Read(TXB0CTRL);
      Txstatus[1] = CanControl.controller.Read(TXB1CTRL);
      Txstatus[2] = CanControl.controller.Read(TXB2CTRL);
      byte canintf = 0; canintf = CanControl.last_interrupt;
      byte canctrl = 0; canctrl = CanControl.controller.Read(CANCTRL);
      
      Serial.println("TXnCTRL: ");
      Serial.println(Txstatus[0], BIN);
      Serial.println(Txstatus[1], BIN);
      Serial.println(Txstatus[2], BIN);
      Serial.print("Last Interrupt: ");
      Serial.println(canintf, BIN);
      Serial.print("CANCTRL: ");
      Serial.println(canctrl, BIN);
      Serial.print("CANSTAT: ");
      Serial.println(CanControl.canstat_register, BIN);
      Serial.println("");
    }
  #endif
}

void loop()
{
  // Fetch any potential messages from the MCP2515
  CanControl.Fetch();

  if (CanControl.Available())
  {

    // Use available CAN packets to assign values to appropriate members of the data structures
    Frame &f = CanControl.Read();
#ifdef DEBUG
    Serial.print("Received: ");
    Serial.println(f.id, HEX);
#endif
    switch (f.id)
    {
    case BMS19_VCSOC_ID: // Voltage/Current of battery
    {
      BMS19_VCSOC packet(f); //Get the voltage and current of the battery pack
      BAT_CURRENT = packet.current / 1000.0;
      tft.updateBatCurr(BAT_CURRENT);
      CAN_RX.reset();
      break;
    }
    /*case BMS19_MinMaxTemp_ID:
    {
      BMS19_MinMaxTemp packet(f);
      MAX_TEMPERATURE = packet.maxTemp;
      updateMaxTemp(MAX_TEMPERATURE);
      CAN_RX.reset();
      break;
    } */
    case MTBA_FRAME0_REAR_LEFT_ID: //Velocity: 19 inch diameter of wheels, figure out conversion factor
    {
      MTBA_F0_RLeft packet(f);
      VELOC = packet.motor_rotating_speed * RPM_TO_MPH;
      CAN_RX.reset();
      break;
    }
    /* case BMS19_BATT_STAT_ID: 
        {
          BMS19_Batt_Stat packet(f);
          VELOC = packet.motor_rotating_speed * RPM_TO_MPH;
          CAN_RX.reset();
          break;
        } */
    case DC_TEMP_0_ID: // Get Max Pack Temp
    {
      DC_Temp_0 packet(f);
      MAX_TEMPERATURE = packet.max_temp;
      avgTemp = packet.avg_temp;
      tft.updateMaxTemp(MAX_TEMPERATURE);
      tft.updateAvgTemp(avgTemp);
      CAN_RX.reset();
      break;
    }
    case DC_INFO_ID:
    {
      DC_Info packet(f); // Get Tripped state of vehicle
      TRIPPED = packet.tripped;
      if (TRIPPED)
      {
        String errorStr = GENERIC_TRIP_STR;
        tft.updateError(errorStr);
        //notif_timer.resOPet(); //res0pet doesn't exsist
      }
      else
      {
        tft.updateError("");
      }

      CAN_RX.reset();
      break;
    }
    /*case BMS_STATUS_EXT_ID:
    {
      BMS_Status_Ext packet(f); // extract the flags
      //if      (packet.flags & BMS_Status_Ext::F_OVERVOLTAGE)    {notif_timer.reset(); steering_wheel.notification = BMS_OVVOLTAGE_STR;}
      //else if (packet.flags & BMS_Status_Ext::F_UNDERVOLTAGE)   {notif_timer.reset(); steering_wheel.notification = BMS_UNVOLTAGE_STR;}
      //else if (packet.flags & BMS_Status_Ext::F_12VLOW)         {notif_timer.reset(); steering_wheel.notification = BMS_12VERR_STR;}
      CAN_RX.reset();
      break;
    }
    case TEL_HEARTBEAT_ID:
    {
      //steering_wheel.telemetrydisplay = 'T';
      //telmetry_timer.reset();
    } */
    }
  }
}