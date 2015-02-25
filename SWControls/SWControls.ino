#include "sc7-can-libinclude.h"
#include <Metro.h>
#include <Switch.h>
#include <serLCD.h>

#include <SPI.h>
 
 #define BIT(n)                   ( 1<<(n) ) 
 #define BIT_SET(y, mask)         ( y |=  (mask) ) 
 #define BIT_CLEAR(y, mask)       ( y &= ~(mask) ) 
 #define BIT_FLIP(y, mask)        ( y ^=  (mask) )
 #define BIT_DIFFERENT(y, x, mask)( y & mask == x & mask )
 #define BIT_CHECK(y, bit, mask)  ( BIT_DIFFERENT(y, (bit)*mask, mask) )

/* Bit masks */
#define FWD_GEAR BIT(0)
#define REV_GEAR BIT(1)
#define HEADLIGHT BIT(2)
#define HAZARDLIGHT BIT(3)
#define CRUISE_CONTROL BIT(4)
#define HORN BIT(5)
#define LEFT_TURN BIT(6)
#define RIGHT_TURN BIT(7)
byte young = 1;
byte old;

//Steering Wheel LCD Info
const int SOC = 8;
const int V = 6;
const int GEAR = 13;
const int CC = 11;
const int LIGHT = 12;
const int RIGHT = 15;
const int LEFT = 1;

//set up pins that connect to switch terminals
   const int fgp =   9;
   const int rgp =   8;
   const int hp =    7;
   const int hzp =   6;
   const int ccp =   3;
   const int hornp = 10;
   const int ltp =   4;
   const int rtp =   5;

//set up metro timer
  //1st: switch state reading timer
  Metro switch_timer = Metro(20);
  //2nd: CAN Transmission timer
  Metro CAN_TX = Metro(1000);
  //3rd: CAN Reception timer
  Metro CAN_RX = Metro(1000);
  //4th: Cruiser Control display timer
  Metro CC_timer = Metro(2000);


// CAN parameters
const byte	   CAN_CS 	 = A0;
const byte	   CAN_INT	 = 1;
const uint16_t CAN_BAUD_RATE = 1000;
const byte     CAN_FREQ      = 16;
const uint16_t RXM0      = MASK_NONE;
const uint16_t RXM1      = MASK_NONE;
const uint16_t RXF0      = MASK_NONE;
const uint16_t RXF1      = MASK_NONE;
const uint16_t RXF2      = MASK_NONE;
const uint16_t RXF3      = MASK_NONE;
const uint16_t RXF4      = MASK_NONE;
const uint16_t RXF5      = MASK_NONE;
      uint16_t CAN_errors;
      
CAN_IO CanControl(CAN_CS,CAN_INT,CAN_BAUD_RATE,CAN_FREQ);

Switch cruisecontrol(ccp);
Switch horn(hornp);

boolean cruisecontroltoggle = false;

serLCD screen(Serial1);

//setup a display structure to store the shenanigans that we neeed to display on LCD
struct LCD{
  char CCdisplay;
  char geardisplay;
  char Lightsdisplay[2];
  int SOCdisplay;
  int Veldisplay;
  boolean LTdisplay;
  boolean RTdisplay;
};
LCD steering_wheel;   

void setup() {
  // Pin Modes
  pinMode(fgp, INPUT_PULLUP);
  pinMode(rgp, INPUT_PULLUP);
  pinMode(hp, INPUT_PULLUP);
  pinMode(hzp, INPUT_PULLUP);
  pinMode(ccp, INPUT_PULLUP);
  pinMode(hornp, INPUT_PULLUP);
  pinMode(ltp, INPUT_PULLUP);
  pinMode(rtp, INPUT_PULLUP);

/*SPST - left turn, right turn, horn, cruise control
SPDT - forward/neutral/reverse, headlight/no light/hazard*/

  //set Serial baud rate to 9600bps
  Serial.begin(9600);

  //CAN setup
  CANFilterOpt filter;
  filter.setRB0(MASK_NONE,DC_DRIVE_ID,0);
  filter.setRB1(MASK_NONE,DC_SWITCHPOS_ID,0,0,0);
  CanControl.Setup(filter, &CAN_errors, RX0IE|RX1IE|ERRIE);
  
  screen.begin();
  screen.clear();
  screen.setBrightness(25);
}

inline void switchBitFromPin(byte pin, byte& out, byte mask){
  switchBit(digitalRead(pin),out, mask);
}

inline void switchBit(bool b, byte& out, byte mask) {
  if (b){
    BIT_SET(out,mask);
  }else{
    BIT_CLEAR(out,mask);
  }
}

/*copy over the blink function from the LCD testing code, used to blink the sides of the display for the turning signals*/


void loop() {  
/*if the metro timer runs out, then check the states of all the switches
    assign the values to the 'young' byte. Reset switch timer.*/
  if (switch_timer.check() == 1){
    old = young; // Store old switch values.
    switchBitFromPin(fgp,  young,FWD_GEAR);
    switchBitFromPin(rgp,  young,REV_GEAR);
	/*Check the forward/reverse bit flags and assign appropriate value for the member in display structure
    if forward bit flag and reverse bit flag are both 1, then the gear character in the display structure is 'N'
    
    else if forward bit flag is 1 and reverse bit flag is 0, then the gear character in the display structure is 'F'
    
    else if reverse bit flag is 1 and the forward bit flag is 0, then the gear character in the display structure is 'R'
    */
    if (FWD_GEAR == 0 && REV_GEAR == 0){
      steering_wheel.geardisplay = 'N';
    }
    else if (FWD_GEAR == 1 && REV_GEAR == 0){
      steering_wheel.geardisplay = 'F';
    }
    else if (REV_GEAR == 1 && FWD_GEAR == 0){
      steering_wheel.geardisplay = 'R';
    }
    
    /*Check the headlight/hazardlight bit flags and assign appropriate value for the member in display structure
    if headlight bit flag and hazardlight bit flag are both 1, then the light string in the display structure is blank
    
    else if headlight bit flag is 1 and hazardlight bit flag is 0, then the light string in the display structure is 'H'
    
    else if hazardlight bit flag is 1 and headlight bit flag is 0, then the light string in the display structure is 'HZ'
    */
	
	switchBitFromPin(hp,   young,HEADLIGHT);
    switchBitFromPin(hzp,  young,HAZARDLIGHT);
    /*if the left turn bit flag is 1, use blink function mentioned above to blink arrows for the turning signals
    
      else if the left turn bit flag is 0, turn off using the LCD library member functions to clear, may need to create additional function to clear just the turn signal area
    */
	
    switchBitFromPin(ltp,  young,LEFT_TURN);
    switchBitFromPin(rtp,  young,RIGHT_TURN);
    //same as with the left turn signal.
	
    cruisecontrol.poll();
    if(cruisecontrol.pushed()){
      BIT_FLIP(young,CRUISE_CONTROL);
    }
    /*assign appropriate value to the cruise control character in the display structure
    
    if cruise control bit flag is 1, then the cruise control character in the display structure is 'C'
    
    else the cruise control character in the display structure is blank*/
    
    horn.poll();
	switchBit(horn.on(), young, HORN);
    switch_timer.reset();
  }
  
  /*If this byte is different from the one in the void setup() or the CAN_TX timer runs out, send CAN packetxxxxx
    and reset CAN_TX timer.*/
  if(young != old || CAN_TX.check()){
    //Serial.print("ERRORS:");
    //Serial.println(CAN_errors,BIN);
    //Serial.println(young,BIN);
    CanControl.Send(SW_Data(young),TXB0);
    CAN_TX.reset();
    old = young;
  }

  if (CanControl.Available()){
   /*Use available CAN packets (BMS SOC and MC Velocity) to assign values to appropriate members of the data structures
    Frame& f = CanControl.Read();
    if (f.id == MC_BUS_STATUS_ID){
      MC_BusStatus receivedMC(f);
      screen.selectLine(1);
      screen.print("MC Bus Current: ");
      screen.selectLine(2);
      screen.print(receivedMC.bus_current);
      CAN_RX.reset();
	}
    else if (f.id == MC_VELOCITY_ID){
      MC_Velocity receivedVel(f);
      screen.selectLine(1);
      screen.print("Velocity: ");
      screen.selectLine(2);
      screen.print(receivedVel.car_velocity);
      CAN_RX.reset();
    }*/
  }
  else if (CAN_RX.check()){
    screen.print("Communic. lost  with DrivCont");
  }         
}
