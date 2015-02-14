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
 #define BIT_CHECK(y, bit, mask)  ( BIT_DIFFERENT(young, (bit)*mask, mask) )

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
  CanControl.Setup(filter, &CAN_errors);
  
  screen.begin();
  screen.clear();
  screen.setBrightness(25);
}

inline void setyoungbit(byte pin, byte& out, byte mask){
  if (digitalRead(pin)==1){
    BIT_SET(out,mask);
  }
  else{
    BIT_CLEAR(out,mask);
  }
}

inline void setyoungbitspecial(byte pin1, byte pin2, byte& out, byte mask1, byte mask2){
  if ((digitalRead(pin1) == 1)&&(digitalRead(pin2) == 1)){
    BIT_CLEAR(out,mask1);
    BIT_CLEAR(out,mask2);
  }
  else if ((digitalRead(pin1) == 0)&&(digitalRead(pin2) == 1)){
    BIT_SET(out,mask2);
    BIT_CLEAR(out,mask1);
  }
  else if ((digitalRead(pin1) == 1)&&(digitalRead(pin2) == 0)){
    BIT_SET(out,mask1);
    BIT_CLEAR(out,mask2);
  }
}

//inline void setyoungbitbutton(byte pin, byte& out, byte mask){
//  button.poll();
//  
//}

void loop() {  
/*if the metro timer runs out, then check the states of all the switches
    assign the values to the 'young' byte. Reset switch timer.*/
  if (switch_timer.check() == 1){
    old = young; // Store old switch values.
    setyoungbit(fgp,  young,FWD_GEAR);
    setyoungbit(rgp,  young,REV_GEAR);
    setyoungbit(hp,   young,HEADLIGHT);
    setyoungbit(hzp,  young,HAZARDLIGHT);
    //setyoungbitspecial(fgp,rgp,young,FWD_GEAR,REV_GEAR);
    //setyoungbitspecial(hp,hzp,young,HEADLIGHT,HAZARDLIGHT);
    setyoungbit(ltp,  young,LEFT_TURN);
    setyoungbit(rtp,  young,RIGHT_TURN);
    cruisecontrol.poll();
    if(cruisecontrol.pushed()){
      if(cruisecontroltoggle == true){
        cruisecontroltoggle = false;
        BIT_SET(young,CRUISE_CONTROL);
      }
      else{
        cruisecontroltoggle = true;
        BIT_CLEAR(young,CRUISE_CONTROL);
      }
    }
    
    horn.poll();
    if(horn.on()){
      BIT_CLEAR(young,HORN);
    }
    else {
      BIT_SET(young,HORN);
    }
    switch_timer.reset();
  }
  
  /*If this byte is different from the one in the void setup() or the CAN_TX timer runs out, send CAN packetxxxxx
    and reset CAN_TX timer.*/
  if(young != old || CAN_TX.check()){
    Serial.print("ERRORS:");
    Serial.println(CAN_errors,BIN);
    Serial.println(young,BIN);
    CanControl.Send(SW_Data(young),TXB0);
    CAN_TX.reset();
    old = young;
  }

  if (can.Available()){
    Frame&f = can.Read();
    if (f.id == MC_BUS_STATUS_ID){
      MC_BusStatus received(f);
      float current = received.bus_current;
      screen.selectLine(1);
      screen.print("MC Bus Current: ");
      screen.selectLine(2);
      screen.print(current);
      CAN_RX.reset();
    }
    else if (f.id == 
    
    DC_Steering received(f);
    
    
    // Call CanControl.Send(Layout); to send a packet
    // Call CanControl.Available();  to check whether a packet is received
    // Frame& f = CanControl.Read(); to get a frame from the queue.
    // DC_Steering packet(f); 		 to convert it to a specific Layout.             
}
