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
  Metro display_timer = Metro(2000);


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
  char notification[16];
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
  CanControl.Setup(filter, &CAN_errors);
  
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

inline void blnk(int a){
 screen.setCursor(1,a);
 if (a == LEFT){
   screen.print("<<");
 }
 else{
   screen.print(">>");
 }
 screen.setCursor(2,a);
 if (a == LEFT){
   screen.print("<<");
 }
 else{
   screen.print(">>");
 }
 delay(500);
 screen.setCursor(1,a);
 screen.print("  ");
 screen.setCursor(2,a);
 screen.print("  ");
 delay(500);
}

inline void defaultdisplay(){
  screen.setCursor(1,4);
  screen.print("SOC  %");
  screen.setCursor(1,SOC);
  screen.print(steering_wheel.SOCdisplay);
  screen.setCursor(1,LIGHT);
  screen.print(steering_wheel.Lightsdisplay);
  screen.setCursor(2,4);
  screen.print("V:");
  screen.setCursor(2,V);
  screen.print(steering_wheel.Veldisplay);
  screen.setCursor(2,CC);
  screen.print(steering_wheel.CCdisplay);
  screen.setCursor(2,GEAR);
  screen.print(steering_wheel.geardisplay);
  if(steering_wheel.LTdisplay){
    blnk(LEFT);
  }
  else if(steering_wheel.RTdisplay){
    blnk(RIGHT);
  }
  else{
    screen.setCursor(1,LEFT);
    screen.print("  ");
    screen.setCursor(2,LEFT);
    screen.print("  ");
    screen.setCursor(1,RIGHT);
    screen.print("  ");
    screen.setCursor(2,RIGHT);
    screen.print("  ");
  }
}

inline void notification(char string[]){
  display_timer.reset();
  while(display_timer.check() == 0){
    screen.selectLine(1);
    screen.print(string);
    delay(500);
    screen.clearLine(1);
    delay(500);
  }
  defaultdisplay();
}

void loop() {  
/*if the metro timer runs out, then check the states of all the switches
    assign the values to the 'young' byte. Reset switch timer.*/
  if (switch_timer.check() == 1){
    old = young; // Store old switch values.
    switchBitFromPin(fgp,  young,FWD_GEAR);
    switchBitFromPin(rgp,  young,REV_GEAR);
    switchBitFromPin(hp,   young,HEADLIGHT);
    switchBitFromPin(hzp,  young,HAZARDLIGHT);
    switchBitFromPin(ltp,  young,LEFT_TURN);
    switchBitFromPin(rtp,  young,RIGHT_TURN);
    	
    cruisecontrol.poll();
    if(cruisecontrol.pushed()){
      BIT_FLIP(young,CRUISE_CONTROL);
    }    
    horn.poll();
	switchBit(horn.on(), young, HORN);
    switch_timer.reset();
  }
  
  //Display shenanigans
  
  if(FWD_GEAR == 0 && REV_GEAR == 0){
    if(steering_wheel.geardisplay != 'N'){
      steering_wheel.geardisplay = 'N';
      notification("NEUTRAL GEAR");
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  else if(FWD_GEAR == 1){
    if(steering_wheel.geardisplay != 'F'){
      steering_wheel.geardisplay = 'F';
      notification("FORWARD GEAR");
      defaultdisplay();
    }
    else{
      defaultdisplay;
    }
  }
  else if(REV_GEAR == 1){
    if(steering_wheel.geardisplay != 'R'){
      steering_wheel.geardisplay = 'R';
      notification("REVERSE GEAR");
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  
  if(HEADLIGHT == 1){
    if(steering_wheel.Lightsdisplay[1] != ' '){
      steering_wheel.Lightsdisplay[0] = 'H';
      steering_wheel.Lightsdisplay[1] = ' ';
      notification("HEADLIGHTS ON");
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  else if(HAZARDLIGHT == 1){
    if(steering_wheel.Lightsdisplay[1] != 'Z'){
      steering_wheel.Lightsdisplay[0] = 'H';
      steering_wheel.Lightsdisplay[1] = 'Z';
      notification("HAZARDLIGHTS ON");
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  else if(HAZARDLIGHT == 1 && HEADLIGHT == 1){
     if(steering_wheel.Lightsdisplay[0] != ' '){
       steering_wheel.Lightsdisplay[0] = ' ';
       steering_wheel.Lightsdisplay[1] = ' ';
       notification("ALL LIGHTS OFF");
       defaultdisplay();
     }
     else{
       defaultdisplay();
     }
  }
     
  
  
  if(CRUISE_CONTROL == 1){
    if(steering_wheel.CCdisplay != 'C'){
      steering_wheel.CCdisplay = 'C';
      notification("CRUISECONTROL ON");
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  else if(CRUISE_CONTROL == 0){
    if(steering_wheel.CCdisplay != ' '){
      steering_wheel.CCdisplay = ' ';
      notification("CRUISECONTROLOFF");
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  
  if(LEFT_TURN == 1){
    if(steering_wheel.LTdisplay != true){
      steering_wheel.LTdisplay = true;
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  else if(LEFT_TURN == 0){
    if(steering_wheel.LTdisplay != false){
      steering_wheel.LTdisplay == false;
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  
  if(RIGHT_TURN == 1){
    if(steering_wheel.RTdisplay != true){
      steering_wheel.RTdisplay = true;
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  }
  else if(RIGHT_TURN == 0){
    if(steering_wheel.RTdisplay != false){
      steering_wheel.RTdisplay = false;
      defaultdisplay();
    }
    else{
      defaultdisplay();
    }
  } 
  delay(3000);
  
  Serial.println(steering_wheel.RTdisplay);
  Serial.println(steering_wheel.LTdisplay);
  Serial.println(steering_wheel.CCdisplay);
  Serial.println(steering_wheel.Lightsdisplay);
  Serial.println(steering_wheel.SOCdisplay);
  Serial.println(steering_wheel.geardisplay);
  Serial.println(steering_wheel.Veldisplay);
  
  delay(3000);
  
  /*If this byte is different from the one in the void setup() or the CAN_TX timer runs out, send CAN packetxxxxx
    and reset CAN_TX timer.*/
  //if(young != old || CAN_TX.check()){
    //Serial.print("ERRORS:");
    //Serial.println(CAN_errors,BIN);
    //Serial.println(young,BIN);
    //CanControl.Send(SW_Data(young),TXB0);
    //CAN_TX.reset();
    //old = young;
 // }

 // if (CanControl.Available()){
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
  //}
  //else if (CAN_RX.check()){
  //  screen.print("Communic. lost  with DrivCont");
  //}         
}
