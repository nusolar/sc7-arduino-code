#include <RA8875.h>
#include <CAN_IO.h>
#include <Metro.h>

#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10

#define RA8875_RESET 9

#define DEBUG

/*Defining the bitwise functions (bitwise operators)
  We're using bits to store data because there are only 8 bytes available for use in a CAN packet.
  We can store all of the necessary data in a single byte and save space. This frees up space to
  send other data that we might need in the future between the steering wheel and the driver controls*/
#define BIT(n)                   ( 1<<(n) )
#define BIT_SET(y, mask)         ( y |=  (mask) )                       //Set the value of BIT = 1
#define BIT_CLEAR(y, mask)       ( y &= ~(mask) )                       //Set the value of BIT = 0
#define BIT_FLIP(y, mask)        ( y ^=  (mask) )                       //Flip the value of BIT
#define BIT_DIFFERENT(y, x, mask)( y & mask == x & mask )               //See if two bits are different values
#define BIT_CHECK(y, bit, mask)  ( BIT_DIFFERENT(y, (bit)*mask, mask) ) //check function utilizing the previous defined different function

//Defining the bit masks
#define FWD_GEAR BIT(0)   // might be diff with the new motors
#define REV_GEAR BIT(1)   // might be diff with the new motors
#define HEADLIGHT BIT(2)
#define HAZARDLIGHT BIT(3)
//#define LAP_TIMER BIT(4)
#define HORN BIT(5)
#define LEFT_TURN BIT(6)
#define RIGHT_TURN BIT(7)

/*These are the bytes containing the above-mentioned bits. The two bytes will be used
  to compare the switch states and determine whether there has been a change or not.*/
char young = 0xFF; //young is continuously assigned to the new switch states
char old;          //old is the previous switch states

//Conversion data for displayed measurements
#define MPS_TO_MPH 2.2369f

//BMS Error Strings
#define BMS_12VERR_STR              String("TRIP: 12V ERR")
#define BMS_DRIVECONTROLSERR_STR    String("TRIP: DCTRLS ERR")
#define BMS_CONTACTERR_STR          String("TRIP: CNTCR ERR")
#define BMS_CURRENTERR_STR          String("TRIP: OVERCURRENT") // This currently doesn't do anything since the BMS doesn't report current trips
#define BMS_OVVOLTAGE_STR           String("TRIP: OVER V")
#define BMS_UNVOLTAGE_STR           String("TRIP: UNDER V")
#define BMS_UNKERR_STR              String("TRIP: UNK. ERR")
#define GENERIC_TRIP_STR            String("SOMETHING IS WRONG WITH YOUR CAR")

//set up pins that connect to switch terminals
//const int fgp =   6;  //forward gear --> don't have these rn for sc7
//const int rgp =   7;  //reverse gear
const int headlightp =    A4;  //headlights
const int hzp =   A5;  //hazardlights
const int brakep = A6; //brakes
const int strobep =   A7;  //strobe light
const int hornp = A8; //horn
const int leftp =   A9;  //left turn
const int rightp =   A10;  //right turn
const int caninterruptp  = 14; // CAN interrupt
const int canchipselp = 52; // CAN chip select
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
const byte     CAN_CS    = 10;
const byte     CAN_INT   = 2; // Interrupt #1
const uint16_t CAN_BAUD_RATE = 1000;
const byte     CAN_FREQ      = 16;
uint16_t errors;

CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ); //Try initializing without interrupts for now

//Indicators:
int SPEED = 0;;
String ERROR = "CAR MISSING";
double BAT_CURRENT = 0;
double MIN_BAT = 0;
double ARRAY_CURRENT = 0; 
double MAX_TEMPERATURE = 0;
boolean LEFT_LIGHT = false;
boolean RIGHT_LIGHT = false;
boolean HAZARD_LIGHT = false;
boolean BRAKES = false;


RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);
uint16_t tx, ty;
void interface() {
 tft.setBackgroundColor(RA8875_BLACK);
 tft.drawRect(5,5,390, 195, RA8875_WHITE);
 tft.drawRect(5,205,390, 270, RA8875_WHITE);
 tft.drawRect(400,5, 390, 470,RA8875_WHITE);
 
 tft.changeMode(TEXT);//Speed Stuff
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,25);
 tft.setFontScale(1);
 tft.print("Speed: ");
 tft.setCursor(180,70);
 tft.setFontScale(5);
 tft.print(SPEED);
 tft.setCursor(335,160);
 tft.setFontScale(1);
 tft.print("mph");

 tft.setFontScale(1);//Array Current
 tft.setCursor(25,225);
 tft.print("Array(A): ");
 tft.setCursor (220,225);
 tft.print(ARRAY_CURRENT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Minimum Battery Voltage
 tft.setCursor (25,285);
 tft.setFontScale(1);
 tft.print("Min V: ");
 tft.setCursor (220,285);
 tft.print(MIN_BAT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Battery Current
 tft.setCursor (25,345);
 tft.setFontScale(1);
 tft.print("Batt A: "); 
 tft.setCursor (220,345);
 tft.print(BAT_CURRENT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Max Temperature
 tft.setCursor (25,405);
 tft.setFontScale(1);
 tft.print("Max Temp: ");
 tft.setCursor (220,405);
 tft.print(MAX_TEMPERATURE);

 tft.drawRect(450,170,150,240,RA8875_WHITE);//Basic Car Model
 tft.drawEllipse(525,320,15,30,RA8875_WHITE);
 tft.drawRect(450,230,20,45,RA8875_WHITE);
 tft.drawRect(580,230,20,45,RA8875_WHITE);
 tft.drawRect(450,365,20,45,RA8875_WHITE);
 tft.drawRect(580,365,20,45,RA8875_WHITE);
 
 tft.fillTriangle(455,150,495,150,475,190,RA8875_YELLOW);//Headlights
 tft.fillTriangle(595,150,555,150,575,190,RA8875_YELLOW);
 
 tft.fillTriangle(440,440,470,425,470,455, RA8875_YELLOW);//Turn Signals
 tft.fillTriangle(610,440,580,425,580,455, RA8875_YELLOW);
 tft.fillRect(470,433,30,14,RA8875_YELLOW);
 tft.fillRect(550,433,30,14,RA8875_YELLOW);

 tft.setCursor(630,270);//Brakes
 tft.setFontScale(2);
 tft.setTextColor(RA8875_RED, RA8875_BLACK);
 tft.print("BRAKE");

 tft.setCursor(407,20);//Error
 tft.setFontScale(1);
 tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
 tft.print("Error: ");
 tft.print(ERROR);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  tft.begin(RA8875_800x480);
  tft.touchBegin(RA8875_INT);
  interface();
}

void loop() {
  // put your main code here, to run repeatedly:

}
