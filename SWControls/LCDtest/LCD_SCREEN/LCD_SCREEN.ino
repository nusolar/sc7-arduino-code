#include <CAN_IO.h>
#include <SPI.h>
#include <Metro.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10
#define RA8875_RESET 9


#define DEBUG 1

//Screen parameters:
const int w = 480; 
const int h = 272;

//CAN parameters: 
const byte     CAN_CS    = 52; //2 CAN computer system
const byte     CAN_INT   = 14; //10 CAN Interrupt 
const uint16_t CAN_BAUD_RATE = 500; // CAN Baud Rate
const byte     CAN_FREQ      = 16; //CAN Frequency 
uint16_t errors;

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);
uint16_t tx, ty;

//CAN vairables 
CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ);

//Indicators:
float VELOC = 0 ;              // Velocity (from CAN) - from DC_Drive
float BAT_CURRENT = 0;        // Battery Current (from CAN) - from BMS_VOLT_CURR
float MIN_BAT = 0;            
float ARRAY_CURRENT = 0;      // Array Current (from CAN) - from DC_Drive
float MAX_TEMPERATURE = 0;    // Max Pack Temperature (from CAN) - from DC_Temp_0

boolean HAZARD_LIGHT;      // Hazard Lights
boolean LEFT_LIGHT;       // Left Turn Signal
boolean RIGHT_LIGHT;      // Right Turn Signal - from 
boolean BRAKES;           // Indicate if brakes are deployed - from DC_INFO
String Err;               // Error message - from DC_INFO
boolean TRIPPED;          // Tripped Status - from DC_INFO

//BMS ERROR STRINGS - from BMS_Status_Ext
#define BMS_12VERR_STR              String("TRIP: 12V ERR") 
#define BMS_DRIVECONTROLSERR_STR    String("TRIP: DCTRLS ERR")
#define BMS_CONTACTERR_STR          String("TRIP: CNTCR ERR")
#define BMS_CURRENTERR_STR          String("TRIP: OVERCURRENT") // This currently doesn't do anything since the BMS doesn't report current trips
#define BMS_OVVOLTAGE_STR           String("TRIP: OVER V")
#define BMS_UNVOLTAGE_STR           String("TRIP: UNDER V")
#define BMS_UNKERR_STR              String("TRIP: UNK. ERR")
#define GENERIC_TRIP_STR            String("SOMETHING IS WRONG WITH YOUR CAR")


//metro timer, these may be diff, check later
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

//WatchDogTimer intialization
unsigned long loopStartTime = 0;
unsigned long loopSumTime = 0;
unsigned long loopCount = 0;

unsigned long previousmillis = 0;

void setup() {
  tft.begin(RA8875_480x272);
  defaultdisplay();
  
  Serial.begin(9600);
  Serial.print("setup");
  
  CanControl.filters.setRB0(MASK_Sxxx, BMS_VOLT_CURR_ID, DC_TEMP_0_ID);
  CanControl.filters.setRB1(MASK_Sxxx, MTBA_FRAME0_REAR_LEFT_ID, DC_INFO_ID, BMS_STATUS_EXT_ID, 0); //**MC_VELOCITY_ID, **MC_PHASE_ID
  CanControl.Setup(RX0IE | RX1IE);
  
  /*#ifdef LOOPBACK
  Serial.print("Set Loopback");
  CanControl.controller.Mode(MODE_LOOPBACK);
  
  #endif
  // Enable WDT to 500 ms timeout
    WDT_Enable(WDT, 0x2000 | 5000| ( 5000 << 16 ));

  */
  #ifdef DEBUG
    Serial.print("CANINTE: " );
    Serial.println(CanControl.controller.Read(CANINTE), BIN);
  #endif
}

void defaultdisplay(){
  tft.setBackgroundColor(RA8875_BLACK);
  tft.drawRect(w/100,h/54,w-w/100,h-h/54, RA8875_WHITE);
  tft.drawRect(w/100,h/54,w/2,h/2, RA8875_WHITE);
  tft.drawRect(w/100,h/54,w/2,h-h/54,RA8875_WHITE);

//  tft.changeMode(TEXT);//Speed Stuff
  tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
  tft.setCursor (w/20,h/10 - 10);
  tft.setFontScale(1); 
  tft.print("Speed: ");
  tft.setCursor(180,100);
  tft.print("mph");
  
  //Array Current
  tft.setFontScale(.75);
  tft.setCursor(w/20,6*h/11);
  tft.print("Array(A): ");

  tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Minimum Battery Voltage
  tft.setCursor (w/20,2*h/3);
  tft.print("Min V: ");
  
  tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Battery Current
  tft.setCursor (w/20,3*h/4 + 5);
  tft.print("Batt A: ");

  tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Max Temperature
  tft.setCursor (w/20, 9*h/10);
  tft.print("Max Temp: ");
  
  tft.drawRect(315,100,90,130,RA8875_WHITE);//Basic Car Model
  tft.drawEllipse(360,185,10,20,RA8875_WHITE);
  tft.drawRect(315,100,15,30,RA8875_WHITE);
  tft.drawRect(390,100,15,30,RA8875_WHITE);
  tft.drawRect(315,200,15,30,RA8875_WHITE);
  tft.drawRect(390,200,15,30,RA8875_WHITE);
  
  tft.setCursor(w/2 + 10, 4*h/5+30);//Brakes
  tft.setFontScale(.5);
  tft.setTextColor(RA8875_RED, RA8875_BLACK);
  tft.print("BRAKE");
  
  tft.setCursor(w/2 + 10, h/10 - 20);//Error
  tft.setFontScale(.5);
  tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
  tft.print("Error: ");
}

/*void updateinterface(){
  
  if(HAZARD_LIGHT)
    {
      tft.fillTriangle(440,440,470,425,470,455, RA8875_YELLOW);//Turn Signals
      tft.fillTriangle(610,440,580,425,580,455, RA8875_YELLOW);
      tft.fillRect(470,433,30,14,RA8875_YELLOW);
      tft.fillRect(550,433,30,14,RA8875_YELLOW); 
    }// Hazard Lights
  else if (LEFT_LIGHT)
    {
      tft.fillTriangle(440,440,470,425,470,455, RA8875_YELLOW);
      tft.fillRect(470,433,30,14,RA8875_YELLOW);
    }  // Left Turn Signal
  else if(RIGHT_LIGHT)
    {
      tft.fillTriangle(610,440,580,425,580,455, RA8875_YELLOW);
      tft.fillRect(550,433,30,14,RA8875_YELLOW);
    }
  if(TRIPPED) {
      tft.print(Err);
    } 
}*/

void loop() {
   // put your main code here, to run repeatedly
  CanControl.Fetch();
  if (CanControl.Available()) {
    // Use available CAN packets to assign values to appropriate members of the data structures
    Frame& f = CanControl.Read();
#ifdef DEBUG
    Serial.print("Received: " );
    Serial.println(f.id, HEX);
#endif 
    switch (f.id)
    {
      case BMS_VOLT_CURR_ID: // Voltage/Current of battery 
      {
          BMS_VoltageCurrent packet(f); //Get the voltage and current of the battery pack
          
          tft.setFontScale(5);
          tft.setCursor (220,285);
          tft.print(packet.voltage / 1000.0);
          tft.setCursor (220,345);
          tft.print(packet.current / 1000.0);
 
          CAN_RX.reset();
          break;
      }
      case DC_DRIVE_ID: //Velocity: 19 inch diameter of wheels, figure out conversion factor
      {
          DC_Drive packet(f);
          tft.setCursor (220,225);
          tft.setFontScale(5);
          tft.print(packet.current);
          tft.setCursor(180,70);
          tft.print(packet.velocity);

          CAN_RX.reset();
          break;
      }
      case DC_TEMP_0_ID: // Get Max Pack Temp
      {
          DC_Temp_0 packet(f); 
          tft.setCursor(200,2);//Error
          tft.setFontScale(5);
          tft.print(packet.max_temp);
          CAN_RX.reset();
          break;
      }
      
      /*case DC_INFO_ID:
      {
          DC_Info packet(f); // Get Tripped state of vehicle
          TRIPPED = packet.tripped;
          if (TRIPPED) {
            Err = GENERIC_TRIP_STR;
            notif_timer.reset();
          }
          Err = "";
          //CAN_RX.reset();
          break;
      }*/
      /*case BMS_STATUS_EXT_ID:
      {
          BMS_Status_Ext packet(f); // extract the flags
          if      (packet.flags & BMS_Status_Ext::F_OVERVOLTAGE)    {notif_timer.reset(); Err = BMS_OVVOLTAGE_STR;}
          else if (packet.flags & BMS_Status_Ext::F_UNDERVOLTAGE)   {notif_timer.reset(); Err = BMS_UNVOLTAGE_STR;}
          else if (packet.flags & BMS_Status_Ext::F_12VLOW)         {notif_timer.reset(); Err = BMS_12VERR_STR;}
          //CAN_RX.reset();
          break;
      }*/
    }
  }
#ifdef DEBUG
Serial.println("No Packets Received.");
#endif
  //Debug for CAN
  CanControl.FetchErrors(); 
  CanControl.FetchStatus();
  
  //  updateinterface();
  delay(100);
}
