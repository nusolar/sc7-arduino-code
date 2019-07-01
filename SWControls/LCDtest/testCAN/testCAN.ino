#include <CAN_IO.h>
#include <SPI.h>
#include <Metro.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10
#define RA8875_RESET 9


#define DEBUG 1

//CAN parameters --> check these, may be diff
const byte     CAN_CS    = 52; //2 CAN computer system
const byte     CAN_INT   = 14; //10 CAN Interrupt 
const uint16_t CAN_BAUD_RATE = 250; // CAN Baud Rate
const byte     CAN_FREQ      = 16; //CAN Frequency 
uint16_t errors;

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);
uint16_t tx, ty;

//CAN vairables 
CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ);

void setup() {
  tft.begin(RA8875_800x480);
  
  Serial.begin(9600);
  Serial.print("setup");
  
  CanControl.filters.setRB0(MASK_Sxxx, BMS_VOLT_CURR_ID, DC_TEMP_0_ID);
  CanControl.filters.setRB1(MASK_Sxxx, MTBA_FRAME0_REAR_LEFT_ID, DC_INFO_ID, BMS_STATUS_EXT_ID, 0); //**MC_VELOCITY_ID, **MC_PHASE_ID
  CanControl.Setup(RX0IE | RX1IE);
  
#ifdef DEBUG
  Serial.print("CANINTE: " );
  Serial.println(CanControl.controller.Read(CANINTE), BIN);
#endif
}

void loop() {
   // put your main code here, to run repeatedly:
  CanControl.Fetch();
  if (CanControl.Available()) {
    // Use available CAN packets to assign values to appropriate members of the data structures
    Frame& f = CanControl.Read();
#ifdef DEBUG
    Serial.print("Received: " );
    Serial.println(f.id, HEX);
#endif 
    if (f.id == DC_DRIVE_ID)
    {
      DC_Drive packet(f);
      Serial.print("Packet Receieved"); 
      tft.setCursor(200,200);//Error
      tft.print(packet.current);
    }
  }
#ifdef DEBUG
Serial.print("No Packets Received.");
#endif
  //Debug for CAN
  CanControl.FetchErrors();
  CanControl.FetchStatus();
}
