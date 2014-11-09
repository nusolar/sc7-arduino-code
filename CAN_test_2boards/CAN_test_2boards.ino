/* LOOPBACK TEST FOR ARDUINO-MCP2515 CAN TXRX
- Alexander Martin, Fall 2014 -
This code puts the MCP2515 in Loopback Mode, where the TX and RX pins are
internally connected. 5 and 10 are sent in data[0] and data[1] bytes in a
CAN frame and received/printed upon reception.

NOTE: Will only work fresh after a power cycle. Reseting the Arduino w/ Serial
or Reset button will cause 2515 to error.
*/

#include <SPI.h>

#include "MCP2515.h"
#include "MCP2515_defs.h"

Frame TXf, RXf;
#define TEST_FID 0x101
#define MODETX

/*
typedef struct
{
      unsigned long id;      // EID if ide set, SID otherwise
      byte srr;                  // Standard Frame Remote Transmit Request
      byte rtr;                  // Remote Transmission Request
      byte ide;                  // Extended ID flag
      byte dlc;                  // Number of data bytes
      byte data[8];            // Data bytes
} Frame;
*/

MCP2515 CCAN(4,5);
void setup() {
  
  // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
  SPI.setClockDivider(10);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(Serial.read() != '1') ;
  if (CCAN.Init(125,20) == false)
    Serial.println("MCP2515 Failed to Initialize");
  
  // Place chip in Loopback mode
  if (CCAN.Mode(MODE_NORMAL) == true)
    Serial.println("MCP2515 in NORMAL MODE");
  else
    Serial.println("MCP2515 failed to set MODE");
    
  //Only enable RX interrupts for now
  CCAN.Write(CANINTE,RX0IF|RX1IF);
  
   /* Set up Transmit Frame
   typedef struct
  {
        unsigned long id;      // EID if ide set, SID otherwise
        byte srr;                  // Standard Frame Remote Transmit Request
        byte rtr;                  // Remote Transmission Request
        byte ide;                  // Extended ID flag
        byte dlc;                  // Number of data bytes
        byte data[8];            // Data bytes
  } Frame; */
   TXf.id = TEST_FID;
   TXf.dlc = 8;
   TXf.data[0] = 0;
   TXf.data[1] = 1;
   TXf.data[2] = 0;
}

/************ TRANSMIT CODE ***********/
#ifdef MODETX
void loop() {
    // Send a Can Message
    CCAN.LoadBuffer(TXB0,TXf);
    CCAN.SendBuffer(TXB0);
   /*
    // Wait for a while to give MCP2515 time to process looped signal
    delay(250);
    while (CCAN.Interrupt() == false)
    {
      // Check for high interrupt pin
      if (millis()%1000 == 0) 
      {
        Serial.print("Waiting for Message...");
      }
    }
    
    // Read the INTF flags.
    byte intf = CCAN.GetInterrupt();
    Frame readframe;
    readframe.data[0] = 3;
    readframe.data[1] = 6;
    if (intf & RX0IF) //if RX0 received
       readframe = CCAN.ReadBuffer(RXB0);
    if (intf & RX1IF) //if RX1 received
       readframe = CCAN.ReadBuffer(RXB1);
       
    // Print data back on Serial
    Serial.print("Messsage Received:");
    Serial.print(readframe.data[0]);
    Serial.println(readframe.data[1]);
    
    // Clear interrupts
    CCAN.ResetInterrupt(RX0IF|RX1IF);
    delay(1);
    Serial.println(CCAN.GetInterrupt(),BIN);
    */
    
    delay(1000);
}

#endif

/************** RECEIVE ****************/
#ifdef MODERX
void loop() {
    while (CCAN.Interrupt() == false)
    {
      // Check for high interrupt pin
      if (millis()%1000 == 0) 
      {
        Serial.print("Waiting for Message...");
      }
    }
    
    // Read the INTF flags.
    byte intf = CCAN.GetInterrupt();
    Frame readframe;
    readframe.data[0] = 3;
    readframe.data[1] = 6;
    if (intf & RX0IF) //if RX0 received
       readframe = CCAN.ReadBuffer(RXB0);
    if (intf & RX1IF) //if RX1 received
       readframe = CCAN.ReadBuffer(RXB1);
       
    // Print data back on Serial
    Serial.print("Messsage Received:");
    Serial.print(readframe.data[0]);
    Serial.println(readframe.data[1]);
    
    // Clear interrupts
    CCAN.ResetInterrupt(0xFFFFFF);
    delay(1);
    Serial.println(CCAN.GetInterrupt(),BIN);
    Serial.println(CCAN.Read(EFLG),BIN);
    Serial.println(CCAN.Read(CANINTE),BIN);
    delay(500);
}
#endif
