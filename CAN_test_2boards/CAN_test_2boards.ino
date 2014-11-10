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
  CCAN.Write(CANINTE,RX0IF|RX1IF|TX0IF|TX1IF|TX2IF|MERRF);
  
  //Setup transmit frame
   TXf.id = TEST_FID;
   TXf.ide = 0;
   TXf.srr = 0;
   TXf.rtr = 0;
   TXf.dlc = 8;
   TXf.data[0] = 9;
   TXf.data[1] = 14;
   TXf.data[2] = 3;
}

/************ TRANSMIT CODE ***********/
#ifdef MODETX
void loop() {
    // Send a Can Message
    CCAN.LoadBuffer(TXB0,TXf);
    CCAN.SendBuffer(TXB0);
    Serial.println(CCAN.Status(),BIN);
    while (!CCAN.Interrupt())
       if (millis()%2000 == 0)
       {
          Serial.println("Waiting for Sent Interrupt");
          Serial.println(CCAN.Status(),BIN);
       }
    byte intr = CCAN.GetInterrupt();
    if (intr & TX0IF)
    {
      Serial.println("Message Sent");
      CCAN.ResetInterrupt(ALLIF);
    }
    else
    {
      Serial.println("Other Error");
      Serial.println(intr,BIN);
      Serial.println(CCAN.Read(TEC));
    }
    
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
        Serial.println(CCAN.Status());
      }
    }
    
    // Read the INTF flags.
    byte intf = CCAN.GetInterrupt();
    delay(10);
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
    Serial.println(CCAN.GetInterrupt(),BIN);
    CCAN.ResetInterrupt(ALLIF);
    delay(10);
    Serial.println(CCAN.GetInterrupt(),BIN);
    Serial.println(CCAN.Read(EFLG),BIN);
    Serial.println(CCAN.Read(CANINTE),BIN);
    delay(500);
}
#endif
