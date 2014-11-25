/* 2-board TEST FOR ARDUINO-MCP2515 CAN TXRX
- Alexander Martin, Fall 2014 -
This code implements basic TX and RX functions for the MCP2515 on the Arduino Due.
It includes one setup routine and two loop routines, on loop for RX and one for TX.
Switching between TX code and RX code can be done by setting the #define below to either
MODERX or MODETX.

NOTE: Will only work fresh after a power cycle. Reseting the Arduino w/ Serial
or Reset button will cause 2515 to error.
*/

#include <SPI.h>

#include "MCP2515.h"
#include "MCP2515_defs.h"

Frame TXf, RXf;

#define TEST_FID 0x101
#define MODETX // Set to either MODERX or MODETX to compile for the TX or RX board for this test.

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
  // Initialize the MCP2515
  if (CCAN.Init(1000,16) == false)
    Serial.println("MCP2515 Failed to Initialize");
  
  // Place chip in Normal mode
  if (CCAN.Mode(MODE_NORMAL) == true)
    Serial.println("MCP2515 in NORMAL MODE");
  else
    Serial.println("MCP2515 failed to set MODE");
    
  // Only enable RX interrupts for now
  CCAN.Write(CANINTE,RX0IF|RX1IF|TX0IF|TX1IF|TX2IF|MERRF);
  
  // Setup transmit frame
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
    //
    Serial.print("2515 Status:");
    Serial.println(CCAN.Status(),BIN);
    while (!CCAN.Interrupt())
    {
      int lastmills;
       if (millis() - lastmills > 2000)
       {
          Serial.println("Waiting for Sent Interrupt (STATUS, TXB0CTRL)");      
          Serial.println(CCAN.Status(),BIN);
          Serial.println(CCAN.Read(TXB0CTRL),BIN);
          lastmills = millis();
        }
       delay(100);
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
    if (intf & MERRF)
    {
        Serial.println("Message Error");
    }
    else
    {
      // Print data back on Serial
      Serial.print("Messsage Received:");
      Serial.print(readframe.data[0]);
      Serial.print(readframe.data[1]);
      Serial.println(readframe.data[2]);
    }
  
    delay(500);
}
#endif
