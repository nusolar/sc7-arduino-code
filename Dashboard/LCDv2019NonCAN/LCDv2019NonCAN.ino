#include <CAN_IO.h>
#include <Metro.h>
#include <SPI.h>
#include <RA8875.h>
#include "src/sc7Dashboard_UI.h"

#define RA8875_INT 4
#define RA8875_CS 10
#define RA8875_RESET 9

#define DEBUG

#define BMS_CURR_VALUE 0.1f
#define BMS_VOLT_VALUE 0.1f
#define BMS_SOC_VALUE 0.5f

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
#define ltp 2  //left turn
#define rtp 3 //right turn

//set up metro timer, these may be diff, check later
//1st: switch state reading timer - frequency at which switches are read
Metro CAN_RX = Metro(1000);
//8th: Telemetry HB timer
Metro disp_timer = Metro(333);

//CAN parameters --> check these, may be diff
const byte CAN_CS = 52;             // CAN computer system
const byte CAN_INT = 14;            //10 CAN Interrupt
const uint16_t CAN_BAUD_RATE = 125; // CAN Baud Rate
const byte CAN_FREQ = 16;           //CAN Frequency
uint16_t errors;

CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ);

RA8875 lcd(RA8875_CS, RA8875_RESET);
sc7Dashboard_UI tft(lcd);
displayData dispData = {};

void setup()
{
  pinMode(ltp, INPUT); // left turn
  pinMode(rtp, INPUT); // right turn

  Serial.begin(115200);
  delay(3000);

  tft.begin();

  // Extended IDs
  const uint32_t RXM0 = 0;
  const uint32_t RXF0 = 0;
  const uint32_t RXF1 = 0;

  // Standard IDs
  const uint32_t RXM1 = 0; // Last three bits free for MPPT offset
  const uint32_t RXF2 = 0;
  const uint32_t RXF3 = 0;
  const uint32_t RXF4 = 0;
  const uint32_t RXF5 = 0; // Place Holder

// Insert DEBUG and LOOPBACK steps
#ifdef DEBUG
    {
    Serial.print(F("Init CAN error: "));
    Serial.println(CanControl.errors, HEX);

    byte Txstatus[3] = {0, 0, 0};
    Txstatus[0] = CanControl.controller.Read(TXB0CTRL);
    Txstatus[1] = CanControl.controller.Read(TXB1CTRL);
    Txstatus[2] = CanControl.controller.Read(TXB2CTRL);
    byte canintf = 0;
    canintf = CanControl.last_interrupt;
    byte canctrl = 0;
    canctrl = CanControl.controller.Read(CANCTRL);

    Serial.println(F("TXnCTRL: "));
    Serial.println(Txstatus[0], BIN);
    Serial.println(Txstatus[1], BIN);
    Serial.println(Txstatus[2], BIN);
    Serial.print(F("Last Interrupt: "));
    Serial.println(canintf, BIN);
    Serial.print(F("CANCTRL: "));
    Serial.println(canctrl, BIN);
    Serial.print(F("CANSTAT: "));
    Serial.println(CanControl.canstat_register, BIN);
  }
#endif
}

void loop()
{
  if (disp_timer.check())
  {
    tft.update(dispData);
    Serial.println(F("Refresh"));
  }
}
