#include <CAN_IO.h>
#include <Metro.h>
#include <SPI.h>
#include <RA8875.h>
#include "src\sc7SW_UI.h"

#define RA8875_INT 4
#define RA8875_CS 10
#define RA8875_RESET 9

void setup() {
  Serial.begin(9600);
  Serial.print("Begin LCD");
  //RA8875 tft = RA8875((uint8_t) RA8875_CS, (uint8_t) RA8875_RESET);
  sc7SW_UI swLCD = sc7SW_UI((int) RA8875_INT, (int) RA8875_CS, (int) RA8875_RESET);

}

void loop() {
  // put your main code here, to run repeatedly:

}
