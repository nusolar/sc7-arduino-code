#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10

#define RA8875_RESET 9

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);

void setup() {
  Serial.begin(9600);
  Serial.println("RA8875 start");
  tft.begin(RA8875_800x480);

  tft.fillScreen(RA8875_WHITE);

  tft.changeMode(TEXT);
  tft.setFontScale(5);
  tft.setCursor(300,220);
  tft.setTextColor(RA8875_BLACK);
  tft.print("30 mph");

  tft.setCursor(600,40);
  tft.setFontScale(1);
  tft.print("Joshua Zhao");

  tft.setCursor(50,430);
  tft.print("121 V");

  tft.setCursor(580,430);
  tft.setFontScale(1);
  tft.print("BAT50AMPPTS0A");
}

void loop() {
  // put your main code here, to run repeatedly:

}
