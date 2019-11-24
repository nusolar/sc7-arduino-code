#include <SPI.h>
#include <RA8875.h>

RA8875 tft = RA8875(10,9);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("RA8875 start");
  
  tft.begin(RA8875_800x480);
  tft.fillScreen(RA8875_WHITE);
  tft.changeMode(TEXT);
  tft.setTextColor(RA8875_BLACK);
  
  tft.setCursor(420,25);
  tft.setFontScale(2);
  tft.print("Christopher Lee");
  
  tft.setCursor(350,220);
  tft.print("30 MPH");
  
  tft.setCursor(20,425);
  tft.print("121 V");

  tft.setCursor(500,425);
  tft.print("BAT50AMPPT50");
 
}

void loop() {
  // put your main code here, to run repeatedly:

}
