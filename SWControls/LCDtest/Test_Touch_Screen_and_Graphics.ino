#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10 

#define RA8875_RESET 9

RA8875 tft = RA8875(RA8875_CS,RA8875_RESET);

//WHEN NEEDING THE IMAGES, MUST USE SD CARD

void setup() {
  Serial.begin(9600);
  Serial.println("RA8875 start");
  tft.begin(RA8875_800x480);
  tft.touchBegin(RA8875_INT);
  
  tft.fillScreen(RA8875_BLACK);
  tft.changeMode(GRAPHIC);
  tft.drawRoundRect(280, 20, 500, 320, 5, RA8875_WHITE);//Body
  tft.drawRoundRect(300, 40, 150, 70, 5, RA8875_WHITE);//Front Square (T) 
  tft.drawRoundRect(300, 250, 150, 70, 5, RA8875_WHITE);//Front Square (B)
  tft.drawRoundRect(610, 40, 150, 100, 5, RA8875_WHITE);//Engines (T)
  tft.drawRoundRect(610, 220, 150, 100, 5, RA8875_WHITE);//Engines (B)
  tft.drawCircle(450, 180, 60, RA8875_WHITE);//Steering Wheel Loc.
  //tft.drawTriangle(10, 240, 110, 290, 110, 190, RA8875_WHITE);
  //tft.fillTriangle(20, 240, 105, 280, 105, 200, RA8875_WHITE);
  //tft.drawTriangle(790, 240, 690, 290, 690, 190, RA8875_WHITE);
  //tft.fillTriangle(780, 240, 695, 280, 695, 200, RA8875_WHITE);
  tft.fillRoundRect(10, 350, 240, 120, 5, RA8875_WHITE);
  tft.fillRoundRect(280, 350, 240, 120, 5, RA8875_WHITE);
  tft.fillRoundRect(550, 350, 240, 120, 5, RA8875_WHITE);

  tft.changeMode(TEXT);
  tft.setFontScale(1);
  tft.setCursor(10,10);
  tft.setTextColor(RA8875_WHITE);
  tft.print("Speed:");
  tft.setCursor(10,60);
  tft.setFontScale(4);
  tft.print("70");
  tft.setCursor(10,140);
  tft.setFontScale(1);
  tft.print("km/h");
  tft.setCursor(190,370);
  tft.setTextColor(RA8875_BLACK);
  tft.setFontScale(3);
  tft.print("V");//CAN ADD VARIABLE?
  tft.setCursor(60,370);
  tft.print("121");//CHANGE FOR CAN on VOLTAGE
  tft.setCursor(460,370);
  tft.print("%");//CAN ADD VARIABLE?
  tft.setCursor(330,370);
  tft.print("98");//CHANGE FOR CAN on BATTERY %
  tft.setCursor(560,370);
  tft.setFontScale(1);
  tft.print("Bat(A):");//CAN ADD VARIABLE?
  tft.setCursor(560,410);
  tft.print("MPPT(A):");//CAN ADD VARIABLE?
  tft.setCursor(700,370);
  tft.print("4");//CHANGE FOR CAN ON BATTERY AMP
  tft.setCursor(700,410);
  tft.print("4");//CHANGE FOR CAN ON MPPT AMP
}

void loop() {


}
