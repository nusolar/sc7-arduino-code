#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10 

#define RA8875_RESET 9

RA8875 tft = RA8875(RA8875_CS,RA8875_RESET);

uint16_t tx, ty;

void interface(){
  tft.drawTriangle(10, 240, 110, 290, 110, 190, RA8875_WHITE);
  tft.fillTriangle(20, 240, 105, 280, 105, 200, RA8875_WHITE);
  tft.drawTriangle(790, 240, 690, 290, 690, 190, RA8875_WHITE);
  tft.fillTriangle(780, 240, 695, 280, 695, 200, RA8875_WHITE);
}
void setup() {
  Serial.begin(9600);
  Serial.println("RA8875 start");

  tft.begin(RA8875_800x480);

  tft.touchBegin(RA8875_INT);
  interface();
}

uint16_t movementChose = 0;

void loop() {
  tft.changeMode(GRAPHIC);

  if (tft.touchDetect()){
    delay(1000);
    tft.touchReadPixel(&tx, &ty);
      tx=tx;ty=ty;
    if (ty >= 190 && ty <= 290){
      if (tx >= 0 && tx <= 110){
        movementChose = 1;
        tft.fillScreen(RA8875_BLACK);
      
        tft.changeMode(TEXT);
        tft.setFontScale(5);
        tft.setCursor(300,220);
        tft.setTextColor(RA8875_WHITE);
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
    }
    if (ty >= 190 && ty<= 290){
      if (tx >= 690 && tx <= 790){
        movementChose = 2;
        tft.fillScreen(RA8875_BLACK);
        tft.changeMode(GRAPHIC);
        tft.drawRoundRect(280, 20, 500, 320, 5, RA8875_WHITE);//Body
        tft.drawRoundRect(300, 40, 150, 70, 5, RA8875_WHITE);//Front Square (T) 
        tft.drawRoundRect(300, 250, 150, 70, 5, RA8875_WHITE);//Front Square (B)
        tft.drawRoundRect(610, 40, 150, 100, 5, RA8875_WHITE);//Engines (T)
        tft.drawRoundRect(610, 220, 150, 100, 5, RA8875_WHITE);//Engines (B)
        tft.drawCircle(450, 180, 60, RA8875_WHITE);//Steering Wheel Loc.
      }
    }
  }

}
