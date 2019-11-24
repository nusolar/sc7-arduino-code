#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10 

#define RA8875_RESET 9

RA8875 tft = RA8875(RA8875_CS,RA8875_RESET);

uint16_t tx, ty;

void interface(){
  tft.fillRoundRect(50, 10, 300, 60, 5, RA8875_WHITE);
  tft.fillRoundRect(450, 10, 300, 60, 5, RA8875_WHITE);
  tft.changeMode(TEXT);
  tft.setTextColor(RA8875_BLACK);
  tft.setCursor(100,20);
  tft.setFontScale(1);
  tft.print("System Check");
  tft.setCursor(530,20);
  tft.print("Speed etc.");
  tft.setCursor(290,220);
  tft.setFontScale(5);
  tft.setTextColor(RA8875_MAGENTA);
  tft.print("NU Solar");
}

void setup() {
  Serial.begin(9600);
  Serial.println("RA8875 start");

  tft.begin(RA8875_800x480);

  tft.touchBegin(RA8875_INT);
  tft.fillScreen(RA8875_BLACK);
  interface();
 
}

uint16_t movementChose = 0;

void loop() {
  tft.changeMode(GRAPHIC);

  if (tft.touchDetect()){
    delay(1000);
    tft.touchReadPixel(&tx, &ty);
      tx=tx;ty=ty;
    if (ty >= 10 && ty <= 60){
      if (tx >= 50 && tx <= 350){
        movementChose = 1;
        tft.fillScreen(RA8875_BLACK);

        tft.changeMode(GRAPHIC);
        tft.drawRoundRect(150, 100, 500, 320, 5, RA8875_WHITE);
        tft.drawRoundRect(160, 110, 150, 70, 5, RA8875_WHITE); 
        tft.drawRoundRect(160, 340, 150, 70, 5, RA8875_WHITE);
        tft.drawRoundRect(490, 110, 150, 100, 5, RA8875_WHITE);
        tft.drawRoundRect(490, 310, 150, 100, 5, RA8875_WHITE);
        tft.drawCircle(310, 260, 60, RA8875_WHITE);

        tft.changeMode(GRAPHIC);
        tft.fillRoundRect(450, 10, 300, 60, 5, RA8875_WHITE);
        tft.fillRoundRect(50, 10, 300, 60, 5, RA8875_MAGENTA);

        tft.changeMode(TEXT);
        tft.setTextColor(RA8875_BLACK);
        tft.setCursor(100,20);
        tft.setFontScale(1);
        tft.print("System Check");

        tft.setCursor(530,20);
        tft.changeMode(TEXT);
        tft.setFontScale(1);
        tft.setTextColor(RA8875_BLACK);
        tft.print("Speed etc.");
      }
    }
    if (ty >= 10 && ty <= 60){
      if (tx >= 450 && tx <= 750){
        movementChose = 2;
        tft.fillScreen(RA8875_BLACK);
        
        tft.changeMode(TEXT);
        tft.setFontScale(5);
        tft.setCursor(300,220);
        tft.setTextColor(RA8875_WHITE);
        tft.print("30 mph");
      
        tft.setCursor(40,410);
        tft.setFontScale(2);
        tft.print("121 V");
      
        tft.setCursor(620,410);
        tft.setFontScale(1);
        tft.print("BAT: 50 A");
        tft.setCursor(620,440);
        tft.setFontScale(1);
        tft.print("MPPT: 0 A");
        
        
        tft.changeMode(GRAPHIC);
        tft.fillRoundRect(50, 10, 300, 60, 5, RA8875_WHITE);
        tft.fillRoundRect(450, 10, 300, 60, 5, RA8875_MAGENTA);
        
        tft.setCursor(530,20);
        tft.changeMode(TEXT);
        tft.setFontScale(1);
        tft.setTextColor(RA8875_BLACK);
        tft.print("Speed etc.");
        tft.changeMode(TEXT);
        tft.setTextColor(RA8875_BLACK);
        tft.setCursor(100,20);
        tft.setFontScale(1);
        tft.print("System Check");
      }
    }
  }
}
