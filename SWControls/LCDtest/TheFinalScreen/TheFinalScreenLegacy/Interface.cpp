#include "Interface.h"


void interface() {
 tft.setBackgroundColor(RA8875_BLACK);
 tft.drawRect(5,5,390, 195, RA8875_WHITE);
 tft.drawRect(5,205,390, 270, RA8875_WHITE);
 tft.drawRect(400,5, 390, 470,RA8875_WHITE);
 
 tft.changeMode(TEXT);//Speed Stuff
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,25);
 tft.setFontScale(1); 
 tft.print("Speed: ");
 tft.setCursor(180,70);
 tft.setFontScale(5);
 tft.print(VELOC);
 tft.setCursor(335,160);
 tft.setFontScale(1);
 tft.print("mph");

 tft.setFontScale(1);//Array Current
 tft.setCursor(25,225);
 tft.print("Array(A): ");
 tft.setCursor (220,225);
 tft.print(ARRAY_CURRENT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Minimum Battery Voltage
 tft.setCursor (25,285);
 tft.setFontScale(1);
 tft.print("Min V: ");
 tft.setCursor (220,285);
 tft.print(MIN_BAT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Battery Current
 tft.setCursor (25,345);
 tft.setFontScale(1);
 tft.print("Batt A: "); 
 tft.setCursor (220,345);
 tft.print(BAT_CURRENT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);//Max Temperature
 tft.setCursor (25,405);
 tft.setFontScale(1);
 tft.print("Max Temp: ");
 tft.setCursor (220,405);
 tft.print(MAX_TEMPERATURE);

 tft.drawRect(450,170,150,240,RA8875_WHITE);//Basic Car Model
 tft.drawEllipse(525,320,15,30,RA8875_WHITE);
 tft.drawRect(450,230,20,45,RA8875_WHITE);
 tft.drawRect(580,230,20,45,RA8875_WHITE);
 tft.drawRect(450,365,20,45,RA8875_WHITE);
 tft.drawRect(580,365,20,45,RA8875_WHITE);
 
 tft.fillTriangle(455,150,495,150,475,190,RA8875_YELLOW);//Headlights
 tft.fillTriangle(595,150,555,150,575,190,RA8875_YELLOW);
 
 tft.fillTriangle(440,440,470,425,470,455, RA8875_YELLOW);//Turn Signals
 tft.fillTriangle(610,440,580,425,580,455, RA8875_YELLOW);
 tft.fillRect(470,433,30,14,RA8875_YELLOW);
 tft.fillRect(550,433,30,14,RA8875_YELLOW);

 tft.setCursor(630,270);//Brakes
 tft.setFontScale(2);
 tft.setTextColor(RA8875_RED, RA8875_BLACK);
 tft.print("BRAKE");

 tft.setCursor(407,20);//Error
 tft.setFontScale(1);
 tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
 tft.print("Error: ");
 tft.print(ERROR);
}
