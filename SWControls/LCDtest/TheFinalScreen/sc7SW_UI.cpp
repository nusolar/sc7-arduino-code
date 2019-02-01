//THINGS TO DO FOR WINTER QUARTER!!!

//1). If there is no input in pins, display error message
//2). Test Blinkers
//3). Test error messages
//4). CAN Input
//5). List of Potential Error Messages: Overcurrent, Undervoltage, Over Temperature (too hot), No Telemetry?, Generic Error, Driver Control Errors, Contact Error, Overvoltage
//6). Forward/Reverse?
//7). Brake message (Speed Red?)
//8). Take off plastic wrap!
//9). Get legit screen protector ($$$$$$$$)
#include <SPI.h>
#include <RA8875.h>
#include "Arduino.h"
#include "sc7SW_UI.h"

sc7SW_UI::sc7SW_UI(int _RA8875_INT, int _RA8875_CS, int _RA8875_RESET)
{
  tft = RA8875(_RA8875_CS, _RA8875_RESET);
  RA8875_INT = _RA8875_INT
  
  Serial.begin(9600);
  tft.begin(RA8875_800x480);
  tft.touchBegin(RA8875_INT);

  // Sets us background interface
  setupInterface();
}

void setupInterface() {
  
 //Screen Background
 tft.setBackgroundColor(RA8875_BLACK);
 tft.drawRect(5,5,390, 195, RA8875_WHITE);
 tft.drawRect(5,205,390, 270, RA8875_WHITE);
 tft.drawRect(400,5, 390, 470,RA8875_WHITE);
 
 //Basic Car Model
 tft.drawRect(600,170,150,240,RA8875_WHITE);
 tft.drawEllipse(675,320,15,30,RA8875_WHITE);
 tft.drawRect(600,230,20,45,RA8875_WHITE);
 tft.drawRect(730,230,20,45,RA8875_WHITE);
 tft.drawRect(600,365,20,45,RA8875_WHITE);
 tft.drawRect(730,365,20,45,RA8875_WHITE);
 
 //Headlights
 tft.fillTriangle(605,150,645,150,625,190,RA8875_YELLOW);
 tft.fillTriangle(745,150,705,150,725,190,RA8875_YELLOW);
 
 //Turn Signals
 tft.fillTriangle(590,440,620,425,620,455, RA8875_YELLOW);
 tft.fillTriangle(760,440,730,425,730,455, RA8875_YELLOW);
 tft.fillRect(620,433,30,14,RA8875_YELLOW);
 tft.fillRect(700,433,30,14,RA8875_YELLOW);
}

//Error Message
void updateError(String error)
{
 tft.setCursor(407,20);
 tft.setFontScale(1);
 tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
 tft.print("Error: ");
 tft.print(error);
}

//Speed  
void updateSpeed(int _speed) 
{
 tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,25);
 tft.setFontScale(1);
 tft.print("Speed: ");
 tft.setCursor(180,70);
 tft.setFontScale(5);
 tft.print(_speed);
 tft.setCursor(330,150);
 tft.setFontScale(1);
 tft.print("mph");
}
 
void updateArrCurr(int _arrCurr)
{
 tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,225);
 tft.setFontScale(2);
 tft.print("Array(A): ");
 tft.setCursor (275,225);
 tft.print(_arrCurr);
}
 
//Minimum Battery Voltage
void updateMinBat(int _minBat)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,285);
 tft.setFontScale(2);
 tft.print("Min V: ");
 tft.setCursor (275,285);
 tft.print(_minBat);
} 
 
//Battery Current
void updateBatCurr(int _batCurr)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,345);
 tft.setFontScale(2);
 tft.print("Batt A: "); 
 tft.setCursor (275,345);
 tft.print(_batCur);
}
 
//Maximum temperature of BATTERY
void updateMaxTemp(int _maxTemp)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,405);
 tft.setFontScale(2);
 tft.print("Max Temp: ");
 tft.setCursor (275,405);
 tft.print(_maxTemp);
}