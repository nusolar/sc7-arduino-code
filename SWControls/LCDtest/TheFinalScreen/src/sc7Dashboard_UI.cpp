#include <SPI.h>
#include <RA8875.h>
#include <Arduino.h>
#include "sc7Dashboard_UI.h"

void sc7Dashboard_UI::begin(void) {
 //Initializes LCD
 tft.begin(RA8875_480x272);

 //Screen Background
 tft.setBackgroundColor(RA8875_BLACK);
 tft.drawRect(0,0, screenWidth/2, screenHeight/3, RA8875_WHITE); // Upper Left Corner
 tft.drawRect(0, screenHeight/3, screenWidth/2, screenHeight * 2/3, RA8875_WHITE); // Lower Left Corner
 tft.drawRect(screenWidth/2, 0, screenWidth/2, screenHeight, RA8875_WHITE);
/* 
 tft.drawRect(5,5,390/2, 195/2, RA8875_WHITE); // Upper Left Corner
 tft.drawRect(5,205/2,390/2, 270/2, RA8875_WHITE); // Lower Left Corner
 tft.drawRect(400/2,5, 390/2, 470/2,RA8875_WHITE); // Right side
 */

 //Basic Car Model
 tft.drawRect(600/2,170/2,150/2,240/2,RA8875_WHITE);
 tft.drawEllipse(675/2,320/2,15/2,30/2,RA8875_WHITE);
 tft.drawRect(600/2,230/2,20/2,45/2,RA8875_WHITE);
 tft.drawRect(730/2,230/2,20/2,45/2,RA8875_WHITE);
 tft.drawRect(600/2,365/2,20/2,45/2,RA8875_WHITE);
 tft.drawRect(730/2,365/2,20/2,45/2,RA8875_WHITE);
 
 //Headlights
 tft.fillTriangle(605/2,150/2,645/2,150/2,625/2,190/2,RA8875_YELLOW);
 tft.fillTriangle(745/2,150/2,705/2,150/2,725/2,190/2,RA8875_YELLOW);
 
 //Turn Signals
 tft.fillTriangle(590/2,440/2,620/2,425/2,620/2,455/2, RA8875_YELLOW);
 tft.fillTriangle(760/2,440/2,730/2,425/2,730/2,455/2, RA8875_YELLOW);
 tft.fillRect(620/2,433/2,30/2,14/2,RA8875_YELLOW);
 tft.fillRect(700/2,433/2,30/2,14/2,RA8875_YELLOW);
}

//Error Message
void sc7Dashboard_UI::updateError(String error)
{
 tft.setCursor(600/2,20/2);
 tft.setFontScale(0);
 tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
 tft.print("Error: ");
 tft.print(error);
}

//Speed  
void sc7Dashboard_UI::updateSpeed(int _speed) 
{
 //tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25/2,25/2);
 tft.setFontScale(0);
 tft.print("Speed: ");
 tft.setCursor(180/2,70/2);
 tft.setFontScale(5);
 tft.print(_speed);
 tft.setCursor(330/2,120/2);
 tft.setFontScale(0);
 tft.print("mph");
}

void sc7Dashboard_UI::updatePackCurr(int _packCurr)
{
 //tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25/2,225/2);
 tft.setFontScale(0);
 tft.print("Pack Curr (A): ");
 tft.setCursor (275/2,225/2);
 tft.print(_packCurr);
}

// Batt Pack Voltage
void sc7Dashboard_UI::updatePackVolt(int _packVolt)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25/2,285/2);
 tft.setFontScale(0);
 tft.print("Pack Volt (V): ");
 tft.setCursor (275/2,285/2);
 tft.print(_packVolt);
} 

// Batt Pack SOC
void sc7Dashboard_UI::updatePackSOC(int _packSOC)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25/2,345/2);
 tft.setFontScale(0);
 tft.print("Pack SOC (%): "); 
 tft.setCursor (275/2,345/2);
 tft.print(_packSOC);
}

//Maximum temperature of BATTERY
void sc7Dashboard_UI::updateMaxTemp(int _maxTemp)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25/2,405/2);
 tft.setFontScale(0);
 tft.print("Max Temp (C): ");
 tft.setCursor (275/2,405/2);
 tft.print(_maxTemp);
}

void sc7Dashboard_UI::updateAvgTemp(int _avgTemp)
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25/2,465/2);
 tft.setFontScale(0);
 tft.print("Avg Temp (C): ");
 tft.setCursor (275/2,465/2);
 tft.print(_avgTemp);
}

void sc7Dashboard_UI::update(const displayData& dispData)
{
    //Clears Screen of text
    //Screen Background
    tft.fillRect(5,205/2,390/2, 270/2, RA8875_BLACK);
    tft.drawRect(0, screenHeight/3, screenWidth/2, screenHeight * 2/3, RA8875_WHITE); // Lower Left Corner
    
    //updates text
    updateError(dispData.err);
    updateSpeed(dispData.speed);
    updatePackCurr(dispData.packCurr);
    updatePackVolt(dispData.packVolt);
    updatePackSOC(dispData.packSOC);
    updateMaxTemp(dispData.maxTemp);
    updateAvgTemp(dispData.avgTemp);
}