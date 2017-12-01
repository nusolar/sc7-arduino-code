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

#include <RA8875.h>



#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10

#define RA8875_RESET 9

//Indicators:
int SPEED = 0;
String ERROR = "";
double BAT_CURRENT = 0;
double MIN_BAT = 0;
double ARRAY_CURRENT = 0; 
int MAX_TEMPERATURE = 0;
boolean LEFT_LIGHT = false;
boolean RIGHT_LIGHT = false;
boolean HAZARD_LIGHT = false;
boolean BRAKES = false;


RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);
uint16_t tx, ty;
void interface() {
  
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
void error_interface()
{
 tft.setCursor(407,20);
 tft.setFontScale(1);
 tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
 tft.print("Error: ");
 tft.print(ERROR);
}

//Speed  
void speed_interface() 
{
 tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,25);
 tft.setFontScale(1);
 tft.print("Speed: ");
 tft.setCursor(180,70);
 tft.setFontScale(5);
 tft.print(SPEED);
 tft.setCursor(330,150);
 tft.setFontScale(1);
 tft.print("mph");
}
 
void array_current()
{
 tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,225);
 tft.setFontScale(2);
 tft.print("Array(A): ");
 tft.setCursor (275,225);
 tft.print(ARRAY_CURRENT);
}
 
//Minimum Battery Voltage
void minimum_voltage()
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,285);
 tft.setFontScale(2);
 tft.print("Min V: ");
 tft.setCursor (275,285);
 tft.print(MIN_BAT);
} 
 
//Battery Current
void battery_current()
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,345);
 tft.setFontScale(2);
 tft.print("Batt A: "); 
 tft.setCursor (275,345);
 tft.print(BAT_CURRENT);
}
 
//Maximum temperature of BATTERY
void battery_max_temp()
{
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,405);
 tft.setFontScale(2);
 tft.print("Max Temp: ");
 tft.setCursor (275,405);
 tft.print(MAX_TEMPERATURE);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  tft.begin(RA8875_800x480);
  tft.touchBegin(RA8875_INT);
  speed_interface();
  array_current();
  minimum_voltage();
  battery_current();
  battery_max_temp(); 
  error_interface();
  interface();
}

void loop() {
  // put your main code here, to run repeatedly:
  SPEED++;
  speed_interface();
  
  BAT_CURRENT = BAT_CURRENT + 1.03;
  battery_current();
  
  ARRAY_CURRENT = ARRAY_CURRENT - .5;
  array_current();
  
  MIN_BAT = MIN_BAT + .7;
  minimum_voltage();
  
  MAX_TEMPERATURE++;
  battery_max_temp();
  
  delay(1000);
  
}
