#include <RA8875.h>


#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10

#define RA8875_RESET 9

//Indicators:
int SPEED = 0;;
String ERROR = "";
double BAT_CURRENT = 0;
double MIN_BAT = 0;
double ARRAY_CURRENT = 0; 
double MAX_TEMPERATURE = 0;
boolean LEFT_LIGHT = false;
boolean RIGHT_LIGHT = false;
boolean HAZARD_LIGHT = false;
boolean BRAKES = false;


RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);
uint16_t tx, ty;
void interface() {
 tft.setBackgroundColor(RA8875_BLACK);
 tft.drawRect(5,5,390, 195, RA8875_WHITE);
 tft.drawRect(5,205,390, 270, RA8875_WHITE);
 tft.drawRect(400,5, 390, 470,RA8875_WHITE);
 
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
 
 tft.changeMode(TEXT);
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,225);
 tft.setFontScale(1);
 tft.print("Array(A): ");
 tft.setCursor (220,225);
 tft.print(ARRAY_CURRENT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,285);
 tft.setFontScale(1);
 tft.print("Min V: ");
 tft.setCursor (220,285);
 tft.print(MIN_BAT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,345);
 tft.setFontScale(1);
 tft.print("Batt A: "); 
 tft.setCursor (220,345);
 tft.print(BAT_CURRENT);
 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor (25,405);
 tft.setFontScale(1);
 tft.print("Max Temp: ");
 tft.setCursor (220,405);
 tft.print(MAX_TEMPERATURE);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  tft.begin(RA8875_800x480);
  tft.touchBegin(RA8875_INT);
  interface();
}

void loop() {
  // put your main code here, to run repeatedly:

}
