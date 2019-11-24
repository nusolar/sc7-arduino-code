#include <SPI.h>
#include <RA8875.h>

#define RA8875_INT 4
#define RA8875_CS 10

#define RA8875_RESET 9

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);

void setup() {
  Serial.begin(9600);
  //??? And do you need tabs?
  Serial.println("RA8875 start");
  //What does this mean?
  tft.begin(RA8875_800x480);

  tft.fillScreen(RA8875_BLACK);

  tft.changeMode(TEXT);
  tft.setCursor(250, 220);
  tft.setFontScale(3);
  tft.setTextColor(RA8875_MAGENTA);
  tft.print("NU Solar");

  tft.setCursor(50, 50);
  tft.setFontScale(1);
  tft.setTextColor(RA8875_WHITE);
  tft.print("Time");
  int i = 0;
}

unsigned long i = 0;

void loop()
{
  tft.setCursor(150, 50);
  if (i >= 0) tft.setTextColor(RA8875_RED, RA8875_BLACK);
  tft.print(i, DEC);
  delay(1000);
  i++;
}
