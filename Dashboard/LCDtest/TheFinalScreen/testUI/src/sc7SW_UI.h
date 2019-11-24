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
#ifndef sc7SW_UI_h
#define sc7SW_UI_h

#include <SPI.h>
#include <RA8875.h>
#include "Arduino.h"


class sc7SW_UI
{
  public:
    sc7SW_UI(int RA8875_INT, int RA8875_CS, int RA8875_RESET);
    RA8875 tft = RA8875(0,0);
    void updateError(String);

    // Update Speed on display
    void updateSpeed(int);
    // Update Array Current on display
    void updateArrCurr(int);
    // Update Minimum Battery on display
    void updateMinBat(int);
    // Update Battery Current on display
    void updateBatCurr(int);
    // Update Max Temperature on display
    void updateMaxTemp(int);
    

  private:
    int _RA8875_INT;
    int _RA8875_CS;
    int _RA8875_RESET;
    
    // Setup display, only called once
    void setupInterface(void);

};
#endif
