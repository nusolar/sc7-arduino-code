#ifndef sc7SW_UI_h
#define sc7SW_UI_h

#include <SPI.h>
#include <RA8875.h>
#include "Arduino.h"

class sc7Dashboard_UI
{
public:
    sc7Dashboard_UI(RA8875 _tft) : 
        tft(_tft) 
        { }
    void updateError(String);

    // Setup display, only called once
    void begin(void);

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
    // Updated Average Temperature on display
    void updateAvgTemp(int);

private:
    RA8875 tft;
};
#endif