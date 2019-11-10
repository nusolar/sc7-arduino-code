#ifndef sc7SW_UI_h
#define sc7SW_UI_h

#include <SPI.h>
#include <RA8875.h>
#include "Arduino.h"

typedef struct
    {
        uint16_t speed = 0;
        uint16_t packCurr = 0;
        uint16_t packVolt = 0;
        uint16_t packSOC = 0;
        uint16_t maxTemp = 0;
        uint16_t avgTemp = 0;

    } displayData;

class sc7Dashboard_UI
{
public:
    sc7Dashboard_UI(RA8875 _tft) : 
        tft(_tft) 
        { }

    // Setup display, only called once
    void begin(void);
    void update(const displayData &dispData);

private:
    RA8875 tft;
    
    // Update Speed on display
    void updateSpeed(int);
    // Update Array Current on display
    void updatePackCurr(int);
    // Update Minimum Battery on display
    void updatePackVolt(int);
    // Update Battery Current on display
    void updatePackSOC(int);
    // Update Max Temperature on display
    void updateMaxTemp(int);
    // Updated Average Temperature on display
    void updateAvgTemp(int);

    const int screenHeight = 272;
    const int screenWidth = 480;
};
#endif