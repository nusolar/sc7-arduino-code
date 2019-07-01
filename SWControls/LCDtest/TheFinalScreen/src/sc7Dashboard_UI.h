#ifndef sc7SW_UI_h
#define sc7SW_UI_h

#include <SPI.h>
#include <RA8875.h>
#include "Arduino.h"

typedef struct
    {
        float speed = 0;
        float packCurr = 0;
        float packVolt = 0;
        float packSOC = 0;
        float maxTemp = 0;
        float avgTemp = 0;
        String err = "";

    } displayData;

class sc7Dashboard_UI
{
public:
    sc7Dashboard_UI(RA8875 _tft) : 
        tft(_tft) 
        { }
    void updateError(String);

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

    int screenHeight = 272;
    int screenWidth = 480;
};
#endif