#ifndef S826_H
#define S826_H


#include "826api.h"
#include <QDebug>

////#ifndef _LINUX
//#include <windows.h>
//#include <conio.h>
////#endif

//#include <stdio.h>
//#include <stdlib.h>

#include <math.h>

#define BOARD   1     // The board identifier is assumed to be always 0 (All switches OFF).
//#define TSETTLE 500   // ADC. Settling time in microseconds.
#define TSETTLE 250 // can be set as low as 0. Lower is faster, but higher is less noise.

#define TMAX  500       // ADC. Max time allowed to read analog signals

#define CONNECTION_LED 5  // The channel that verifies that the program has connected to the board.

// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLING
// These examples employ very simple error handling: if an error is detected, the example functions will immediately return an error code.
// This behavior may not be suitable for some real-world applications but it makes the code easier to read and understand. In a real
// application, it's likely that additional actions would need to be performed. The examples use the following X826 macro to handle API
// function errors; it calls an API function and stores the returned value in errcode, then returns immediately if an error was detected.

#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { qWarning()<<"ERROR: "<<errcode; return errcode;}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Counter mode: Clock set tp 1MHz, Counting down, preload when 0 reached, preload upon start, assert ExtOut when not 0
#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_OM_NOTZERO)

#define CLEAR_BITS  (0x00000000)

// // Configure a counter channel to operate as a periodic timer and start it running. (Counts down and does not preload at zero)
//#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_START | S826_CM_OM_NOTZERO)

class S826
{
public:
    // VARIABLES
    bool boardConnected = false;
    bool flag_range_changed = true;
    int boardNumber;
//    int tsettle = 0;  // Analog input (ADC) settling time in µs
    int prev_range = 8;
    int count_range = 16;

    // Analog output range codes
    uint rangeDAC = S826_DAC_SPAN_10_10; // -10V to +10V
    // Analog input range codes
    uint rangeADC = S826_ADC_GAIN_1;     // -10V to +10V
    // (For more range codes see 826api.h)


    uint rangeCodesDAC[8]  = {0};
    uint rangeCodesADC[16] = {0};

    // This is for output range codes
    const int RANGE_PARAM[4][3] = { {  0,  5,  5},    // rangeCode 0  [minVoltage,rangeV,maxVoltage]
                                    {  0, 10, 10},    // rangeCode 1  [minVoltage,rangeV,maxVoltage]
                                    { -5, 10,  5},    // rangeCode 2  [minVoltage,rangeV,maxVoltage]
                                    {-10, 20, 10}  }; // rangeCode 3  [minVoltage,rangeV,maxVoltage]
    // FUNCTIONS

    // Creator function
    S826();
    // Destroyer function
    ~S826();

    // Initialization and shutdown functions
    int init(void);
    int close(void);

    short int rangeTest(uint rangeCode, double outputV);

    // privatize
    int analogInit(uint rangeADC, uint rangeDAC);  // Initialize all analog inputs and outputs.

    // Analog Outputs
    int analogWrite(uint chan, uint rangeCode, double outputV); // Set 1 AO channel to some voltage.
    int analogWriteAll(uint rangeCode[8], double outputV[8]);   // Set all AO channels to corresponding voltages.

    // Analog Inputs
    int analogRead(uint chan, double *aiV); // Read the value from 1 AI channel.
    int analogReadAll(double *aiV);

    // Digital Outputs
    int digitalWrite(uint chan, bool outputV);     // Set 1 DO channel to be high/low voltage.
    int digitalWriteAll(uint outputV);             // Set all DO channel to be high/low voltage.

    // Timer related functions
    int periodicTimerStart(uint counter, uint period);
    int periodicTimerStop(uint counter);
    int periodicTimerClear(uint counter);
    int periodicTimerWait(uint counter, uint *timestamp);
    void printCurrentTime(void);
    int waitUsePeriodicTimer(uint waitTime);

};

#endif // S826_H
