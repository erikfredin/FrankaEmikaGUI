#ifndef DAQ_H
#define DAQ_H

#include <stdio.h>
#include "NIDAQmx.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>

class daq
{
public:
    daq();
    ~daq();
    bool enableDAQ = false;
    double analogInputVoltages[8] = {0.0};
    double analogRawInputVoltages[8] = {0.0};
    // Averaging and Data Sampling Variables
    const static int maxSamples = 10;
    int numSamples = 10; // Number of samples per datapoint on all channels
//    double sampleRate = 31000.0; // internal clock speed
    double sampleRate = 25000.0; // internal clock speed
    constexpr static double maxSampleRate = 250000.0/8.0; // this rate is shared among the channels used


    TaskHandle taskHandle = 0;
    int32      error=0;
    char       errBuff[2048]={'\0'};
    int32 samplesReceived = 0;
//    float64 data[8*numSamples]; // This is what the function would be, but it is not allowed in C
    // Make a data array that is much larger than is needed for all operations.
    // Only store the data up to the point that is needed.
    float64 data[8*maxSamples];

    double timeout = 10.0;




//    void dataAcquisition8(double datastream[8]);
//    void dataAcquisition8(double datastream[8], int numSamples, double sampleRate);
    void dataAcquisition(void);

    void dataAcquisition8(double datastream[8]);

//    void dataAcquisition16(double datastream[16]);

    bool isEnabled(void);
    bool isConnected(void);
    void setupTask(void);
    void finishTask(void);
    void setNumSamples(int numSamp);
    void setSampleRate(double sampRate);

    int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
    int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

    // example datafile name
    QString filename = "C:/Users/MicroRoboticsLab/Documents/data1000.txt";

};

#endif // DAQ_H
