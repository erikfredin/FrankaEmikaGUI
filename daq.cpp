#include "daq.h"

//#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

daq::daq()
{
    // CONSTRUCTOR
    // Set up channels here?
}

daq::~daq()
{
    qInfo() << "Disconnecting the DAQ...";
    // TODO delete daq data stream or something;
}

bool daq::isConnected(void)
{
    return enableDAQ;
    // TODO Check if the daq is actually connected
}

bool daq::isEnabled(void)
{
    return enableDAQ;
}

void daq::setupTask(void)
{
    DAQmxCreateTask("MyTask", &taskHandle);
    DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai0:7", "8_AI_Channels", DAQmx_Val_Cfg_Default, -10, 10, DAQmx_Val_Volts, NULL);
    DAQmxCfgSampClkTiming(taskHandle, "", sampleRate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numSamples);
//    DAQmxCfgSampClkTiming(taskHandle, "", sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numSamples);


//    DAQmxRegisterEveryNSamplesEvent(taskHandle,DAQmx_Val_Acquired_Into_Buffer,1000,0,EveryNCallback,NULL);
//    DAQmxRegisterDoneEvent(taskHandle,0,DoneCallback,NULL);

}

void daq::finishTask()
{
    if( DAQmxFailed(error) )
    {
            DAQmxGetExtendedErrorInfo(errBuff,2048);
    }
    if( taskHandle!=0 )
    {

        DAQmxClearTask(taskHandle);
    }
    if( DAQmxFailed(error) )
    {
        printf("DAQmx Error: %s\n",errBuff);
    }
}

void daq::setNumSamples(int numSamp)
{

    // TODO: check for application start first

    if (numSamp < maxSamples)
    {
        numSamples = numSamp;
    }
    else
    {
        numSamples = maxSamples;
        qInfo() << "Number of samples was set too high. Instead, number of samples was set to the maximum: " << maxSamples;
    }
}

void daq::setSampleRate(double sampRate)
{
    // TODO: check for application start first
    sampleRate = sampRate;
}

void daq::dataAcquisition(void)
{
    TaskHandle taskHandle = 0;
    int32      error=0;
    char       errBuff[2048]={'\0'};
    int32 samplesReceived = 0;
//    float64 data[8*numSamples]; // This is what the function would be, but it is not allowed in C
    // Make a data array that is much larger than is needed for all operations.
    // Only store the data up to the point that is needed.
    float64 data[8*maxSamples];
    double timeout = 10.0;

    // Voltage readings are set to max and mins of +-10 V

    // Recommended sampleRate = 10000.0;
    // Recommended numSamples = 10;

    DAQmxCreateTask("MyTask", &taskHandle);
    DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai0:7", "8_AI_Channels", DAQmx_Val_Cfg_Default, -10, 10, DAQmx_Val_Volts, NULL);
    DAQmxCfgSampClkTiming(taskHandle, "", sampleRate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numSamples);
    // Start Code
    DAQmxStartTask(taskHandle);
    // Read Code
    // This could be the part that is repeated and cut out the top and bottom code to speed up calcs.
    DAQmxReadAnalogF64(taskHandle, numSamples, timeout, DAQmx_Val_GroupByChannel, data, 8*numSamples, &samplesReceived, NULL);

    if( samplesReceived > 0 )
    {
//        qInfo() << "Acquired " << (int)samplesReceived << " samples.";

        for (int i = 0; i < 8; i++)
        {
            double sum = 0.0;
            for (int j = 0; j < numSamples; j++)
            {
                sum += data[i*numSamples+j];
//                qDebug() << "Datapoint: " << i*numSamples+j << " is " << data[i*numSamples+j];
            }
            // Average over numSamples is the value measured
            analogInputVoltages[i] = sum/numSamples;
        }
    }
    else
    {
        qInfo() << "No samples were read.";
    }

//    if( DAQmxFailed(error) )
//    {
//            DAQmxGetExtendedErrorInfo(errBuff,2048);
//    }
    if( taskHandle!=0 )
    {
        // Stop Code
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
    }
//    if( DAQmxFailed(error) )
//    {
//        printf("DAQmx Error: %s\n",errBuff);
//    }



//    DAQmxStopTask(taskHandle_x);
//    DAQmxClearTask(taskHandle_x);

}

void daq::dataAcquisition8(double datastream[8])
{

    // Read Data

//    float64 data[8*numSamples]; // This is what the function would be, but it is not allowed in C
    // Make a data array that is much larger than is needed for all operations.
    // Only store the data up to the point that is needed.

    // reset array
//        data[8*maxSamples];

    // Start Code
    DAQmxStartTask(taskHandle);
    // Now ready to begin reading from these channels

    DAQmxReadAnalogF64(taskHandle, numSamples, timeout, DAQmx_Val_GroupByChannel, data, 8*numSamples, &samplesReceived, NULL);

    // Synthesize and find the average of the samples for each channel
    if( samplesReceived > 0 )
    {
//            qInfo() << "Acquired " << (int)samplesReceived << " samples.";
//            qInfo() << "Number of Samples: " << numSamples;
        for (int i = 0; i < 8; i++)
        {
            double sum = 0.0;
            for (int j = 0; j < numSamples; j++)
            {
                sum += data[i*numSamples+j];
//                    qDebug() << "Datapoint: " << i*numSamples+j << " is " << data[i*numSamples+j];
            }
            // Average over numSamples is the value measured
            datastream[i] = sum/numSamples;

        }
    }
    else
    {
        qInfo() << "No samples were read.";
    }

    // Stop Code
    DAQmxStopTask(taskHandle);


}

//int32 CVICALLBACK daq::EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
//{
//    int32       error=0;
//    char        errBuff[2048]={'\0'};
//    static int  totalRead=0;
//    int32       read=0;
//    float64     data[1000];

//    /*********************************************/
//    // DAQmx Read Code
//    /*********************************************/
//    DAQmxReadAnalogF64(taskHandle,1000,10.0,DAQmx_Val_GroupByScanNumber,data,1000,&read,NULL);

//    // Synthesize and find the average of the samples for each channel
//    if( read > 0 )
//    {
//        qInfo() << "Acquired " << (int)read << " samples.";

//        for (int i = 0; i < 8; i++)
//        {
//            double sum = 0.0;
//            for (int j = 0; j < nSamples; j++)
//            {
//                sum += data[i*nSamples+j];
//                qDebug() << "Datapoint: " << i*nSamples+j << " is " << data[i*nSamples+j];
//            }
//            // Average over numSamples is the value measured
//            analogInputVoltages[i] = sum/nSamples;
//        }
//    }
//    else
//    {
//        qInfo() << "No samples were read.";
//    }

//    if( read>0 ) {
//        qInfo() << "Acquired " << (int)read << " samples. Total " << (int)(totalRead+=read);
////        fflush(stdout);
//    }



////Error:
//    if( DAQmxFailed(error) ) {
//        DAQmxGetExtendedErrorInfo(errBuff,2048);
//        /*********************************************/
//        // DAQmx Stop Code
//        /*********************************************/
//        DAQmxStopTask(taskHandle);
//        DAQmxClearTask(taskHandle);
//        printf("DAQmx Error: %s\n",errBuff);
//    }
//    return 0;
//}

//int32 CVICALLBACK daq::DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
//{
//    int32   error=0;
//    char    errBuff[2048]={'\0'};

//    // Check to see if an error stopped the task.
////	DAQmxErrChk (status);

////Error:
//    if( DAQmxFailed(error) ) {
//        DAQmxGetExtendedErrorInfo(errBuff,2048);
//        DAQmxClearTask(taskHandle);
//        printf("DAQmx Error: %s\n",errBuff);
//    }
//    return 0;
//}
