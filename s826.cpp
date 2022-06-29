/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Code written by Adam Schonewille on 2021-03-31
// Strongly based off of code created by Jiachen Zhang on 2014-08-12 which was last modified on 2015-08-04.
// Program modified by Adam Schonewille on 2021-04-01
// Program modified by Adam Schonewille on 2021-04-12
// Program modified by ... on ... (Please follow this format to add any following modification info.)
//

#include "s826.h"

S826::S826()
{
    // When new board object is created, try to initialize the board with init()
//    init();

    // This would be the correct way to go about it?
}

S826::~S826()
{
    // When destroyed, close connection to the board
    qInfo() << "Closing connection to S826 board...";
    close();
}

// Board Initialization:
int S826::init(void)
{
    // Space out attempt for this code.
    qInfo() << " ";
    qInfo() << "Attempting to connect to the S826 board...";
    // Initialize the errorcode as no error
    int errcode = S826_ERR_OK;
    // open S826 driver and find all 826 boards
    // S826_SystemOpen() returns a number from 0-65535 where bitwise is the board number(s) found
    // Here 0 means no board and 1 mean board 0 found
    // for only 1 board connected the function should return 2^(ID#)
    int boardflags  = S826_SystemOpen();

    qInfo() << "S826_SystemOpen() returned the value: " << boardflags;
    // if boardflag is positive -> the bitwise flags of the connected board(s).
    // if boardflag is negative -> there was a problem connecting to the board. See 826API for info.

    if (boardflags < 0)
    {
        // Problem during opening
        errcode = boardflags;
        qWarning() << "Fatal Problem During Board Opening.";
        qInfo() << " ";
        return errcode;
    }
    else if (boardflags == 0)
    {
        qWarning() << "No boards were found. Try restarting the PC.";
        qInfo() << " ";
        return errcode;
    }
    else if ( (boardflags & (1 << BOARD) ) == 0)
    {
        // Connected Board(s) do no match the board that was specified in this code.
        qWarning() << "Connected board has a different ID than the one specified in the code";
        qWarning() << "Connected board ID is: " << log2(boardflags);
        qWarning() << "NOTE: a non-whole number result means multiple wrong boards";
        qWarning() << "Board ID specified in this code is: " << BOARD;
    }
    else if (boardflags != (1 << BOARD))
    {
        qWarning() << "Multiple boards are detected. Errors may occur.";
        qWarning() << "For more info see: http://www.sensoray.com/support/826_boardID.htm";
        boardConnected = true;
        boardNumber = BOARD;
    }
    else
    {
        qInfo() << "The board number is: " << log2(boardflags);
        qInfo() << "NOTE: a non-whole number result means multiple connected boards";
        boardConnected = true;
        boardNumber = BOARD;
    }

    // TODO
    // Initialize the number of inputs and outputs and set all to zero.

    if (boardConnected)
    {
        // Set status LED to Blink for connected status
        qInfo() << "Configuring S826 status LED...";
        S826_CounterStateWrite(BOARD, CONNECTION_LED, 0);     // halt channel if it's running
        S826_CounterModeWrite(BOARD, CONNECTION_LED, TMR_MODE);     // configure counter as periodic timer
        // NOTE: Don't need to start/unhalt the counter, the led is on as soon as 1MHz clock is set.

        // Set up all the analog channels that will be used for inputs and outputs
//        X826( analogInit(rangeADC, rangeDAC) );
        errcode = analogInit(rangeADC, rangeDAC);
        qInfo() << "Successfully initialized the Analog input and output channels";
//        qInfo() << errcode;
    }
    qInfo() << " ";
    return errcode;
}


int S826::close(void)
{
    // Write values to zero and get rid of board connection
    // Clear the Connection LED to show disconnection.
    if (boardConnected)
    {
        periodicTimerClear(CONNECTION_LED);
        S826_AdcEnableWrite(BOARD, 0);  // disable ADC conversions
    }

    S826_SystemClose();   // The return value always equals 0.
    return 0;
}


// Test whether the output voltage is inside the range or not.
// Inputs:  rangeCode:    0: 0 +5V;   1: 0 +10V;   2: -5 +5V;   3: -10 +10V.
//          outputV  : Desired analog output voltage (can be positive and negative).
// Outputs:  0 : inside the range;
//          -1 : outside the range.

short int S826::rangeTest(uint rangeCode, double outputV)
{
    switch(rangeCode)
    {
    case S826_DAC_SPAN_0_5: // 0
        // V > 5 or V < 0
        if ( (outputV>RANGE_PARAM[0][2]) || (outputV<RANGE_PARAM[0][0]) )
        {
            return -1;
        }
        break;
    case S826_DAC_SPAN_0_10: // 1
        // V > 10 or V < 0
        if ( (outputV>RANGE_PARAM[1][2]) || (outputV<RANGE_PARAM[1][0]) )
        {
            return -1;
        }
        break;
    case S826_DAC_SPAN_5_5: // 2
        // V > 5 or V < -5
        if ( (outputV>RANGE_PARAM[2][2]) || (outputV<-RANGE_PARAM[2][0]) )
        {
            return -1;
        }
        break;
    case S826_DAC_SPAN_10_10: // 3
        // V > 10 or V < -10
        if ( (outputV>RANGE_PARAM[3][2]) || (outputV<RANGE_PARAM[3][0]) )
        {
            return -1;
        }
        break;
    default:
        qWarning() << "Error: Range selection is invalid.";
        return -1;
        break;
    }
    // else return 0
    return 0;
}


// Initialize all Analog Input channels and all Analog Output channels
// Initialize inputs by assigning a slot to each channel.
// Input:   rangeADC : the range code to be set for all ADC channels
//          rangeDAC : the range code to be set for all DAC channels
// Output:  -1 :
//           0 : if successful, else the program doesn't run?

int S826::analogInit(uint rangeADC, uint rangeDAC)
{
    // If analog inputs or output voltages are not all the same, this will need to be changed in this function.
    int errcode = S826_ERR_OK;

    // Configure Analog Inputs
    qInfo() << "Configuring S826 analog inputs...";

    for (int j = 0; j<S826_NUM_ADC; j++) // cycle through all 16 ADC channels and assign and timeslot
    {
        // Configure channel j to timeslot j.
        X826( S826_AdcSlotConfigWrite(BOARD, j, j, TSETTLE, rangeADC) );  // program ADC timeslot attributes: slot, chan, 0us settling time
        // It is important to record range codes when set to speed up processes later on.
        rangeCodesADC[j] = rangeADC;
    }
    X826( S826_AdcSlotlistWrite(BOARD, 0xFFFF, S826_BITWRITE)   );  // enable all ADC timeslots
//        X826( S826_AdcTrigModeWrite(BOARD, 0x80 | (48 + counter))        );  // enable ADC hardware triggering, use counter channel's ExtOut as trigger
    X826( S826_AdcTrigModeWrite(BOARD, 0)                       );  // disable ADC hardware triggering, use continuous mode
    X826( S826_AdcEnableWrite(BOARD, 1)                         );  // enable ADC conversions
    double trash;
    X826( analogReadAll(&trash)                                 );  // discard first adc data sample (because it might need some settling time)
    qDebug() << "Read analog values gives: " << trash;


    qInfo() << "Configuring S826 analog outputs...";
    for (int i=0; i<S826_NUM_DAC; i++)
    {
//        qInfo() << "Writing to Analog Output: " << i;
        // Set the output range for all analogue outputs to be rangeDAC (runmode 0)
        X826( S826_DacRangeWrite(BOARD, i, rangeDAC, 0) );  // program dac output range: -10V to +10V
        // NOTE: DacRangeWrite also initializes the setpoint to zero
        // It is important to record range codes when set to speed up processes later on.
        rangeCodesDAC[i] = rangeDAC;
    }
//    qDebug() << errcode;
    return errcode;


//    int errcode = S826_ERR_OK;

//    // These ...Old variables are used to receive the existing setting of the ADC.
//    //uint chanOld =0;
//    //uint tsettleOld = 0;
//    //uint rangeOld = 0;
//    //X826(S826_AdcSlotConfigRead(board, slot, &chanOld, &tsettleOld, &rangeOld));
//    //printf("Reading result %i, %i, %i\n",chan,tsettle,range);
//    //if(chanOld != chan)
//    //	printf("Warning: This AI slot has been assigned to channel %i!\n",chanOld);

////	uint i = 0;
//    for (uint i = 0; i<16; i++)
//    {
//        X826( S826_AdcSlotConfigWrite(BOARD,i,i,TSETTLE,range) );   // Configure channel i to timeslot i.
//    }
//    //uint slotlist = ((uint)1)<<slot;
//    X826( S826_AdcSlotlistWrite(BOARD,chan,0) );  // Program the conversion slot list. Write Mode: 0 (write).

//    // Disable hardware triggering.
//    uint trigmode = 0;
//    X826( S826_AdcTrigModeWrite(BOARD,trigmode) );
//    //S826_AdcTrigModeRead(board,&trigmode);
//    //printf("Trigmode is 0x%x.\n",trigmode);

//    unsigned short int enable = 1;                   // Set to 1 to enable, or 0 to disable ADC conversion bursts.
//    //uint enableRead = 0;
//    X826( S826_AdcEnableWrite(BOARD,enable) );         // Enable or disable ADC conversions.
//    //X826(S826_AdcEnableRead(board,&enableRead));
//    //printf("The enable status is %i.\n",enableRead);

//    return errcode;


}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ANALOG OUTPUT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Set 1 Analog Output channel.
// Inputs:  board    : board identifier.
//          chan     : DAC channel # in the range 0 to 7.
//          rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//          outputV  : Desired analog output voltage (can be positive and negative).
// Output:  -1 : Unsuccessful write to analog channel
//           0 : Successful write to analog channel
int S826::analogWrite(uint chan, uint rangeCode, double outputV)
{
    int errcode   = S826_ERR_OK;
//    int minVoltage= 0;            // The lowest possible voltage under specific range setting.
//    uint rangeV   = 0;            // Corresponding range voltage span under specific rangeCode.
    uint setpoint = 0;            // Value used in S826 subroutine.

    // Verify that the range code is the same as that initialized for the channel
    uint slot_setpoint  = 0;   // Buffer to store the setpoint
    uint slot_rangeCode = 0;   // Buffer to store the range-code (0: 0-5V; 1: 0-10V; 2: +-5V; 3: +-10V)
    uint runmode = 0;
    // Read what the rangeCode was initialized to:
    // NOTE: if this action takes a long time, can use a stored value check instead.
    X826( S826_DacRead(BOARD, chan, &slot_rangeCode, &slot_setpoint, runmode) );
    if (slot_rangeCode != rangeCode)
    {
        qWarning() << "Error: Slot voltage range code does not match initialized range code.";
        return -1;
    }

    // Test that the desired voltage is within the bounds of the selected voltage range
    int testInput = rangeTest(rangeCode,outputV);
    if (testInput == -1)
    {
        qWarning() << "Error: Output voltage is outside the range.";
        return -1;
    }

    // Find the corresponding uint for the voltage value in that range
    switch (rangeCode)
    { // conversion is based on dac output range:
        case S826_DAC_SPAN_0_5:
            setpoint = (uint)(outputV * 0xFFFF / 5);
            break; // 0 to +5V
        case S826_DAC_SPAN_0_10:
            setpoint = (uint)(outputV * 0xFFFF / 10);
            break; // 0 to +10V
        case S826_DAC_SPAN_5_5:
            setpoint = (uint)(outputV * 0xFFFF / 10) + 0x8000;
            break; // -5V to +5V
        case S826_DAC_SPAN_10_10:
            setpoint = (uint)(outputV * 0xFFFF / 20) + 0x8000;
            break; // -10V to +10V
        default:
            printf("Error: Range selection is wrong.");
            return -1;
            break;
    }

    return S826_DacDataWrite(BOARD, chan, setpoint, 0); // program DAC output


//    switch(rangeCode)
//    {
//    case 0:
//        minVoltage = RANGE_PARAM[0][0]; // min is 0
//        rangeV     = RANGE_PARAM[0][1]; // Span is 5V.
//        break;
//    case 1:
//        minVoltage = RANGE_PARAM[1][0]; // min is 0
//        rangeV     = RANGE_PARAM[1][1]; // Span is 10V.
//        break;
//    case 2:
//        // why is this check necessary when it is chceked above?
//        if ((outputV>5)||(outputV<-5))
//        {
//            qWarning() << "Error: Output voltage is outside the range.";
//            return -1;
//        }
//        minVoltage = RANGE_PARAM[2][0]; // min is -5
//        rangeV     = RANGE_PARAM[2][1]; // Span is 10V.
//        break;
//    case 3:
//        minVoltage = RANGE_PARAM[3][0]; // min is -10
//        rangeV     = RANGE_PARAM[3][1]; // Span is 20V.
//        break;
//    default:
//        printf("Error: Range selection is wrong.");
//        return -1;
//        break;
//    }

//    if ( rangeCode != prev_range )
//    {
//        if (count_range != 0)
//        {
//            count_range--;
//        }
//        else
//        {
//            flag_range_changed = true;
//            count_range = 16; //# of channels
//        }
//    }
//    qDebug() << "minVoltage is " << minVoltage << " and rangeV is " << rangeV;
//    qDebug() << "outputV is " << outputV;
//    qDebug() << "Fractional result is " << (outputV-minVoltage)/rangeV;

//    setpoint = (uint)((outputV - minVoltage)/rangeV * 0xFFFF);   // Calc the corresponding setpoint value for the DAQ.
//    qDebug() << "setpoint value is " << setpoint;

//    if(flag_range_changed){//added to fix issue with high frequency noise (resetting of amp after programming)
//        X826(S826_DacRangeWrite(BOARD,chan,rangeCode,0));   // Program DAC output range.
//        if(count_range == 0)
//        {
//            flag_range_changed = false;
//            prev_range = rangeCode;
//        }
//    }

//    X826(S826_DacDataWrite(BOARD,chan,setpoint,0));     // Set the desired output voltage.

//    return errcode;

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Set all the 8 Analog Output channels together.
// Input: board    : board identifier.
//        rangeCode:   0: 0 +5V;   1: 0 +10V;   2: -5 +5V;   3: -10 +10V.
//        outputV  : Desired analog output voltage (can be positive and negative).
// Output:  -1 : Unsuccessful write to any analog channel
//           0 : Successful write to all analog channels
int S826::analogWriteAll(uint rangeCode[8], double outputV[8])
{

    int errcode   = S826_ERR_OK;
    for (int i = 0; i<S826_NUM_DAC; i++)
    {
        // For each channel, attempt to write the voltage to the slot
        errcode = analogWrite(i, rangeCode[i], outputV[i]);
        // analogWrite will handle validity checks prior to writing data and will
        // return 0 if successful
        if (errcode != 0)
        {
            // There was an unsuccessful writing attempt
            qWarning() << "A problem occured during analog write.";
            return errcode;
        }
    }

    return errcode;

//    int minVoltage= 0;            // The lowest possible voltage under specific range setting.
//    uint rangeV   = 0;            // Corresponding range voltage span under specific rangeCode.
//    uint setpoint = 0;            // Value used in S826 subroutine.
//    short int testInput = 0;
////    int i;

//    for (int i=0; i<8; i++)
//    {

//        // OLD CODE WAS REPETITIVE.
//        // UTILIZE OTHER FUNCTIONS:
////        // Perform range test.
////        testInput = rangeTest(rangeCode[i], outputV[i]);
////        if (testInput == -1) {printf("Error: Output voltage is outside the range."); return -1;}

////            switch(rangeCode[i])
////            {
////                case 0:
////                    miniV = 0;
////                    rangeV = 5;   // Span is 5V.
////                    break;
////                case 1:
////                    miniV = 0;
////                    rangeV = 10;   // Span is 10V.
////                    break;
////                case 2:
////                    miniV = -5;
////                    rangeV = 10;   // Span is 10V.
////                    break;
////                case 3:
////                    miniV = -10;
////                    rangeV = 20;   // Span is 20V.
////                    break;
////                default:
////                    printf("Error: Range selection is wrong.");
////                    return -1;
////                    break;
////            }
////            setpoint = (uint)((outputV[i] - miniV)/rangeV * 0xFFFF);   // Calc the corresponding setpoint value for the DAQ.
////            X826( S826_DacRangeWrite(BOARD, i, rangeCode[i], 0)   );      // program dac output range

////            X826( S826_DacDataWrite(BOARD, i, setpoint, 0)  );

//        // try to write each individual value to the corresponding channels on the S826
//        int writeAttempt = 0;
//        writeAttempt = analogWrite(i, rangeCode[i], outputV[i]);
//        if (writeAttempt == -1)
//        {
//            // There was an unsuccessful writting attempt
//            errcode = writeAttempt;
//        }
//        // iterate over the next channels
//    }

//    return errcode;


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ANALOG INPUTS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Read the ADC value(s) of interested channel(s). (Since slots are directly mapped to channels, slotlist and channel list are interchangable)
// Input:   chan: Bit-wise. Each bit stands for one channel. The corresponding bit will be set if one channel is of interest.
//          *aiV: Pointer to an array which is used to receive the ADC result.
// Output:  errcode
//          *aiV: Pointer to variable to that changes value (P-B-R) in integer
int S826::analogRead(uint chan, double *aiV)
{
    // Creates 16bit ADC Buffer for storing data
    int adcbuf[16];
    // set up the adc slot list; we are only interested in one slot
    // Create slotlist of channels to read from (will just be the one channel bit)
    uint slotlist = 1 << chan;
    // wait for data to arrive on the slot of interest (in response to adc hardware trigger)
    int errcode = S826_AdcRead(BOARD, adcbuf, NULL, &slotlist, TMAX); // S826_AdcRead(BOARD, buf, tstamp, &chan, tmax)
    // this application doesn't care about adc missed triggers
    if (errcode == S826_ERR_MISSEDTRIG)
        errcode = S826_ERR_OK;
    // copy ADC data to slotdata buffer
    *aiV = adcbuf[chan] & 0xFFFF;

    return errcode;





//    uint errcode = S826_ERR_OK;

//    //uint status = 0;
//    int buf[16];
//    uint tstamp[16];
//    //uint slotlist = ((uint)1)<<slot;
//    uint tmax = 1000;

//    //uint halfRangeV = 0;
//    //time_t mytime;
//    //mytime=time(NULL);
//    //printf("slotlist is %i.\n",slotlist);
//    //X826(S826_AdcSlotConfigRead(board,slot,&chan,&tsettle,&rangeCode));
//    //printf("I am here. S826_AdcSlotConfigRead. The rangeCode is %i.\n",rangeCode);
//    //uint i = 1;
//    //for(;i<100;i++)
//    //{
//    //	X826(S826_AdcStatusRead(board,&status));
//    //	printf("Status is %i.\n",status);
//        //delay(100);
//    //	usleep(1000);
//    //}
//    //if ((status >> slot) && 1)
//    //{
//    X826( S826_AdcRead(BOARD, buf, tstamp, &chan, tmax) );

//    //printf("I am here. S826_AdcRead.\n");
//        //printf("The time is now %s", ctime(&mytime));
//    //print_current_time_with_ms ();
//    //}
//    //printf("buf is %i",buf[0]);

//    uint bufNew = 0;
//    double resolution = 0;   // Resolution.
//    uint chanOld    = 0;   // Buffer to store the AI channel #.
//    uint tsettleOld = 0;   // Buffer to store the settle-time (uS).
//    uint rangeCodeOld   = 0;   // Buffer to store the range-code (0: +-10V; 1: +-5V; 2: +-2V; 3: +-1V).

////    uint i = 0;
//    for (uint i = 0; i<16; i++)
//    {
//        // channel in binary bitwise AND with 1 left shifted i times
//        if (chan & (1<<i)) // e.g. 10 & (1 << 8) IS 1010 & (0001 << 8) GIVES 0000 0000 0000 1010 & 0000 0001 0000 0000 RESULTS IN 0000 0000 0000 0000
//        {
//            bufNew = buf[i] & 0xFFFF; // This is the same as just buf[i]? Passes value not ref.
//            //printf("The bufNew is 0x%x.\n",bufNew);
//            X826( S826_AdcSlotConfigRead(BOARD,i,&chanOld,&tsettleOld,&rangeCodeOld) );

//            switch (rangeCodeOld)
//            {
//                case 0:
//                    //halfRangeV = 10;
//                    resolution = 20.0 / 65536.0;
//                    break;
//                case 1:
//                    //halfRangeV = 5;
//                    resolution = 10.0 / 65536.0;
//                    break;
//                case 2:
//                    //halfRangeV = 2;
//                    resolution = 4.0 / 65536.0;
//                    break;
//                case 3:
//                    //halfRangeV = 1;
//                    resolution = 2.0 / 65536.0;
//                    break;
//                //default:
//                //	resolution = 20 / 2^16;
//                //	break;
//            }
//            if (bufNew & 0x8000) // The reading is negative.
//            {
//                //printf("The reading is negative.\n");
//                //actualV = - (~(bufNew - 1))/0x7FFF * halfRangeV;
//                //actualV = ((~(bufNew - 1))&0xFFFF) * resolution;
//                aiV[i] = ((~(bufNew - 1))&0xFFFF) * resolution;
//                //printf("Test: actualV is %fV.\n",actualV);
//                aiV[i] = -1 * aiV[i];
//                //printf("The reading is %f.\n",aiV[i]);
//            }
//            else 		     // The reading is positive.
//            {
//                //printf("The reading is positive.\n");
//                aiV[i] = bufNew * resolution;
//                //printf("The reading is %f.\n",aiV[i]);
//            }
//        }
//    }
//    //printf("The resolution is %f.\n",resolution);
//    //printf("The AI signal is 0x%x, i.e. %fV.\n",bufNew,actualV);
//    //fprintf(fp," %f\n",actualV);

//    return errcode;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Read the ADC value(s) of interested channel(s). also called slotlist
// Input:   chan: Bit-wise. Each bit stands for one channel. The corresponding bit will be set if one channel is of interest.
//          *aiV: Pointer to an array which is used to receive the ADC result.
// Output:  errcode
//          *aiV: Pointer to an array which is used to receive the ADC results
//              Values passed by reference are in Volts
int S826::analogReadAll(double *aiV)
{
    // Creates 16 slot ADC Buffer for storing data from all slots (channels)
    int ADC_Buffer[16];
    // set up the adc slot list; we are only interested in one slot
    // Create slotlist of channels to read from ( All bits are "1" )
    uint slotlist = 0xFFFF;
    // wait for data to arrive on the slot of interest (in response to adc hardware trigger)
    int errcode = S826_AdcRead(BOARD, ADC_Buffer, NULL, &slotlist, TMAX); // S826_AdcRead(BOARD, buf, tstamp, &chan, tmax)
    // this application doesn't care about adc missed triggers
    if (errcode == S826_ERR_MISSEDTRIG)
        errcode = S826_ERR_OK;

    // ADC_Buffer now contains all of our analog inputs in integer format
    uint singleADC_reading = 0;

    double resolution   = 0;   // Resolution of signal in V/integer based on range code
    uint slot_chan      = 0;   // Buffer to store the AI channel number
    uint slot_tsettle   = 0;   // Buffer to store the settle-time (µs)
    uint slot_rangeCode = 0;   // Buffer to store the range-code (0: +-10V; 1: +-5V; 2: +-2V; 3: +-1V)

//    uint i = 0;
    for (uint i = 0; i<16; i++)
    {
        singleADC_reading = ADC_Buffer[i] & 0xFFFF; // This is the same as just buf[i]? Passes value not ref.
//        qDebug() << "Buffer Slot " << i << " has a value of: " << singleADC_reading;
        // This explicitly checks the set range code
        X826( S826_AdcSlotConfigRead(BOARD,i,&slot_chan,&slot_tsettle,&slot_rangeCode) );
        // Could speed up code by assuming that the range code is the one that was initialized at the start.
        switch (slot_rangeCode)
        {
            case 0:
                //halfRangeV = 10;
                resolution = 20.0 / 65536.0;
                break;
            case 1:
                //halfRangeV = 5;
                resolution = 10.0 / 65536.0;
                break;
            case 2:
                //halfRangeV = 2;
                resolution = 4.0 / 65536.0;
                break;
            case 3:
                //halfRangeV = 1;
                resolution = 2.0 / 65536.0;
                break;
            //default:
            //	resolution = 20 / 2^16;
            //	break;
        }

        if (singleADC_reading & 0x8000) // The reading is negative if the leading bit is "1".
        {
//            qDebug() << "The reading is negative.";
            aiV[i] = ( ( ~(singleADC_reading-1) ) & 0xFFFF ) * resolution; // subtract 1 due to range of integers being −32,768 to 32,767
            aiV[i] = -1.0 * aiV[i];
        }
        else 		     // The reading is positive.
        {
//            qDebug() << "The reading is positive.";
            aiV[i] = singleADC_reading * resolution;
        }
//        qDebug() << "Buffer Slot " << i << " has a value of: " << aiV[i];
    }

    return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DIGITAL OUPUT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// See:
// https://github.com/atelier-ritz/Coil-System-Core/blob/master/Code/s826_subroutine.c
// if digital commands and or timers need to be implemented

int S826::digitalWrite(uint chan, bool outputV)
{
    return 0;
}

int S826::digitalWriteAll(uint outputV)
{
    return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TIMERS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Configure a counter channel to operate as a periodic timer and start it running.
int S826::periodicTimerStart(uint counter, uint period)
{
    int errcode;
    int board = BOARD;
    X826( S826_CounterStateWrite(board, counter, 0)                         );     // halt channel if it's running
    X826( S826_CounterModeWrite(board, counter, TMR_MODE)                   );     // configure counter as periodic timer
    X826( S826_CounterSnapshotConfigWrite(board, counter, 4, S826_BITWRITE) );     // capture snapshots at zero counts
    X826( S826_CounterPreloadWrite(board, counter, 0, period)               );     // timer period in microseconds
    X826( S826_CounterStateWrite(board, counter, 1)                         );     // start timer
    return errcode;
}

// Halt channel operating as periodic timer.
int S826::periodicTimerStop(uint counter)
{
    return S826_CounterStateWrite(BOARD, counter, 0);
}

int S826::periodicTimerClear(uint counter)
{
    return S826_CounterModeWrite(BOARD, counter, CLEAR_BITS);
}

// Wait for periodic timer event.
int S826::periodicTimerWait(uint counter, uint *timestamp)
{
    uint counts, tstamp, reason;    // counter snapshot
    int errcode = S826_CounterSnapshotRead(BOARD, counter, &counts, &tstamp, &reason, S826_WAIT_INFINITE);    // wait for timer snapshot
    if (timestamp != NULL)
        *timestamp = tstamp;
    return errcode;
}

void S826::printCurrentTime(void)
{

}
int waitUsePeriodicTimer(uint waitTime)
{
    return 0;
}


