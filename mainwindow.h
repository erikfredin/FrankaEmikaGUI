
#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>

#include <fstream>
#include <QtSerialPort/QSerialPort>
#include <QFile>

// all important math functions and libraries are contained in "magneticmathfunctions.h"



#include "ui_mainwindow.h"

#include <QDebug>
#include <QTimer>
#include <ctime>
#include <QElapsedTimer>

#include <string>
#include <iostream>
#include <QString>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include <Core>


#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Dense>
#include <franka/duration.h>
#include <franka/model.h>


#include "examples_common.h"

//#include <QSerialPortInfo>
#include <math.h>
#include <iomanip>
#include <time.h>
#include <queue>


#include "s826.h"               // The necessary API is included within this include.
#include "daq.h"

#include "gamepadmonitor.h"
#include <QtGamepad/QGamepad>
//#include "qcustomplot.h"
#include "frankathread.h"
#include "robotstatus.h"
//#include <boost/>
#include <thread>

#include <liborl/liborl.h>
#include <memory>


#include <chrono>

//#include "src/model.h"
//using keras2cpp::Model;
//using keras2cpp::Tensor;

#include <fdeep/fdeep.hpp>

#include <boost/algorithm/string.hpp>

#include "electromagnet_calibration.h"
#include "scalorPotential.h"


// These are defined in the S826api header internally. Left here for reference
#define AOUT0_PIN   42
#define AOUT1_PIN   44
#define AOUT2_PIN   46
#define AOUT3_PIN   48
#define AOUT4_PIN   41
#define AOUT5_PIN   43
#define AOUT6_PIN   45
#define AOUT7_PIN   47

// AIN0 - AIN7  ARE CURRENT SENSE FROM EM0-EM7, respectively
#define NEG_AIN0_PIN    3
#define POS_AIN0_PIN    4
#define NEG_AIN1_PIN    5
#define POS_AIN1_PIN    6
#define NEG_AIN2_PIN    7
#define POS_AIN2_PIN    8
#define NEG_AIN3_PIN    9
#define POS_AIN3_PIN    10
#define NEG_AIN4_PIN    11
#define POS_AIN4_PIN    12
#define NEG_AIN5_PIN    13
#define POS_AIN5_PIN    14
#define NEG_AIN6_PIN    15
#define POS_AIN6_PIN    16
#define NEG_AIN7_PIN    17
#define POS_AIN7_PIN    18
// AIN8 - AIN15 ARE THERMOCOUPLE SENSE FROM EM0 - EM7, respectively
#define NEG_AIN8_PIN    21
#define POS_AIN8_PIN    22
#define NEG_AIN9_PIN    23
#define POS_AIN9_PIN    24
#define NEG_AIN10_PIN   25
#define POS_AIN10_PIN   26
#define NEG_AIN11_PIN   27
#define POS_AIN11_PIN   28
#define NEG_AIN12_PIN   29
#define POS_AIN12_PIN   30
#define NEG_AIN13_PIN   31
#define POS_AIN13_PIN   32
#define NEG_AIN14_PIN   33
#define POS_AIN14_PIN   34
#define NEG_AIN15_PIN   35
#define POS_AIN15_PIN   36

//#define _USE_MATH_DEFINES
//#include <cmath>

#define _USE_MATH_DEFINES
#include <math.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

using namespace orl;

using namespace std::literals;
//class ElectromagnetCalibration;

class MainWindow : public QMainWindow
{
    Q_OBJECT



public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    // set Franka robot controller ip address here
    std::string fci_ip = "192.168.100.2";

//    bool isRobotConnected = false;
//    Eigen::Vector3d position_d;
//    Eigen::Quaterniond  orientation_d;

    Eigen::Vector3d robottip_eulerAngles;
    Eigen::Vector3d robottip_position;

    double Robot_tip_posisition[3] = {0.0, 0.0, 0.0};
    double Robot_orient[3] = {0.0, 0.0, 0.0};
    double Robot_joint[7] = { 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0};

    double RobotEE_offset[3] = {0.0, 0.0, 0.0};

    void SetControlMode(const int & controlMode);
//    void SetFileName(const std::string & fileNameBase);
//    void SetLogEnabled();

    void Record(void);


    //~~~~~~~~~~~~ ELECTROMAGNETIC ACTUATION SYSTEM ~~~~~~~~~~~~//
    //-----moved from Adams's code-------
    const static int     numAct = 8; // Shouldn't ever change
    const static int     numField = 3;
    const static int     numGrad = 5;
    const static int     numProberead = 3;
    const static int     numProbePos = 3;

    constexpr static double maxAllowableTemp  = 90.0;  // [C] Maximum temperature that the system is allowed to operate at. Kill currents if temperature exceeds
    constexpr static double maxCurrent = 24.0; // [A], maxvalue based on Adam's code
//    constexpr static double maxCurrent = 23.5; // [A], decrease to 23.5A to make it safer
    // MAX COMMAND SIGNALS Found through current experimentation with clamp meter -Adam 2021/09/13
//    constexpr static double maxCurrentCommand[numAct] = { 3.5555, 3.5976, 3.7431, 6.1806, 3.5449, 3.5449, 3.5764, 3.5036 }; // [V] For max 24 A
    constexpr static double maxCurrentCommand[numAct] = { 3.5555/2.0, 3.5976/2.0, 3.7431/2.0, 3.5389/2.0, 3.5449/2.0, 3.5449/2.0, 3.5764/2.0, 3.5036/2.0 }; // [V] For max 12 A (old EM7 6.1806/2.0)

    constexpr static double deg2rad = M_PI/180.0; // factor for changing degrees to radians

    // Considering the system as built:
    // Declare the Electromagnets (EM1, EM2, EM3, EM4, EM5, EM6, EM7, EM8)
    // in the order that they are wired. See comment below
//    const double d1 =  0.287/sqrt(2.0);
//    const double d2 =  0.093*sqrt(2.0);
//    const double h1 = -0.300;
//    double pAct_cartesion[numField][numAct] = {
//    //    EM1, EM4, EM6, EM7, EM2, EM8, EM5, EM3  (As wired)
//        { -d1, 0.0,  d1,  d2, -d2,  d1, 0.0, -d1 },
//        { -d1, -d2, -d1, 0.0, 0.0,  d1,  d2,  d1 },
//        {  h1,  h1,  h1,  h1,  h1,  h1,  h1,  h1 }
//        };
    const double d1 =  0.287/sqrt(2.0);
    const double d2 =  0.093*sqrt(2.0);
    const double h1 = -0.37897/2.0;
    double pAct_cartesion[numField][numAct] = {
    //    EM1, EM2, EM3,  EM4,  EM5, EM6, EM7,  EM8  (As wired)
        { -d1, -d2, -d1,  0.0,  0.0, d1,   d2,  d1 },
        { -d1,  0.0, d1,  -d2,  d2,  -d1,  0.0, d1 },
        {  h1,  h1,  h1,   h1,  h1,  h1,   h1,  h1 }
        };

//    const double m = 6324.48; // [Am^2] this is the theoretical magnetic moment on each EM based on simulation
    // 2*pi*(0.12+0.360/2)^3*(0.035366483)/mu_0 = 4.7745e+03 [A m^2] From 2021/01/13 COMSOL Simulations
    const double m = 4774.5; // [Am^2] this is most recent number. It is also similar to a volume-scaled permanent magnet
    double mAct_cartesion[numField][numAct] = { // All are in positive z direction
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {   m,   m,   m,   m,   m,   m,   m,   m}
        };

    /// GAME CONTROLLER OBJECT
    GamepadMonitor connectedGamepad; // Gamepad monitor for the connected game controller (Use Xbox)

    /// PLOTTING AND CALLBACKS TIMERS
    const int captionRefreshPeriod = 10; // 20 ms correlates to 50 Hz
    const int callbackRefreshPeriod = 10; // 20 ms correlates to 50 Hz

    /// S826 BOARD FOR CONTROL
    S826 s826;

//    std::string name1 = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess.yaml";
//    ElectromagnetCalibration mymodel(name1);

//ElectromagnetCalibration mymodel(std::string("C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess.yaml"));

    /// DAQ FOR FEEDBACK
    daq DAQ;
    double gaussmeterCalibratedConstants[3] = {102.67, 103.25, 104.66}; // mT/V
    double gaussCalibCons_new[3] = {102.8047, 103.17, 104.27}; // mT/V

    /// CONTROL BOOLEANS and STATE VALUES
    bool overheatingFlag = false;
    bool isGradientControlled = false;
    bool controllerState; // State for enabling or disabling the game controller.
    bool useCalibratedFieldValues = true;
    // Enable/Disable Plots State
    bool magneticPlotState; // State for enabling realtime plotting of the magnetic field.
    bool gradientPlotState; // State for enabling realtime plotting of field gradients on the magnetic field plot.
    bool magneticLegendState;
    bool inputPlotState; // State for enabling the realtime plotting of amplifier currents in the EMs.
    bool setpointPlotState; // State for enabling realtime plotting on current setpoints on the input plot.
    bool inputLegendState;
    bool temperaturePlotState;
    bool temperatureLegendState;


    int currentControllerMode; // Specifies the state of the game controller. This dictates how the joystick inputs are used.
    int currentSubroutineMode; // Specifies the state of the Subroutines.
    bool subroutineState;

    /// CALIBRATION CONSTANTS

    /// CONTROL MATRICES
    // Declare the control matrix for relating the currents to the magnetic field and gradients
    // This matrix considers the gradients as well as the field components
    double N[numAct][numField+numGrad] = { {  0.0036336,	-0.00078,	-0.004188,	-0.0172944,	 0.01758,	-0.0034944,	 0.0017976,	 0.0040032 },
                                           {  0.0037728,	 0.0181776,	 0.003588,	-0.0010728,	 0.0008232,	-0.0040848,	-0.0170712,	-0.0036168 },
                                           { -0.0007968,	 0.0125472,	-0.0012336,	 0.012252,	 0.01212,	-0.0011952,	 0.0121992,	-0.0011472 },
                                           { -0.0153312,	 0.15354,	-0.0191832,	-0.0795432,	-0.0935904,	-0.012216,	 0.1548408,	-0.0230928 },
                                           { -0.0383184,	 0.0036216,	 0.0412104,	-0.0063,	-0.0008544,	-0.0371352,	 0.015732,	 0.0364968 },
                                           { -0.0083544,	 0.0153336,	 0.0099864,	 0.2313216,	-0.2274,	 0.0070104,	-0.011592,	-0.01098   },
                                           { -0.0187776,	-0.0901176,	-0.0144816,	 0.1492368,	 0.1640112,	-0.0203568,	-0.0964416,	-0.0139104 },
                                           { -0.0108264,	-0.247728,	-0.0094248,	 0.0090024,	-0.0205296,	 0.0097824,	 0.2304216,	 0.0087936 } };
    // Declare the inverse of the control matrix relating the magnetic field components to the currents
    double invN[numField+numGrad][numAct] =  {0.0};
    // Declare the control matrix for relating the currents to the magnetic field
    // This matrix does not consider the gradients, only the field components
    // Uses these calibrated field values unless useCalibratedFieldValues is set to false
    double M[numField][numAct] =   { { 0.0036336,	-0.00078,	-0.004188,	-0.0172944,	0.01758,	-0.0034944,	 0.0017976,	 0.0040032},
                                     { 0.0037728,	 0.0181776,	 0.003588,	-0.0010728,	0.0008232,  -0.0040848,	-0.0170712,	-0.0036168},
                                     {-0.0007968,	 0.0125472,	-0.0012336,	 0.012252,	0.01212,	-0.0011952,	 0.0121992,	-0.0011472} };
    // Declare the psuedo-inverse of the non-square control matrix relating the currents to the magnetic field components
    double pseudoinvM[numAct][numField] =  {0.0};

    /// CALIBRATION CONSTANTS
    // These are the calibration constants for each electromagnet current signals
//    const double currentControlAdj[numAct] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; // [T/V]
//    const double currentControlAdj[numAct] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2}; // [V/A] Max 5V for 25 A Output (OLD ESTIMATES)
    const double currentControlAdj[numAct] = {1.0/6.7501, 1.0/6.6705, 1.0/6.4118, 1.0/6.7818, 1.0/6.7703, 1.0/6.7703, 1.0/6.7107, 1.0/6.8500}; // [V/A] Max Command signal for 24 A Output shown above
//    const double currentSenseAdj[numAct] = {8.05, 8.05, 8.05, 8.05, 8.05, 8.05, 8.05, 8.05}; // [A/V] (OLD ESTIMATES)
    const double currentSenseAdj[numAct] = {6.7501, 6.6705, 6.4118, 3.8831, 6.7703, 6.7703, 6.7107, 6.8500}; // [A/V] (OLD ESTIMATES)
//    const double currentSenseAdj[numAct] = {1.0/0.6338, 1.0/0.6516, 1.0/0.6791, 1.0/0.64, 1.0/0.6422, 1.0/0.6395, 1.0/0.6341, 1.0/0.6392}; // [A/V] (NEW ESTIMATES) (NOTE EM7 is not calibrated)
    const double temperatureSenseAdj[numAct] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}; // [deg C/V]

 //    const double ATINanoVoltageOffsets[6] = {-1.26987250,	3.78201850,	2.1132725,	2.020594,	-0.4165031,	-0.75069465}; // just magnet experiments
    const double ATINanoVoltageOffsets[6] = {-1.3171104,	3.8164696,	2.2166968,	1.9402872,	-0.44156372,	-1.0281404}; // Gripper experiments

    double ATINanoCalibrationMatrix[6][6] =       {{  0.00749,	 0.02250,	 0.01747,	-0.80775,	-0.02452,	 0.81951},
                                                   { -0.03933,	 0.97698,	 0.01759,	-0.43942,	 0.00040,	-0.49738},
                                                   {  0.92086,	 0.03400,	 0.93714,	-0.00522,	 0.97298,	 0.01115},
                                                   { -0.55209,	 5.99193,	 5.23393,	-2.69998,	-5.44013,	-3.14306},
                                                   { -5.88807,	-0.35924,	 2.76672,	 4.91801,	 3.46558,	-4.95869},
                                                   { -0.17835,	 3.57785,	-0.02007,	 3.30164,	-0.08660,	 3.66835}};

    /// CURRENTS AND FIELD VALUES
    // should there be gradients considered for the local field? if so, how do you transform them to find the global frame equivalent?
    double B_Local_Desired[numField] = {0.0};      // [T?]
    double B_Global_Desired[numField+numGrad] = {0.0};     // [T?]
//    double B_Global_Desired_Input[8] = {0.0};
    double B_Global_Output[numField+numGrad]  = {0.0}; // [T] theoretical field values based on measured currents
    double measuredCurrents[numAct]     = {0.0}; // [A] read from amplifiers
    double measuredTemperatures[numAct] = {0.0}; // [C] thermocouple feedback readings converted to deg C
    double inputAnalogVoltages[16] = {0.0}; // [V] feedback voltage values. Contains both coil current monitor and thermocouples voltages
    double ATINanoForceTorque[6] = {0.0}; //
    // Input Analog Voltages are as follows:
    // AIN0 - AIN7  ARE CURRENT SENSE FROM EM0-EM7, respectively
    // AIN8 - AIN15 ARE THERMOCOUPLE SENSE FROM EM0 - EM7, respectively
    double currentSetpoints[numAct]     = {0.0}; // [A] out to amplifiers
    double outputAnalogVoltages[numAct] = {0.0}; // [V] Voltage values to go out to the current amplifiers
    // Output Analog Voltages are as follows:
    // AOUT0 - AOUT7 ARE CURRENT REFERENCES FOR EM0 - EM7, respectively



    double          msdFieldvalue[numProberead];
    double          ProbePos[numProbePos];

    double          msdCoilCurrent[numAct];
    double          Daqraw[numProberead];



    bool            LogEnabled = false;

//    FrankaThread    *frankathread;
    FrankaThread    *frankathread = new FrankaThread;
//    QThread *thread = new QThread;

    bool            isRobotStreaming = false;

    void            robotfreedraginthread(void);

    // initial variable for registration between robot and coil table
//    Eigen::MatrixXd     Ac;
    const static int        registerdataNum = 9;
    double     robotdata[4][registerdataNum]; //size=4*9
    //cooornates of nut center 1,2,3,4,5,6,7,8,0
    double     tabledata[4][registerdataNum] = {  {-202.94*0.001,   -131.52*0.001,   -202.94*0.001,     0.0,            0.0,              202.94*0.001,     131.52*0.001,     202.94*0.001,     0.0},
                                                  {-202.94*0.001,    0.0,            202.94*0.001,      -131.52*0.001,  131.52*0.001,     -202.94*0.001,     0.0,             202.94*0.001,     0.0},
                                                  {0.0,             6.5*0.001,       14.7*0.001,        8.8*0.001,       10.3*0.001,        17.4*0.001,       20.9*0.001,      12.6*0.001,     85.3*0.001},
                                                  {1.0,             1.0,                1.0,            1.0,            1.0,                1.0,              1.0,             1.0,           1.0}};

    Eigen::Matrix4d     transT2R;

    int                 registerdataCount = 0;
    void    registration(double tabledata[4][registerdataNum], double robotdata[4][registerdataNum] ); //output Tt2r, Probot = Tt2r*Ptable
//    void    collectrobotdata(void);

    // define field calibration cubic space
    //! cubic range is 200-200-100 mm
//    double       leftlowercorner[3] = {-100.0, -100.0, 50.0}; //table origin is at the table center (0,0,0)
//    double       righttopcorner[3] = {100.0, 100.0, 150.0};
    //! cubic range is 100-100-50 mm
    double       leftlowercorner[3] = {-50.0, -50.0, 75.0}; //table origin is at the table center (0,0,0)
    double       righttopcorner[3] = {50.0, 50.0, 125.0};
    double       incstep = 10.0;
    bool         CalibrationDataCollet = false;
    double       cmdCoilCurrent[numAct] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int          currentloop = 1;
    int          currentcount = 0;
    int          robotmoveloop = 20;
    int          robotmovecount = 0;
    bool         robotloopisdone = true;
    bool         robotinitialized = false;
    int          Robotmotionsuccess = 0;
//    bool         currenttakebreak = false;
    double       zeroCurrent[numAct] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double       robotposcmd[3]; //unit: m
    double       currentcooldownloop = 10.0;
    double       tempCoilCurrent[numAct] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    std::array<double, 16> current_EEpose;

    // inputs for NN predictor
    double NN_B1[3] = {30.0, 10.0,   0.0};
    double NN_B2[3] = {20.0, 5.0, 0.0};
    double NN_P1[3] = {50.0, -50.0, 60.0};
    double NN_P2[3] = {30.0, -50.0, 60.0};
    std::vector<float> DNNinputprepare(double NN_B1[3], double NN_P1[3], double NN_B2[3], double NN_P2[3]);
    double       predictCoilCurrent[numAct] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



protected:

    // these are for the new dual force sensing tool with three FBG array.
    //void addNote(string s);
//    bool OpenFiles(const std::string &fileNameBase);
//    void CloseFiles(void);

    //flags set by the user



    std::ofstream  LogFileAllData;


    std::string     FileNameBase = std::string("");
    std::string     FullFileName = std::string("ASDF");
    std::string     FilePath = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data/";


    char			Delim = ',';
    int             NumWritten = 0;
    int             ControlMode;
    QElapsedTimer   currentTime;


private:
    Ui::MainWindow *ui;

    double  lastTime;
    double  plotPeriod = 4.0; // in seconds. 10 s for slow scrolling, 5 s for faster.





public slots:
    void        experimental_control(void);
    void        freedrag(void);
    void        translationaldrag(void);
//    void SetFileName(std::string & fileNameBase);
//    void SetFileNamenew(std::string & fileNameBase);
    void        SetLogEnabled();
    bool        OpenFiles(std::string &fileNameBase);
    void        CloseFiles(void);
    //functions that can be accessed via the command pattern
    void        EnableLog();
    void        DisableLog();
    void        Setfilename(void);
    void        Robotconnect(void);

    void        frankathreadcontrol(void);
//    void        frankathreadquit(void);
    void        robotstreaming(void);
    void        pilotthreadon(void);
    void        pilotthreadoff(void);
    void        robotrecovery(void);
    void        enableDAQ(void);
    void        clearcurrent(void);
    void        DNNpredict(void);
    void        updateDNNinput(void);
    void        runDNNcurrent(void);
    void        moveRot_P1(void);
    void        moveRot_P2(void);
    void        initializeFranka(void);
    void        runGlobalfield(void);

    void        Cartesiantest(void);

    void        enableController(void);
    void        setFrankaguidingmode(void);

private slots:
    void       callbacks(void);
    void       updateCaption(void);
    void       updateRobotEE(void);
    void       updateCurrents_CalibrationOnly(double I_command[8]);
    void       updateCurrents(void);
    void       registerdatacollect(void);

    void       calibratesetflag(void);
    void       calibratesetflagoff(void);

};
#endif // MAINWINDOW_H
