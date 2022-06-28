#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>

#include <fstream>
#include <QtSerialPort/QSerialPort>
#include <QFile>

// all important math functions and libraries are contained in "magneticmathfunctions.h"



#include "ui_mainwindow.h"
#include <QtGamepad/QGamepad>

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
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

//#include <QSerialPortInfo>
#include <math.h>
#include <iomanip>
#include <time.h>
#include <queue>


#include "s826.h"               // The necessary API is included within this include.
#include "daq.h"


//#define _USE_MATH_DEFINES
//#include <cmath>

#define _USE_MATH_DEFINES
#include <math.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    /// PLOTTING AND CALLBACKS TIMERS
    const int captionRefreshPeriod = 10; // 20 ms correlates to 50 Hz
    const int callbackRefreshPeriod = 10; // 20 ms correlates to 50 Hz
    // set Franka robot controller ip address here
    std::string fci_ip = "172.16.0.2";


//    Eigen::Vector3d position_d;
//    Eigen::Quaterniond  orientation_d;

    Eigen::Vector3d robottip_eulerAngles;
    Eigen::Vector3d robottip_position;

    bool RobotIsOn = false;

    double Robot_pos[3];
    double Robot_orient[3];
    double Robot_joint[7];

    double RobotEE_offset[3];

    void IsLogging(bool &isLogging) const;
    void SetFileName(const std::string & fileNameBase);
    void SetLogEnabled();
    void SetFilePath(const std::string& path);
    void SetControlMode(const int & controlMode);

    void Configure(const std::string & filename);
    void Startup(void);
    void Record(void);
    void Cleanup(void);

    //~~~~~~~~~~~~ ELECTROMAGNETIC ACTUATION SYSTEM ~~~~~~~~~~~~//
    const static int     numAct = 8; // Shouldn't ever change
    const static int     numField = 3;
    const static int     numGrad = 5;
    const static int     numProberead = 3;
    const static int     numProbePos = 3;

    constexpr static double maxAllowableTemp  = 90.0;  // [C] Maximum temperature that the system is allowed to operate at. Kill currents if temperature exceeds
    constexpr static double maxCurrent = 24.0; // [A]
    // MAX COMMAND SIGNALS Found through current experimentation with clamp meter -Adam 2021/09/13
//    constexpr static double maxCurrentCommand[numAct] = { 3.5555, 3.5976, 3.7431, 6.1806, 3.5449, 3.5449, 3.5764, 3.5036 }; // [V] For max 24 A
    constexpr static double maxCurrentCommand[numAct] = { 3.5555/2.0, 3.5976/2.0, 3.7431/2.0, 3.5389/2.0, 3.5449/2.0, 3.5449/2.0, 3.5764/2.0, 3.5036/2.0 }; // [V] For max 12 A (old EM7 6.1806/2.0)

    constexpr static double deg2rad = M_PI/180.0; // factor for changing degrees to radians

    // Considering the system as built:
    // Declare the Electromagnets (EM1, EM2, EM3, EM4, EM5, EM6, EM7, EM8)
    // in the order that they are wired. See comment below
    const double d1 =  0.287/sqrt(2.0);
    const double d2 =  0.093*sqrt(2.0);
    const double h1 = -0.300;
    double pAct_cartesion[numField][numAct] = {
    //    EM1, EM4, EM6, EM7, EM2, EM8, EM5, EM3  (As wired)
        { -d1, 0.0,  d1,  d2, -d2,  d1, 0.0, -d1 },
        { -d1, -d2, -d1, 0.0, 0.0,  d1,  d2,  d1 },
        {  h1,  h1,  h1,  h1,  h1,  h1,  h1,  h1 }
        };

//    const double m = 6324.48; // [Am^2] this is the theoretical magnetic moment on each EM based on simulation
    // 2*pi*(0.12+0.360/2)^3*(0.035366483)/mu_0 = 4.7745e+03 [A m^2] From 2021/01/13 COMSOL Simulations
    const double m = 4774.5; // [Am^2] this is most recent number. It is also similar to a volume-scaled permanent magnet
    double mAct_cartesion[numField][numAct] = { // All are in positive z direction
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {   m,   m,   m,   m,   m,   m,   m,   m}
        };

    double          ProbeReading[numProberead];
    double          ProbePos[numProbePos];
    double          cmdCoilCurrent[numAct];
    double          mrdCoilCurrent[numAct];
    double          Daqraw[numProberead];

    bool            LogEnabled = false;



protected:

    // these are for the new dual force sensing tool with three FBG array.
    //void addNote(string s);
    bool OpenFiles(const std::string &fileNameBase);
    void CloseFiles(void);

    //flags set by the user

    //functions that can be accessed via the command pattern
    void EnableLog();
    void DisableLog();

    std::ofstream  LogFileAllData;


    std::string     FileNameBase = std::string("");
    std::string     FullFileName = std::string("ASDF");
    std::string     FilePath = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data";


    char			Delim = ',';
    int             NumWritten = 0;
    int             ControlMode;
    QElapsedTimer   currentTime;


private:
    Ui::MainWindow *ui;

    double  lastTime;
    double  plotPeriod = 4.0; // in seconds. 10 s for slow scrolling, 5 s for faster.





public slots:
    void experimental_control(void);
    void freedrag(void);
    void translationaldrag(void);

private slots:
    void callbacks(void);
    void updateCaption(void);
    void updateRobotEE(void);
};
#endif // MAINWINDOW_H
