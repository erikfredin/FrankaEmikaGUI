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

#include <iostream>


#include <iostream>

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

    QVector<double> Robot_pos;
    QVector<double> Robot_orient;

private:
    Ui::MainWindow *ui;

    QElapsedTimer currentTime;
    double lastTime;
    double plotPeriod = 4.0; // in seconds. 10 s for slow scrolling, 5 s for faster.

public slots:
    void connectioncheck(void);

private slots:
    void callbacks(void);
    void updateCaption(void);
};
#endif // MAINWINDOW_H
