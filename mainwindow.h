#ifndef MAINWINDOW_H
#define MAINWINDOW_H



#include <QMainWindow>

#include <fstream>
#include <QtSerialPort/QSerialPort>
#include <QFile>

// all important math functions and libraries are contained in "magneticmathfunctions.h"

//#include "callbacks.h"   // Does this need to be called?


#include "ui_mainwindow.h"
#include <QtGamepad/QGamepad>

#include <QDebug>
#include <QTimer>
#include <ctime>
#include <QElapsedTimer>

#include <expat_external>

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

private:
    Ui::MainWindow *ui;

    QElapsedTimer currentTime;
    double lastTime;
    double plotPeriod = 4.0; // in seconds. 10 s for slow scrolling, 5 s for faster.

public slots:
    void connectioncheck(void);

private slots:
    void callbacks(void);
};
#endif // MAINWINDOW_H
