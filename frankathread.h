#ifndef FRANKATHREAD_H
#define FRANKATHREAD_H

#include <QtCore>

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
#include "robotstatus.h"



class FrankaThread : public QThread
{
public:
    FrankaThread();
    ~FrankaThread();
//    void run(std::string fci_ip, bool isRobotconnect);
    void run();
    bool isStop;

private:
    std::string     fci_ip = "172.16.0.2";
//    bool            isRobotconnect = false;
};

#endif // FRANKATHREAD_H
