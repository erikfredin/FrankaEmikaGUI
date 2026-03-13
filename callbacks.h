#ifndef CALLBACKS_H
#define CALLBACKS_H


#include "mainwindow.h"
#include "gripper_cam_move.h"
#include <franka/robot.h>
#include <franka/exception.h>
#include "franka_funcs.h"
#include <iostream>
#include <ctime> // For time_t and time()

// This callbacks function is separated for better organization of the code

Eigen::Matrix<double, 8, 1> coilCurrents;
double I_command[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int theta1;
int theta2;
double K1;
double K2;

Eigen::Vector2d qd;
Eigen::Vector2d k_joints;
//static Eigen::Vector2d offset {0.0, -1e-04};
//float k_joints = 0.0009; //0.0003;

Eigen::Matrix<double,2,8> mMu;
Eigen::Vector2d tauK;
Eigen::Vector2d tauInt;
static cv::Mat mat;

double dist_max = 0.005; //10 mm
double time_interpl = 0.5; // seconds
time_t current_time;
time_t last_cmd_time;
std::once_flag flag;
double time_limit = 1; //seconds




#endif // CALLBACKS_H
