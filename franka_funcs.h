#ifndef FRANKA_FUNCS_H
#define FRANKA_FUNCS_H

#pragma once

#include <array>
#include <iostream>
#include <functional>
#include <cmath>
#define _USE_MATH_DEFINES

#include "frankathread.h"

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <atomic>
#include "gamepadpoller.h"



bool EE_moveInX(franka::Robot& robot, const double& x_meters, const double& duration_sec = 5.0);
bool EE_moveInY(franka::Robot& robot, const double& y_meters, const double& duration_sec = 5.0);
bool EE_moveInZ(franka::Robot& robot, const double& z_meters, const double& duration_sec = 5.0);
bool EE_rotateAboutX(franka::Robot& robot, const double& angleX_rad, const double& duration_sec = 5.0);
bool EE_rotateAboutY(franka::Robot& robot, const double& angleY_rad, const double& duration_sec = 5.0);
bool EE_rotateAboutZ(franka::Robot& robot, const double& angleZ_rad, const double& duration_sec = 5.0);

std::array<double, 16> getEndEffectorHTM_COLMAJOR(franka::Robot& robot);
std::array<double, 3> getEndEffectorXYZ(franka::Robot& robot);
std::array<double, 9> getEndEffectorRotMtx_COLMAJOR(franka::Robot& robot);
std::array<double, 7> getJointAngles(franka::Robot& robot);

struct HTM {std::array<double, 16> data;}; // only used for an atomic variable for thread safe logging
static std::atomic<HTM> currentHTM; // column-major
void interpolateHTMs(const double* start, const double* end, double interpVal, double* ret);
void franka_moveRelativeInEE(franka::Robot& robot, const std::array<double, 16>& endHTM, double duration_sec);
void makeHTM(double x, double y, double z, double rx, double ry, double rz, std::array<double, 16>& endHTM);
void franka_teleopEE(franka::Robot& robot, std::atomic<TwistCmd>& cmdSource, std::atomic<bool>& finishFlag);

#endif // FRANKA_FUNCS_H
