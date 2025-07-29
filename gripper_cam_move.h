#ifndef GRIPPER_CAM_MOVE_H
#define GRIPPER_CAM_MOVE_H

#include <franka_funcs.h>
#include <Eigen>
#include <fstream>
#include "magserialrobot.h"
#include "nlohmann/json.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;

//MagSerialRobot& magbot();


// Params
extern const Eigen::Matrix4d T_gripper;
extern const Eigen::Matrix4d T_Cam2EE; // Still need to determine/calibrate this!

// Functions
Eigen::Matrix4d get_EE_TMat(franka::Robot& robot);
void get_and_print_EE_TMat(franka::Robot& robot);

void move_camera_rel_gripper(franka::Robot& robot, double x, double y, double z);
Eigen::Matrix4d get_gripper2cam_transform(franka::Robot& robot);
void point_camera_to_gripper(franka::Robot& robot);
void record_data(franka::Robot& robot, nlohmann::json& j, double coil_currents[8], int& group_idx, int& sample_cntr, cv::VideoCapture capCam, double time);
Eigen::Matrix<double, 8, 1> run_open_loop(MagSerialRobot& magbot, int theta1, int theta2, double K1, double K2);
extern string gripper_data_path;


extern const Eigen::Matrix<double,8,8> mCurrentToFieldMatrix;
extern const Eigen::Matrix<double,8,8> mCoilMatrix;
nlohmann::json toJsonArray(const Eigen::Matrix4d& T);



#endif // GRIPPER_CAM_MOVE_H
