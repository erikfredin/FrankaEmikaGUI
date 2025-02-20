// franka_move_cartesian.hpp

#include "franka_funcs.h"

// motion with zero starting and ending velocity and acceleration - very smooth and most likely to never trip the franka impedence limits
// startVal = starting value
// deltaVal = value change at interpVal = 1.0
// interpVal should be limited to (0, 1)
static double cycloidal_motion(const double& startVal, const double& deltaVal, const double& interpVal){
    return startVal + deltaVal * M_1_PI * (M_PI * interpVal - 0.5 * std::sin(2.0 * M_PI * interpVal));
}
// NOTE: always using column-major format for htm arrays
// interpolateFcn returns nothing (void fcn) and takes in:
//  const double& interpVal - the interpolation value, always = current_execution_time / duration_sec
//  const std::array<double, 16>& curHtm - the robot EE htm BEFORE motion starts
//  std::array<double, 16>& newHTM - the variable where the new pose at the interpolation value should be stored. this will be initialized to curHTM at the start of the motion.
static bool EE_move_timeInterpolated(
    franka::Robot& robot,
    const double& duration_sec,
    const std::function<void(const double&, const std::array<double, 16>&, std::array<double, 16>&)>& interpolateFcn
    ){
    std::array<double, 16> initPose = robot.readOnce().O_T_EE_c;
    std::array<double, 16> newPose = initPose;
    double time = 0.0;
    try {
        robot.stop();
        robot.control([&](const franka::RobotState& robot_state, const franka::Duration& period)->franka::CartesianPose{
            time += period.toSec();
            interpolateFcn(time/duration_sec, initPose, newPose);
            if (time >= duration_sec){return franka::MotionFinished(newPose);}
            return newPose;
        });
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        robot.automaticErrorRecovery();
        return false;
    }
}
bool EE_moveInX(franka::Robot& robot, const double& x_meters, const double& duration_sec){
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM, std::array<double, 16>& newHTM)->void {
        newHTM[12] = cycloidal_motion(curHTM[12], x_meters, interpVal);
    };
    return EE_move_timeInterpolated(robot, duration_sec, interpFunc);
}
bool EE_moveInY(franka::Robot& robot, const double& y_meters, const double& duration_sec){
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM, std::array<double, 16>& newHTM)->void {
        newHTM[13] = cycloidal_motion(curHTM[13], y_meters, interpVal);
    };
    return EE_move_timeInterpolated(robot, duration_sec, interpFunc);
}
bool EE_moveInZ(franka::Robot& robot, const double& z_meters, const double& duration_sec){
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM, std::array<double, 16>& newHTM)->void {
        newHTM[14] = cycloidal_motion(curHTM[14], z_meters, interpVal);
    };
    return EE_move_timeInterpolated(robot, duration_sec, interpFunc);
}
bool EE_rotateAboutX(franka::Robot& robot, const double& angleX_rad, const double& duration_sec){
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM, std::array<double, 16>& newHTM)->void {
        double rotAngle = cycloidal_motion(0.0, angleX_rad, interpVal);
        double ctheta = std::cos(rotAngle);
        double stheta = std::sin(rotAngle);
        Eigen::Matrix3d rotMtx;
        rotMtx <<   1.0, 0.0, 0.0,
                    0.0, ctheta, -stheta,
                    0.0, stheta, ctheta;
        Eigen::Matrix4d transformMatrix = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(curHTM.data());
        transformMatrix.block<3,3>(0,0) = rotMtx * transformMatrix.block<3,3>(0,0);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(newHTM.data()) = transformMatrix;
    };
    return EE_move_timeInterpolated(robot, duration_sec, interpFunc);
}
bool EE_rotateAboutY(franka::Robot& robot, const double& angleY_rad, const double& duration_sec){
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM, std::array<double, 16>& newHTM)->void {
        double rotAngle = cycloidal_motion(0.0, angleY_rad, interpVal);
        double ctheta = std::cos(rotAngle);
        double stheta = std::sin(rotAngle);
        Eigen::Matrix3d rotMtx;
        rotMtx <<   ctheta, 0.0, stheta,
                    0.0, 1.0, 0.0,
                    -stheta, 0.0, ctheta;
        Eigen::Matrix4d transformMatrix = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(curHTM.data());
        transformMatrix.block<3,3>(0,0) = rotMtx * transformMatrix.block<3,3>(0,0);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(newHTM.data()) = transformMatrix;
    };
    return EE_move_timeInterpolated(robot, duration_sec, interpFunc);
}
bool EE_rotateAboutZ(franka::Robot& robot, const double& angleZ_rad, const double& duration_sec){
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM, std::array<double, 16>& newHTM)->void {
        double rotAngle = cycloidal_motion(0.0, angleZ_rad, interpVal);
        double ctheta = std::cos(rotAngle);
        double stheta = std::sin(rotAngle);
        Eigen::Matrix3d rotMtx;
        rotMtx <<   ctheta, -stheta, 0.0,
                    stheta, ctheta, 0.0,
                    0.0, 0.0, 1.0;
        Eigen::Matrix4d transformMatrix = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(curHTM.data());
        transformMatrix.block<3,3>(0,0) = rotMtx * transformMatrix.block<3,3>(0,0);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(newHTM.data()) = transformMatrix;
    };
    return EE_move_timeInterpolated(robot, duration_sec, interpFunc);
}
