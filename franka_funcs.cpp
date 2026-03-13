// franka_move_cartesian.hpp

#include "franka_funcs.h"
#include <array>
#include <cmath>

using namespace Eigen;
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

// [x y z] elements in meters from base
std::array<double, 16> getEndEffectorHTM_COLMAJOR(franka::Robot& robot){return robot.readOnce().O_T_EE_c;} // Also O_T_EE_c
// meters from base
std::array<double, 3> getEndEffectorXYZ(franka::Robot& robot){
    std::array<double, 16> htm = getEndEffectorHTM_COLMAJOR(robot);
    return {htm[12], htm[13], htm[14]}; // [x, y, z]
}
// rotation relative to the robot base
std::array<double, 9> getEndEffectorRotMtx_COLMAJOR(franka::Robot& robot){
    std::array<double, 16> htm = getEndEffectorHTM_COLMAJOR(robot);
    return {
        htm[0], htm[1], htm[2], // Rot[:, 0]
        htm[4], htm[5], htm[6], // Rot[:, 1]
        htm[8], htm[9], htm[10] // Rot[:, 2]
    };
}
// rad, starting from base joint angle and ending at EE joint angle
std::array<double, 7> getJointAngles(franka::Robot& robot) {return robot.readOnce().q;}

void interpolateHTMs(const double* start, const double* end, double interpVal, double* ret){
    ret[12] = cycloidal_motion(start[12], end[12] - start[12], interpVal);
    ret[13] = cycloidal_motion(start[13], end[13] - start[13], interpVal);
    ret[14] = cycloidal_motion(start[14], end[14] - start[14], interpVal);
    using namespace Eigen;
    Quaterniond quatStart(Map<const Matrix<double,4,4,ColMajor>>(start).block<3,3>(0,0));
    Quaterniond quatEnd(Map<const Matrix<double,4,4,ColMajor>>(end).block<3,3>(0,0));
    Quaterniond quatInterpolated = quatStart.slerp(cycloidal_motion(0.0, 1.0, interpVal), quatEnd);
    Matrix3d rotInterpolated = quatInterpolated.toRotationMatrix();
    Map<Matrix<double,4,4,ColMajor>> retMat(ret);
    retMat.block<3,3>(0,0) = rotInterpolated;
}


void franka_moveRelativeInEE(franka::Robot& robot, const std::array<double, 16>& endHTM, double duration_sec) {

    std::array<double, 16> initHTM = robot.readOnce().O_T_EE_c;
    std::array<double, 16> newHTM  = initHTM;
    // Compute final pose when the delta is expressed in the EE frame
    // (right-multiply by the delta) - this is the only difference really
    Matrix4d finalHTM =
        Map<const Matrix<double,4,4,ColMajor>>(initHTM.data()) *
        Map<const Matrix<double,4,4,ColMajor>>(endHTM.data());
    double time = 0.0;
    try {
        robot.stop();
        robot.control([&](const franka::RobotState& state, const franka::Duration& period) -> franka::CartesianPose {
            // Update shared state for logging/telemetry
            HTM snapshot;
            std::copy_n(state.O_T_EE_c.data(), 16, snapshot.data.data());
            currentHTM.store(snapshot, std::memory_order_release);

            if (time >= duration_sec) {
                //return franka::MotionFinished(newHTM);
            }

            time += period.toSec();

            // Interpolate between init and final in world coordinates
            interpolateHTMs(initHTM.data(), finalHTM.data(), time / duration_sec, newHTM.data());
            return newHTM;
        });
        //return true;
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        robot.automaticErrorRecovery();
    }
    //return false;
}



// Build a 4x4 homogeneous transform in COLUMN-MAJOR layout.
// Translation: (x, y, z)
// Rotation:    rx about X, ry about Y, rz about Z (radians)
// Composition: R = Rz * Ry * Rx (yaw * pitch * roll)
void makeHTM(double x, double y, double z,
             double rx, double ry, double rz,
             std::array<double, 16>& endHTM)
{
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double cz = std::cos(rz), sz = std::sin(rz);

    // Rotation matrix for ZYX (yaw-pitch-roll)
    const double r00 =  cz*cy;
    const double r01 =  cz*sy*sx - sz*cx;
    const double r02 =  cz*sy*cx + sz*sx;

    const double r10 =  sz*cy;
    const double r11 =  sz*sy*sx + cz*cx;
    const double r12 =  sz*sy*cx - cz*sx;

    const double r20 = -sy;
    const double r21 =  cy*sx;
    const double r22 =  cy*cx;

    // Column-major storage: m[col*4 + row]
    endHTM[0]  = r00; endHTM[1]  = r10; endHTM[2]  = r20; endHTM[3]  = 0.0;
    endHTM[4]  = r01; endHTM[5]  = r11; endHTM[6]  = r21; endHTM[7]  = 0.0;
    endHTM[8]  = r02; endHTM[9]  = r12; endHTM[10] = r22; endHTM[11] = 0.0;
    endHTM[12] = x;   endHTM[13] = y;   endHTM[14] = z;   endHTM[15] = 1.0;
}

static inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

void franka_teleopEE(franka::Robot& robot, std::atomic<TwistCmd>& cmdSource,
                     std::atomic<bool>& finishFlag) {
  try {
    robot.stop();  // ensure no other motion is active

    robot.control([&](const franka::RobotState& state,
                      const franka::Duration& period) -> franka::CartesianPose {
      static std::array<double, 16> out = state.O_T_EE_c;  // init with current

      if (finishFlag.load(std::memory_order_relaxed)) {
        return franka::MotionFinished(out);
      }

      // read command (already thresholded in the poller)
      TwistCmd cmd = cmdSource.load(std::memory_order_relaxed);

      // extra safety clamps
      const double v_lin_max = 0.10;
      const double v_ang_max = 0.8;
      cmd.vx = clamp(cmd.vx, -v_lin_max, v_lin_max);
      cmd.vy = clamp(cmd.vy, -v_lin_max, v_lin_max);
      cmd.vz = clamp(cmd.vz, -v_lin_max, v_lin_max);
      cmd.wx = clamp(cmd.wx, -v_ang_max, v_ang_max);
      cmd.wy = clamp(cmd.wy, -v_ang_max, v_ang_max);
      cmd.wz = clamp(cmd.wz, -v_ang_max, v_ang_max);

      const double dt = period.toSec();

      // If all zeros => hold pose (instant stop)
      if (std::abs(cmd.vx)+std::abs(cmd.vy)+std::abs(cmd.vz)+
          std::abs(cmd.wx)+std::abs(cmd.wy)+std::abs(cmd.wz) < 1e-12) {
        return out;
      }

      // Current EE pose
      Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::ColMajor>> Tcur(state.O_T_EE_c.data());

      // Small delta transform in EE frame from twist*dt
      Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
      Eigen::Vector3d w(cmd.wx, cmd.wy, cmd.wz);
      const double theta = w.norm() * dt;
      Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
      if (theta > 1e-9) dR = Eigen::AngleAxisd(theta, w.normalized()).toRotationMatrix();
      Eigen::Vector3d dp(cmd.vx*dt, cmd.vy*dt, cmd.vz*dt);
      dT.topLeftCorner<3,3>() = dR;
      dT.topRightCorner<3,1>() = dp;

      // Apply as right-multiply (EE-frame command)
      Eigen::Matrix4d Tnext = Tcur * dT;

      // Back to column-major array
      Eigen::Map<Eigen::Matrix<double,4,4,Eigen::ColMajor>>(out.data()) = Tnext;
      return out;
    });

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    robot.automaticErrorRecovery();
  }
}


