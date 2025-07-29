#include "gripper_cam_move.h"
#include "math.h"
#include <iomanip>
#include "nlohmann/json.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace Eigen;
using namespace nlohmann;

// Define constant variables here (needs to be done in source file at runtime).
// Define these constant variables only here!!
const Eigen::Matrix4d T_gripper = (Eigen::Matrix4d() <<
    0,  0, -1, 0.478749,
    -1, 0,  0, 0.00236887,
    0,  1,  0,  0.227299,
    0,  0,  0,         1).finished(); // Translation is rough, rotation ideal (i.e., may be view degrees off). Calibrate position later!!

const Eigen::Matrix4d T_Cam2EE = (Eigen::Matrix4d() <<
                                  0, 0, 1, 0,
                                  -1, 0, 0, 0,
                                  0, -1, 0, 0.05,
                                  0, 0, 0, 1).finished(); // Translation is made up PLACEHOLDER. Rotation is ideal transform, but may need adjustment via calibration.


string gripper_data_path = "C:\\Users\\MicroRoboticsLab\\Desktop\\Erik\\SLL\\8-DoF Data\\Flexible Train Data";

const Eigen::Matrix<double,8,8> mCurrentToFieldMatrix {
              {3.6,   -0.7,  -4.1, -17.2,   17.5,  -3.4,   1.7,   4.0}, // mT/%A
              {3.7,   18.1,   3.5,  -1.0,    0.8,  -4.0, -17.0,  -3.6}, // mT/%A
             {-0.7,   12.5,  -1.2,  12.2,   12.1,  -1.1,  12.1,  -1.1}, // mT/%A
            {-15.3,  153.5, -19.1, -79.5,  -93.5, -12.2, 154.8, -23.0}, // mT/m.%A
            {-38.3,    3.6,  41.2,  -6.3,   -0.8, -37.1,  15.7,  36.4}, // mT/m.%A
             {-8.3,   15.3,   9.9, 231.3, -227.4,   7.0, -11.5, -10.9}, // mT/m.%A
            {-18.7,  -90.1, -14.4, 149.2,  164.0, -20.3, -96.4, -13.9}, // mT/m.%A
            {-10.8, -247.7,  -9.4,   9.0,  -20.5,   9.7, 230.4,   8.7} // mT/m.%A
    };

const Eigen::Matrix<double,8,8> mCoilMatrix = mCurrentToFieldMatrix * 0.001 / 24.0; //T/A and T/m.A

//--------------------------------- Initialize Magbot -------------------------
static MagSerialRobot makeMagbot()
{
    /* ---- fixed parameters ---- */
    constexpr int numLinks = 2;

    /* 1. plain C arrays → constructor wants *non-const* double* / int* ---- */
    double linkLength [numLinks] = { 7.22e-3,  7.77e-3 };
    double linkTwist  [numLinks] = { 1.571  ,  0.0     };
    double linkOffset [numLinks] = { 0.0    ,  0.0     };
    double jointAngle [numLinks] = { 0.0    ,  0.0     };
    int    jointType  [numLinks] = { JOINTREV,JOINTREV };

    /* 2. Eigen vectors that need the “<<” syntax -------------------------- */
    Eigen::Vector3d magnetLocal[3];
    magnetLocal[0] <<  0.0       , 0.0, 0.0;
    magnetLocal[1] << 35.859e-3  , 0.0, 0.0;
    magnetLocal[2] << -16.088e-3 , 0.0, 0.0;

    Eigen::Vector3d magnetPosLocal[3];
    magnetPosLocal[0] << -3.20e-3, 0.0    , 0.0;
    magnetPosLocal[1] << -3.72e-3, 0.0    , 0.0;
    magnetPosLocal[2] << -4.01e-3, 0.94e-3, 0.0;

    /* 3. T-matrix --------------------------------------------------------- */
    Eigen::Matrix4d T;
    T << 0, -1, 0, 0,
         1,  0, 0, 0,
         0,  0, 1, 0,
         0,  0, 0, 1;

    /* ---- construct the robot ------------------------------------------- */
    MagSerialRobot bot(numLinks,
                       linkLength,
                       linkTwist,
                       linkOffset,
                       jointAngle,
                       jointType,
                       magnetLocal,
                       magnetPosLocal);

    /* ---- extra run-time configuration ---------------------------------- */
    bot.m_set_q(Eigen::Vector2d{0.0, 0.0});
    bot.m_change_DH_params(linkLength, linkTwist,
                           linkOffset, jointAngle, jointType);
    bot.m_change_magnets(magnetLocal, magnetPosLocal);
    bot.m_set_Tbase(T);

    return bot;            // NRVO/move → no copy needed
}



/* ---- accessor: first call builds the single instance, thread-safe ------ */
/*
MagSerialRobot& magbot()
{
    static MagSerialRobot instance = makeMagbot();
    return instance;
}*/

//------------------------------ Functions --------------------------------
Eigen::Matrix4d get_EE_TMat(franka::Robot& robot){
    std::array<double, 9> rot = getEndEffectorRotMtx_COLMAJOR(robot);
    std::array<double, 3> pos = getEndEffectorXYZ(robot);

    // Note function for rot gives transposed rotation matrix! Transpose back here.
    Eigen::Matrix4d T_EE;
    T_EE << rot[0], rot[3], rot[6], pos[0],
            rot[1], rot[4], rot[7], pos[1],
            rot[2], rot[5], rot[8], pos[2],
            0,       0,     0,      1;

    return T_EE;
}

json toJsonArray(const Eigen::Matrix4d& T)
{
    json j = json::array();
    for (int r = 0; r < 4; ++r)
    {
        json row = json::array();
        for (int c = 0; c < 4; ++c)
            row.push_back(T(r, c));   // access element (row, col)
        j.push_back(std::move(row));
    }
    return j;
}

void get_and_print_EE_TMat(franka::Robot& robot){

    Eigen::Matrix4d T_EE = get_EE_TMat(robot);

    cout << "Current Franka Transform:" << endl << T_EE << endl;
}


void move_camera_rel_gripper(franka::Robot& robot, double x, double y, double z){
    /* This function moves the camera relative to the grippers base link origin
    Args:
        * robot: franka robot object
        * x: desired x position relative to gripper origin, expressed in global franka csys
        * y: desired y position relative to gripper origin, expressed in global franka csys
        * z: desired z position relative to gripper origin, expressed in global franka csys
    */


    // TODO: This might be wrong since T_EE2Cam is expressed in EE or cam coordinate frame,
    // not global Franka coordinate frame. Either Convert T_EE2Cam to global or use transform
    // mat multiplication altogether.
    Eigen::Matrix4d T_EE = get_EE_TMat(robot);
    Eigen::Matrix4d T_Cam2F = T_EE * T_Cam2EE;


    double dx = x + T_gripper(0, 3) - T_Cam2F(0, 3);
    double dy = y + T_gripper(1, 3) - T_Cam2F(1, 3);
    double dz = z + T_gripper(2, 3) - T_Cam2F(2, 3);

    EE_moveInX(robot, dx);
    EE_moveInY(robot, dy);
    EE_moveInZ(robot, dz);

}

void point_camera_to_gripper(franka::Robot& robot){
    /* Automatically rotated frame EE such that the camera points at the gripper's base.
       This is possible because we know the exact location of the gripper and the camera.
    */

    Matrix4d T_EE = get_EE_TMat(robot);
    Matrix4d T_cam2F = T_EE * T_Cam2EE;
    Matrix3d Rot_cam_d; // Desired camera orientation

    // Make z-axis point at gripper
    Vector3d gripper_pos = T_gripper.block<3,1>(0, 3);
    Vector3d z_axis = gripper_pos - T_cam2F.block<3,1>(0, 3);

    // Make y-axis point down first (remember, coordinates here given in Franka csys)
    Vector3d down(0, 0, -1);
    Vector3d x_axis = down.cross(z_axis).normalized();
    // Calc new, correct y_axis based on x and z
    Vector3d y_axis = x_axis.cross(z_axis).normalized();

    // Check that y-axis is actually pointing downwards
    double angle = y_axis.dot(down);
    if (angle > M_PI / 2){
        cout << "Flipped y-axis!" << endl;
        y_axis = -y_axis;
        x_axis = y_axis.cross(z_axis);
    }

    // Now, compute rotation delta and turn into euler angles
    Rot_cam_d.col(0) = x_axis;
    Rot_cam_d.col(1) = y_axis;
    Rot_cam_d.col(2) = z_axis;
    Matrix3d delta_rot = Rot_cam_d * T_cam2F.block(0, 0, 3, 3);
    Vector3d euler = delta_rot.eulerAngles(2, 1, 0);
    // Send to Franka
    //EE_rotateAboutX(robot, euler[0], 4);
    EE_rotateAboutY(robot, euler[1], 4);
    //EE_rotateAboutZ(robot, euler[2], 4);
}


Eigen::Matrix4d get_gripper2cam_transform(franka::Robot& robot){
    /* Returns the transformation matrix from the gripper base csys to camera csys.
       The resulting matrix (position, rotations) is given in the camera's coordinate frame.
    */

    Eigen::Matrix4d gripper2cam;
    Eigen::Matrix4d T_EE = get_EE_TMat(robot);

    gripper2cam = T_Cam2EE.inverse() * T_EE.inverse() * T_gripper;
    return gripper2cam;
}

Eigen::Matrix<double, 8, 1> run_open_loop(MagSerialRobot& magbot, int theta1, int theta2, double K1, double K2){
    // --------------------- Open loop Magbot control -------------------
    // static keyword means that magbot instance is only created once, and then re-used after, even during later function calls.
    static Eigen::Vector2d qd;
    static Eigen::Vector2d k_joints;
    //static Eigen::Vector2d offset {0.0, -1e-04};
    //float k_joints = 0.0009; //0.0003;

    static Eigen::Matrix<double,2,8> mMu;
    //Eigen::Matrix<double, Eigen::Dynamic, 1> tauK;
    static Eigen::Vector2d tauK;
    static Eigen::Vector2d tauInt;
    static Eigen::Matrix<double, 8, 1> coilCurrents;

    qd << (M_PI*theta1)/180, (M_PI*theta2)/180; // Also convert to rad

    //cout << "Desired Angles (deg):" << (qd(0)/M_PI)*180 << "," << (qd(1)/M_PI)*180 << endl;


    // Determine the internal generalized forces for the desired position
    magbot.m_set_q(qd);

    // Determine the actuation matrix for the gripper in its present state
    mMu = magbot.m_calc_actuation_matrix() * mCoilMatrix;

    tauInt = magbot.m_calc_internal_gen_forces();
    k_joints << K1 / 1000.0, K2 / 1000.0;
    tauK = k_joints.cwiseProduct(qd) + tauInt;
    cout << "TauK1: " << tauK(0) << ", TauK2: " << tauK(1) << endl;
    // Calculate the required coil currents to produce the desired generalized forces
    coilCurrents = mMu.completeOrthogonalDecomposition().solve(tauK);

    // Check that coil currents do not exceed allowed amount
    if(coilCurrents.lpNorm<Eigen::Infinity>() > 24.0)
    {
        std::cout << "Requested currents exceed maximum allowable 24 A. "
                  << "Reducing currents automatically..." << std::endl;
        coilCurrents = coilCurrents / coilCurrents.lpNorm<Eigen::Infinity>() * 24.0;
    }

    return coilCurrents;
}

void record_data(franka::Robot& robot, nlohmann::json& j, double coil_currents[8], int& group_idx, int& sample_cntr, cv::VideoCapture capCam, double time){

    // Create savename and add
    static std::string imgstr = "image";
    imgstr.append(std::to_string(sample_cntr));
    imgstr.append(".png");
    j["data"][group_idx]["images"].push_back(imgstr);

    // Save current image
    static cv::Mat mat;
    capCam.read(mat);

    std::string path = gripper_data_path;
    path.append("\\group").append(std::to_string(group_idx)).append("\\").append(imgstr);
    cv::imwrite(path, mat);

    // Append current end effector position
    static Eigen::Matrix4d T_EE = get_EE_TMat(robot);
    j["data"][group_idx]["Franka Transforms"].push_back(toJsonArray(T_EE));

    // Append coil currents
    json coilarray = nlohmann::json::array();
    for(int i = 0; i < 8; i++){
        coilarray.push_back(coil_currents[i]);
    }
    j["data"][group_idx]["coil currents"].push_back(coilarray);
    j["time"].push_back(time);

    // Reset imgstr
    imgstr = "image";
}

