#include "callbacks.h"

//void callback(void)
//{
//    qDebug() << "Callback executed.";

//}

void MainWindow::callbacks(void)
{
    // Execute code here
//    qInfo() << connectedGamepad.joystickValues[0];
//    qDebug() << "Code callback executing...";

    //We connect Franka every turns of callback , could be improved...
    franka::Robot robot(fci_ip);
    //read franka robot pose
    franka::RobotState initial_state = robot.readOnce();
    // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    position_d = position_d*1000;//from m to mm
//    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d rotation(initial_transform.linear());
    Eigen::Vector3d eulerangle = rotation.eulerAngles(0, 1, 2);

    Eigen::Affine3d EEinFlange(Eigen::Matrix4d::Map(initial_state.F_T_EE.data()));
    Eigen::Vector3d EEinFpos(EEinFlange.translation());
    EEinFpos = EEinFpos*1000; //from m to mm
//    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d EEinFrot(EEinFlange.linear());
    Eigen::Vector3d EEinFeulerangle = EEinFrot.eulerAngles(0, 1, 2);


    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());


    Robot_pos[0] = position_d[0];
    Robot_pos[1] = position_d[1];
    Robot_pos[2] = position_d[2];

    Robot_orient[0] = eulerangle[0];
    Robot_orient[1] = eulerangle[1];
    Robot_orient[2] = eulerangle[2];

//    std::cout<<"robot EE posisiton is: " << position_d <<std::endl;
//    std::cout<<"robot EE orientation is: " << eulerangle <<std::endl;

//    std::cout<<"robot EEinF posisiton is: " << EEinFpos <<std::endl;
//    std::cout<<"robot EEinF orientation is: " << EEinFeulerangle <<std::endl;

//    std::cout<<"robot measured tau: "<<initial_tau_measured<<std::endl;




}
