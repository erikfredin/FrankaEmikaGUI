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
//    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d rotation(initial_transform.linear());

    Eigen::Vector3d eulerangle = rotation.eulerAngles(0, 1, 2);


    Robot_pos[0] = position_d[0];
    Robot_pos[1] = position_d[1];
    Robot_pos[2] = position_d[2];

    Robot_orient[0] = eulerangle[0];
    Robot_orient[1] = eulerangle[1];
    Robot_orient[2] = eulerangle[2];

    std::cout<<"robot EE posisiton is: " << position_d <<std::endl;
    std::cout<<"robot EE quaternion orientation is: " << eulerangle <<std::endl;




}
