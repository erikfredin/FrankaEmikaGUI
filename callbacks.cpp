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

    //read franka robot pose
    franka::Robot robot(fci_ip);
    franka::RobotState initial_state = robot.readOnce();
    // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    Robot_pos[0] = position_d[0];
    Robot_pos[1] = position_d[1];
    Robot_pos[2] = position_d[2];

//    Robot_orient[0] = orientation_d[0];
//    Robot_orient[1] = orientation_d[1];
//    Robot_orient[2] = orientation_d[2];

    std::cout<<"robot EE posisiton is: " << position_d <<std::endl;
    std::cout<<"robot EE quaternion orientation is: " << orientation_d <<std::endl;




}
