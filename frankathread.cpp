
#include "frankathread.h"
#include <Qtcore>
#include <QDebug>
#include <franka_funcs.h>


FrankaThread::FrankaThread()
{
    isStop = false;
    std::cout<<"Franka robot thread created!"<<std::endl;
}

FrankaThread::~FrankaThread()
{
}



//void FrankaThread::run(std::string fci_ip, bool isRobotconnect)

void FrankaThread::run() {
    // Provide a default robot IP or read from command line
    std::cout << "Enterering Franka Thread......" << std::endl;
     //std::string robot_ip = "192.168.100.1";  // Replace with your robot's IP address
     //if (argc > 1) {
     //  robot_ip = argv[1];
     //}

     try {
       // Connect to robot
       franka::Robot robot(fci_ip);

       // Use default collisions / control behavior (example; adapt to your needs!)
       setDefaultBehavior(robot);

       // Example: Move end-effector +10 cm in X over 5 seconds
       std::cout << "Moving +0.1 m in X..." << std::endl;
       bool success = EE_moveInX(robot, 0.1, 5.0);
       if (!success) {
         std::cerr << "Motion in X failed!" << std::endl;
       }

       // Example: Move end-effector -5 cm in Y over 3 seconds
       std::cout << "Moving -0.05 m in Y..." << std::endl;
       success = EE_moveInY(robot, -0.05, 3.0);
       if (!success) {
         std::cerr << "Motion in Y failed!" << std::endl;
       }

       // Example: Rotate about Z by 45 degrees (pi/4 radians) over 4 seconds
       std::cout << "Rotating 45 deg about Z..." << std::endl;
       success = EE_rotateAboutZ(robot, M_PI / 4, 4.0);
       if (!success) {
         std::cerr << "Rotation about Z failed!" << std::endl;
       }

       std::cout << "Done with motions." << std::endl;

     } catch (const franka::Exception& e) {
       std::cerr << "Franka exception: " << e.what() << std::endl;
     }
}


