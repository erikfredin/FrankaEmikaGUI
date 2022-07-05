#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // TIMERS
    // Define the timer for updating the graphics and readouts on the GUI
    QTimer *captiontimer = new QTimer(this);
    connect(captiontimer, SIGNAL(timeout()), this, SLOT(updateCaption()));	// show fps,... in timer
    captiontimer->start(captionRefreshPeriod); //Default 200. Set to 20 ms for a faster 50Hz refresh rate
    // Define the timer for executing runtime code know as the callbacks.
    QTimer *callbacktimer = new QTimer(this);
    connect(callbacktimer, SIGNAL(timeout()), this, SLOT(callbacks()));
    callbacktimer->start(callbackRefreshPeriod); //Default 200. Set to 20 ms for a faster 50Hz refresh rate
//    QElapsedTimer currentTime;
    // Start the timer to keep track of the current elapsed time/
    currentTime.start();
    lastTime = currentTime.elapsed();

    // PUSH BUTTONS
    connect(ui->pushButton_experimental_control,SIGNAL(clicked()),SLOT(experimental_control()));
    connect(ui->pushButton_freedrag,SIGNAL(clicked()),SLOT(freedrag()));
    connect(ui->pushButton_translationaldrag,SIGNAL(clicked()),SLOT(translationaldrag()));
    connect(ui->pushButton_EEoffsetupdate,SIGNAL(clicked()),SLOT(updateRobotEE()));
    connect(ui->pushButton_robotconnect,SIGNAL(clicked()),SLOT(Robotconnect()));
    connect(ui->pushButton_pilotthreadon,SIGNAL(clicked()),SLOT(pilotthreadon()));
    connect(ui->pushButton_pilotthreadoff,SIGNAL(clicked()),SLOT(pilotthreadoff()));
    connect(ui->pushButton_registercollect,SIGNAL(clicked()),SLOT(registerdatacollect()));
    connect(ui->pushButton_calibrationcollect,SIGNAL(clicked()),SLOT(calibratesetflag()));






//    connect(ui->pushButton_setfilename,SIGNAL(clicked()),SLOT(SetFileName(ui->lineEdit_EE_x->text())));

    connect(ui->pushButton_setfilename,SIGNAL(clicked()),SLOT(Setfilename()));
    connect(ui->pushButton_logon,SIGNAL(clicked()),SLOT(SetLogEnabled()));
    connect(ui->pushButton_logoff,SIGNAL(clicked()),SLOT(CloseFiles()));
    connect(ui->pushButton_logpause,SIGNAL(clicked()),SLOT(DisableLog()));
    connect(ui->pushButton_logcontinue,SIGNAL(clicked()),SLOT(EnableLog()));

    //check box
//
    connect(ui->checkBox_pilot,SIGNAL(clicked()),SLOT(frankathreadcontrol()));
    connect(ui->checkBox_streaming,SIGNAL(clicked()),SLOT(robotstreaming()));



//    connect(ui->lineEdit_EE_x,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );
//    connect(ui->lineEdit_EE_y,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );
//    connect(ui->lineEdit_EE_z,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );

    //we assign transformation matrix here
    transT2R<<  -33.8486, 434.531, 5191.75, -252.754,
                 426.209, 19.3244, 3612.1, -427.56,
                 -57.2238, 53.3836, 571.997, 20.8684,
                 -0.229218, 0.416058, -11.5944, 1.84834;

    std::cout<< "transformation from table to Franka is " << std::endl <<transT2R <<  std::endl;
}



MainWindow::~MainWindow()
{
    delete frankathread;
    std::cout<<"delete Franka thread---"<<std::endl;
    delete ui;
}


void MainWindow::updateCaption(void)
{
    // Update labels on the GUI to reflect parameters saved in code

    // for testing connection
    ui->label_robot_px->setText(tr("%1").arg(Robot_tip_posisition[0]));
    ui->label_robot_py->setText(tr("%1").arg(Robot_tip_posisition[1]));
    ui->label_robot_pz->setText(tr("%1").arg(Robot_tip_posisition[2]));
    ui->label_robot_ea1->setText(tr("%1").arg(Robot_orient[0]));
    ui->label_robot_ea2->setText(tr("%1").arg(Robot_orient[1]));
    ui->label_robot_ea3->setText(tr("%1").arg(Robot_orient[2]));

    ui->label_q1->setText(tr("%1").arg(Robot_joint[0]));
    ui->label_q2->setText(tr("%1").arg(Robot_joint[1]));
    ui->label_q3->setText(tr("%1").arg(Robot_joint[2]));
    ui->label_q4->setText(tr("%1").arg(Robot_joint[3]));
    ui->label_q5->setText(tr("%1").arg(Robot_joint[4]));
    ui->label_q6->setText(tr("%1").arg(Robot_joint[5]));
    ui->label_q7->setText(tr("%1").arg(Robot_joint[6]));

}


void MainWindow::updateRobotEE(void)
{

    RobotEE_offset[0] = ui->lineEdit_EE_x->text().toDouble();
    RobotEE_offset[1] = ui->lineEdit_EE_y->text().toDouble();
    RobotEE_offset[2] = ui->lineEdit_EE_z->text().toDouble();
    double RobotEE_offset_meter[3];
    RobotEE_offset_meter[0] = RobotEE_offset[0]*0.001;
    RobotEE_offset_meter[1] = RobotEE_offset[1]*0.001;
    RobotEE_offset_meter[2] = RobotEE_offset[2]*0.001;
    std::array<double, 16> EEtoFlange = {1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, RobotEE_offset_meter[0], RobotEE_offset_meter[1],RobotEE_offset_meter[2],1.0};
//    franka::Robot::setEE(EEtoFlange);
    franka::Robot robot(fci_ip);
    robot.setEE(EEtoFlange);
    std::cout<< "EE_offset is updated to: " << RobotEE_offset_meter[0] <<" "<< RobotEE_offset_meter[1]<<" "<< RobotEE_offset_meter[2] << std::endl;
}


void MainWindow::Robotconnect(void)
{
    std::string pingtest = "ping -c1 -s1 " + fci_ip + "> /dev/null 2>&1";
//    int x = system("ping -c1 -s1 8.8.8.8  > /dev/null 2>&1");
    int x = system(pingtest.c_str());
    if (x==0){
        std::cout<<"Robot connection succecced"<<std::endl;
//        isRobotConnected = true;

    }else{
        std::cout<<"Robot connection failed"<<std::endl;
//        isRobotConnected = false;
    }
    //    try {

////        franka::Robot robot(fci_ip);
//        isRobotConnected = true;

//    }  catch (int) {
//        std::cerr<<"Franka is not connected, check IP settings"<<std::endl;
//        return -1;
//    }
}

void MainWindow::translationaldrag()
{

    qDebug()<<"Translational drag.....";
    franka::Robot robot(fci_ip);
    //Impendence control

      // Compliance parameters
      const double translational_stiffness{150.0};
      const double rotational_stiffness{10.0};
      Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
      stiffness.setZero();
      stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      damping.setZero();
      damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
      damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                             Eigen::MatrixXd::Identity(3, 3);

      try {
        // connect to robot
    //    franka::Robot robot(argv[1]);

          setDefaultBehavior(robot);
          // First move the robot to a suitable joint configuration
          std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
          MotionGenerator motion_generator(0.5, q_goal);
          std::cout << "WARNING: This example will move the robot! "
                    << "Please make sure to have the user stop button at hand!" << std::endl
                    << "Press Enter to continue..." << std::endl;
      //    std::cin.ignore();
          robot.control(motion_generator);
          std::cout << "Finished moving to initial joint configuration." << std::endl;


        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();

        franka::RobotState initial_state = robot.readOnce();

        // equilibrium point is the initial position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.linear());

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            impedance_control_callback = [&](const franka::RobotState& robot_state,
                                             franka::Duration /*duration*/) -> franka::Torques {
          // get state variables
          std::array<double, 7> coriolis_array = model.coriolis(robot_state);
          std::array<double, 42> jacobian_array =
              model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

          // convert to Eigen
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
          Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
          Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position(transform.translation());
          Eigen::Quaterniond orientation(transform.linear());

          // compute error to desired equilibrium pose
          // position error
          Eigen::Matrix<double, 6, 1> error;
          error.head(3) << position - position_d;

          // orientation error
          // "difference" quaternion
          if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
          }
          // "difference" quaternion
          Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
          error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
          // Transform to base frame
          error.tail(3) << -transform.linear() * error.tail(3);

          // compute control
          Eigen::VectorXd tau_task(7), tau_d(7);

          // Spring damper system with damping ratio=1
//          tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
          tau_task << jacobian.transpose() * ( - damping * (jacobian * dq));
          tau_d << tau_task + coriolis;

          std::array<double, 7> tau_d_array{};
          Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
          return tau_d_array;
        };

        // start real-time control loop
        std::cout << "WARNING: Collision thresholds are set to high values. "
                  << "Make sure you have the user stop at hand!" << std::endl
                  << "After starting try to push the robot and see how it reacts." << std::endl;
        robot.control(impedance_control_callback);

      } catch (const franka::Exception& e) {
          std::cout << e.what() << std::endl;
          std::cout << "Running error recovery..." << std::endl;
          robot.automaticErrorRecovery();
      //    return -1;
        }

}


void MainWindow::robotfreedraginthread()
{
    if(isRobotConnected)
    {

        qDebug()<<"free drag.....";
        franka::Robot robot(fci_ip);
        //Impendence control

          // Compliance parameters
          const double translational_stiffness{150.0};
          const double rotational_stiffness{10.0};
          Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
          stiffness.setZero();
          stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
          stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
          damping.setZero();
          damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                             Eigen::MatrixXd::Identity(3, 3);
          damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                                 Eigen::MatrixXd::Identity(3, 3);

        try
        {

            setDefaultBehavior(robot);
            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Vector3d position_d(initial_transform.translation());
            Eigen::Quaterniond orientation_d(initial_transform.linear());

            // set collision behavior
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            // define callback for the torque control loop
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback = [&](const franka::RobotState& robot_state,
                                                 franka::Duration /*duration*/) -> franka::Torques {
              // get state variables
              std::array<double, 7> coriolis_array = model.coriolis(robot_state);
              std::array<double, 42> jacobian_array =
                  model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

              // convert to Eigen
              Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
              Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
              Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
              Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
              Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
              Eigen::Vector3d position(transform.translation());
              Eigen::Quaterniond orientation(transform.linear());

              // compute error to desired equilibrium pose
              // position error
              Eigen::Matrix<double, 6, 1> error;
              error.head(3) << position - position_d;

              // orientation error
              // "difference" quaternion
              if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
              }
              // "difference" quaternion
              Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
              error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
              // Transform to base frame
              error.tail(3) << -transform.linear() * error.tail(3);

              // compute control
              Eigen::VectorXd tau_task(7), tau_d(7);

              // Spring damper system with damping ratio=1
    //          tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
              tau_task << jacobian.transpose() * ( - damping * (jacobian * dq));
              tau_d << tau_task + coriolis;

              std::array<double, 7> tau_d_array{};
              Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
              return tau_d_array;
            };

            // start real-time control loop
            std::cout << "WARNING: Collision thresholds are set to high values. "
                      << "Make sure you have the user stop at hand!" << std::endl
                      << "After starting try to push the robot and see how it reacts." << std::endl;
            robot.control(impedance_control_callback);

        } catch (const franka::Exception& e) {
          std::cout << e.what() << std::endl;
          std::cout << "Running error recovery..." << std::endl;
          robot.automaticErrorRecovery();
      //    return -1;
        }
    }
    else
    {
        qInfo()<<"robot is not connected, run Robotconnect first...";
    }

}



void MainWindow::freedrag()
{
    if(isRobotConnected)
    {

        qDebug()<<"free drag.....";
        franka::Robot robot(fci_ip);
        //Impendence control

          // Compliance parameters
          const double translational_stiffness{150.0};
          const double rotational_stiffness{10.0};
          Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
          stiffness.setZero();
          stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
          stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
          damping.setZero();
          damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                             Eigen::MatrixXd::Identity(3, 3);
          damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                                 Eigen::MatrixXd::Identity(3, 3);

        try
        {

            setDefaultBehavior(robot);
            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Vector3d position_d(initial_transform.translation());
            Eigen::Quaterniond orientation_d(initial_transform.linear());

            // set collision behavior
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            // define callback for the torque control loop
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback = [&](const franka::RobotState& robot_state,
                                                 franka::Duration /*duration*/) -> franka::Torques {
              // get state variables
              std::array<double, 7> coriolis_array = model.coriolis(robot_state);
              std::array<double, 42> jacobian_array =
                  model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

              // convert to Eigen
              Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
              Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
              Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
              Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
              Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
              Eigen::Vector3d position(transform.translation());
              Eigen::Quaterniond orientation(transform.linear());

              // compute error to desired equilibrium pose
              // position error
              Eigen::Matrix<double, 6, 1> error;
              error.head(3) << position - position_d;

              // orientation error
              // "difference" quaternion
              if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
              }
              // "difference" quaternion
              Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
              error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
              // Transform to base frame
              error.tail(3) << -transform.linear() * error.tail(3);

              // compute control
              Eigen::VectorXd tau_task(7), tau_d(7);

              // Spring damper system with damping ratio=1
    //          tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
              tau_task << jacobian.transpose() * ( - damping * (jacobian * dq));
              tau_d << tau_task + coriolis;

              std::array<double, 7> tau_d_array{};
              Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
              return tau_d_array;
            };

            // start real-time control loop
            std::cout << "WARNING: Collision thresholds are set to high values. "
                      << "Make sure you have the user stop at hand!" << std::endl
                      << "After starting try to push the robot and see how it reacts." << std::endl;
            robot.control(impedance_control_callback);

        } catch (const franka::Exception& e) {
          std::cout << e.what() << std::endl;
          std::cout << "Running error recovery..." << std::endl;
          robot.automaticErrorRecovery();
      //    return -1;
        }
    }
    else
    {
        qInfo()<<"robot is not connected, run Robotconnect first...";
    }


}


void MainWindow::experimental_control()
{


  if(isRobotConnected)
  {
    franka::Robot robot(fci_ip);
    try {

        setDefaultBehavior(robot);
        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl;
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set the joint impedance.
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        // Set the collision behavior.
        std::array<double, 7> lower_torque_thresholds_nominal{
            {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
        std::array<double, 7> upper_torque_thresholds_nominal{
            {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
        std::array<double, 7> lower_torque_thresholds_acceleration{
            {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
        std::array<double, 7> upper_torque_thresholds_acceleration{
            {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
        std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
        std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
        std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
        std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
        robot.setCollisionBehavior(
            lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
            lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
            lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
            lower_force_thresholds_nominal, upper_force_thresholds_nominal);
        double time_max = 4.0;
        double v_max = 0.1;
        double angle = M_PI / 4.0;
        double time = 0.0;
        robot.control([=, &time](const franka::RobotState&,
                                 franka::Duration period) -> franka::CartesianVelocities {
          time += period.toSec();
          double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
          double v_x = std::cos(angle) * v;
          double v_z = -std::sin(angle) * v;
          franka::CartesianVelocities output = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
          if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
          }
          return output;
        });
      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
//        return -1;
      }
    }
      else {
          qInfo()<<"robot is not connected, run Robotconnect first......";
      }

}

void MainWindow::robotstreaming(void)
{
    if(ui->checkBox_streaming->checkState())
    {
        isRobotreading = true;
        qInfo()<<"robot is streaming";
    }
    else
    {
        isRobotreading = false;

        qInfo()<<"robot stops streaming";
    }

}




void MainWindow::frankathreadcontrol(void)
{
    if(ui->checkBox_pilot->checkState())
    {
//        frankathread.run(fci_ip, isRobotConnected);
        frankathread->start();
        std::cout<<"thread run"<<std::endl;
    }
    else
    {
        frankathread->isStop = true;
//        std::cout<<"isStop = " <<frankathread->isStop<<std::endl;
//        qInfo()<<"isStop = "<<frankathread->isStop;
//        frankathread->quit();
//        frankathread->exit();
        frankathread->terminate();
        std::cout<<"thread exit"<<std::endl;
    }

}

//void MainWindow::frankathreadquit(void)
//{
//    frankathread.quit();
//    frankathread.exit(0);
//}



void    MainWindow::pilotthreadon(void)
{
//    std::thread t1(robotfreedraginthread);
    qInfo()<<"not using anymore...";

}


void    MainWindow::pilotthreadoff(void)
{
    qInfo()<<"not using anymore...";
}

void  MainWindow::SetLogEnabled(){

    //if we not already logging then we should reset the latest and start.
    LogEnabled = true;
    std::cout<<"...........Recording in progress......"<<std::endl;
}

void  MainWindow::EnableLog(){
    LogEnabled = true;
    std::cout<<"...........Recording continueing ......"<<std::endl;
}

void  MainWindow::DisableLog(){
    LogEnabled=false;
    std::cout<<"...........Recording paused......"<<std::endl;
}



void MainWindow::Record(void)
{
    //default
    if (!LogFileAllData.is_open())
    {
        std::cerr<<" Log file is not open, creating a default file"<<std::endl;
        std::string Defaultbase = "Default";
        if (!OpenFiles(Defaultbase))
            std::cerr<<" Log file failed to open"<<std::endl;
    }

        LogFileAllData<<std::setprecision(4)<<currentTime.elapsed()/1000.0
                     <<Delim<<std::setprecision(0)<<NumWritten<<Delim;


        LogFileAllData<<std::setprecision(7);

        //covert probe reading to filed at coil system frame
        // Bx = ProbeBy
        // By = -ProbeBx
        // Bz = ProbeBz
        msdFieldvalue[0] = DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1];
        msdFieldvalue[1] = -DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0];
        msdFieldvalue[2] = DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2];

       int i;

       for (i = 0; i < numProberead; i++)
           LogFileAllData<<msdFieldvalue[i]<<Delim;

       for (i = 0; i < numProbePos; i++)
//               LogFileAllData<<ProbePos[i]<<Delim;
           LogFileAllData<<Robot_tip_posisition[i]<<Delim;



       for (i = 0;i < numAct; i++)
           LogFileAllData<<cmdCoilCurrent[i]<<Delim;

       for (i = 0;i < numAct; i++)
           LogFileAllData<<measuredCurrents[i]<<Delim;

       for (i = 0;i < numProberead; i++)
           LogFileAllData<<DAQ.analogRawInputVoltages[i]<<Delim;

       LogFileAllData<<0;

        LogFileAllData<<std::endl;
        NumWritten++;

        //LogRawData.setf(ios::fixed); //Print floating point numbers using fixed point notation.
        ////ios::showpoint  Print a decimal point for all floating point numbers, even when
        ////it's not needed (e.g. the number is exactly an integer).
        ////The precision of numbers can be changed as follows. You can also set the width,
        ////i.e. the minimum number of spaces used to print the next.
        ////These featuers are used, for example, to make output items line up in columns
        ////when printed. Both of these features require that you include the iomanip header file.
        //LogRawData.precision(0);  // print 0 digits after decimal point


}

bool MainWindow::OpenFiles(std::string &fileNameBase){
    //Robot.GetRobotControlModeQR(LastStateIndex,          ControlMode);
    std::cout << "Opening new LogFile in Control Mode: " <<std::endl;
//    CloseFiles();

    LogFileAllData.clear();

    //LogFileReadme.clear();

    std::string dateTime;
//    osaGetDateTimeString(dateTime);

    // Find time to create a unique filename for saving
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H-%M-%S",timeinfo);
    std::string temp(buffer);
//    dateTime = QString::fromStdString(temp);
    dateTime = temp;
    std::string fileName;

    FullFileName = std::string(dateTime+std::string("-")+fileNameBase+std::string("-RobotLog"));
    fileName = FilePath + dateTime+std::string("-")+fileNameBase+std::string("-RobotLog")+std::string(".csv");
    LogFileAllData.open(fileName.c_str(), std::ios::out | std::ios::app);

    //TODO: add a file that describes all the files that are written
    //		should add the format of each file in there.
    //fileName=dirName+std::string("Log6Readme")+std::string(".txt");
    //LogFileReadme.open(fileName.c_str(), std::ios::out | std::ios::app);

    if (LogFileAllData.fail() ){
        std::cerr << "Can't Open files: " << fileName << std::endl;
        return false;
    }
    else {
        //add header
        LogFileAllData.precision(4);
        LogFileAllData.setf(std::ios::fixed);

            //LogFileForces<<"Ticks"<<Delim<<"TimeStamp(origin:"<<timeOriginInSec<<")"<<Delim<<"Valid"<<Delim;

            LogFileAllData<<"TimeStamp(s)"<<Delim<<"Ticks"<<Delim;


            int i;
            for (i = 0; i < numProberead; i++)
                LogFileAllData<<"msdFieldvalue_mT_"<<i+1<<Delim;

            for (i = 0; i < numProbePos; i++)
                LogFileAllData<<"ProbePos_mm_"<<i+1<<Delim;


            for (i = 0;i < numAct; i++)
                LogFileAllData<<"cmdCoilCurrent_A_"<<i+1<<Delim;

            for (i = 0;i < numAct; i++)
                LogFileAllData<<"mrdCoilCurrent_A_"<<i+1<<Delim;

            for (i = 0;i < numProberead; i++)
                LogFileAllData<<"Daqraw_v"<<i+1<<Delim;

            LogFileAllData<<"Test";

            LogFileAllData<<std::endl;

        std::cout << "Log files opened at " << fileName <<std::endl;
    }
    return true;
}

void MainWindow::SetControlMode(const int & controlMode)
{
    ControlMode = controlMode;
}

//void MainWindow::SetFileNamenew(std::string & fileNameBase)
//{
//    FileNameBase = fileNameBase;
//    LogEnabled = false;
//    CloseFiles();
//    OpenFiles(FileNameBase);
//}


void MainWindow::Setfilename(void)
{
    FileNameBase = ui->lineEdit_filename->text().toStdString();
    std::cout<<"filename set as: "<<FileNameBase<<std::endl;
    LogEnabled = false;
    CloseFiles();
    OpenFiles(FileNameBase);
}

void MainWindow::CloseFiles(void){
    LogFileAllData.close();
    LogEnabled = false;
    NumWritten=0;
    std::cout<<"Log file closed and was saved as: "<<FullFileName<<std::endl;
}



void MainWindow::updateCurrents_CalibrationOnly(double I_command[8])
{
    // This is the only function that should tell the S826 to send currents to the coils
    // Reads from Variables:
    //      B_Global_Desired : The desire field at the time
    //      isGradientControlled : Boolean to determine if inverse or pseudoInverse is necessary
    // Sets the Variables:
    //      currentSetpoints : The target current in the coils

    // NOTE: This code finds the inverse of the control matrices every single time it is called.
    //      This isn't necessary if the tool is stationary in the workspace, since the control
    //      matrix, as well as its inverse, will be unchanged.

    // Calling updateCurrents will:
    // 1. Check for overheating
    // 2. Determine the currents needed to generate the desired field
    // 3. Calculate the theoretical magnetic field produced by said currents
    // 4. Send analog output command to change the coil currents if the system is not overheating

    if (!overheatingFlag)
    {
        // Send current setpoints as normal
        // Need to convert to voltages first

        // TODO convert amps to volts
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = I_command[i] * currentControlAdj[i]; // Voltage = (Amps) * (Volts/Amp)
            // TODO limit voltages sent to S826
        }
    }
    else
    {
        // Only send 0s to the currents to turn them off because the system is overheating!
//        clearCurrentsSlot();
        // calling clear currents make an inf loop
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
        }
    }
    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
        qInfo() << "Wrote values to the S826 in updateCurrents_CalibrationOnly submode";
    }
}

void MainWindow::updateCurrents(void)
{
    // This is the only function that should tell the S826 to send currents to the coils
    // Reads from Variables:
    //      B_Global_Desired : The desire field at the time
    //      isGradientControlled : Boolean to determine if inverse or pseudoInverse is necessary
    // Sets the Variables:
    //      currentSetpoints : The target current in the coils

    // NOTE: This code finds the inverse of the control matrices every single time it is called.
    //      This isn't necessary if the tool is stationary in the workspace, since the control
    //      matrix, as well as its inverse, will be unchanged.

    // Calling updateCurrents will:
    // 1. Check for overheating
    // 2. Determine the currents needed to generate the desired field
    // 3. Calculate the theoretical magnetic field produced by said currents
    // 4. Send analog output command to change the coil currents if the system is not overheating

    if (!overheatingFlag)
    {

        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
        }
    }
    else
    {
        // Only send 0s to the currents to turn them off because the system is overheating!
//        clearCurrentsSlot();
        // calling clear currents make an inf loop
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
        }


    }

    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
        qInfo() << "Wrote values to the S826.";
    }


}


//if(InitRobotRegister==true)
//      {
//          //collect datapoint from master, number of total points is defined as NumRobotsRegisterData;
//          if (NumCollectPointRobotRigster < NumRobotsRegisterData)
//          {
//              if (PedalsValue[0] >20 && PedalsValue[1] < 20)
//              {
//                  RobtRegisterPedalcount++;
//                  if(RobtRegisterPedalcount > 300)
//                  {
//                      std::cout<<"Collect robots registration data:"<<NumCollectPointRobotRigster+1<<"/"<<NumRobotsRegisterData<<std::endl;
//                      CollectPointRobotsRegister[0][NumCollectPointRobotRigster] = tipx;
//                      CollectPointRobotsRegister[1][NumCollectPointRobotRigster] = tipy;
//                      CollectPointRobotsRegister[2][NumCollectPointRobotRigster] = tipz;
//                      CollectPointRobotsRegister[3][NumCollectPointRobotRigster] = 1;
//                      OnelapOverRobotRegister = true;
//                      Point_RobotRegsiter[0] = tipx; // for logging
//                      Point_RobotRegsiter[1] = tipy;
//                      Point_RobotRegsiter[2] = tipz;
//                   }
//                  else
//                      std::cout << "Please press left pedal[ " << RobtRegisterPedalcount+1 <<"/300]"<< std::endl;
//              }
//              if (PedalsValue[0] < 20 && OnelapOverRobotRegister == true)
//              {
//                  std::cerr<<"Finished "<<NumCollectPointRobotRigster+1<<"/"<<NumRobotsRegisterData<<" dataset collection!!!"<<std::endl;
//                  RobtRegisterPedalcount = 0; //clear RobotRegisterCount avoid accumulating all time
//                  NumCollectPointRobotRigster++;
//                  OnelapOverRobotRegister = false;
//              }
//          }
//          else  //got all the data from master, next step is to receive datapoint from slave, and calculte transformation T
//          {
////                 while(true) //wait until get all slave robot data
////                 {
//              if(FlagSlavedatareceive == 1)
//              {
//                  // data from slave is in vctDoubleVec, should be like [P1x, P1y, P1z, P2x, P2y, P2z, ...,],
//                  // so need convert to vctFixedSizeMatrix<double, 4, NumRobotsRegisterData>:
//                  // [P1x, P2x, P3x, ...
//                  //  P1y, P2y, P3y, ...
//                  //  P1z, P2z, P3z, ...
//                  //  1,    1,   1,  ...]
//                  unsigned long k = 0;
//                  for (unsigned long col=0; col<NumRobotsRegisterData; col++) {
//                      for (unsigned long row = 0; row < 3; row++) {
//                          SlaveRobotPointsMat[row][col] = SlaveRegistPointreceive[k];
//                          k++;
//                      }
//                      SlaveRobotPointsMat[3][col] = 1;
//                  }
//                  ControlAlg.ComputeTwoRobotsTransformation(CollectPointRobotsRegister, SlaveRobotPointsMat, TransformM2S);
//                  std::cerr<<"Finished robots registration:"<<TransformM2S<<std::endl;
//                  InitRobotRegister = false;
////                    break;
//      }

//void robotControlAlgorithm::ComputeTwoRobotsTransformation(const vctFixedSizeMatrix<double, 4, NumRobotsRegisterData> & MasterPoint,
//                                                  const vctFixedSizeMatrix<double, 4, NumRobotsRegisterData> & SlavePoint,
//                                                vctFrm4x4 & Tm2s) //Tm2s: transformation from master frame to slave frame
//              {   //Prolem is get T from T<4,4>*Masterpoint<4,n> = Slavepoint<4,n>,
//                  //it is xA=B, make transpose for both sides, we get A_transpose*x_transpose = B_transpose
//                  //denotes as Atxt = Bt
//                  //then xt = (At.tranpose*At).inverse*At.transpose*Bt
//                  //finally x = xt.transpose
//              // not sure should use vctDoulbeMat or vctFixedSizeMatrix.....
//              //    vctDoubleMat A, B, At, Bt, pinvAt;
//              //    A.SetSize(4, NumRobotsRegisterData);
//              //    B.SetSize(4, NumRobotsRegisterData);
//              //    At.SetSize(NumRobotsRegisterData, 4);
//              //    Bt.SetSize(NumRobotsRegisterData, 4);
//              //    pinvAt.SetSize(4, NumRobotsRegisterData);
//                  vctFixedSizeMatrix<double, 4, NumRobotsRegisterData> A, B;
//                  vctFixedSizeMatrix<double, NumRobotsRegisterData, 4> At, Bt;
//                  vct4x4 doubleAt, invdoubleAt;
//                  vctFixedSizeMatrix<double, 4, NumRobotsRegisterData> pinvAt;
//                  vct4x4 xt;
//                  A = MasterPoint;
//                  B = SlavePoint;
//                  At = A.Transpose();
//                  Bt = B.Transpose();
//                  doubleAt.ProductOf(A, At);
//                  invdoubleAt = doubleAt;
//                  // Compute inverse and check result
//                  nmrInverse(invdoubleAt);
//                  pinvAt.ProductOf(invdoubleAt,A);
//                  xt.ProductOf(pinvAt,Bt);
//                  Tm2s = xt.Transpose();

void MainWindow:: registerdatacollect(void)
{
    if(registerdataCount<registerdataNum)
    {
        registerdataCount++;
        for(int i=0; i<3; i++)
            robotdata[i][registerdataCount] = Robot_tip_posisition[i];
        robotdata[3][registerdataCount] = 1;
        std::cout<<"register data collected....["<<registerdataCount<<"/"<<registerdataNum<<"]...Robot tip: "<<Robot_tip_posisition[0]<<" "<<Robot_tip_posisition[1]<<" "<<Robot_tip_posisition[2]<<std::endl;
        if(registerdataCount==registerdataNum)
        {
            qInfo()<<"register data collect finished...calculating registration matrix...";
            registration(robotdata, tabledata);
        }

    }
    else
    {
        qInfo()<<"register is done...";
    }
}

void MainWindow::calibratesetflag(void)
{
    CalibrationDataCollet = true;
}


void MainWindow::registration(double tabledata[4][registerdataNum], double robotdata[4][registerdataNum] ) //output Tt2r, Probot = Tt2r*Ptable
{
//Prolem is get T from T<4,4>*Masterpoint<4,n> = Slavepoint<4,n>,
          //it is xA=B, make transpose for both sides, we get A_transpose*x_transpose = B_transpose
          //denotes as Atxt = Bt
          //then xt = (At.tranpose*At).inverse*At.transpose*Bt
          //finally x = xt.transpose
      // not sure should use vctDoulbeMat or vctFixedSizeMatrix.....
      //    vctDoubleMat A, B, At, Bt, pinvAt;
      //    A.SetSize(4, NumRobotsRegisterData);
      //    B.SetSize(4, NumRobotsRegisterData);
      //    At.SetSize(NumRobotsRegisterData, 4);
      //    Bt.SetSize(NumRobotsRegisterData, 4);
      //    pinvAt.SetSize(4, NumRobotsRegisterData);
          Eigen::MatrixXd A(4, registerdataNum);
          Eigen::MatrixXd B(4, registerdataNum);
          Eigen::MatrixXd At(registerdataNum, 4);
          Eigen::MatrixXd Bt(registerdataNum, 4);
          Eigen::Matrix4d doubleAt;
          Eigen::Matrix4d invdoubleAt;
          Eigen::MatrixXd pinvAt(4, registerdataNum);
          Eigen::Matrix4d xt;
            //manually convert double array to eigen matrix
          for(int i = 0; i<4; i++)
              for(int j=0; j<registerdataNum; j++)
                {
                  A(i,j) = tabledata[i][j];
                  B(i,j) = robotdata[i][j];
              }
          At = A.transpose();
          Bt = B.transpose();
          doubleAt = A*At;
          invdoubleAt = doubleAt.inverse();
          // Compute inverse and check result

          pinvAt = invdoubleAt*A;
          xt = pinvAt*Bt;
          transT2R = xt.transpose();
//          std::string sep = "\n----------------------------------------\n";
        std::cout<<"Register matrix is: "<<std::endl;
        for (int k=0; k<4; k++) {
            for (int l=0; l<4; l++) {
                std::cout<< transT2R(k,l) <<", ";
            }
            std::cout<<"\n"<<std::endl;

        }


//          vctFixedSizeMatrix<double, 4, registerdataNum> A, B;
//          vctFixedSizeMatrix<double, registerdataNum, 4> At, Bt;
//          vct4x4 doubleAt, invdoubleAt;
//          vctFixedSizeMatrix<double, 4, registerdataNum> pinvAt;
//          vct4x4 xt;
//          A = MasterPoint;
//          B = SlavePoint;
//          At = A.Transpose();
//          Bt = B.Transpose();
//          doubleAt.ProductOf(A, At);
//          invdoubleAt = doubleAt;
//          // Compute inverse and check result
//          nmrInverse(invdoubleAt);
//          pinvAt.ProductOf(invdoubleAt,A);
//          xt.ProductOf(pinvAt,Bt);
//          Tm2s = xt.Transpose();
}
