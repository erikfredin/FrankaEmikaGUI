#include "mainwindow.h"
#include "ui_mainwindow.h"

const auto model = fdeep::load_model("C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/fdeep_model.json"); //no normalization layer model
//std::cout<<"load model!"<<std::endl;

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

    // Set up the game controller.
    // This work better here than it does in the objects constructor.
    // Probably due to a timing issue and load on the computer during launch.
    connectedGamepad.reconnectController();

    // Set up the S826 board.
    int errcode = s826.init();
    qInfo() << "S826.init() returned the value: " << errcode;
    qInfo() << " ";

    // PUSH BUTTONS
    connect(ui->pushButton_experimental_control,SIGNAL(clicked()),SLOT(experimental_control()));
    connect(ui->pushButton_freedrag,SIGNAL(clicked()),SLOT(freedrag()));
    connect(ui->pushButton_translationaldrag,SIGNAL(clicked()),SLOT(translationaldrag()));
    connect(ui->pushButton_EEoffsetupdate,SIGNAL(clicked()),SLOT(updateRobotEE()));
    connect(ui->pushButton_robotconnect,SIGNAL(clicked()),SLOT(Robotconnect()));
//    connect(ui->pushButton_pilotthreadon,SIGNAL(clicked()),SLOT(pilotthreadon()));
//    connect(ui->pushButton_pilotthreadoff,SIGNAL(clicked()),SLOT(pilotthreadoff()));
    connect(ui->pushButton_registercollect,SIGNAL(clicked()),SLOT(registerdatacollect()));

    connect(ui->pushButton_calibrationcollect,SIGNAL(clicked()),SLOT(calibratesetflag()));
    connect(ui->pushButton_calibrationcollect_off,SIGNAL(clicked()),SLOT(calibratesetflagoff()));

    connect(ui->pushButton_clearcurrent,SIGNAL(clicked()),SLOT(clearcurrent()));

    connect(ui->pushButton_robotrecovery,SIGNAL(clicked()),SLOT(robotrecovery()));


//    connect(ui->pushButton_setfilename,SIGNAL(clicked()),SLOT(SetFileName(ui->lineEdit_EE_x->text())));

    connect(ui->pushButton_setfilename,SIGNAL(clicked()),SLOT(Setfilename()));
    connect(ui->pushButton_logon,SIGNAL(clicked()),SLOT(SetLogEnabled()));
    connect(ui->pushButton_logoff,SIGNAL(clicked()),SLOT(CloseFiles()));
    connect(ui->pushButton_logpause,SIGNAL(clicked()),SLOT(DisableLog()));
    connect(ui->pushButton_logcontinue,SIGNAL(clicked()),SLOT(EnableLog()));

    connect(ui->pushButton_DNNpredict,SIGNAL(clicked()),SLOT(DNNpredict()));
    connect(ui->pushButton_dnn_inputupdate,SIGNAL(clicked()),SLOT(updateDNNinput()));
    connect(ui->pushButton_run_dnncurent,SIGNAL(clicked()),SLOT(runDNNcurrent()));





    //check box
//
    connect(ui->checkBox_pilot,SIGNAL(clicked()),SLOT(frankathreadcontrol()));
    connect(ui->checkBox_streaming,SIGNAL(clicked()),SLOT(robotstreaming()));
    connect(ui->checkBox_enableDAQ,SIGNAL(clicked()),SLOT(enableDAQ()));



//    connect(ui->lineEdit_EE_x,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );
//    connect(ui->lineEdit_EE_y,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );
//    connect(ui->lineEdit_EE_z,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );

    //we assign transformation matrix here
    transT2R<<  0.0118,  -1.0013,  0.0049,  0.5496,
                0.9993,  0.0160,   -0.0067, -0.0422,
                0.0043,  0.0084,   1.0043,  0.0493,
                0.0,     0.0,      0.0,     1.0;

    std::cout<< "transformation from table to Franka is " << std::endl <<transT2R <<  std::endl;
}



MainWindow::~MainWindow()
{
//    franka::Robot robot(fci_ip);
//    robot.stop();
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

    ui->lineEdit_EE_x->setText(tr("%1").arg(RobotEE_offset[0]));
    ui->lineEdit_EE_y->setText(tr("%1").arg(RobotEE_offset[1]));
    ui->lineEdit_EE_z->setText(tr("%1").arg(RobotEE_offset[2]));

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

//    Eigen::Vector3d a;
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


    try {
//        const double execution_time = 2.0;
//        Robot franka(fci_ip); // IP-Address or hostname of the robot
//        franka.absolute_cart_motion(0.5,0,0.3, execution_time);


       franka::Robot robot(fci_ip);
        std::cout<<"connected!"<<std::endl;
        setDefaultBehavior(robot);
        std::cout<<"set default!"<<std::endl;

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
//        std::cin.ignore();
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

        double time_max = 3.0;
        double v_max = 0.1;
        double angle = M_PI / 4.0;
        double time = 0.0;
        robot.control([=, &time](const franka::RobotState&,
                                 franka::Duration period) -> franka::CartesianVelocities {
          time += period.toSec();

          double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
//          double v_x = std::cos(angle) * v;
//          double v_z = -std::sin(angle) * v;
          double v_x = 0.01;
          double v_y = 0.01;
          double v_z = 0.01;

          franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
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

void MainWindow::robotstreaming(void)
{
    if(ui->checkBox_streaming->checkState())
    {
        isRobotStreaming = true;
        qInfo()<<"robot is streaming";
    }
    else
    {
        isRobotStreaming = false;

        qInfo()<<"robot stops streaming";
    }

}


void MainWindow::robotrecovery(void)
{
    std::cout << "Running error recovery..." << std::endl;
    franka::Robot robot(fci_ip);
    robot.automaticErrorRecovery();
    std::cout << "Error recovery done...move robot to initial pos" << std::endl;
//    setDefaultBehavior(robot);
//    std::cout<<"set default!"<<std::endl;

//    // First move the robot to a suitable joint configuration
//    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//    MotionGenerator motion_generator(0.5, q_goal);
//    std::cout << "WARNING: This example will move the robot! "
//              << "Please make sure to have the user stop button at hand!" << std::endl;
////        std::cin.ignore();
//    robot.control(motion_generator);
//    std::cout << "Finished moving to initial joint configuration." << std::endl;
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
        msdFieldvalue[0] = DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0];
        msdFieldvalue[1] = DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1];
        msdFieldvalue[2] = DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2];

       int i;

       for (i = 0; i < numProberead; i++)
           LogFileAllData<<msdFieldvalue[i]<<Delim;

       for (i = 0; i < numProbePos; i++)
//               LogFileAllData<<ProbePos[i]<<Delim;
           LogFileAllData<<Robot_tip_posisition[i]<<Delim;

       for (i = 0; i < numProbePos; i++)
           LogFileAllData<<robotposcmd[i]<<Delim;


       for (i = 0;i < numAct; i++)
           LogFileAllData<<cmdCoilCurrent[i]<<Delim;

       for (i = 0;i < numAct; i++)
           LogFileAllData<<measuredCurrents[i]<<Delim;

       for (i = 0;i < numProberead; i++)
           LogFileAllData<<DAQ.analogRawInputVoltages[i]<<Delim;

       for (i = 0; i<16; i++)
           LogFileAllData<<current_EEpose[i]<<Delim;

       LogFileAllData<<Robotmotionsuccess;

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
                LogFileAllData<<"robotMsdPos_mm_"<<i+1<<Delim;

            for (i = 0; i < numProbePos; i++)
                LogFileAllData<<"tableCmdPos_mm_"<<i+1<<Delim;


            for (i = 0;i < numAct; i++)
                LogFileAllData<<"cmdCoilCurrent_A_"<<i+1<<Delim;

            for (i = 0;i < numAct; i++)
                LogFileAllData<<"msdCoilCurrent_A_"<<i+1<<Delim;

            for (i = 0;i < numProberead; i++)
                LogFileAllData<<"Daqraw_v"<<i+1<<Delim;

            for (i = 0; i<16; i++)
                LogFileAllData<<"EEpose"<<i<<Delim;

            LogFileAllData<<"Robotmotionsuccess";

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
            std::cerr<<"Overheating!....Sending 0A to coil "<<i<<std::endl;
        }
    }
    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
//        qInfo() << "Wrote values to the S826 in updateCurrents_CalibrationOnly submode";
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


void MainWindow::clearcurrent(void)
{
    for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
        }

    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
        qInfo() << "setting zero to the S826.";
    }
}

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

void MainWindow::calibratesetflagoff(void)
{
    CalibrationDataCollet = false;
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


void MainWindow::enableDAQ()
{
    DAQ.enableDAQ = ui->checkBox_enableDAQ->checkState();
    if (DAQ.isEnabled())
    {
        DAQ.setupTask();
    }
    else
    {
        DAQ.finishTask();
        for ( int i = 0; i < numAct; i++ )
        {
            DAQ.analogInputVoltages[i] = 0.0;
        }
    }
}


void MainWindow::DNNpredict()
{
//    const auto model = fdeep::load_model("C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/fdeep_model.json"); //no normalization layer model
//    std::cout<<"load model!"<<std::endl;
    int inputsize = 92;
    std::vector<float> input = {};

    input = DNNinputprepare(NN_B1, NN_P1, NN_B2, NN_P2);

    const auto result = model.predict(
    {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(inputsize)),
           input )});

    std::cout << fdeep::show_tensors(result) << std::endl;

    std::string outstring = fdeep::show_tensors(result);
    std::string cleanstring = {};
    for (int k=0; k<outstring.size(); k++)
    {
        if (outstring[k] != '[' && outstring[k] != ']' && outstring[k] != ' ')
        {
            cleanstring.push_back(outstring[k]);
        }

    }

    std::string delimiters(" ,");
    std::vector<std::string> parts;
    boost::split(parts, cleanstring, boost::is_any_of(delimiters));

    std::vector<float> dataout = {};
    int strsize = parts.size();

    for(int k=0; k<strsize; k++)
    {
        dataout.push_back(std::stof(parts[k]));
    }

    std::cout<<"final dataout is "<<std::endl;
    for (float cc : dataout)
    {
        std::cout<<cc<<",\t";
    }
    std::cout<<std::endl;

    for(int k=0; k<numAct; k++)
    {
        if(dataout[k]>maxCurrent)
            predictCoilCurrent[k] = maxCurrent;
        else if (dataout[k]<-maxCurrent)
            predictCoilCurrent[k] = -maxCurrent;
        else
            predictCoilCurrent[k] = dataout[k];
    }

    ui->label_dnn_predict_I1->setText(tr("%1").arg(predictCoilCurrent[0]));
    ui->label_dnn_predict_I2->setText(tr("%1").arg(predictCoilCurrent[1]));
    ui->label_dnn_predict_I3->setText(tr("%1").arg(predictCoilCurrent[2]));
    ui->label_dnn_predict_I4->setText(tr("%1").arg(predictCoilCurrent[3]));
    ui->label_dnn_predict_I5->setText(tr("%1").arg(predictCoilCurrent[4]));
    ui->label_dnn_predict_I6->setText(tr("%1").arg(predictCoilCurrent[5]));
    ui->label_dnn_predict_I7->setText(tr("%1").arg(predictCoilCurrent[6]));
    ui->label_dnn_predict_I8->setText(tr("%1").arg(predictCoilCurrent[7]));

}

std::vector<float> MainWindow::DNNinputprepare(double NN_B1[3], double NN_P1[3], double NN_B2[3], double NN_P2[3] )
{
    std::vector<float> input = {};
    //create input data
    double r1[numAct] = {};
    double r2[numAct] = {};
    double r1_sup2[numAct] = {};
    double r1_sup_3[numAct] = {};
    double r2_sup2[numAct] = {};
    double r2_sup_3[numAct] = {};
    double r1_xy[numAct] = {};
    double r1_xz[numAct] = {};
    double r1_yz[numAct] = {};
    double r2_xy[numAct] = {};
    double r2_xz[numAct] = {};
    double r2_yz[numAct] = {};
    for (int j= 0; j<numAct; j++)
    {
        r1[j] = sqrt(pow((NN_P1[0]-pAct_cartesion[0][j]),2) + pow((NN_P1[1]-pAct_cartesion[1][j]),2)+ pow((NN_P1[2]-pAct_cartesion[2][j]),2));
        r1_sup2[j] = pow(r1[j],2);
        r1_sup_3[j] = pow(r1[j], -3);
        double r1_vec[3] = {NN_P1[0]-pAct_cartesion[0][j], NN_P1[1]-pAct_cartesion[1][j], NN_P1[2]-pAct_cartesion[2][j]};
        r1_xy[j] = r1_vec[0]*r1_vec[1];
        r1_xz[j] = r1_vec[0]*r1_vec[2];
        r1_yz[j] = r1_vec[1]*r1_vec[2];

        r2[j] = sqrt(pow((NN_P2[0]-pAct_cartesion[0][j]),2) + pow((NN_P2[1]-pAct_cartesion[1][j]),2)+ pow((NN_P2[2]-pAct_cartesion[2][j]),2));
        r2_sup2[j] = pow(r2[j],2);
        r2_sup_3[j] = pow(r2[j], -3);
        double r2_vec[3] = {NN_P2[0] - pAct_cartesion[0][j], NN_P2[1] - pAct_cartesion[1][j], NN_P2[2] - pAct_cartesion[2][j]};
        r2_xy[j] = r2_vec[0]*r2_vec[1];
        r2_xz[j] = r2_vec[0]*r2_vec[2];
        r2_yz[j] = r2_vec[1]*r2_vec[2];
     }
    for (int k=0; k<3; k++)
    {
        input.push_back(NN_B1[k]);
    }
    for (int k=0; k<3; k++)
    {
        input.push_back(NN_P1[k]);
    }
    for (int k=0; k<3; k++)
    {
        input.push_back(NN_B2[k]);
    }
    for (int k=0; k<3; k++)
    {
        input.push_back(NN_P2[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r1_sup2[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r2_sup2[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r1_sup_3[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r2_sup_3[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r1_xy[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r1_xz[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r1_yz[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r2_xy[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r2_xz[k]);
    }
    for (int k=0; k<numAct; k++)
    {
        input.push_back(r2_yz[k]);
    }

    std::vector<float> sample = { 2.24063043e+01, -1.48057763e+01, -6.44080494e+00,  6.99414861e-03,
                       -8.01251138e-03,  9.77839592e-02,  1.70210778e+01, -1.44074961e+01,
                       -1.75598573e+01,  2.29818797e-02, -3.41654689e-02,  1.02230576e-01,
                        1.64592241e-01,  1.01774340e-01,  1.71096466e-01,  9.78269326e-02,
                        1.02042214e-01,  1.58914681e-01,  9.80948065e-02,  1.65418906e-01,
                        1.64623236e-01,  1.10136663e-01,  1.92357349e-01,  9.51044114e-02,
                        1.13078436e-01,  1.45967498e-01,  9.80461844e-02,  1.73701611e-01,
                        1.49756597e+01,  3.07994213e+01,  1.41298790e+01,  3.26822800e+01,
                        3.06782222e+01,  1.57853411e+01,  3.25484999e+01,  1.48635410e+01,
                        1.49714305e+01,  2.73591204e+01,  1.18532331e+01,  3.40956525e+01,
                        2.62984597e+01,  1.79314642e+01,  3.25727146e+01,  1.38131978e+01,
                        4.09217931e-02, -1.10986111e-03, -4.42859870e-02,  8.63842748e-04,
                       -9.75924139e-04, -3.81950944e-02,  9.97779715e-04,  4.13351255e-02,
                        6.03074627e-02,  3.97913500e-02,  6.03074627e-02,  2.00920179e-03,
                        2.00920179e-03, -5.62890592e-02, -3.57729464e-02, -5.62890592e-02,
                        5.59965151e-02, -2.30174581e-03, -6.06000068e-02,  3.54804024e-02,
                       -4.00838940e-02,  5.59965151e-02, -2.30174581e-03, -6.06000068e-02,
                        3.81297197e-02, -5.27869276e-03, -5.35671494e-02,  2.23743290e-03,
                       -3.80780629e-03, -3.03722240e-02,  3.70831936e-03,  4.26689069e-02,
                        6.59048281e-02,  4.50711479e-02,  6.59048281e-02,  6.70417228e-03,
                        6.70417228e-03, -5.24964836e-02, -3.16628033e-02, -5.24964836e-02,
                        4.92340564e-02, -9.96659945e-03, -6.91672553e-02,  2.84003761e-02,
                       -4.83335750e-02,  4.92340564e-02, -9.96659945e-03, -6.91672553e-02};
    return input;
}

void MainWindow::updateDNNinput(void)
{

    NN_B1[0] = ui->lineEdit_dnnInput_b1x->text().toDouble();
    NN_B1[1] = ui->lineEdit_dnnInput_b1y->text().toDouble();
    NN_B1[2] = ui->lineEdit_dnnInput_b1z->text().toDouble();
    NN_B2[0] = ui->lineEdit_dnnInput_b2x->text().toDouble();
    NN_B2[1] = ui->lineEdit_dnnInput_b2y->text().toDouble();
    NN_B2[2] = ui->lineEdit_dnnInput_b2z->text().toDouble();

    NN_P1[0] = ui->lineEdit_dnnInput_p1x->text().toDouble();
    NN_P1[1] = ui->lineEdit_dnnInput_p1y->text().toDouble();
    NN_P1[2] = ui->lineEdit_dnnInput_p1z->text().toDouble();
    NN_P2[0] = ui->lineEdit_dnnInput_p2x->text().toDouble();
    NN_P2[1] = ui->lineEdit_dnnInput_p2y->text().toDouble();
    NN_P2[2] = ui->lineEdit_dnnInput_p2z->text().toDouble();

    std::cout<< "DNN Inputs are updated as: " << std::endl;
    std::cout << "   NN_B1: ";
    for(int k=0; k<3; k++)
        std::cout << NN_B1[k]<<"   ";
    std::cout << "NN_P1:  ";
    for(int k=0; k<3; k++)
        std::cout << NN_P1[k]<<"   ";
    std::cout <<std::endl << "   NN_B2: ";
    for(int k=0; k<3; k++)
        std::cout << NN_B2[k]<<"   ";
    std::cout << "NN_P2:  ";
    for(int k=0; k<3; k++)
        std::cout << NN_P2[k]<<"   ";
    std::cout <<std::endl;
}


void MainWindow::runDNNcurrent(void)
{
    MainWindow::updateCurrents_CalibrationOnly(predictCoilCurrent);
}


void MainWindow::moveRot_P1(void)
{
    std::cout << std::endl<< "command position in table frame is: "<<NN_P1[0]<<", "<<NN_P1[1]<<", "<<NN_P1[2]<<std::endl;

    //covert cmd position in table frame to robot frame
    Eigen::Vector4d pos_cmd(NN_P1[0], NN_P1[1], NN_P1[2], 1);
    Eigen::Vector4d pos_robot = transT2R*pos_cmd;

    double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};

    std::cout<< "command position in robot frame is: "<<abs_robotpos[0]<<", "<<abs_robotpos[1]<<", "<<abs_robotpos[2]<<std::endl;

    //move robot in absolute position
    franka::Robot robot(fci_ip);
    try{
        setDefaultBehavior(robot);
        // Set additional parameters always before the control loop, NEVER in the control loop!

//                            std::array<double, 16> initial_pose;
        double time = 0.0;

        /// ----------------robot control loop---------------------------------
        robot.control([=, &time](const franka::RobotState& robot_state,
                                 franka::Duration period) -> franka::CartesianVelocities
//                            robot.control([&time, &initial_pose, &abs_robotpos]( const franka::RobotState& robot_state,
//                                                                 franka::Duration period) -> franka::CartesianPose
        {
          time += period.toSec();
//                              if (time == 0.0) {
//                                initial_pose = robot_state.O_T_EE_c; //Last commanded end effector pose of motion generation in base frame.
//                                //robot_state.O_T_EE; Measured end effector pose in base frame.
//                                //robot_state.O_T_EE_d; Last desired end effector pose of motion generation in base frame.
//                              }
//                              std::array<double, 16> new_pose = initial_pose;
          double tolerance = 0.001; //1mm
          double error[3];
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

          //default unit is meter and rad, so we keep that
          Robot_tip_posisition[0] = current_EEpose[12];
          Robot_tip_posisition[1] = current_EEpose[13];
          Robot_tip_posisition[2] = current_EEpose[14];

//                              qInfo() << "command position in robot frame is: "<<abs_robotpos[0]<<", "<<abs_robotpos[1]<<", "<<abs_robotpos[2];
//                              qInfo() << "current robot position is: "<<current_pose[12]<<", "<<current_pose[13]<<", "<<current_pose[14];
          for(int k=0; k<3; k++)
          {
              double temp_e = abs_robotpos[k]-current_EEpose[12+k];
              if (temp_e>0)
                 direction[k] = 1.0;
              else
                  direction[k] = -1.0;

              if (abs(temp_e)>=tolerance)
                  error[k] = temp_e;
              else
              {
                  error[k] = 0.0;
                  direction[k] = 0.0;
              }
          }
//                              std::cout<<"pos error is: "<<error[0]<<" "<<error[1]<<" "<<error[2]<<std::endl;
//                              double detp = 0.00001; //0.1mm

//                              new_pose[12] = current_pose[12]+direction[0]*detp;
//                              new_pose[13] = current_pose[13]+direction[1]*detp;
//                              new_pose[14] = current_pose[14]+direction[2]*detp;
          double v_x = 0.002;
          double v_y = 0.002;
          double v_z = 0.002;
          double maxv = 0.010;  //10mm/s
          double maxDelt_e = 0.05; //50mm
          double A = maxv/(maxDelt_e*maxDelt_e);
          double v_cmd[3] = {0.0};

          for (int k=0; k<3; k++)
          {
              if(abs(error[k])<=maxDelt_e)
                   v_cmd[k] = maxv*sin((M_PI/2)*(error[k]/maxDelt_e));
//                                      v_cmd[k] = A*pow(error[k],2);
              else
                  v_cmd[k] = maxv*direction[k];
          }

//                              franka::CartesianVelocities output = {{direction[0]*v_x, direction[1]*v_y, direction[2]*v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{v_cmd[0], v_cmd[1], v_cmd[2], 0.0, 0.0, 0.0}};
          if (abs(error[0])<tolerance && abs(error[1])<tolerance && abs(error[2])<tolerance) {
            std::cout << "single motion Finished: ["<<robotmovecount+1<<"/"<<robotmoveloop <<"]" << std::endl;
            std::cout << "motion error is "<<error[0]<<" " <<error[1]<<" "<<error[2]<<std::endl;
            output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            Robotmotionsuccess = 1;
            return franka::MotionFinished(output);
          }
          return output;
        });
    }catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::cout << "Running error recovery..." << std::endl;
    Robotmotionsuccess = 0;
    robot.automaticErrorRecovery();
    }
}

void MainWindow::moveRot_P2(void)
{
    std::cout << std::endl<< "command position in table frame is: "<<NN_P2[0]<<", "<<NN_P2[1]<<", "<<NN_P2[2]<<std::endl;

    //covert cmd position in table frame to robot frame
    Eigen::Vector4d pos_cmd(NN_P2[0], NN_P2[1], NN_P2[2], 1);
    Eigen::Vector4d pos_robot = transT2R*pos_cmd;

    double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};

    std::cout<< "command position in robot frame is: "<<abs_robotpos[0]<<", "<<abs_robotpos[1]<<", "<<abs_robotpos[2]<<std::endl;

    //move robot in absolute position
    franka::Robot robot(fci_ip);
    try{
        setDefaultBehavior(robot);
        // Set additional parameters always before the control loop, NEVER in the control loop!

//                            std::array<double, 16> initial_pose;
        double time = 0.0;

        /// ----------------robot control loop---------------------------------
        robot.control([=, &time](const franka::RobotState& robot_state,
                                 franka::Duration period) -> franka::CartesianVelocities
//                            robot.control([&time, &initial_pose, &abs_robotpos]( const franka::RobotState& robot_state,
//                                                                 franka::Duration period) -> franka::CartesianPose
        {
          time += period.toSec();
//                              if (time == 0.0) {
//                                initial_pose = robot_state.O_T_EE_c; //Last commanded end effector pose of motion generation in base frame.
//                                //robot_state.O_T_EE; Measured end effector pose in base frame.
//                                //robot_state.O_T_EE_d; Last desired end effector pose of motion generation in base frame.
//                              }
//                              std::array<double, 16> new_pose = initial_pose;
          double tolerance = 0.001; //1mm
          double error[3];
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

          //default unit is meter and rad, so we keep that
          Robot_tip_posisition[0] = current_EEpose[12];
          Robot_tip_posisition[1] = current_EEpose[13];
          Robot_tip_posisition[2] = current_EEpose[14];

//                              qInfo() << "command position in robot frame is: "<<abs_robotpos[0]<<", "<<abs_robotpos[1]<<", "<<abs_robotpos[2];
//                              qInfo() << "current robot position is: "<<current_pose[12]<<", "<<current_pose[13]<<", "<<current_pose[14];
          for(int k=0; k<3; k++)
          {
              double temp_e = abs_robotpos[k]-current_EEpose[12+k];
              if (temp_e>0)
                 direction[k] = 1.0;
              else
                  direction[k] = -1.0;

              if (abs(temp_e)>=tolerance)
                  error[k] = temp_e;
              else
              {
                  error[k] = 0.0;
                  direction[k] = 0.0;
              }
          }
//                              std::cout<<"pos error is: "<<error[0]<<" "<<error[1]<<" "<<error[2]<<std::endl;
//                              double detp = 0.00001; //0.1mm

//                              new_pose[12] = current_pose[12]+direction[0]*detp;
//                              new_pose[13] = current_pose[13]+direction[1]*detp;
//                              new_pose[14] = current_pose[14]+direction[2]*detp;
          double v_x = 0.002;
          double v_y = 0.002;
          double v_z = 0.002;
          double maxv = 0.010;  //10mm/s
          double maxDelt_e = 0.05; //50mm
          double A = maxv/(maxDelt_e*maxDelt_e);
          double v_cmd[3] = {0.0};

          for (int k=0; k<3; k++)
          {
              if(abs(error[k])<=maxDelt_e)
                   v_cmd[k] = maxv*sin((M_PI/2)*(error[k]/maxDelt_e));
//                                      v_cmd[k] = A*pow(error[k],2);
              else
                  v_cmd[k] = maxv*direction[k];
          }

//                              franka::CartesianVelocities output = {{direction[0]*v_x, direction[1]*v_y, direction[2]*v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{v_cmd[0], v_cmd[1], v_cmd[2], 0.0, 0.0, 0.0}};
          if (abs(error[0])<tolerance && abs(error[1])<tolerance && abs(error[2])<tolerance) {
            std::cout << "single motion Finished: ["<<robotmovecount+1<<"/"<<robotmoveloop <<"]" << std::endl;
            std::cout << "motion error is "<<error[0]<<" " <<error[1]<<" "<<error[2]<<std::endl;
            output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            Robotmotionsuccess = 1;
            return franka::MotionFinished(output);
          }
          return output;
        });
    }catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::cout << "Running error recovery..." << std::endl;
    Robotmotionsuccess = 0;
    robot.automaticErrorRecovery();
    }
}
