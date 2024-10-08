#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <franka/robot.h>
#include <franka/exception.h>
#include <QTimer>

using namespace orl;
using namespace std::literals;

//const auto DNNmodel = fdeep::load_model("C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/fdeep_model.json"); //no normalization layer model
//std::cout<<"load model!"<<std::endl;

//std::string initialguess = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess.yaml";
//ElectromagnetCalibration mymodel(initialguess);
//std::cout<<"load calibration file!"<<std::endl;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), robot(fci_ip)
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

    stateUpdateTimer = new QTimer(this);
    connect(stateUpdateTimer, &QTimer::timeout, this, &MainWindow::updateRobotState);
    stateUpdateTimer->start(100); // Update every 100 ms

    // PUSH BUTTONS
    connect(ui->pushButton_moveEE, SIGNAL(clicked()), SLOT(on_pushButton_moveEE_clicked()));
    connect(ui->pushButton_experimental_control,SIGNAL(clicked()),SLOT(experimental_control()));
    connect(ui->pushButton_Control_ml,SIGNAL(clicked()),SLOT(experimental_control_ml()));
    connect(ui->pushButton_freedrag,SIGNAL(clicked()),SLOT(freedrag()));
    connect(ui->pushButton_translationaldrag,SIGNAL(clicked()),SLOT(translationaldrag()));
    connect(ui->pushButton_EEoffsetupdate,SIGNAL(clicked()),SLOT(updateRobotEE()));
    connect(ui->pushButton_robotconnect,SIGNAL(clicked()),SLOT(Robotconnect()));
//    connect(ui->pushButton_pilotthreadon,SIGNAL(clicked()),SLOT(pilotthreadon()));
//    connect(ui->pushButton_pilotthreadoff,SIGNAL(clicked()),SLOT(pilotthreadoff()));
    connect(ui->pushButton_registercollect,SIGNAL(clicked()),SLOT(registerdatacollect()));

    connect(ui->pushButton_calibrationcollect,SIGNAL(clicked()),SLOT(calibratesetflag()));
    connect(ui->pushButton_calibrationcollect_off,SIGNAL(clicked()),SLOT(calibratesetflagoff()));

    connect(ui->pushButton_calibrationcollect_sequence_on,SIGNAL(clicked()),SLOT(calibratesetflag_sequence()));
    connect(ui->pushButton_calibrationcollect_sequence_off,SIGNAL(clicked()),SLOT(calibratesetflagoff_sequence()));


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
    connect(ui->pushButton_initializerobot,SIGNAL(clicked()),SLOT(initializeFranka()));

    connect(ui->pushButton_moveRot2_P1,SIGNAL(clicked()),SLOT(moveRot_P1()));
    connect(ui->pushButton_moveRot2_P2,SIGNAL(clicked()),SLOT(moveRot_P2()));

    connect(ui->pushButton_runGlobalField,SIGNAL(clicked()),SLOT(runGlobalfield()));

    connect(ui->pushButton_Cartesiantest,SIGNAL(clicked()),SLOT(Cartesiantest()));

    connect(ui->pushButton_initialprobeorient,SIGNAL(clicked()),SLOT(initialProbeOrient()));
    connect(ui->pushButton_CalibrateCoiltable,SIGNAL(clicked()),SLOT(CalibrateCoiltable()));

    connect(ui->pushButton_Fullworkspace_MoveRobot,SIGNAL(clicked()),SLOT(Fullworkspace_MoveRobot()));

    connect(ui->pushButton_Fullworkspace_run,SIGNAL(clicked()),SLOT(runFullWorkspacefield()));

    connect(ui->pushButton_Validata_data_collect,SIGNAL(clicked()),SLOT(Validation_datacollect_pushbutton()));

    connect(ui->pushButton_updateCurrent,SIGNAL(clicked()),SLOT(UpdateCurrent_fromGUI()));

    connect(ui->pushButton_robotOrient_run,SIGNAL(clicked()),SLOT(FrankaOrientAdjust()));







    //check box
//
    connect(ui->checkBox_pilot,SIGNAL(clicked()),SLOT(frankathreadcontrol()));
    connect(ui->checkBox_streaming,SIGNAL(clicked()),SLOT(robotstreaming()));
    connect(ui->checkBox_currentfeedback,SIGNAL(clicked()),SLOT(currentfeedback()));
    connect(ui->checkBox_enableDAQ,SIGNAL(clicked()),SLOT(enableDAQ()));

    connect(ui->checkBox_controllerEnable,SIGNAL(clicked()),SLOT(enableController()));

    connect(ui->checkBox_guidingmode,SIGNAL(clicked()),SLOT(setFrankaguidingmode()));

    connect(ui->checkBox_validation_datacollect,SIGNAL(clicked()),SLOT(Validation_datacollect()));

    connect(ui->checkBox_udpmode,SIGNAL(clicked()),SLOT(enableUDP()));


//    connect(ui->lineEdit_EE_x,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );
//    connect(ui->lineEdit_EE_y,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );
//    connect(ui->lineEdit_EE_z,SIGNAL(editingFinished()),SLOT( updateRobotEE() ) );

    //we assign transformation matrix here

//    transT2R<< -0.00154759037336260,	-0.999982247374182,     0.00835101244978627,	0.564180613920046,
//                1.00000109635735,       -0.00152205831827103,	-0.00193260051127602,	-0.0188112812049862,
//                0.00194690720224443,	0.00834849788023952,	0.999963649556847,      0.0500355897415963,
//                0.0,     0.0,	0.0,	1.0;

//    transT2R<<  -0.00155386058946349,	-0.999954382073213, 	0.00830767687423786,	0.564466737536956,
//                0.999986825617582,      -0.00157122681481832,	-0.00213670207361007,	-0.0188948524267936,
//                0.00214933885311591,	0.00830429191316800,	0.999956569897929,      0.0500234630678109,
//                0.0,	0.0,	0.0,	1.0;

    transT2R<<  0.00198243,	-0.99997,	0.00384,	0.56825,
                0.99997951,	0.001983,	0.0,        -0.02338,
                0.0,    	0.003849,	0.99998,	0.051278,
                0.0,	0.0,	0.0,	1.0;


    transUS2EE <<     0.0000,    0.0000,          0.0,    0.0072,
                         -0.0012,   -0.0002,         0.0,    0.0845,
                         -0.0001,    0.0011,         0.0,    0.1819,
                          0.0000,   -0.0000,         0.0,    1.0000 ;

    std::cout<< "transformation from table to Franka is " << std::endl <<transT2R <<  std::endl;

//    ElectromagnetCalibration mycalibration("InitialGuess.yaml");
//    std::string name1 = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess.yaml";
//    ElectromagnetCalibration mymodel(name1);

//    std::cout<< "Initialized ElectromagnetCalibration "  <<  std::endl;

    socket = new QUdpSocket(this);
    bool result =  socket->bind(QHostAddress::AnyIPv4, SERVER_PORT);
    qDebug() << result;
    if(result)
    {
        qDebug() << "PASS";
//        UDPflag = true;
    }
    else
    {
        qDebug() << "FAIL";
//        UDPflag = false;
    }

    connect(socket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()),Qt::QueuedConnection);

    socket_send = new QUdpSocket(this);



//    host  = new QHostAddress("192.168.1.101");
//    bcast = new QHostAddress("192.168.31.124");

//    socket_send->bind(QHostAddress("192.168.31.124"), SERVER_PORT);

//    socket_send->connectToHost(QHostAddress("192.168.31.124"),SERVER_PORT);

//    QByteArray *datagram = makeNewDatagram(); // data from external function
//    QByteArray Data; // Message for send
//       Data += "SAMP";
//       Data += "SAMP";
//       Data += "SAMP";
//       Data += "SAMP";
//    socket_send->write(Data);

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DESTROYER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

MainWindow::~MainWindow()
{
//    franka::Robot robot(fci_ip);
//    robot.stop();
    delete frankathread;
    std::cout<<"delete Franka thread---"<<std::endl;
    clearcurrent();
    std::cout<<"clear current---"<<std::endl;
    delete ui;
    socket->~QUdpSocket();
    socket_send->~QUdpSocket();
}


void MainWindow::updateCaption(void)
{
    // Update labels on the GUI to reflect parameters saved in code

    // for testing connection
    ui->label_robot_px->setText(tr("%1").arg(Robot_tip_position[0]));
    ui->label_robot_py->setText(tr("%1").arg(Robot_tip_position[1]));
    ui->label_robot_pz->setText(tr("%1").arg(Robot_tip_position[2]));
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

//    ui->lineEdit_EE_x->setText(tr("%1").arg(RobotEE_offset[0]));
//    ui->lineEdit_EE_y->setText(tr("%1").arg(RobotEE_offset[1]));
//    ui->lineEdit_EE_z->setText(tr("%1").arg(RobotEE_offset[2]));

    ui->label_current1_1->setText(tr("%1").arg(measuredCurrents_feed[0]));
    ui->label_current1_2->setText(tr("%1").arg(measuredCurrents_feed[1]));
    ui->label_current1_3->setText(tr("%1").arg(measuredCurrents_feed[2]));
    ui->label_current1_4->setText(tr("%1").arg(measuredCurrents_feed[3]));
    ui->label_current1_5->setText(tr("%1").arg(measuredCurrents_feed[4]));
    ui->label_current1_6->setText(tr("%1").arg(measuredCurrents_feed[5]));
    ui->label_current1_7->setText(tr("%1").arg(measuredCurrents_feed[6]));
    ui->label_current1_8->setText(tr("%1").arg(measuredCurrents_feed[7]));

}

void MainWindow::enableController(void)
{
    // For Enabling or disabling the gamepad or game controller.
    controllerState = ui->checkBox_controllerEnable->checkState();

    // Enable the controller if the checkbox is checked
    if (controllerState)
    {
        connectedGamepad.enableController();
    }
    else
    {
        // If checkbox is unchecked, disable controller and reset field values
        connectedGamepad.disableController();
    }
}


void MainWindow::updateRobotEE(void)
{

    RobotEE_offset[0] = ui->lineEdit_EE_x->text().toDouble();
    RobotEE_offset[1] = ui->lineEdit_EE_y->text().toDouble();
    RobotEE_offset[2] = ui->lineEdit_EE_z->text().toDouble();
    double RobotEE_offset_meter[3];
    RobotEE_offset_meter[0] = RobotEE_offset[0]*0.001; //transform mm in GUI to meter for Franka arm
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


void MainWindow::on_pushButton_moveEE_clicked()
{
    // Retrieve input values
    double x_distance_cm = ui->lineEdit_x_distance_cm->text().toDouble();
    double y_distance_cm = ui->lineEdit_y_distance_cm->text().toDouble();
    double z_distance_cm = ui->lineEdit_z_distance_cm->text().toDouble();

    double rotate_x_deg = ui->lineEdit_rotate_x_deg->text().toDouble();
    double rotate_y_deg = ui->lineEdit_rotate_y_deg->text().toDouble();
    double rotate_z_deg = ui->lineEdit_rotate_z_deg->text().toDouble();

    // Convert units
    double x_meters = x_distance_cm / 100.0;
    double y_meters = y_distance_cm / 100.0;
    double z_meters = z_distance_cm / 100.0;

    double angleX_rad = rotate_x_deg * M_PI / 180.0;
    double angleY_rad = rotate_y_deg * M_PI / 180.0;
    double angleZ_rad = rotate_z_deg * M_PI / 180.0;

    double duration_sec = 5.0; // You can also make this an input

    // Perform movements
    try {
        if (x_meters != 0.0) {
            EE_moveInX(x_meters, duration_sec);
        }
        if (y_meters != 0.0) {
            EE_moveInY(y_meters, duration_sec);
        }
        if (z_meters != 0.0) {
            EE_moveInZ(z_meters, duration_sec);
        }
        if (angleX_rad != 0.0) {
            EE_rotateAboutX(angleX_rad, duration_sec);
        }
        if (angleY_rad != 0.0) {
            EE_rotateAboutY(angleY_rad, duration_sec);
        }
        if (angleZ_rad != 0.0) {
            EE_rotateAboutZ(angleZ_rad, duration_sec);
        }
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        robot.automaticErrorRecovery();
    }
}

void MainWindow::updateRobotState()
{
    try {
        franka::RobotState robot_state = robot.readOnce();

        // Get joint positions
        std::array<double, 7> q = robot_state.q;



    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        robot.automaticErrorRecovery();
    }
}

// Cycloidal motion function with zero starting and ending velocity and acceleration
double MainWindow::cycloidal_motion(const double& startVal, const double& deltaVal, const double& interpVal)
{
    return startVal + deltaVal * M_1_PI * (M_PI * interpVal - 0.5 * std::sin(2.0 * M_PI * interpVal));
}

// Time-interpolated movement function
bool MainWindow::EE_move_timeInterpolated(
    const double& duration_sec,
    const std::function<void(const double&, const std::array<double, 16>&, std::array<double, 16>&)>& interpolateFcn)
{
    std::array<double, 16> initPose = robot.readOnce().O_T_EE_c;
    std::array<double, 16> newPose = initPose;
    double time = 0.0;

    try {
        robot.stop();
        robot.control([&](const franka::RobotState& robot_state, const franka::Duration& period) -> franka::CartesianPose {
            time += period.toSec();
            double interpVal = time / duration_sec;
            if (interpVal > 1.0) interpVal = 1.0;
            interpolateFcn(interpVal, initPose, newPose);
            if (time >= duration_sec) {
                return franka::MotionFinished(newPose);
            }
            return newPose;
        });
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        robot.automaticErrorRecovery();
        return false;
    }
}

bool MainWindow::EE_moveInX(const double& x_meters, const double& duration_sec)
{
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM,
                          std::array<double, 16>& newHTM) {
        newHTM = curHTM;
        newHTM[12] = cycloidal_motion(curHTM[12], x_meters, interpVal);
    };
    return EE_move_timeInterpolated(duration_sec, interpFunc);
}

bool MainWindow::EE_moveInY(const double& y_meters, const double& duration_sec)
{
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM,
                          std::array<double, 16>& newHTM) {
        newHTM = curHTM;
        newHTM[13] = cycloidal_motion(curHTM[13], y_meters, interpVal);
    };
    return EE_move_timeInterpolated(duration_sec, interpFunc);
}

bool MainWindow::EE_moveInZ(const double& z_meters, const double& duration_sec)
{
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM,
                          std::array<double, 16>& newHTM) {
        newHTM = curHTM;
        newHTM[14] = cycloidal_motion(curHTM[14], z_meters, interpVal);
    };
    return EE_move_timeInterpolated(duration_sec, interpFunc);
}

bool MainWindow::EE_rotateAboutX(const double& angleX_rad, const double& duration_sec)
{
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM,
                          std::array<double, 16>& newHTM) {
        double rotAngle = cycloidal_motion(0.0, angleX_rad, interpVal);
        double ctheta = std::cos(rotAngle);
        double stheta = std::sin(rotAngle);

        Eigen::Matrix3d rotMtx;
        rotMtx << 1.0, 0.0, 0.0,
                  0.0, ctheta, -stheta,
                  0.0, stheta, ctheta;

        Eigen::Map<const Eigen::Matrix4d> curHTMMatrix(curHTM.data());
        Eigen::Matrix4d newHTMMatrix = curHTMMatrix;
        newHTMMatrix.block<3, 3>(0, 0) = rotMtx * curHTMMatrix.block<3, 3>(0, 0);
        Eigen::Map<Eigen::Matrix4d>(newHTM.data()) = newHTMMatrix;
    };
    return EE_move_timeInterpolated(duration_sec, interpFunc);
}

bool MainWindow::EE_rotateAboutY(const double& angleY_rad, const double& duration_sec)
{
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM,
                          std::array<double, 16>& newHTM) {
        double rotAngle = cycloidal_motion(0.0, angleY_rad, interpVal);
        double ctheta = std::cos(rotAngle);
        double stheta = std::sin(rotAngle);

        Eigen::Matrix3d rotMtx;
        rotMtx << ctheta, 0.0, stheta,
                  0.0, 1.0, 0.0,
                  -stheta, 0.0, ctheta;

        Eigen::Map<const Eigen::Matrix4d> curHTMMatrix(curHTM.data());
        Eigen::Matrix4d newHTMMatrix = curHTMMatrix;
        newHTMMatrix.block<3, 3>(0, 0) = rotMtx * curHTMMatrix.block<3, 3>(0, 0);
        Eigen::Map<Eigen::Matrix4d>(newHTM.data()) = newHTMMatrix;
    };
    return EE_move_timeInterpolated(duration_sec, interpFunc);
}

bool MainWindow::EE_rotateAboutZ(const double& angleZ_rad, const double& duration_sec)
{
    auto interpFunc = [&](const double& interpVal, const std::array<double, 16>& curHTM,
                          std::array<double, 16>& newHTM) {
        double rotAngle = cycloidal_motion(0.0, angleZ_rad, interpVal);
        double ctheta = std::cos(rotAngle);
        double stheta = std::sin(rotAngle);

        Eigen::Matrix3d rotMtx;
        rotMtx << ctheta, -stheta, 0.0,
                  stheta, ctheta, 0.0,
                  0.0, 0.0, 1.0;

        Eigen::Map<const Eigen::Matrix4d> curHTMMatrix(curHTM.data());
        Eigen::Matrix4d newHTMMatrix = curHTMMatrix;
        newHTMMatrix.block<3, 3>(0, 0) = rotMtx * curHTMMatrix.block<3, 3>(0, 0);
        Eigen::Map<Eigen::Matrix4d>(newHTM.data()) = newHTMMatrix;
    };
    return EE_move_timeInterpolated(duration_sec, interpFunc);
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
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
//        std::array<double, 7> q_goal = { {-1.3875, -1.5927, 1.6031, -1.8722, 0.0507, 1.7052, -0.5713} };

        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
//        std::cin.ignore();
//        robot.control(motion_generator);
//        std::cout << "Finished moving to initial joint configuration." << std::endl;

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
          double omega_x = 0.02;
          double omega_y = 0.02;
          double omega_z = 0.05;

//          franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{ 0.0, 0.0, 0.0, 0.0,omega_y, 0.0 }};
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
void MainWindow::experimental_control_ml()
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
        //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
        std::array<double, 7> q_goal = { {-1.3875, -1.5927, 1.6031, -1.8722, 0.0507, 1.7052, -0.5713} };

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
                    {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.} };
                std::array<double, 7> upper_torque_thresholds_nominal{
                    {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0} };
                std::array<double, 7> lower_torque_thresholds_acceleration{
                    {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0} };
                std::array<double, 7> upper_torque_thresholds_acceleration{
                    {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0} };
                std::array<double, 6> lower_force_thresholds_nominal{ {30.0, 30.0, 30.0, 25.0, 25.0, 25.0} };
                std::array<double, 6> upper_force_thresholds_nominal{ {40.0, 40.0, 40.0, 35.0, 35.0, 35.0} };
                std::array<double, 6> lower_force_thresholds_acceleration{ {30.0, 30.0, 30.0, 25.0, 25.0, 25.0} };
                std::array<double, 6> upper_force_thresholds_acceleration{ {40.0, 40.0, 40.0, 35.0, 35.0, 35.0} };
                robot.setCollisionBehavior(
                    lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                    lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                    lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                    lower_force_thresholds_nominal, upper_force_thresholds_nominal);
                double time_max = 10.0;
                double v_max = 0.1;
                double angle = M_PI / 4.0;
                double time = 0.0;
                robot.control([=, &time](const franka::RobotState&,
                    franka::Duration period) -> franka::CartesianVelocities {
                        time += period.toSec();
                        double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
                        double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
                        //double v_x = std::cos(angle) * v;
                        double v_y = -std::sin(angle) * v;
                        franka::CartesianVelocities output = { {0.0, v_y, 0.0, 0.0, 0.0, 0.0} };
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
    std::cout << "Error recovery done..." << std::endl;
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
           LogFileAllData<<Robot_tip_position[i]<<Delim; //probe position in robot frame

       for (i = 0; i < numProbePos; i++)
           LogFileAllData<<robotposcmd[i]<<Delim; //probe position in table frame


       for (i = 0;i < numAct; i++)
           LogFileAllData<<cmdCoilCurrent[i]<<Delim;

       for (i = 0;i < numAct; i++)
           LogFileAllData<<measuredCurrents[i]<<Delim;

       for (i = 0;i < numProberead; i++)
           LogFileAllData<<DAQ.analogRawInputVoltages[i]<<Delim;

       for (i = 0; i<16; i++)
           LogFileAllData<<current_EEpose[i]<<Delim;

       LogFileAllData<<Robotmotionsuccess<<Delim;

       for (i = 0; i<2; i++)
           LogFileAllData<<Field_command_validation[i]<<Delim;

       LogFileAllData<<Field_command_validation[2];

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
                LogFileAllData<<"robotMsdPos_m_"<<i+1<<Delim;

            for (i = 0; i < numProbePos; i++)
                LogFileAllData<<"tableCmdPos_m_"<<i+1<<Delim;


            for (i = 0;i < numAct; i++)
                LogFileAllData<<"cmdCoilCurrent_A_"<<i+1<<Delim;

            for (i = 0;i < numAct; i++)
                LogFileAllData<<"msdCoilCurrent_A_"<<i+1<<Delim;

            for (i = 0;i < numProberead; i++)
                LogFileAllData<<"Daqraw_v"<<i+1<<Delim;

            for (i = 0; i<16; i++)
                LogFileAllData<<"EEpose"<<i<<Delim;

            LogFileAllData<<"Robotmotionsuccess"<<Delim;


            for (i = 0; i<2; i++)
                LogFileAllData<<"Field_command"<<i+1<<Delim;

            LogFileAllData<<"Field_command 3";

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
            //update I_command for logger here
            cmdCoilCurrent[i] = I_command[i];
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
            //update I_command for logger here
            cmdCoilCurrent[i] = 0.0;
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
        // Calculate the necessary current setpoints based on the desired global field
        if (isGradientControlled)
        {
            // Consider Gradient components here:
            // N is numField+numGrad x numAct
            cv::invert(cv::Mat(numField+numGrad,numAct,CV_64F,N),cv::Mat(numAct,numField+numGrad,CV_64F,invN),cv::DECOMP_SVD); // Singular Value Decomposition is slowest but it can handle nonsquare
            // This would have been made into a simpler function but I suck at pointers and functions of matrices
            // It currently makes 2 matrices from the values of N and invN, takes the inverse of N and stores it in invN

            // Multiply the inverse matrix with the desired field and the resulting vector is the necessary currents
            // If the desired fields result in currents that are larger than what is possible to generate, then
            // all currents will need to be scaled down by the same factor.
            double maxCurrentBuf = 0.0;
            // Explicit matrix multiplication here
            for (int i = 0; i < numAct; i++)
            {
                double sum = 0.0;
                for ( int j = 0; j < numField+numGrad; j++)
                {
                    sum += invN[i][j]*B_Global_Desired[j];
                }
                currentSetpoints[i] = sum;
                // Check that none of the currents are above the max by finding the max and scaling all down accordingly afterwards.
                if ( abs(currentSetpoints[i]) > maxCurrentBuf)
                {
                    maxCurrentBuf = abs(currentSetpoints[i]);
                }
            }
            // Scale down all if necessary
            // Currents are normalized and fall on a scale between 0-1 representing currents
            // from zero to max current (0 A - 25 A)
            if (maxCurrentBuf > 1.0)
            {
                double scalingFactor = 1.0/maxCurrentBuf;
                for (int i = 0; i < numAct; i++)
                {
                    // Multiply all setpoints by (positive and less than unity) scaling factor
                    currentSetpoints[i] *= scalingFactor;
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }
            else
            {
                for (int i = 0; i < numAct; i++)
                {
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }

        }
        else
        {
            // Don't worry about gradients:
            // M is numField x numAct
            cv::invert(cv::Mat(numField,numAct,CV_64F,M),cv::Mat(numAct,numField,CV_64F,pseudoinvM),cv::DECOMP_SVD); // Singular Value Decomposition is slowest but it can handle nonsquare
            // This would have been made into a simpler function but I suck at pointers and functions of matrices
            // It currently makes 2 matrices from the values of M and pseudoinvM, takes the inverse of M and stores it in pseudoinvM

            // Multiply the inverse matrix with the desired field and the resulting vector is the necessary currents
            double maxCurrentBuf = 0.0;
            for (int i = 0; i < numAct; i++)
            {
                double sum = 0.0;
                for ( int j = 0; j < numField; j++)
                {
                    sum += pseudoinvM[i][j]*B_Global_Desired[j];
                }
                currentSetpoints[i] = sum;
                // Check that none of the currents are above the max by finding the max and scaling all down accordingly afterwards.
                if ( abs(currentSetpoints[i]) > maxCurrentBuf)
                {
                    maxCurrentBuf = abs(currentSetpoints[i]);
                }
            }
            // Scale down all currents if necessary (if max required current is > max allowable current)
            // And scale up normalized currents to theiir actual values.
            if ( maxCurrentBuf > 1.0 )
            {
                double scalingFactor = 1.0/maxCurrentBuf;
                for (int i = 0; i < numAct; i++)
                {
                    // Multiply all setpoints by (positive and less than unity) scaling factor
                    currentSetpoints[i] *= scalingFactor;
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }
            else
            {
                for (int i = 0; i < numAct; i++)
                {
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }
        }


        // No matter the state, find the theoretical magnetic field produced by the setpoint currents
        for (int k = 0; k < numField+numGrad; k++)
        {
            double sum = 0.0;
            for (int l = 0; l < numAct; l++)
            {
                sum += N[k][l]*currentSetpoints[l]/maxCurrent;
            }
            B_Global_Output[k] = sum;
        }
        // Send current setpoints as normal
        // Need to convert to voltages first

        // TODO convert amps to volts
        std::cout<< "wrote current " ;
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = currentSetpoints[i] * currentControlAdj[i]; // Voltage = (Amps) * (Volts/Amp)
            // TODO limit voltages sent to S826
            std::cout<<i<<": "<<currentSetpoints[i] <<",\t";
        }
        std::cout<< std::endl;

    }
    else
    {
        // Only send 0s to the currents to turn them off because the system is overheating!
//        clearCurrentsSlot();
        // calling clear currents make an inf loop
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
            std::cerr << "Currents Clearing for coil "<<i+1<<std::endl;
        }


    }

    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
//        qInfo() << "Wrote values to the S826.";
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
        {
            robotdata[i][registerdataCount] = Robot_tip_position[i];
            tabledata[i][registerdataCount] = Robot_tip_position[i]; //tempory set same value as the robot data
        }
        robotdata[3][registerdataCount] = 1;
        std::cout<<"register data collected....["<<registerdataCount<<"/"<<registerdataNum<<"]...Robot tip: "<<Robot_tip_position[0]<<" "<<Robot_tip_position[1]<<" "<<Robot_tip_position[2]<<std::endl;
//        if(registerdataCount==registerdataNum)
//        {
//            qInfo()<<"register data collect finished...calculating registration matrix...";
//            registration(tabledata, robotdata);
//        }
        MainWindow::Record();

    }
    else
    {
        qInfo()<<"register is done...";
    }
}

void MainWindow::calibratesetflag(void)
{
    CalibrationDataCollet_Random = true;
}

void MainWindow::calibratesetflagoff(void)
{
    CalibrationDataCollet_Random = false;
}


void MainWindow::calibratesetflag_sequence(void)
{
    CalibrationDataCollet_sequence = true;
}

void MainWindow::calibratesetflagoff_sequence(void)
{
    CalibrationDataCollet_sequence = false;
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
    const auto DNNmodel = fdeep::load_model("C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/fdeep_model.json"); //no normalization layer model
    //std::cout<<"load model!"<<std::endl;

    int inputsize = 92;
    std::vector<float> input = {};

    input = DNNinputprepare(NN_B1, NN_P1, NN_B2, NN_P2);

    const auto result = DNNmodel.predict(
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

    std::vector<double> sample = { 2.24063043e+01, -1.48057763e+01, -6.44080494e+00,  6.99414861e-03,
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

    NN_P1[0] = ui->lineEdit_dnnInput_p1x->text().toDouble()*0.001;
    NN_P1[1] = ui->lineEdit_dnnInput_p1y->text().toDouble()*0.001;
    NN_P1[2] = ui->lineEdit_dnnInput_p1z->text().toDouble()*0.001;
    NN_P2[0] = ui->lineEdit_dnnInput_p2x->text().toDouble()*0.001;
    NN_P2[1] = ui->lineEdit_dnnInput_p2y->text().toDouble()*0.001;
    NN_P2[2] = ui->lineEdit_dnnInput_p2z->text().toDouble()*0.001;

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

    std::cout<< "command position in robot frame is: "<<pos_robot(0)<<", "<<pos_robot(1)<<", "<<pos_robot(2)<<std::endl;

//    //--we use liborl instead libfranka
//    try{
//        const double execution_time = 2.0;
//        orl::Robot robot(fci_ip); // IP-Address or hostname of the robot
//        Pose goal_pose({pos_robot(0),pos_robot(1),pos_robot(2)}, {-0,0,0});
//        auto pose_generator = PoseGenerators::MoveToPose(goal_pose);
//        apply_speed_profile(pose_generator, SpeedProfiles::QuinticPolynomialProfile());
//        robot.move_cartesian(pose_generator, execution_time);

//    }catch (const franka::Exception& e) {
//    std::cout << e.what() << std::endl;
//    std::cout << "Running error recovery..." << std::endl;
//    Robotmotionsuccess = 0;
//    franka::Robot robot(fci_ip);
//    robot.automaticErrorRecovery();
//    }

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
          Robot_tip_position[0] = current_EEpose[12];
          Robot_tip_position[1] = current_EEpose[13];
          Robot_tip_position[2] = current_EEpose[14];

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
            std::cout << "Moving to P1 Finished!" << std::endl;
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
          Robot_tip_position[0] = current_EEpose[12];
          Robot_tip_position[1] = current_EEpose[13];
          Robot_tip_position[2] = current_EEpose[14];

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
            std::cout << "Moving to P2 Finished!" << std::endl;
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

void MainWindow::initializeFranka(void)
{
    franka::Robot robot(fci_ip);
    try{

        setDefaultBehavior(robot);
        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
    //    std::array<double, 7> q_goal = {{-0.0690753, -0.0159581, -0.00171238, -1.94264, -0.0153294, 1.91711, 0.724667}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration in callback." << std::endl;
        robotinitialized = true;
    }catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::cout << "Running error recovery..." << std::endl;
    Robotmotionsuccess = 0;
    robot.automaticErrorRecovery();
    }
}


void MainWindow::runGlobalfield(void)
{
    B_Global_Desired[0] = ui->lineEdit_Global_Bx->text().toDouble()/1000.0; //mT to T
    B_Global_Desired[1] = ui->lineEdit_Global_By->text().toDouble()/1000.0;
    B_Global_Desired[2] = ui->lineEdit_Global_Bz->text().toDouble()/1000.0;

    for (double cc : B_Global_Desired)
        std::cout<<cc<<" ";
    std::cout<<std::endl;
    updateCurrents();

  /*  mymodel.useOffset(true);

    std::string calibration_1 = "calibrate_file1";

    std::vector<MagneticMeasurement> dataList;

//    Eigen::Vector3d Field; *< The measured Field in Tesla. A 3x1 Vector*
//    Eigen::Vector3d Position;*< The position of the measurement in meters. A 3x1 Vector *
//    Eigen::VectorXd AppliedCurrentVector; *< The applied current vector in Amps. A Nx1 Vector *

    Eigen::Vector3d Field = Eigen::Vector3d(1,-3,0.5);
    Eigen::Vector3d Position = Eigen::Vector3d(1,-3,0.5);
    Eigen::VectorXd AppliedCurrentVector {{1,-3,0.5,9,2,3,4,5}};

    struct MagneticMeasurement data
    (
        Field, *< The measured Field in Tesla. A 3x1 Vector*
        Position,*< The position of the measurement in meters. A 3x1 Vector *
        AppliedCurrentVector *< The applied current vector in Amps. A Nx1 Vector *
    );


    dataList.push_back(data);

    data.Field = Eigen::Vector3d(3,-6,-9.5);
    data.Position = Eigen::Vector3d(2,-2,4.5);
    for(int i=0; i<8; i++)
        data.AppliedCurrentVector(i) = i+2.2;
    dataList.push_back(data);

    mymodel.calibrate(calibration_1, dataList);
//    mymodel.writeCalibration("calibratedfile.yaml");*/



        std::cout<<"B_Global_Desired: ";
}


void MainWindow::Cartesiantest(void)
{

   // set desired orientation of EE in robot base frame
    Eigen::Matrix3d abs_robotOrient_y;
    abs_robotOrient_y << 1.0,  0.0,  0.0,
                       0.0,  0.0,  1.0,
                       0.0,  -1.0, 0.0;
    Eigen::Matrix3d abs_robotOrient_ny;
    abs_robotOrient_ny << 1.0,  0.0,  0.0,
                       0.0,  0.0,    -1.0,
                       0.0,  1.0,     0.0;

    Eigen::Matrix3d abs_robotOrient_x;
    abs_robotOrient_x <<0.0,  0.0,  1.0,
                        0.0,  -1.0, 0.0,
                        1.0,  0.0,  0.0;
    Eigen::Matrix3d abs_robotOrient_nx;
    abs_robotOrient_nx <<0.0,  0.0,  -1.0,
                         0.0,  -1.0, 0.0,
                         -1.0,  0.0,  0.0;
    Eigen::Matrix3d abs_robotOrient_nz;
    abs_robotOrient_nz <<1.0,  0.0, 0.0,
                        0.0,  -1.0, 0.0,
                        0.0,  0.0,  -1.0;

    FrankaAbsOrientmotion(abs_robotOrient_ny);

  //  try {
//
            /*using namespace orl;
            const double execution_time = 2.0;
            Robot robot(fci_ip);
            Pose current_pose =robot.get_current_pose();

            Position current_position = current_pose.getPosition();

            Pose goal_pose({current_position[0],current_position[1],current_position[2]}, {-M_PI,0,0});
            auto pose_generator = PoseGenerators::MoveToPose(goal_pose);
            apply_speed_profile(pose_generator, SpeedProfiles::QuinticPolynomialProfile());
            robot.move_cartesian(pose_generator, execution_time);
*/

//            robot.absolute_cart_motion(0.4, 0.15, 0.20, execution_time, StopConditions::Force(5));


//            auto pose_generator = PoseGenerators::AbsoluteMotion({0.5,0,0.3});
//            apply_speed_profile(pose_generator, SpeedProfiles::QuinticPolynomialProfile());
//            robot.move_cartesian(pose_generator, execution_time);

//            orl::Robot robot(fci_ip);
//            std::cout << "WARNING: This example will move the robot! "
//                      << "Please make sure to have the user stop button at hand!" << std::endl
//                      << "Double tap the robot to continue..." << std::endl;
//            robot.double_tap_robot_to_continue();
//            std::array<double, 7> q_goal = {{0.542536, 0.258638, -0.141972, -1.99709, -0.0275616, 2.38391, 1.12856}};
//            robot.joint_motion(q_goal, 0.2);

//            orl::Pose pregrasp_pose(robot.get_current_pose());
//            pregrasp_pose.set_position(0.4, 0.15, 0.20);
//            pregrasp_pose.set_RPY(-M_PI, 0, 0);

//            orl::Pose move_pose(robot.get_current_pose());
//            move_pose.set_position(0.7, 0.15, 0.20);
//            move_pose.set_RPY(-M_PI, 0, 0);


//            robot.cart_motion(pregrasp_pose, 5);
//            robot.cart_motion(move_pose, 5, orl::StopConditions::Force(5));
//            robot.cart_motion(pregrasp_pose, 5);


//            robot.joint_motion(q_goal, 0.2);

//            const double execution_time = 2.0;
//            Robot franka("franka"); // IP-Address or hostname of the robot
//            franka.absolute_cart_motion({0.5,0,0.3}, execution_time);

//            const double execution_time = 2.0;
//            Robot franka("franka"); // IP-Address or hostname of the robot
//            Pose goal_pose({0,2,3}, {-M_PI,0,0});
//            franka.move_cartesian(goal_pose, execution_time);

//        } catch (const franka::Exception &e) {
//            std::cout << e.what() << std::endl;
//        }
}

void MainWindow::setFrankaguidingmode(void)
{
    franka::Robot robot(fci_ip);
    // For Enabling or disabling the gamepad or game controller.
    bool FrankaGuidieState = ui->checkBox_guidingmode->checkState();

    // Enable the controller if the checkbox is checked
    if (FrankaGuidieState)
    {

        try{

            setDefaultBehavior(robot);
            robot.setGuidingMode({true, true, true, true, true, true}, false);
            std::cout<<"set franka guide mode"<<std::endl;

        }catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        Robotmotionsuccess = 0;
        robot.automaticErrorRecovery();
        }
    }
    else
    {
        try{

            setDefaultBehavior(robot);
            robot.setGuidingMode({false, false, false, false, false, false}, false);

        }catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        Robotmotionsuccess = 0;
        robot.automaticErrorRecovery();
        }
    }
}


void MainWindow::initialProbeOrient(void)
{
    Eigen::Matrix3d Probeintable;
    Probeintable << 0.0,  -1.0,  0.0,
                    -1.0,  0.0,  0.0,
                    0.0,  0.0,  -1.0;
    Eigen::Matrix3d rotT2R;
    rotT2R = transT2R.block<3,3>(0,0);  //	 matrix.block<p,q>(i,j);
    Eigen::Matrix3d ProbeinRobot;
    //desired initial Probe orient in robot frame
    ProbeinRobot = rotT2R*Probeintable;
    std::cout<<"ProbeinRobot: "<<std::endl <<ProbeinRobot<<std::endl;
    //now control franka to rotate to this Probe orient
    FrankaAbsOrientmotion(ProbeinRobot);

//    FrankaAbsOrientmotion_stepsolution(ProbeinRobot);
}


void MainWindow::initialProbeOrient_test(void)
{
    Eigen::Matrix3d Probeintable;
    Probeintable << 0.0,  -1.0,  0.0,
                    -1.0,  0.0,  0.0,
                    0.0,  0.0, -1.0;
    Eigen::Matrix3d rotT2R;
    rotT2R = transT2R.block<3,3>(0,0);  //	 matrix.block<p,q>(i,j);
    Eigen::Matrix3d ProbeinRobot;
    //desired initial Probe orient in robot frame
    ProbeinRobot = rotT2R*Probeintable;
    std::cout<<"ProbeinRobot: " <<std::endl<<ProbeinRobot<<std::endl;
    //now control franka to rotate to this Probe orient
    //transform rotation to euler angle
    Eigen::Vector3d desired_eulerangle = ProbeinRobot.eulerAngles(0, 1, 2); //x, y, z

    std::cout<<"desired_eulerangle: "<<desired_eulerangle<<std::endl;

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

          double tolerance = 0.003; //0.001rad -> 0.057degree
          double error[3] = {0.0, 0.0, 0.0};
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position_d(initial_transform.translation());
      //    position_d = position_d*1000;//from m to mm
      //    Eigen::Quaterniond orientation_d(initial_transform.linear());
          Eigen::Matrix3d measured_rotation(initial_transform.linear());


          //calculate pose difference
          Eigen::Matrix3d pose_diff = measured_rotation.inverse()*ProbeinRobot;

          Eigen::Vector3d eulerangle_diff = pose_diff.eulerAngles(0,1,2); //x,y,z

//          Eigen::Vector3d measured_eulerangle = rotation.eulerAngles(2, 1, 0);
//          std::cout<<"measured_eulerangle: "<<measured_eulerangle<<std::endl;

          for(int k=0; k<3; k++)
          {
              double temp_e = eulerangle_diff[k];

              if(M_PI-abs(temp_e)<=tolerance || abs(temp_e)<=tolerance )
              {
                  error[k] = 0.0;
                  direction[k] = 0.0;
              }
              else if(temp_e<=M_PI_2 &&temp_e>0)
              {
                  error[k] = temp_e;
                  direction[k] = 1.0;
              }
              else if(temp_e<M_PI &&temp_e>M_PI_2)
              {
                  error[k] = temp_e-M_PI;
                  direction[k] = -1.0;
              }
              else if (temp_e>-M_PI_2 && temp_e<0)
              {
                  error[k] = temp_e;
                  direction[k] = -1.0;
              }
              else if (temp_e>-M_PI && temp_e<-M_PI_2)
              {
                  error[k] = M_PI+temp_e;
                  direction[k] = 1.0;
              }

           /*   if (temp_e>M_PI)
              {
                 direction[k] = -1.0;
                 error[k] = M_PI - temp_e;
              }
              else if (abs(temp_e-M_PI)<tolerance)
              {
                  error[k] = 0.0;
                  direction[k] = 0.0;
              }
              else
              {
                  error[k] = temp_e;
                  direction[k] = 1.0;
              }

              if (abs(temp_e)>=tolerance)
                  error[k] = temp_e;
              else
              {
                  error[k] = 0.0;
                  direction[k] = 0.0;
              }*/

          }

          std::cout<<"diff_eulerangle x-y-z: "<<eulerangle_diff[0]<< " "<<eulerangle_diff[1]<< " " <<eulerangle_diff[2]<<std::endl;
          std::cout<<"error x-y-z : "<<error[0] <<" "<<error[1]<<" "<<error[2] <<std::endl;

          double max_omega = 0.030;  //rad/s
          double maxDelt_e = 0.1; // 1rad = 57degree

//          double A = maxv/(maxDelt_e*maxDelt_e);
          double v_cmd[3] = {0.0};

          for (int k=0; k<3; k++)
          {
              if(abs(error[k])<=maxDelt_e)
                   v_cmd[k] = max_omega*sin((M_PI/2)*(error[k]/maxDelt_e));
//                                      v_cmd[k] = A*pow(error[k],2);
              else
                  v_cmd[k] = max_omega*direction[k];
          }
          std::cout<<"v_cmd: "<<v_cmd[0] <<" "<<v_cmd[1]<<" "<<v_cmd[2] <<std::endl<<std::endl<<std::endl;


//         franka::CartesianVelocities output = {{direction[0]*v_x, direction[1]*v_y, direction[2]*v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, v_cmd[0], v_cmd[1], v_cmd[2]}};

//          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,}};
          if (abs(error[0])<tolerance && abs(error[1])<tolerance && abs(error[2])<tolerance) {
            std::cout << "Orientation initialized Finished" << std::endl;
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

    //read franka robot pose
    franka::RobotState initial_state = robot.readOnce();
    // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Matrix3d rotation(initial_transform.linear());

    std::cout<<"ProbeinRobot_desired: " <<std::endl<<ProbeinRobot<<std::endl;
    std::cout<<"ProbeinRobot_actual: " <<std::endl<<rotation<<std::endl;
}


void MainWindow::FrankaAbscartmotion(double abs_robotpos[3])
{
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
          Robot_tip_position[0] = current_EEpose[12];
          Robot_tip_position[1] = current_EEpose[13];
          Robot_tip_position[2] = current_EEpose[14];

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

//          double v_x = 0.002;
//          double v_y = 0.002;
//          double v_z = 0.002;
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
            std::cout << "single motion Finished: ["<<robotmovecount+1<<"/100 ]" << std::endl;
            std::cout << "motion error is "<<error[0]<<" " <<error[1]<<" "<<error[2]<<std::endl;
            output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            Robotmotionsuccess = 1;
            robotmovecount++;
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


void MainWindow::FrankaRelativecartmotion(double relative_robotpos[3])
{
    std::cout<< "command relative position in robot frame is: "<<relative_robotpos[0]<<", "<<relative_robotpos[1]<<", "<<relative_robotpos[2]<<std::endl;

    //move robot in relative position
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
          double tolerance = 0.0005; //1mm
          double error[3];
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

          //default unit is meter and rad, so we keep that
          Robot_tip_position[0] = current_EEpose[12];
          Robot_tip_position[1] = current_EEpose[13];
          Robot_tip_position[2] = current_EEpose[14];

//                              qInfo() << "command position in robot frame is: "<<abs_robotpos[0]<<", "<<abs_robotpos[1]<<", "<<abs_robotpos[2];
//                              qInfo() << "current robot position is: "<<current_pose[12]<<", "<<current_pose[13]<<", "<<current_pose[14];
          for(int k=0; k<3; k++)
          {
              double temp_e = relative_robotpos[k];
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

//          double v_x = 0.002;
//          double v_y = 0.002;
//          double v_z = 0.002;
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
            std::cout << "single motion Finished: ["<<robotmovecount+1<<"/100 ]" << std::endl;
            std::cout << "motion error is "<<error[0]<<" " <<error[1]<<" "<<error[2]<<std::endl;
            output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            Robotmotionsuccess = 1;
            robotmovecount++;
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


//does not work, because eigen library does not output right euler angles
void MainWindow::FrankaAbsOrientmotion_test(Eigen::Matrix3d abs_robotOrient)
{
    std::cout<<"ProbeinRobot: " <<abs_robotOrient<<std::endl;
    //now control franka to rotate to this Probe orient
    //transform rotation to euler angle
//    Eigen::Vector3d desired_eulerangle = ProbeinRobot.eulerAngles(2, 1, 0); //z, y, x

//    std::cout<<"desired_eulerangle: "<<desired_eulerangle<<std::endl;

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

          double tolerance = 0.001; //rad -> 0.057degree
          double error[3] = {0.0, 0.0, 0.0};
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position_d(initial_transform.translation());
      //    position_d = position_d*1000;//from m to mm
      //    Eigen::Quaterniond orientation_d(initial_transform.linear());
          Eigen::Matrix3d measured_rotation(initial_transform.linear());


          //calculate pose difference
          Eigen::Matrix3d pose_diff = measured_rotation.inverse()*abs_robotOrient;

          Eigen::Vector3d eulerangle_diff = pose_diff.eulerAngles(2,1,0);

//          Eigen::Vector3d measured_eulerangle = rotation.eulerAngles(2, 1, 0);
//          std::cout<<"measured_eulerangle: "<<measured_eulerangle<<std::endl;

          for(int k=0; k<3; k++)
          {
              double temp_e = eulerangle_diff[k];

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

          std::cout<<"diff_eulerangle x-y-z: "<<eulerangle_diff[2]<< " "<<eulerangle_diff[1]<< " " <<eulerangle_diff[0]<<std::endl;
          std::cout<<"error x-y-z : "<<error[2] <<" "<<error[1]<<" "<<error[0] <<std::endl;

          double max_omega = 0.050;  //rad/s
          double maxDelt_e = 0.1; // 1rad = 57degree

//          double A = maxv/(maxDelt_e*maxDelt_e);
          double v_cmd[3] = {0.0};

          for (int k=0; k<3; k++)
          {
              if(abs(error[k])<=maxDelt_e)
                   v_cmd[k] = max_omega*sin((M_PI/2)*(error[k]/maxDelt_e));
//                                      v_cmd[k] = A*pow(error[k],2);
              else
                  v_cmd[k] = max_omega*direction[k];
          }
          std::cout<<"v_cmd: "<<v_cmd[2] <<" "<<v_cmd[1]<<" "<<v_cmd[0] <<std::endl<<std::endl<<std::endl;


//         franka::CartesianVelocities output = {{direction[0]*v_x, direction[1]*v_y, direction[2]*v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, v_cmd[2], v_cmd[1], v_cmd[0]}};

//          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,}};
          if (abs(error[0])<tolerance && abs(error[1])<tolerance && abs(error[2])<tolerance) {
            std::cout << "Orientation initialized Finished" << std::endl;
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



void MainWindow::FrankaAbsOrientmotion(Eigen::Matrix3d abs_robotOrient)
{
    std::cout<<"desired EE in Robot base frame: " <<abs_robotOrient<<std::endl;
    //now control franka to rotate to this Probe orient
    //transform rotation to euler angle
//    Eigen::Vector3d desired_eulerangle = ProbeinRobot.eulerAngles(2, 1, 0); //z, y, x

//    std::cout<<"desired_eulerangle: "<<desired_eulerangle<<std::endl;

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

          double tolerance = 0.001; //rad -> 0.057degree
          double error[3] = {0.0, 0.0, 0.0};
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position_d(initial_transform.translation());
      //    position_d = position_d*1000;//from m to mm
      //    Eigen::Quaterniond orientation_d(initial_transform.linear());
          Eigen::Matrix3d measured_rotation(initial_transform.linear());
//         std::cout<<"measured EE in Robot base frame: "<<std::endl <<measured_rotation<<std::endl;

          //calculate pose difference
          Eigen::Matrix3d pose_diff = abs_robotOrient*measured_rotation.transpose(); //we rotate around fixed base frame, so we pre-multiply

//          Eigen::Vector3d eulerangle_diff = pose_diff.eulerAngles(2,1,0);

          //calculate pose difference euler angle
          std::array<double, 3> eulerangle_diff = RotMat2EulerAngle(pose_diff);

//          double eulerangle_diff[3] = {thetaX,thetaY,thetaZ};

//          Eigen::Vector3d measured_eulerangle = rotation.eulerAngles(2, 1, 0);
//          std::cout<<"measured_eulerangle: "<<measured_eulerangle<<std::endl;

          for(int k=0; k<3; k++)
          {
              double temp_e = eulerangle_diff[k];

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

          std::cout<<"diff_eulerangle x-y-z: "<<eulerangle_diff[0]<< " "<<eulerangle_diff[1]<< " " <<eulerangle_diff[2]<<std::endl;
          std::cout<<"error x-y-z : "<<error[0] <<" "<<error[1]<<" "<<error[2] <<std::endl;

          double max_omega = 0.050;  //rad/s
          double maxDelt_e = 0.1; // 1rad = 57degree

//          double A = maxv/(maxDelt_e*maxDelt_e);
          double v_cmd[3] = {0.0};

          for (int k=0; k<3; k++)
          {
              if(abs(error[k])<=maxDelt_e)
                   v_cmd[k] = max_omega*sin((M_PI/2)*(error[k]/maxDelt_e));
//                                      v_cmd[k] = A*pow(error[k],2);
              else
                  v_cmd[k] = max_omega*direction[k];
          }
          std::cout<<"v_cmd: "<<v_cmd[0] <<" "<<v_cmd[1]<<" "<<v_cmd[2] <<std::endl<<std::endl<<std::endl;


//         franka::CartesianVelocities output = {{direction[0]*v_x, direction[1]*v_y, direction[2]*v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, v_cmd[0], v_cmd[1], v_cmd[2]}};

//          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,}};
          if (abs(error[0])<tolerance && abs(error[1])<tolerance && abs(error[2])<tolerance) {
            std::cout << "Orientation initialized Finished" << std::endl;
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

std::array<double, 3>  MainWindow::RotMat2EulerAngle(Eigen::Matrix3d RotMat)
{
    //R = rotx*roty*rotz
    double r02 = RotMat(0,2);
    double r10 = RotMat(1,0);
    double r11 = RotMat(1,1);
    double r12 = RotMat(1,2);
    double r00 = RotMat(0,0);
    double r01 = RotMat(0,1);
    double r22 = RotMat(2,2);
    double thetaY = 0.0;
    double thetaX = 0.0;
    double thetaZ = 0.0;
    if( r02<1.0)
    {
      if ( r02>-1.0)
      {
          thetaY = asin(r02);
          thetaX = atan2 (-1.0*r12, r22 ) ;
          thetaZ = atan2 (-1.0*r01, r00 ) ;
    }
      else // r 0 2 = −1
      {
      // Not a u n i q u e s o l u t i o n : t h e t aZ − t he t aX = a t a n 2 ( r10 , r 1 1 )
          thetaY = -M_PI_2;
          thetaX = -atan2(r10, r11) ;
          thetaZ = 0 ;
       }
    }
    else // r 0 2 = +1
    {
    // Not a u n i q u e s o l u t i o n : t h e t aZ + t he t aX = a t a n 2 ( r10 , r 1 1 )
          thetaY = M_PI_2;
          thetaX = atan2(r10, r11) ;
          thetaZ = 0 ;
    }

    std::array<double, 3> eulerangle = {thetaX,thetaY,thetaZ};
    return eulerangle;
}

void MainWindow::FrankaAbsOrientmotion_stepsolution(Eigen::Matrix3d abs_robotOrient)
{
    std::cout<<"desired EE in Robot base frame: " <<abs_robotOrient<<std::endl;
    //calculate desired pose euler angle
    std::array<double, 3> eulerangle_desired = RotMat2EulerAngle(abs_robotOrient);
    std::cout<<"desired EE euler angle in Robot base frame: " <<eulerangle_desired[0]<<" "<<eulerangle_desired[1]<<" "<<eulerangle_desired[2]<<std::endl;
    //now control franka to rotate to this Probe orient
    //transform rotation to euler angle
//    Eigen::Vector3d desired_eulerangle = ProbeinRobot.eulerAngles(2, 1, 0); //z, y, x

//    std::cout<<"desired_eulerangle: "<<desired_eulerangle<<std::endl;

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

          double tolerance = 0.001; //rad -> 0.057degree
          double error[3] = {0.0, 0.0, 0.0};
          double direction[3];
          current_EEpose = robot_state.O_T_EE; //not sure whether should use _d

         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position_d(initial_transform.translation());
      //    position_d = position_d*1000;//from m to mm
      //    Eigen::Quaterniond orientation_d(initial_transform.linear());
          Eigen::Matrix3d measured_rotation(initial_transform.linear());
//         std::cout<<"measured EE in Robot base frame: "<<std::endl <<measured_rotation<<std::endl;

          //calculate pose difference
          Eigen::Matrix3d pose_diff = measured_rotation.inverse()*abs_robotOrient;

          //calculate current pose euler angle
          std::array<double, 3> eulerangle_current = RotMat2EulerAngle(measured_rotation);

//          Eigen::Vector3d eulerangle_diff = pose_diff.eulerAngles(2,1,0);

          std::array<double, 3> eulerangle_diff;
          for(int m=0; m<3; m++)
            eulerangle_diff[m] = eulerangle_desired[m] - eulerangle_current[m];

//          double eulerangle_diff[3] = {thetaX,thetaY,thetaZ};

//          Eigen::Vector3d measured_eulerangle = rotation.eulerAngles(2, 1, 0);
//          std::cout<<"measured_eulerangle: "<<measured_eulerangle<<std::endl;

          for(int k=0; k<3; k++)
          {
              double temp_e = eulerangle_diff[k];

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

          std::cout<<"diff_eulerangle x-y-z: "<<eulerangle_diff[0]<< " "<<eulerangle_diff[1]<< " " <<eulerangle_diff[2]<<std::endl;
          std::cout<<"error x-y-z : "<<error[0] <<" "<<error[1]<<" "<<error[2] <<std::endl;

          double max_omega = 0.050;  //rad/s
          double maxDelt_e = 0.1; // 1rad = 57degree

//          double A = maxv/(maxDelt_e*maxDelt_e);
          double v_cmd[3] = {0.0};

          for (int k=0; k<3; k++)
          {
              if(abs(error[k])<=maxDelt_e)
                   v_cmd[k] = max_omega*sin((M_PI/2)*(error[k]/maxDelt_e));
//                                      v_cmd[k] = A*pow(error[k],2);
              else
                  v_cmd[k] = max_omega*direction[k];
          }
          std::cout<<"v_cmd: "<<v_cmd[0] <<" "<<v_cmd[1]<<" "<<v_cmd[2] <<std::endl<<std::endl<<std::endl;


//         franka::CartesianVelocities output = {{direction[0]*v_x, direction[1]*v_y, direction[2]*v_z, 0.0, 0.0, 0.0}};
          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, v_cmd[0], v_cmd[1], v_cmd[2]}};

//          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,}};
          if (abs(error[0])<tolerance && abs(error[1])<tolerance && abs(error[2])<tolerance) {
            std::cout << "Orientation initialized Finished" << std::endl;
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


void MainWindow::ReadFrankaPoseStatus(void)
{

    franka::Robot robot(fci_ip);
    //read franka robot pose
    franka::RobotState initial_state = robot.readOnce();
    // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    current_EEpose = initial_state.O_T_EE;
    Eigen::Vector3d position_d(initial_transform.translation());
//    position_d = position_d*1000;//from m to mm
//    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d rotation(initial_transform.linear());
    Eigen::Vector3d eulerangle = rotation.eulerAngles(0, 1, 2);

    Eigen::Affine3d EEinFlange(Eigen::Matrix4d::Map(initial_state.F_T_EE.data()));
    Eigen::Vector3d EEinFpos(EEinFlange.translation());
//    EEinFpos = EEinFpos*1000; //from m to mm
//    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d EEinFrot(EEinFlange.linear());
    Eigen::Vector3d EEinFeulerangle = EEinFrot.eulerAngles(0, 1, 2);

    //default units are meter and rad, so we keep that
    Robot_tip_position[0] = position_d[0];
    Robot_tip_position[1] = position_d[1];
    Robot_tip_position[2] = position_d[2];

    Robot_orient[0] = eulerangle[0];
    Robot_orient[1] = eulerangle[1];
    Robot_orient[2] = eulerangle[2];

    current_EEpose = initial_state.O_T_EE;

    for(int i=0; i<7; i++){
        Robot_joint[i] = initial_state.q.data()[i];
    }
}

vector<vector<double>> MainWindow::readCSVfile(string filename)
{
     vector<vector<double>> CSVcontent;

    cout<<"File name is "<<filename<<std::endl;

//    vector<string> row;
    vector<double> row;
    string line, word;
    bool headflag = true;

    ifstream file (filename, ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            if(headflag==true) //get rid of table head
            {
                headflag = false;
            }
            else
            {
                row.clear();

                stringstream str(line);

                while(getline(str, word, ','))
                    row.push_back(stod(word));
                CSVcontent.push_back(row);
            }
        }
        std::cout<<"data dimension is: "<<CSVcontent.size() <<" x " <<CSVcontent[0].size() <<std::endl;
    }
    else
        cout<<"Could not open the file\n";

    return CSVcontent;

}

void MainWindow::CalibrateCoiltable()
{
//    string fname = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data/coilTableCalibrationData.csv";
//    string fname = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data/coilTableCalibrationData-noOffset.csv";
    string fname = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data/coilTableCalibrationData-onlycoil4.csv";


    //prepare data as b_field(3x1), posistion(3x1), current(8x1)
    //prepare data as b_field(3x1), posistion(3x1), current(1x1) - for single coil test
    vector<vector<double>> Fulldata;
    Fulldata = readCSVfile(fname);
    std::vector< MagneticMeasurement > meas_vec;
//    Eigen::Vector3d b_field;
//    Eigen::Vector3d pos;
//    Eigen::VectorXd cur;

    /*
     *
     *  \brief calibrationDataPoint
     *  \param Field the field value in Tesla
     *  \param Pos the position of the measurement in meters
     *  \param CurrentVec the applied current vector in Amps
     */
//    MagneticMeasurement( const Eigen::Vector3d& Field, const Eigen::Vector3d& Pos, const Eigen::VectorXd& CurrentVec );

    for(int k=0; k<Fulldata.size(); k++)
    {
        std::vector<double> singleline = Fulldata[k];
        Eigen::Vector3d b_field;
        Eigen::Vector3d pos;
//        Eigen::VectorXd cur(8);
        Eigen::VectorXd cur(1); //- for single coil test
        b_field << singleline[0]*0.001, singleline[1]*0.001, singleline[2]*0.001; //unit: Tesla
        pos << singleline[3], singleline[4], singleline[5]; //unit: meter
//        cur << singleline[6], singleline[7], singleline[8], singleline[9], singleline[10], singleline[11], singleline[12], singleline[13]; //unit: Ample
        cur << singleline[6];
        meas_vec.push_back(MagneticMeasurement( b_field, pos, cur ) );
    }

//    std::string initialguess = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess.yaml";
//    std::string initialguess = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess_noOffset.yaml";
//    std::string initialguess = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess_noInterfere.yaml";
      std::string initialguess = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/InitialGuess_onlyCoil4.yaml";

    ElectromagnetCalibration CoiltableModel(initialguess);

    std::cout << "Default has: " << CoiltableModel.getNumberOfCoils() << " Coils." << std::endl;
        for( unsigned int i = 0; i < CoiltableModel.getNumberOfCoils(); i++ )
            std::cout << "Default has Coil " << i << " has : " << CoiltableModel.getNumberOfSources( i ) << " Sources." << std::endl;
//    CoiltableModel.useOffset(false);
    // pass meas_vec to cal
//    CoiltableModel.calibrate( "CoiltableModel_Dec_2022_noOffset", meas_vec, true, true, ElectromagnetCalibration::HEADING_THEN_POSITION, 0.07, 0.6 );
     CoiltableModel.calibrate( "CoiltableModel_Dec_2022_coil4", meas_vec, true, true, ElectromagnetCalibration::HEADING_THEN_POSITION, 0.01, 1.5 ); //0.01m=1cm;0.6m=60cm
//    void calibrate(std::string calibrationName, const std::vector<MagneticMeasurement>& dataList, bool printProgress = true, bool printStats = true, calibration_constraints constraint = HEADING_THEN_POSITION, double minimumSourceToCenterDistance = -1, double maximumSourceToCenterDistance = -1, double converganceTolerance = 1e-12, int maxIterations = 10000, int numberOfConvergedIterations = 1 );

    // write to file...
    CoiltableModel.writeCalibration( "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/CoiltableModel_Dec_2022_coil4.yaml" );


}



void MainWindow::runFullWorkspacefield(void)
{
//    std::string CoilModel = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Franka_Emika_GUI/CoiltableModel_Nov_2022_Nconst-2.yaml";

    /**
     * @brief  loads a new calibration file
     *  @param a string pointing to the location of the yaml formated calbiration file
     */
//    bool loadCalibration(std::string fileName);

    ElectromagnetCalibration CoiltableModel(CoilModel);
//    std::cout << "Default has: " << CoiltableModel.getNumberOfCoils() << " Coils." << std::endl;
    CoiltableModel.useOffset(true);

    Eigen::Vector3d B_FullWorkspace;
    Eigen::VectorXd G_FullWorkspace(numGrad);

//    double P_FullWorkspace[3] = {0.0};
    Eigen::Vector3d P_FullWorkspace;

    //mm to meter
    P_FullWorkspace << ui->lineEdit_Fullworkspace_Px->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Py->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Pz->text().toDouble()/1000.0;

    //mT to T
    B_FullWorkspace << ui->lineEdit_Fullworkspace_Bx->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_By->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Bz->text().toDouble()/1000.0;

    //mT to T
    G_FullWorkspace << ui->lineEdit_Fullworkspace_Gxx->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Gxy->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Gxz->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Gyy->text().toDouble()/1000.0, ui->lineEdit_Fullworkspace_Gyz->text().toDouble()/1000.0;

    Eigen::MatrixXd actuationMatrix = Eigen::MatrixXd::Zero(numField,numAct);

    actuationMatrix = CoiltableModel.fieldCurrentJacobian( P_FullWorkspace );
    /*
     * @brief returns the 3xN matrix mapping field at a point to the current in each of the N sources
     *  @param position is the position in the workspace the field is desired
     *
     *  This function does not check to see if the point is actually in the calibrated workspace.
     */
    //    Eigen::MatrixXd fieldCurrentJacobian( const Eigen::Vector3d& position = Eigen::Vector3d::Zero() ) const;

    Eigen::MatrixXd pinvAMatrix = actuationMatrix.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd demandCurrent;

    demandCurrent = pinvAMatrix*B_FullWorkspace;

    std::cout<<"demand current is" <<demandCurrent<<std::endl<<std::endl;

    double I_command[numAct] = {0.0};
    for(int k=0; k<numAct; k++)
        I_command[k] = demandCurrent[k];
    updateCurrents_CalibrationOnly(I_command);

}



void MainWindow::Fullworkspace_MoveRobot()
{
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_z = 0.0;

    robot_x = ui->lineEdit_Fullworkspace_Px->text().toDouble()/1000.0; //mm to meter
    robot_y = ui->lineEdit_Fullworkspace_Py->text().toDouble()/1000.0;
    robot_z = ui->lineEdit_Fullworkspace_Pz->text().toDouble()/1000.0;

    // move robot to desired position
    std::cout<< "command position in table frame is: "<<robot_x<<", "<<robot_y<<", "<<robot_z<<std::endl;
    //covert cmd position in table frame to robot frame
    Eigen::Vector4d pos_cmd(robot_x, robot_y, robot_z, 1);
    Eigen::Vector4d pos_robot = transT2R*pos_cmd;
    double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};

    // run robot
    FrankaAbscartmotion( abs_robotpos);

}



void MainWindow::runCoilmodel_field(Eigen::Vector3d B_FullWorkspace, Eigen::Vector3d P_FullWorkspace)
{

    ElectromagnetCalibration CoiltableModel(CoilModel);
//    std::cout << "Default has: " << CoiltableModel.getNumberOfCoils() << " Coils." << std::endl;
    CoiltableModel.useOffset(true);

    Eigen::MatrixXd actuationMatrix = Eigen::MatrixXd::Zero(numField,numAct);

    actuationMatrix = CoiltableModel.fieldCurrentJacobian( P_FullWorkspace );
    /*
     * @brief returns the 3xN matrix mapping field at a point to the current in each of the N sources
     *  @param position is the position in the workspace the field is desired
     *
     *  This function does not check to see if the point is actually in the calibrated workspace.
    */
    //    Eigen::MatrixXd fieldCurrentJacobian( const Eigen::Vector3d& position = Eigen::Vector3d::Zero() ) const;

    Eigen::MatrixXd pinvAMatrix = actuationMatrix.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd demandCurrent;

    demandCurrent = pinvAMatrix*B_FullWorkspace;


    std::cout<<"command current is" <<demandCurrent<<std::endl<<std::endl;

//    double I_command[numAct] = {0.0};
    for(int k=0; k<numAct; k++)
        cmdCoilCurrent[k] = demandCurrent[k];
    updateCurrents_CalibrationOnly(cmdCoilCurrent);

}


void MainWindow::Validation_datacollect(void)
{
    ValidationDataCollect_Random = ui->checkBox_validation_datacollect->checkState();
    std::cout<<"Validation data collect is "<<ValidationDataCollect_Random<<std::endl;
}


void MainWindow::Validation_datacollect_pushbutton(void)
{
    ValidationDataCollect_Random = true;
    ValidateData_maxloop = ui->lineEdit_Validate_data_num->text().toInt();
    std::cout<<"Validation data collect is "<<ValidationDataCollect_Random<<std::endl;
}

void MainWindow::UpdateCurrent_fromGUI(void)
{
    double I_command[8] = {0.0};
    I_command[0] =  ui->lineEdit_CurrentCmd_1->text().toDouble();
    I_command[1] =  ui->lineEdit_CurrentCmd_2->text().toDouble();
    I_command[2] =  ui->lineEdit_CurrentCmd_3->text().toDouble();
    I_command[3] =  ui->lineEdit_CurrentCmd_4->text().toDouble();
    I_command[4] =  ui->lineEdit_CurrentCmd_5->text().toDouble();
    I_command[5] =  ui->lineEdit_CurrentCmd_6->text().toDouble();
    I_command[6] =  ui->lineEdit_CurrentCmd_7->text().toDouble();
    I_command[7] =  ui->lineEdit_CurrentCmd_8->text().toDouble();
    if (!overheatingFlag)
    {
        // Send current setpoints as normal
        // Need to convert to voltages first
        std::cout<<"Send currents to coils---"<<std::endl;
        // TODO convert amps to volts
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = I_command[i] * currentControlAdj[i]; // Voltage = (Amps) * (Volts/Amp)
            // TODO limit voltages sent to S826
            //update I_command for logger here
            cmdCoilCurrent[i] = I_command[i];

            std::cout<<I_command[i]<<", ";
        }
        std::cout<<std::endl;
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
            //update I_command for logger here
            cmdCoilCurrent[i] = 0.0;
        }
    }
    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
        qInfo() << "Wrote values to the S826 in updateCurrents_GUI mode";
        std::cout<<std::endl;
    }

}


void MainWindow::currentfeedback(void)
{
    if(ui->checkBox_currentfeedback->checkState())
    {
        currentStreaming = true;
        qInfo()<<"current is streaming";
    }
    else
    {
        currentStreaming = false;

        qInfo()<<"current stops streaming";
    }

}

void MainWindow::FrankaOrientAdjust(void)
{
    double Rotangle[3] = {0.0}; //unit: radian
    Rotangle[0] =  ui->lineEdit_Robot_rotx->text().toDouble();
    Rotangle[1] =  ui->lineEdit_Robot_rotx->text().toDouble();
    Rotangle[2] =  ui->lineEdit_Robot_rotx->text().toDouble();
//    //change unit to radian
//    for (int k=0; k<3; k++) {
//        Rotangle[k] = Rotangle[k]*M_PI/180.0;
//    }

    try {
//        const double execution_time = 2.0;
//        Robot franka(fci_ip); // IP-Address or hostname of the robot
//        franka.absolute_cart_motion(0.5,0,0.3, execution_time);


       franka::Robot robot(fci_ip);

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

        double time_max = 0.5;
        double time = 0.0;
        robot.control([=, &time](const franka::RobotState&,
                                 franka::Duration period) -> franka::CartesianVelocities {
          time += period.toSec();
          double v_x = 0.0;
          double v_y = 0.0;
          double v_z = 0.0;
          double omega_x = Rotangle[0];
          double omega_y = Rotangle[1];
          double omega_z = Rotangle[2];

          franka::CartesianVelocities output = {{ v_x, v_y, v_z, omega_x,omega_y, omega_z }};
          if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion!" << std::endl;
            return franka::MotionFinished(output);
          }
          return output;
        });
      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
//        return -1;
      }


}


void MainWindow::processPendingDatagrams()
 {
    QHostAddress sender;
    uint16_t port;
    if (socket->hasPendingDatagrams())
    {
         QByteArray datagram;
         datagram.resize(socket->pendingDatagramSize());
         socket->readDatagram(datagram.data(),datagram.size(),&sender,&port); //char* datagram.data(), qint64

//        std::cout <<"Message From :: " << (sender.toString()).toStdString() <<std::endl;
//        std::cout <<"Port From :: "<< port <<std::endl;
//        std::cout <<"Message :: " << datagram.data() <<std::endl;
        senderPort = port;
        senderIP =  sender.toString();
        std::string datastring = datagram.data();
//        std::cout <<"String ::" << datastring <<std::endl;
//        std::cout <<"data size ::" << datagram.size()<<std::endl;

        if(datagram.size()>5)
        {
//            std::cerr<<"in plane"<<std::endl;

            USrobotVelticalMove = 0.0;
            USrobotUpward = true;
            USrobotDownward = false;

            OutPlaneCount = 0;
            InPlaneCount++;

            if(InPlaneCount>=5)
            {
                capsuleInplane = true;
                std::cerr<<"In plane"<<std::endl;
            }
            capsuleOutplane = false;
            std::cerr<<"In plane count: "<<InPlaneCount<<std::endl;


            string image_x = "";
            string image_y = "";
            //split received string to float
            int kk = 1;

            for(kk=1; kk<datagram.size(); kk++)
            {
                if(datastring[kk]==',')
                    break;
                else
                    image_x.push_back(datastring[kk]);
            }
            for(int mm=kk+2; mm<datagram.size(); mm++)
            {
                if(datastring[mm]==']')
                    break;
                else
                    image_y.push_back(datastring[mm]);
            }
//            double scale = 20/172.4;
            USimage_pos << stod(image_x)*UStomm_scale, stod(image_y)*UStomm_scale, 0.0;
//            std::cout <<"image position ::" << USimage_pos<<std::endl;

        }
        else if (datagram.size()<5)
        {
            InPlaneCount = 0;
            OutPlaneCount++;
            if(OutPlaneCount>=5)
            {
                capsuleOutplane = true;
                std::cerr<<"out plane"<<std::endl;
            }
            capsuleInplane = false;
            std::cerr<<"out plane count: "<<OutPlaneCount<<std::endl;

        }
        else
        {
            std::cerr<<"Wrong datagram from upd "<<std::endl;
        }

     /*   PBYTE pPacketByte = reinterpret_cast<byte*>(datagram.data());

            // Left Device
            for (int i = 0; i < 3; i++)
            {
//                m_Outputs.LeftDesiredPosition[i] = datagram.toFloat();

//                m_Outputs.LeftDesiredPosition[i] = qToLittleEndian(*((unsigned __int64*)pPacketByte));
                m_Outputs.LeftDesiredPosition[i] = ntohd(*((unsigned __int64*)pPacketByte));
                pPacketByte += sizeof(unsigned __int64);
            }
            for (int i = 0; i < 3; i++)
            {
                m_Outputs.LeftHaptic.Gimbal[i] = ntohd(*((unsigned __int64*)pPacketByte));
                pPacketByte += sizeof(unsigned __int64);
            }

            m_Outputs.LeftHaptic.GreyButton = *((unsigned char*)pPacketByte);
            pPacketByte += sizeof(unsigned char);
            m_Outputs.LeftHaptic.WhiteButton = *((unsigned char*)pPacketByte);
//        qDebug() <<"Message Desired Position :: " << m_Outputs.LeftDesiredPosition[0] << m_Outputs.LeftDesiredPosition[1]<< m_Outputs.LeftDesiredPosition[2];
            std::cout <<"Message Desired Gimbal :: " << m_Outputs.LeftHaptic.Gimbal[0]*180/M_PI <<std::endl; // << m_Outputs.LeftHaptic.Gimbal[1]*180/M_PI<< m_Outputs.LeftHaptic.Gimbal[2]*180/M_PI;
            std::cout <<"Message Grey button:: " << m_Outputs.LeftHaptic.GreyButton <<std::endl;
//        qDebug() <<"Message white button:: " << m_Outputs.LeftHaptic.WhiteButton;
*/
    }
}

void MainWindow::enableUDP(void)
{
    // When checkbox is changed, update the boolean
    UDPflag = ui->checkBox_udpmode->checkState();
    std::cout<< "UPD mode:: "<<UDPflag<<std::endl;
//    if(!UDPflag)
//    {

//    }

}


std::array<double, 8>  MainWindow::GenerateRandomCurrent()
{
    double maxIncrement = 5;
    double maxCurrent = 12;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(0,maxIncrement*2); // distribution in range [a, b]
    double randCurrentInc[8] = {0.0};

    for (int i = 0; i < 8; i++)
    {
        randCurrentInc[i] =  dist6(rng)-maxIncrement; //generate random current in the range of [-maxCurrent, maxcurrent]
        randCurrent[i] = randCurrent[i]+randCurrentInc[i];
        if (randCurrent[i]>maxCurrent)
            randCurrent[i] = maxCurrent-5.0;
        if (randCurrent[i]<(-maxCurrent))
            randCurrent[i] = -maxCurrent +5.0;
    }
    std::cout << "rand currents: "<<randCurrent[0]<<", "<<randCurrent[1]<<", "<<randCurrent[2]<<", "<<randCurrent[3]<<", "<<randCurrent[4]<<", "<<randCurrent[5]<<", "<<randCurrent[6]<<", "<<randCurrent[7] <<std::endl;
    return randCurrent;
}

/*
Eigen::Matrix<double, 6, 6> MainWindow::calculateAdjointVelMatrix(Eigen::Affine3d initial_transform)
{
    Eigen::Vector3d position(initial_transform.translation());
//    position_d = position_d*1000;//from m to mm
//    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d rotMat(initial_transform.linear());
//    Eigen::Matrix3d rotMat = HomogenousT.block<3,3>(0,0);
//    Eigen::Vector3d TransVec = HomogenousT.block<3,1>(0,3);
    Eigen::Matrix<double, 6, 6> AdjointVelMatrix;
//    a.block<2,2>(1,1) = m;
    AdjointVelMatrix.block<3,3>(0,0) = rotMat;
    AdjointVelMatrix.block<3,3>(3,3) = rotMat;
    Eigen::Matrix3d zero3 = Eigen::Array33f::Zero();
    AdjointVelMatrix.block<3,0>(3,3) = zero3;
    Eigen::Matrix3d P_skew;
    P_skew <<  0,        -position(2),  position(1),
            position(2),  0,            -position(0),
            -position(1), position(0),  0;
    AdjointVelMatrix.block<0,3>(3,3) = P_skew*rotMat;
    return AdjointVelMatrix;

}*/
