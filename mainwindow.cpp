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
    connect(ui->pushButton_ConnectionCheck,SIGNAL(clicked()),SLOT(connectioncheck()));

}



MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::updateCaption(void)
{
    // Update labels on the GUI to reflect parameters saved in code

    // for testing connection
    ui->label_robot_px->setText(tr("%1").arg(Robot_pos[0]));
    ui->label_robot_py->setText(tr("%1").arg(Robot_pos[1]));
    ui->label_robot_pz->setText(tr("%1").arg(Robot_pos[2]));
    ui->label_robot_ea1->setText(tr("%1").arg(Robot_orient[0]));
    ui->label_robot_ea2->setText(tr("%1").arg(Robot_orient[1]));
    ui->label_robot_ea3->setText(tr("%1").arg(Robot_orient[2]));

}


void MainWindow::connectioncheck()
{
  qDebug()<<"checking connection.....press enter to continue...";
//  std::cin.ignore();
//  std::string fci_ip = "172.16.0.2";
  franka::Robot robot(fci_ip);

// read robot status
//  try {
//    franka::Robot robot(fci_ip);

//    size_t count = 0;
//    robot.read([&count](const franka::RobotState& robot_state) {
//      // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
//      // should not be done in a control loop.
//      std::cout << robot_state << std::endl;
//      return count++ < 100;
//    });

//    std::cout << "Done." << std::endl;
//  } catch (franka::Exception const& e) {
//    std::cout << e.what() << std::endl;
//  }


//Cartesian pose control
  try {
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
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    std::array<double, 16> initial_pose;
    double time = 0.0;
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      constexpr double kRadius = 0.3;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = kRadius * std::sin(angle);
      double delta_z = kRadius * (std::cos(angle) - 1);
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;
      if (time >= 20.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
//    return -1;
  }

// Cartesian velocity control
  try {
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
//    return -1;
  }


 //joint position control
  try {
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
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    std::array<double, 7> initial_position;
    double time = 0.0;
    robot.control([&initial_position, &time](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));
      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3] + delta_angle,
                                        initial_position[4] + delta_angle, initial_position[5],
                                        initial_position[6] + delta_angle}};
      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
//    return -1;
  }


// joint velocity control
  try {
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
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    double time_max = 1.0;
    double omega_max = 1.0;
    double time = 0.0;
    robot.control(
        [=, &time](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
          time += period.toSec();
          double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
          franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
          if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
//    return -1;
  }

//Impendence control

//  // Compliance parameters
//  const double translational_stiffness{150.0};
//  const double rotational_stiffness{10.0};
//  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
//  stiffness.setZero();
//  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
//  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
//  damping.setZero();
//  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
//                                     Eigen::MatrixXd::Identity(3, 3);
//  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
//                                         Eigen::MatrixXd::Identity(3, 3);

//  try {
//    // connect to robot
////    franka::Robot robot(argv[1]);
//    setDefaultBehavior(robot);
//    // load the kinematics and dynamics model
//    franka::Model model = robot.loadModel();

//    franka::RobotState initial_state = robot.readOnce();

//    // equilibrium point is the initial position
//    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
//    Eigen::Vector3d position_d(initial_transform.translation());
//    Eigen::Quaterniond orientation_d(initial_transform.linear());

//    // set collision behavior
//    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//    // define callback for the torque control loop
//    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
//        impedance_control_callback = [&](const franka::RobotState& robot_state,
//                                         franka::Duration /*duration*/) -> franka::Torques {
//      // get state variables
//      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
//      std::array<double, 42> jacobian_array =
//          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

//      // convert to Eigen
//      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
//      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
//      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
//      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
//      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
//      Eigen::Vector3d position(transform.translation());
//      Eigen::Quaterniond orientation(transform.linear());

//      // compute error to desired equilibrium pose
//      // position error
//      Eigen::Matrix<double, 6, 1> error;
//      error.head(3) << position - position_d;

//      // orientation error
//      // "difference" quaternion
//      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
//        orientation.coeffs() << -orientation.coeffs();
//      }
//      // "difference" quaternion
//      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
//      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
//      // Transform to base frame
//      error.tail(3) << -transform.linear() * error.tail(3);

//      // compute control
//      Eigen::VectorXd tau_task(7), tau_d(7);

//      // Spring damper system with damping ratio=1
//      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
//      tau_d << tau_task + coriolis;

//      std::array<double, 7> tau_d_array{};
//      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
//      return tau_d_array;
//    };

//    // start real-time control loop
//    std::cout << "WARNING: Collision thresholds are set to high values. "
//              << "Make sure you have the user stop at hand!" << std::endl
//              << "After starting try to push the robot and see how it reacts." << std::endl
//              << "Press Enter to continue..." << std::endl;
////    std::cin.ignore();
//    robot.control(impedance_control_callback);

//  } catch (const franka::Exception& ex) {
//    // print exception
//    std::cout << ex.what() << std::endl;
//  }



}
