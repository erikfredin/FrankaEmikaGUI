#include "callbacks.h"



void MainWindow::callbacks(void)
{
    // Execute code here
//    qInfo() << connectedGamepad.joystickValues[0];
//    qDebug() << "Code callback executing...";

    //We connect Franka every turns of callback , could be improved...


    if (isRobotStreaming){
        franka::Robot robot(fci_ip);
        //read franka robot pose
        franka::RobotState initial_state = robot.readOnce();
        // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
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


        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());


        //default unit is meter and rad, so we keep that
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


    // ~~~~~~~~~~~~~~~~~~~~~~~~ GENERAL CALLBACKS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // These are commands that are executed every iteration no matter the settings
    // to ensure that the system is running properly. As such, they should not be
    // modified unless you know what you are doing.
    // moved from Adam's code

    // Read Thermocouples and Current Monitors
    if (s826.boardConnected)
    {
        // Read all 16 input channels and store in pass-by-reference array:
        int err = s826.analogReadAll(inputAnalogVoltages);
//        qInfo() << err;
//        qInfo()<<"s826 connected";
    }

    for (int t = 0; t < 8; t++)
    {
        // Record new temperatures and currents from conversions
        // Input Analog Voltages are as follows:
        // AIN0 - AIN7  CURRENT SENSE FROM EM0-EM7, respectively
        // AIN8 - AIN15 THERMOCOUPLE SENSE FROM EM0 - EM7, respectively
        measuredCurrents[t] = inputAnalogVoltages[t]*currentSenseAdj[t]; // [A] read from amplifiers
        measuredTemperatures[t] = inputAnalogVoltages[t+8]*temperatureSenseAdj[t]; // [deg C] read from thermocouples
        if(currentStreaming)
        {
            measuredCurrents_feed[t] = measuredCurrents[t];
        }
        // Check that the temperature in any core is not above the max value
      /* if (measuredTemperatures[t] > maxAllowableTemp)
        {
            overheatingFlag = true;
            // set all currents to 0 and reset all desired field strengths
            std::cerr << "Coil "<< t <<" is Overheating!!!"<<std::endl ;
            updateCurrents_CalibrationOnly(zeroCurrent);
            std::cerr << "Currents Cleared!"<<std::endl;
            Robotmotionsuccess = 0;
            CalibrationDataCollet_Random = false;
            CalibrationDataCollet_sequence = false;
            std::cerr << "robot stopped!"<<std::endl;
            // at the end of callbacks, re-evaluate the temperatures.
        }*/
    }
//     std::cout<<"mrd current: "<<measuredCurrents[7]<<" "<<std::endl;
//    std::cout<<"mrd current: "<<measuredCurrents[0]<<" "<<measuredCurrents[1]<<" "<<measuredCurrents[2]<<" "<<measuredCurrents[3]<<" "<<measuredCurrents[4]<<" "<<measuredCurrents[5]<<" "<<measuredCurrents[6]<<" "<<measuredCurrents[7]<<std::endl;
//    std::cout<<"mrd tempera: "<<measuredTemperatures[0]<<" "<<measuredTemperatures[1]<<" "<<measuredTemperatures[2]<<" "<<measuredTemperatures[3]<<" "<<measuredTemperatures[4]<<" "<<measuredTemperatures[5]<<" "<<measuredTemperatures[6]<<" "<<measuredTemperatures[7]<<std::endl;


    // READ FROM DAQ
    if (DAQ.isEnabled())
    {
        // Read analog inputs from the DAQ by reading values and passing by ref.
        DAQ.dataAcquisition8(DAQ.analogInputVoltages);
        DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
        std::cout<<std::endl <<"Daq reading is: "<<DAQ.analogRawInputVoltages[0]<<" "<<DAQ.analogRawInputVoltages[1]<<" "<<DAQ.analogRawInputVoltages[2]<<std::endl;
        std::cout<<"Field measurement is: "<<DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2]<<std::endl;

//        DAQ.dataAcquisition();
        //Get Forces and torques from values
        double tempVoltages[6];
        double originalDAQVol[6];
        for (int v = 0; v<6; v++)
        {
            tempVoltages[v] = DAQ.analogInputVoltages[v]-ATINanoVoltageOffsets[v];
            originalDAQVol[v] = DAQ.analogInputVoltages[v];
        }
//        qInfo()<<"DAQ connected!";

    }



    if(CalibrationDataCollet_Random)
    {

        if (!robotinitialized) //not initialized
        {
            franka::Robot robot(fci_ip);
            setDefaultBehavior(robot);
            // First move the robot to a suitable joint configuration
//            std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            std::array<double, 7> q_goal = {{-0.0690753, -0.0159581, -0.00171238, -1.94264, -0.0153294, 1.91711, 0.724667}};
            MotionGenerator motion_generator(0.5, q_goal);
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration in callback." << std::endl;
            robotinitialized = true;
        }
        else
        {
            if (currentcount < currentloop)
            {
                if(robotloopisdone)
                {
                    double diff_current[numAct];
                    double inc_current[numAct];
                    // those code is copied from https://stackoverflow.com/questions/13445688/how-to-generate-a-random-number-in-c
                    // and is used to generate a random number, which is placed here for local use
                    std::random_device dev;
                    std::mt19937 rng(dev());
                    std::uniform_int_distribution<std::mt19937::result_type> dist6(0,maxCurrent*2); // distribution in range [a, b]
                    //
                    for (int i = 0; i < 8; i++)
                    {
                      if(!overheatingFlag)
                      {
                          cmdCoilCurrent[i] =  dist6(rng)-maxCurrent; //generate random current in the range of [-maxCurrent, maxcurrent]
                          diff_current[i] = cmdCoilCurrent[i] - tempCoilCurrent[i];
                          inc_current[i] = diff_current[i]/currentcooldownloop;
                      }
                      else
                          cmdCoilCurrent[i] =  0.0;
                    }
                    std::cout << "cmd currents: "<<cmdCoilCurrent[0]<<", "<<cmdCoilCurrent[1]<<", "<<cmdCoilCurrent[2]<<", "<<cmdCoilCurrent[3]<<", "<<cmdCoilCurrent[4]<<", "<<cmdCoilCurrent[5]<<", "<<cmdCoilCurrent[6]<<", "<<cmdCoilCurrent[7] <<std::endl;
//                    updateCurrents_CalibrationOnly(zeroCurrent);
//                    std::cout<<"reset currents to all coils..."<<currentTime.elapsed()<<std::endl;
//                    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

//                    double send_current[numAct];
//                    for(int k=0; k<currentcooldownloop; k++)
//                    {
//                        for (int j=0; j<numAct; j++)
//                            send_current[j] = tempCoilCurrent[j]+(k+1)*inc_current[j];
//                        updateCurrents_CalibrationOnly(send_current);
//                        std::cout<<"update cur: "<< k<< " " <<send_current[0]<<", "<<send_current[1]<<", "<<send_current[2]<<", "<<send_current[3]<<", "<<send_current[4]<<", "<<send_current[5]<<", "<<send_current[6]<<", "<<send_current[7] <<std::endl;
//                        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//                    }
//                    std::cout<<"Done!"<<std::endl;
                    updateCurrents_CalibrationOnly(cmdCoilCurrent);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    std::cout<<"Set current to cmd values!"<<std::endl;

                    // Read Thermocouples and Current Monitors
                    if (s826.boardConnected)
                    {
                        // Read all 16 input channels and store in pass-by-reference array:
                        int err = s826.analogReadAll(inputAnalogVoltages);
                //        qInfo() << err;
                //        qInfo()<<"s826 connected";
                    }
                    for (int t = 0; t < 8; t++)
                    {
                        measuredCurrents[t] = inputAnalogVoltages[t]*currentSenseAdj[t]; // [A] read from amplifiers
                    }
                    std::cout<<"mrd current: "<<measuredCurrents[0]<<" "<<measuredCurrents[1]<<" "<<measuredCurrents[2]<<" "<<measuredCurrents[3]<<" "<<measuredCurrents[4]<<" "<<measuredCurrents[5]<<" "<<measuredCurrents[6]<<" "<<measuredCurrents[7]<<std::endl;

//                    for(int k=0; k<numAct; k++)
//                        tempCoilCurrent[k] = cmdCoilCurrent[k];

                    robotloopisdone = false;
                }
                if(!robotloopisdone)
                {
                    if (robotmovecount < robotmoveloop)
                    {
                        // those code is copied from https://stackoverflow.com/questions/13445688/how-to-generate-a-random-number-in-c
                        // and is used to generate a random number, which is placed here for local use
    //                    double       leftlowercorner[3] = {-50.0, -50.0, 50.0}; //table origin is at the table center (0,0,0)
    //                    double       righttopcorner[3] = {50.0, 50.0, 100.0};
                        std::random_device dev1;
                        std::mt19937 rng1(dev1());
                        std::random_device dev2;
                        std::mt19937 rng2(dev2());
                        std::random_device dev3;
                        std::mt19937 rng3(dev3());
                        std::uniform_int_distribution<std::mt19937::result_type> pos_x(0,righttopcorner[0]*2); // distribution in range [0, 100]
                        std::uniform_int_distribution<std::mt19937::result_type> pos_y(0,righttopcorner[1]*2); // distribution in range [0, 100]
                        std::uniform_int_distribution<std::mt19937::result_type> pos_z(0,(righttopcorner[2]-leftlowercorner[2])); // distribution in range [0, 50]

                        robotposcmd[0] = (pos_x(rng1)-righttopcorner[0])*0.001; //rand [-50, 50]
                        robotposcmd[1] = (pos_y(rng2)-righttopcorner[1])*0.001; //rand [-50, 50]
                        robotposcmd[2] = (pos_z(rng3)+leftlowercorner[2])*0.001; //rand [75, 125]

//                        robotposcmd[0] = 0.0;
//                        robotposcmd[1] = 0.0,
//                        robotposcmd[2] = 0.1; //100mm
                        std::cout << std::endl<< "command position in table frame is: "<<robotposcmd[0]<<", "<<robotposcmd[1]<<", "<<robotposcmd[2]<<std::endl;

                        //covert cmd position in table frame to robot frame
                        Eigen::Vector4d pos_cmd(robotposcmd[0], robotposcmd[1], robotposcmd[2], 1);
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

                        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());

                        //default unit is meter and rad, so we keep that
                        Robot_tip_position[0] = position_d[0];
                        Robot_tip_position[1] = position_d[1];
                        Robot_tip_position[2] = position_d[2];

                        Robot_orient[0] = eulerangle[0];
                        Robot_orient[1] = eulerangle[1];
                        Robot_orient[2] = eulerangle[2];

                        for(int i=0; i<7; i++){
                            Robot_joint[i] = initial_state.q.data()[i];
                        }

                        if (DAQ.isEnabled())
                        {
                            // Read analog inputs from the DAQ by reading values and passing by ref.
                            DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
                        }

                    // record
//                        if(LogEnabled){
                             MainWindow::Record();
//                        }
//                        else
//                        {
//                            qInfo()<<"log is not enabled!";
//                        }

                        robotmovecount++;
                    }
                    else
                    {
                        robotmovecount = 0;
                        robotloopisdone = true;
                        currentcount++;
                        std::cout<<"single current loop is done ["<<currentcount<<"/"<<currentloop <<"]"<<std::endl;
                        if (currentcount == currentloop)
                        {
                            qInfo()<<"calibration data collection is done! set current to zeros";
                            MainWindow::CloseFiles();
                        }
                    }
                }
            }
            else
            {

//                qInfo()<<"calibration data collection is done! set current to zeros";
                double I_zeros[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                updateCurrents_CalibrationOnly(I_zeros);
//                // set robot to intial pos
//                franka::Robot robot(fci_ip);
//                setDefaultBehavior(robot);
//                // First move the robot to a suitable joint configuration
//                std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//                MotionGenerator motion_generator(0.5, q_goal);
//                robot.control(motion_generator);
//                std::cout << "Finished moving to initial joint configuration." << std::endl;
            }

        }


    }



    if (CalibrationDataCollet_sequence == true)
    {
        /// 9. DATA COLLECTION FOR FIELD CALIBRATION WITH NEURAL NETWORK
        //
        // This function collects/records field [Bx,By,Bz] (fetched from gaussmeter/DAQ),
        // gaussmeter probe position [Px,Py,Pz] (fetched from robot)
        // and coil currents [I1, I2,...,I8] (measured from S826).
        // The function also sends commands to S826 with desired currents and robot positions
        // The currents for 8 coils are generated randomly, for each set of currents, robot moves a full workspace
        // ---
        // --
        // -
        double step_x = 0.04; //unit: meter
        double step_y = 0.04;
        double step_z = 0.03;


        if (robotinitflag == false)
        {
            //move robot to initcorner
            //need to manualy move robot roughly close to the initconner before run below code!
//            double abs_robotpos[3] = {robot_x, robot_y, robot_z};
            //covert cmd position in table frame to robot frame
            qInfo() << "Initial position in table is: "<<robotinitcorner[0]<<robotinitcorner[1]<<robotinitcorner[2];
            Eigen::Vector4d pos_cmd(robotinitcorner[0], robotinitcorner[1], robotinitcorner[2], 1);
            Eigen::Vector4d pos_robot = transT2R*pos_cmd;
            double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};

            FrankaAbscartmotion( abs_robotpos);

            qInfo() << "robot is initilized!!!";
            robotinitflag = true;
        }
        else //robot is initilized
        {
            if (loopcount<loop)
            {
                if (singleloopdone == false)
                {
                    if (robot_x >= robotrange_x[0] && robot_x <= robotrange_x[1])
                    {
                        if (robot_y >= robotrange_y[0]&& robot_y <= robotrange_y[1])
                        {
                            if (robot_z >= robotrange_z[0]-0.001&& robot_z <= robotrange_z[1])
                            {
                                // move robot to desired position
                                std::cout<< "command position in table frame is: "<<robot_x<<", "<<robot_y<<", "<<robot_z<<std::endl;
                                //covert cmd position in table frame to robot frame
                                Eigen::Vector4d pos_cmd(robot_x, robot_y, robot_z, 1);
                                robotposcmd[0] = robot_x;
                                robotposcmd[1] = robot_y;
                                robotposcmd[2] = robot_z;
                                Eigen::Vector4d pos_robot = transT2R*pos_cmd;
                                double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};
                                // run robot
                                FrankaAbscartmotion( abs_robotpos);
                                //read robot status
                                ReadFrankaPoseStatus();
                                if (DAQ.isEnabled())
                                {
                                    // Read analog inputs from the DAQ by reading values and passing by ref.
                                    DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
                                    std::cout<<"Field measurement is: "<<DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2]<<std::endl;

                                }

                            // record
        //                        if(LogEnabled){
                                     MainWindow::Record();
        //                        }
        //                        else
        //                        {
        //                            qInfo()<<"log is not enabled!";
        //                        }

                                if (robotZupwardFlag == true)
                                {
                                    robot_z = robot_z + step_z;
                                }
                                if (robotZdownwardFlag == true)
                                {
                                    robot_z = robot_z - step_z;
                                }
                            }
                            else
                            {
                                if (robotYpositivewardFlag == true)
                                {
                                    robot_y = robot_y + step_y;
                                }
                                if (robotYnegtivewardFlag == true)
                                {
                                    robot_y = robot_y - step_y;
                                }

                                if (robot_z < robotrange_z[0])
                                {
                                    robot_z = robotrange_z[0];
                                    robotZupwardFlag = true;
                                    robotZdownwardFlag = false;
                                }
                                if (robot_z > robotrange_z[1])
                                {
                                    robot_z = robotrange_z[1];
                                    robotZupwardFlag = false;
                                    robotZdownwardFlag = true;
                                }

                                if(robot_y >= robotrange_y[0]&& robot_y <= robotrange_y[1])
                                {
                                    // move robot to desired position
                                    std::cout<< "command position in table frame is: "<<robot_x<<", "<<robot_y<<", "<<robot_z<<std::endl;
                                    //covert cmd position in table frame to robot frame
                                    Eigen::Vector4d pos_cmd(robot_x, robot_y, robot_z, 1);
                                    robotposcmd[0] = robot_x;
                                    robotposcmd[1] = robot_y;
                                    robotposcmd[2] = robot_z;
                                    Eigen::Vector4d pos_robot = transT2R*pos_cmd;
                                    double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};
                                    // run robot
                                    FrankaAbscartmotion( abs_robotpos);
                                    //read robot status
                                    ReadFrankaPoseStatus();
                                    if (DAQ.isEnabled())
                                    {
                                        // Read analog inputs from the DAQ by reading values and passing by ref.
                                        DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
                                        std::cout<<"Field measurement is: "<<DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2]<<std::endl;

                                    }

                                // record
            //                        if(LogEnabled){
                                         MainWindow::Record();
            //                        }
            //                        else
            //                        {
            //                            qInfo()<<"log is not enabled!";
            //                        }


                                    if (robotZupwardFlag == true)
                                    {
                                        robot_z = robot_z + step_z;
                                    }
                                    if (robotZdownwardFlag == true)
                                    {
                                        robot_z = robot_z - step_z;
                                    }
                                }
                            }
                        }
                        else
                        {
                            if (robotXpositivewardFlag == true)
                            {
                                robot_x = robot_x + step_x;
                            }
                            if (robotXnegtivewardFlag == true)
                            {
                                robot_x = robot_x - step_x;
                            }
                            if (robot_z < robotrange_z[0])
                            {
                                robot_z = robotrange_z[0];
                                robotZupwardFlag = true;
                                robotZdownwardFlag = false;
                            }
                            if (robot_z > robotrange_z[1])
                            {
                                robot_z = robotrange_z[1];
                                robotZupwardFlag = false;
                                robotZdownwardFlag = true;
                            }

                            if (robot_y < robotrange_y[0])
                            {
                                robot_y = robotrange_y[0];
                                robotYpositivewardFlag = true;
                                robotYnegtivewardFlag = false;
                            }
                            if (robot_y > robotrange_y[1])
                            {
                                robot_y = robotrange_y[1];
                                robotYpositivewardFlag = false;
                                robotYnegtivewardFlag = true;
                            }
                            if(robot_x >= robotrange_x[0] && robot_x <= robotrange_x[1])
                            {
                                // move robot to desired position
                                std::cout<< "command position in table frame is: "<<robot_x<<", "<<robot_y<<", "<<robot_z<<std::endl;
                                //covert cmd position in table frame to robot frame
                                Eigen::Vector4d pos_cmd(robot_x, robot_y, robot_z, 1);
                                robotposcmd[0] = robot_x;
                                robotposcmd[1] = robot_y;
                                robotposcmd[2] = robot_z;
                                Eigen::Vector4d pos_robot = transT2R*pos_cmd;
                                double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};
                                // run robot
                                FrankaAbscartmotion( abs_robotpos);
                                //read robot status
                                ReadFrankaPoseStatus();
                                if (DAQ.isEnabled())
                                {
                                    // Read analog inputs from the DAQ by reading values and passing by ref.
                                    DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
                                    std::cout<<"Field measurement is: "<<DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2]<<std::endl;

                                }

                            // record
        //                        if(LogEnabled){
                                     MainWindow::Record();
        //                        }
        //                        else
        //                        {
        //                            qInfo()<<"log is not enabled!";
        //                        }


                                if (robotZupwardFlag == true)
                                {
                                    robot_z = robot_z + step_z;
                                }
                                if (robotZdownwardFlag == true)
                                {
                                    robot_z = robot_z - step_z;
                                }
                            }
                        }
                    }
                    else // single loop data collection is done!
                    {
                        qInfo()<<"Loop " << loopcount <<" is done..........";
                        singleloopdone = true;

                        if (robot_z < robotrange_z[0])
                        {
                            robot_z = robotrange_z[0];
                            robotZupwardFlag = true;
                            robotZdownwardFlag = false;
                        }
                        if (robot_z > robotrange_z[1])
                        {
                            robot_z = robotrange_z[1];
                            robotZupwardFlag = false;
                            robotZdownwardFlag = true;
                        }

                        if (robot_y < robotrange_y[0])
                        {
                            robot_y = robotrange_y[0];
                            robotYpositivewardFlag = true;
                            robotYnegtivewardFlag = false;
                        }
                        if (robot_y > robotrange_y[1])
                        {
                            robot_y = robotrange_y[1];
                            robotYpositivewardFlag = false;
                            robotYnegtivewardFlag = true;
                        }
                        if (robot_x < robotrange_x[0])
                        {
                            robot_x = robotrange_x[0];
                            robotXpositivewardFlag = true;
                            robotXnegtivewardFlag = false;
                        }
                        if (robot_x > robotrange_x[1])
                        {
                            robot_x = robotrange_x[1];
                            robotXpositivewardFlag = false;
                            robotXnegtivewardFlag = true;
                        }

                    }
                }
                else if (singleloopdone == true)
                {
                    if (loopcount<loop) //avoid update current at the last loop
                    {
                        for (int i = 0; i < 8; i++)
                        {
                          cmdCoilCurrent[i] =  CurrentPool[loopcount][i]; //generate random current in the range of [-maxCurrent, maxcurrent]
                        }
                        qInfo() << "Current is: "<<cmdCoilCurrent[0]<<", "<<cmdCoilCurrent[1]<<", "<<cmdCoilCurrent[2]<<", "<<cmdCoilCurrent[3]<<", "<<cmdCoilCurrent[4]<<", "<<cmdCoilCurrent[5]<<", "<<cmdCoilCurrent[6]<<", "<<cmdCoilCurrent[7];
                        updateCurrents_CalibrationOnly(cmdCoilCurrent);
                        qInfo() << "loop is " <<loopcount;

                    }
                    singleloopdone = false;
                    loopcount++;
                }
            }
            else // loop reach max count, data collection finished!
            {
                qInfo() << "!!!!!!!!!!!Data loop is finished, send 0 to S826!!!";
                updateCurrents_CalibrationOnly(zeroCurrent); //send 0 to S826
                Datacollectdoneflag = true;
            }
        }
    }





    if(ValidationDataCollect_Random )
    {
        if(Num_validation<ValidateData_maxloop)
        {
            std::cout<<std::endl;
            std::cout<<"random collect num is "<<Num_validation <<std::endl;
            // get random B field Bx, By, Bz, in the range of [-10,10]
//            double max_B_random = 12.0; //unit: mT
            // get random P position Px, Py, Pz in the range of [-80,80], [-80,80], [30,120]
//            double B_random[3] = {0.0}; //unit: T
            double P_random[3] = {0.0};
            // those code is copied from https://stackoverflow.com/questions/13445688/how-to-generate-a-random-number-in-c
            // and is used to generate a random number, which is placed here for local use
//            std::random_device dev;
//            std::mt19937 rng(dev());
//            std::uniform_int_distribution<std::mt19937::result_type> dist_B(0,max_B_random*2); // distribution in range [a, b], unit: mT
//            //
//            for (int i = 0; i < 3; i++)
//            {

//                  B_random[i] =  (dist_B(rng)-max_B_random)*0.001; //generate random field in the range of [-maxB, maxB], unit: T
//            }

//            std::cout<< "command field B is: "<<B_random[0]*1000.0<<", "<<B_random[1]*1000.0<<", "<<B_random[2]*1000.0 <<"mT"<<std::endl;

            //unit: meter - cm, this random function only generates integers
            std::random_device dev1;
            std::mt19937 rng1(dev1());
            std::uniform_int_distribution<std::mt19937::result_type> dist_Px(0,robotrange_x[1]*2*100); // distribution in range [a, b]
            P_random[0] = (dist_Px(rng1)-robotrange_x[1]*100)*0.01;

            std::random_device dev2;
            std::mt19937 rng2(dev2());
            std::uniform_int_distribution<std::mt19937::result_type> dist_Py(0,robotrange_y[1]*2*100); // distribution in range [a, b]
            P_random[1] = (dist_Py(rng2)-robotrange_y[1]*100)*0.01;

            std::random_device dev3;
            std::mt19937 rng3(dev3());
            std::uniform_int_distribution<std::mt19937::result_type> dist_Pz(robotrange_z[0]*100, robotrange_z[1]*100); // distribution in range [a, b]
            P_random[2] = dist_Pz(rng3)*0.01;

            //calculate current using Coil model
//            Eigen::Vector3d B_command;
//            B_command <<B_random[0], B_random[1], B_random[2];
            Eigen::Vector3d P_command;
            P_command <<P_random[0], P_random[1], P_random[2];

            // run current
//            runCoilmodel_field(B_command, P_command);
            std::array<double, 8> RandCurrentOut = GenerateRandomCurrent();
            for(int k=0;k<8;k++)
                cmdCoilCurrent[k] = RandCurrentOut[k];
            updateCurrents_CalibrationOnly(cmdCoilCurrent);

            //move robot to P_command
            // move robot to desired position
            std::cout<< "command position in table frame is: "<<P_random[0]<<", "<<P_random[1]<<", "<<P_random[2]<<std::endl;
            //covert cmd position in table frame to robot frame
            Eigen::Vector4d pos_cmd(P_random[0], P_random[1], P_random[2], 1);
            Eigen::Vector4d pos_robot = transT2R*pos_cmd;
            double abs_robotpos[3] = {pos_robot(0), pos_robot(1), pos_robot(2)};

            // run robot
            FrankaAbscartmotion( abs_robotpos);
            //read robot status
            ReadFrankaPoseStatus(); //update robot tip position and orientation

//            std::cout<< "Field command is: "<<B_random[0]*1000.0<<", "<<B_random[1]*1000.0<<", "<<B_random[2]*1000.0 <<"mT"<<std::endl;
            if (DAQ.isEnabled())
            {
                // Read analog inputs from the DAQ by reading values and passing by ref.
                DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
                std::cout<<"Field measurement is: "<<DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2]<<std::endl;

            }
            else
                std::cerr<<"DAQ is not enabled!!!"<<std::endl;

            //update Field_command_validation and P_command for logging
            for(int k=0; k<3; k++)
            {
//                Field_command_validation[k] = B_random[k]*1000.0; //from T to mT
                robotposcmd[k] = P_random[k];
            }

        // record
            MainWindow::Record();

            Num_validation++;

        }
        else
        {
            std::cout<<"random collect is done! " <<std::endl;
            ValidationDataCollect_Random = false;
        }

    }




    //Receiving UDP package from CTR-- once udp received, active robot follow  control!
    if(UDPflag == true)
    {
//        qDebug() << "udp mode !";

//        QByteArray Data; // Message for send
//        Data += "SAMP";
//        socket_send->write(Data);
//        QByteArray Data;
//            Data.append("Hello from UDP");

//        socket_send->writeDatagram(Data, QHostAddress("192.168.31.124"), US_PORT);



        double USmidplane_x = 256; //unit: pixel
//        double scale = 20/172.4;
        double USmidplane_x_mm = USmidplane_x*UStomm_scale; //unit: mm
//        isGradientControlled = false; //set default as false
        processPendingDatagrams();

//        connect(socket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()),Qt::QueuedConnection);



        if(capsuleOutplane) //move robot up/down to search for the capsule
        {
            std::cout<<"capsule out plane motion........"<<std::endl;
            double maxSearchRange = 4*0.001; //unit:meter
            double deltaMove = 1*0.001; //unit:meter
            if (USrobotUpward==true)
            {
                if(USrobotVelticalMove<maxSearchRange)
                {
                    double Relativemoiton[3] = {0.0, 0.0, deltaMove};
                    ReadFrankaPoseStatus();
                    double abs_robotpos[3] = {Robot_tip_position[0]+Relativemoiton[0], Robot_tip_position[1]+Relativemoiton[1], Robot_tip_position[2]+Relativemoiton[2]};
                    FrankaAbscartmotion(abs_robotpos);
//                    FrankaRelativecartmotion(Relativemoiton);
                    std::cerr << "robot moving Upward!!!"<<std::endl;
                    USrobotVelticalMove = USrobotVelticalMove+deltaMove;
                }
                else
                {
                    USrobotUpward = false;
                    USrobotDownward = true;
                }
            }

            if (USrobotDownward==true)
            {
                if(USrobotVelticalMove>(-maxSearchRange))
                {
                    double Relativemoiton[3] = {0.0, 0.0, -deltaMove};
                    ReadFrankaPoseStatus();
                    double abs_robotpos[3] = {Robot_tip_position[0]+Relativemoiton[0], Robot_tip_position[1]+Relativemoiton[1], Robot_tip_position[2]+Relativemoiton[2]};
                    FrankaAbscartmotion(abs_robotpos);
//                    FrankaRelativecartmotion(Relativemoiton);
                    std::cerr << "robot moving Downward!!!"<<std::endl;
                    USrobotVelticalMove = USrobotVelticalMove-deltaMove;
                }
                else
                {
                    USrobotUpward = true;
                    USrobotDownward = false;
                }
            }

        }

        if(capsuleInplane)
        {
            std::cout << "Capsule in plane motion...... "<<std::endl;
            USrobotVelticalMove = 0.0;
            USrobotUpward = true;
            USrobotDownward = false;
            //control robot to desired inplane position
            //------------ use absolute control----------------
            //convert Image frame to Robot frame

            franka::Robot robot(fci_ip);
            //read franka robot pose
            franka::RobotState initial_state = robot.readOnce();
            // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
            Eigen::Matrix4d FK_robot = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());

            std::cout << "Capsule position in image (mm) is: "<<std::endl<<USimage_pos <<std::endl;
            Eigen::Vector4d USimageCenter(USmidplane_x_mm,USimage_pos(1),0.0, 1.0);
//            Eigen::Vector4d USimageCenter(USimage_pos(1),USmidplane_x_mm,0.0, 1.0); //swap x and y at US image
            Eigen::Vector4d USimageCenter_robot = FK_robot*transUS2EE*USimageCenter;
            std::cout << "Image center position in image (mm) is: "<<std::endl<<USimageCenter <<std::endl;

            Eigen::Vector4d UScapsule_cmd(USimage_pos[0], USimage_pos[1], USimage_pos[2], 1.0);
//            Eigen::Vector4d UScapsule_cmd(USimage_pos[1], USimage_pos[0], USimage_pos[2], 1.0); //swap x and y at US image
            Eigen::Vector4d UScapsule_robot = FK_robot*transUS2EE*UScapsule_cmd;

//            std::cout << "Image center position in Robot (mm) is: "<<std::endl<<USimageCenter_robot <<std::endl;
//            std::cout << "Capsule position in Robot (mm) is: "<<std::endl<<UScapsule_robot <<std::endl;

            //read current robot EE position
            ReadFrankaPoseStatus();
//            double deltamotion[3] = {UScapsule_robot(0)-USimageCenter_robot(0), UScapsule_robot(1)-USimageCenter_robot(1),UScapsule_robot(2)-USimageCenter_robot(2)};

            //we use relative EE increasement instead of Image increasement, Y-axis of EE is for the US probe moving left and right

//            Eigen::Vector4d capsule_EE(0.0, USimage_pos(0)-USimageCenter(0),0.0,1.0);
//            Eigen::Vector4d capsule_Robot = FK_robot*capsule_EE;

//            double abs_robotpos[3] = {capsule_Robot[0], capsule_Robot[1], capsule_Robot[2]};
//            std::cerr << "robot moving in plane!!! delta in EE::"<<capsule_EE[0]<<", "<<capsule_EE[1]<<", "<<capsule_EE[2] << std::endl;

            //we use relative Global increasement instead of Image increasement, X-axis of Global is for the US probe moving left and right

//            double delta_image[3] = {-(USimage_pos(0)-USimageCenter(0)), -(USimage_pos(1)-USimageCenter(1)),-(USimage_pos(2)-USimageCenter(2))};
            double delta_image[3] = {-(USimage_pos(0)-USimageCenter(0))*0.001, 0.0, 0.0};
            double abs_robotpos[3] = {Robot_tip_position[0]+delta_image[0], Robot_tip_position[1]+delta_image[1], Robot_tip_position[2]+delta_image[2]};
            std::cerr << "robot moving in plane!!! delta in image (m)::"<<delta_image[0]<<", "<<delta_image[1]<<", "<<delta_image[2] << std::endl;


            FrankaAbscartmotion(abs_robotpos);

//             std::cout << "Current position in image (pixel) is: "<<USimage_pos <<std::endl;
//             double delta_x = USimage_pos[0] - USmidplane_x;

            //-------use relative control --------
           /* franka::Robot robot(fci_ip);
            //read franka robot pose
            franka::RobotState initial_state = robot.readOnce();
            // EE in base frame, 4x4 matrix: initial_state.O_T_EE.data();
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Matrix<double, 6, 6> AdjointVel = calculateAdjointVelMatrix(initial_transform);
            // obtain moving direction
            */


        }


        QByteArray Data;
        Data.append("MotionCompleted");



    //        socket->write(Data);

        socket_send->writeDatagram(Data, QHostAddress("192.168.31.124"), US_PORT);
//        std::cout<<"ready to send response!"<<std::endl;
//        socket_send->writeDatagram(Data, QHostAddress(senderIP), senderPort);
//        std::cout<<"Sent response to IP: "<< senderIP.toStdString() <<" senderPort: "<< senderPort <<std::endl;

//        std::cout<<"Sent response to US computer" <<std::endl;


    }


 //record
    if(LogEnabled && (!CalibrationDataCollet_Random))
         MainWindow::Record();


    if (connectedGamepad.enabled) // Update Direct Local B-field
        {
            //// 1. DIRECT LOCAL B-FIELD CONTROL:
            //
            // Here the joystick directly controls the desired Bx, By, and Bz field components
            // in the tool frame. This assumes that the desired fields are relative to the
            // tool which, when at rest, points in the z-direction. The code takes the
            // desired local frame magnetic field values and computes the necessary field in
            // the global frame to be sent to the actuators.
            //
            // The assigned joystick axes are as follows:
            // Left Y-axis:     Bx field
            // Right X-axis:    By field
            // L2 and R2:       Bz field

    //        qDebug() << "Working";

            // Set desired B local field
            std::cout<<"joystick values"<<std::endl;
            for (int i=0; i<10; i++)
                std::cout<<"[ "<< i << "]:  "<<connectedGamepad.joystickValues[i]<<"    "; // Increase nonlinearly to add sensitivity and control
           std::cout<<std::endl;

    }




}
