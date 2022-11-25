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
        Robot_tip_posisition[0] = position_d[0];
        Robot_tip_posisition[1] = position_d[1];
        Robot_tip_posisition[2] = position_d[2];

        Robot_orient[0] = eulerangle[0];
        Robot_orient[1] = eulerangle[1];
        Robot_orient[2] = eulerangle[2];

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
        // Check that the temperature in any core is not above the max value
        if (measuredTemperatures[t] > maxAllowableTemp)
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
        }
    }
//    std::cout<<"mrd current: "<<measuredCurrents[0]<<" "<<measuredCurrents[1]<<" "<<measuredCurrents[2]<<" "<<measuredCurrents[3]<<" "<<measuredCurrents[4]<<" "<<measuredCurrents[5]<<" "<<measuredCurrents[6]<<" "<<measuredCurrents[7]<<std::endl;
//    std::cout<<"mrd tempera: "<<measuredTemperatures[0]<<" "<<measuredTemperatures[1]<<" "<<measuredTemperatures[2]<<" "<<measuredTemperatures[3]<<" "<<measuredTemperatures[4]<<" "<<measuredTemperatures[5]<<" "<<measuredTemperatures[6]<<" "<<measuredTemperatures[7]<<std::endl;


    // READ FROM DAQ
    if (DAQ.isEnabled())
    {
        // Read analog inputs from the DAQ by reading values and passing by ref.
        DAQ.dataAcquisition8(DAQ.analogInputVoltages);
        DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
//        std::cout<<std::endl <<"Daq reading is: "<<DAQ.analogRawInputVoltages[0]<<" "<<DAQ.analogRawInputVoltages[1]<<" "<<DAQ.analogRawInputVoltages[2]<<std::endl;
//        std::cout<<"Field calculation is: "<<DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2]<<std::endl;

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
                        Robot_tip_posisition[0] = position_d[0];
                        Robot_tip_posisition[1] = position_d[1];
                        Robot_tip_posisition[2] = position_d[2];

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
