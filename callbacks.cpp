#include "callbacks.h"



void MainWindow::callbacks(void)
{
    // Execute code here
//    qInfo() << connectedGamepad.joystickValues[0];
//    qDebug() << "Code callback executing...";

    //We connect Franka every turns of callback , could be improved...


    if (isRobotreading){
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
        Robot_pos[0] = position_d[0];
        Robot_pos[1] = position_d[1];
        Robot_pos[2] = position_d[2];

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
            qWarning() << "System is Overheating!!!" ;
            updateCurrents();
            qInfo() << "Currents Cleared.";
            // at the end of callbacks, re-evaluate the temperatures.
        }
    }

    // READ FROM DAQ
    if (DAQ.isEnabled())
    {
        // Read analog inputs from the DAQ by reading values and passing by ref.
        DAQ.dataAcquisition8(DAQ.analogInputVoltages);
        DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
//        DAQ.dataAcquisition();
        //Get Forces and torques from values
        double tempVoltages[6];
        double originalDAQVol[6];
        for (int v = 0; v<6; v++)
        {
            tempVoltages[v] = DAQ.analogInputVoltages[v]-ATINanoVoltageOffsets[v];
            originalDAQVol[v] = DAQ.analogInputVoltages[v];
        }

//        double A[6][6] = {0.0};
//        double J[6] = {0.0};
////        MatrixMultVect6(A, tempVoltages, J);
//        MatrixMultVect6(ATINanoCalibrationMatrix, tempVoltages, ATINanoForceTorque);
//        //this is for ATI force/torque output
//        for (int v = 0; v<6; v++)
//        {
//            DAQ.analogInputVoltages[v] = ATINanoForceTorque[v];
//        }
        //this is for gaussmeter output
//        for (int v = 0; v<3; v++)
//        {
//            DAQ.analogInputVoltages[v] = originalDAQVol[v]*gaussmeterCalibratedConstants[v];
//        }
    }


//    std::cout<<"robot EE posisiton is: " << position_d <<std::endl;
//    std::cout<<"robot EE orientation is: " << eulerangle <<std::endl;

//    std::cout<<"robot EEinF posisiton is: " << EEinFpos <<std::endl;
//    std::cout<<"robot EEinF orientation is: " << EEinFeulerangle <<std::endl;

//    std::cout<<"robot measured tau: "<<initial_tau_measured<<std::endl;


    if(LogEnabled){
         MainWindow::Record();
    }

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
            std::cout<<"joystick values: " <<connectedGamepad.joystickValues <<std::endl; // Increase nonlinearly to add sensitivity and control
    }




}
