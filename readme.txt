**Franka end effector offset
- for guassmeter probe, RobotEE_offset[3] = {0.0, 0.0, 325.4}  /unit:mm old------//338.2} 
- for registration bar, RobotEE_offset[3] = {0.0, 0.0, 80} /unit:mm

**Robot register data collect:
- check "pilot thread' to enalbe freedrag of the robot, and drag the robot to target #1
- uncheck "pilot thread" to kill the thread
- click "Robot recovery" to auto recover Franka
- check "Streaming data from Franka" to fetch current Franka robot tip position
- click "Robot register data collect", to record robottip position for online registration
- #set file name and click "Log on" if you want to save the tippostion to file
- #click "pause" to pause the data logging
- uncheck "Streaming data from Franka" to stop connection with Franka controller
------repeat following until all points are reached-----------------------
- check "pilot thread' to enalbe freedrag of the robot, and drag the robot to target #2
- uncheck "pilot thread" to kill the thread
- click "Robot recovery" to auto recover Franka
- check "Streaming data from Franka" to fetch current Franka robot tip position
- click "Robot register data collect", to record robottip position for online registration
- #click "continue" to continue the data logging
- #click "pause" to pause the data logging
- uncheck "Streaming data from Franka" to stop connection with Franka controller
--------------------------------------------------------------------------
- when all 9 target position are collected, the registration matrix is calculated and displayed,
  then remember to UPDATE matrix "transT2R" mannualy. 
- #click "Log off" to stop logging and save data to .csv file.


**Calibration data collect
------------before run the programe
- Manually move robot roughly to the init position
- manually set motion range of robot in mainwindow.h, currently it is [-0.08,0.08], [-0.08,0.08], [0.06, 0.15] //unit: meter, origin is set on xy plane center of the table
- mannually set "currentloop" and "robotmoveloop", robot will move "robotmoveloop" times and 
  loop inside of "currentloop"
--------------do after start the programe
- Click "Initial Probe Orient"!!!!!!!!!!
- Set EE_offset!!!!!!!!!
- check "Enable DAQ" to collect data from guassmeter probe
- Set file name
- click "Calibration data collect ON" and logging will start automatically,
  coil currents and robot position are randomly assigned each time
- after all loops are passed, calibration shuts down automatically, logging will stops, 
  and the data will be saved to .csv file. 
- click "Calibration data collect ON" if you want to stop the process early


**DNN predict
- input B1, P1, B2, P2, click "update". Note that P1, P2 should be in [-50,50], [-50,50], [75, 125]
- click "DNN predict", look at predicted current, if no problem
- click "Run DNN currents", send current to S826
- click "Move Robot to P1" to move Franka tip to P1
- click "Move Robot to P2" to move Franka tip to P2



Debug:
Franka_Progrom:
B_Global_Desired: 0.01 0 0 0 0 0 0 0 
0: 15.731, 1: -0.0495944, 2: -20.0479, 3: 1.03966, 4: -1.92972,	5: -19.9266, 6: -0.230327, 7: 19.1166,	
setting zero to the S826.
EM_program
Input B = 10mT, 0,0
0: 1.33573 1: -0.261861 2: -1.46208 3: -6.31208 4: 6.18944 5: -1.26992 6: 0.407749 7: 1.4175 



//    const double m = 6324.48; // [Am^2] this is the theoretical magnetic moment on each EM based on simulation
    // 2*pi*(0.12+0.360/2)^3*(0.035366483)/mu_0 = 4.7745e+03 [A m^2] From 2021/01/13 COMSOL Simulations
    const double m = 4774.5; // [Am^2] this is most recent number. It is also similar to a volume-scaled permanent magnet
    double mAct_cartesion[numField][numAct] = { // All are in positive z direction
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {   m,   m,   m,   m,   m,   m,   m,   m}
        };





