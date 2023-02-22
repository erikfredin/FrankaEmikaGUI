**Franka end effector offset
- for guassmeter probe, RobotEE_offset[3] = {0.0, 0.0, 325.3} (327.1 for the cover tip)  /unit:mm old------//338.2} 
- for registration bar, RobotEE_offset[3] = {0.0, 0.0, 80} /unit:mm
- for registration bar of ultrasound probe, RobotEE_offset[3] = {0.0, 0.0, 75.35} unit:mm

**Robot register data collect:
- update RobotEE with registration bar's length
- set file name to save the data to file
- check "pilot thread' to enalbe freedrag of the robot, and drag the robot to target #1
- uncheck "pilot thread" to kill the thread
- click "Robot recovery" to auto recover Franka
- check "Streaming data from Franka" to fetch current Franka robot tip position
- click "Robot register data collect", to record robottip position for online registration
- uncheck "Streaming data from Franka" to stop connection with Franka controller
------repeat following until all points are reached-----------------------
- check "pilot thread' to enalbe freedrag of the robot, and drag the robot to target #2
- uncheck "pilot thread" to kill the thread
- click "Robot recovery" to auto recover Franka
- check "Streaming data from Franka" to fetch current Franka robot tip position
- click "Robot register data collect", to record robottip position for online registration
- uncheck "Streaming data from Franka" to stop connection with Franka controller
--------------------------------------------------------------------------
- when all target positions are collected, the registration matrix is calculated and displayed,
  then remember to UPDATE matrix "transT2R" mannualy. 
- #click "Log off" to stop logging and save data to .csv file.


**Calibration data collect
-we have two modes, one is random, one is scanning full workspace at a grid step, called sequence
------------before run the programe
- Manually move robot roughly to the init position
- manually set motion range of robot in mainwindow.h, currently it is [-0.08,0.08], [-0.08,0.08], [0.03, 0.12] //unit: meter, origin is set on xy plane center of the table
- (for random mode) mannually set "currentloop" and "robotmoveloop", robot will move "robotmoveloop" times and loop inside of "currentloop"
- (for sequence mode)set desired currents at header file, we collect data per coil, so mannually update current each time
--------------do after start the programe
- Click "Initial Probe Orient"!!!!!!!!!!
- Set EE_offset!!!!!!!!!
- check "Enable DAQ" to collect data from guassmeter probe
- Set file name
- click "Calibration data collect ON" and logging will start automatically,
  <coil currents and robot position are randomly assigned each time in random mode>
- after all loops are passed, calibration shuts down automatically, logging will stops, 
  and the data will be saved to .csv file. 
- click "Calibration data collect OFF" if you want to stop the process early


**validation data collect
- set maximun data loop in Callback, use cotinuose collect mode, or set maxdataset # in GUI
- init robot
- set robot to init pose
- Set Franka EE_offset on GUI!!!!!!!!!!
- set filename
- enable DAQ
- check "Validation_DataCollect" or click "Collect"



***Coil model calibrate
- update dataset file name in MainWindows.cpp -> CalibrateCoiltable()
- update initial yaml file in MainWindows.cpp -> CalibrateCoiltable()
- change model file save name in MainWindows.cpp -> CalibrateCoiltable()
- run program and click Clibratecoiltable button


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


To do:
3d print new holder for gaussmeter,
re-register the robot with coil table
orientation control, rotate one axis after the other one, in sequence, not parallel
re-collect current data
measure output current???


----Franka robot debug-------

◆ franka::CartesianVelocities::CartesianVelocities	(	const std::array< double, 6 > & 	cartesian_velocities	)	
## cartesian_velocities-> Desired Cartesian velocity w.r.t. O-frame (base frame) {dx in [m/s], dy in [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.


◆ O_T_EE
std::array<double, 16> franka::CartesianPose::O_T_EE {}
Homogeneous transformation OT_EE, column major, that transforms from the end effector frame EE to base frame O.
Equivalently, it is the desired end effector pose in base frame.




Lsqnonlin (optimization) 

Solve nonlinear least-squares (nonlinear data-fitting) problems 

Tft =	 0.0165615641503394	-0.999856276573472	-0.00553760253506452	0.565589066909011 

0.999825417988350	0.0165044583017847	0.00756797445081788	-0.0190198566832415 

-0.00746220266756481	-0.00564890231151272	0.999946269462639	0.0531115277798854 

0	0	0	1 

 

3-point method 

H_ft = 

0.006691	-0.999957526650883	0.00633840476828956	0.563327058095494 

0.999871250339639	0.00678260457249358	0.0145423182285803	-0.0204573825283624 

-0.0145846914607849	0.00624028600800332	0.999874164885528	0.0496227521092186 

0	0	0	1 

H_ft_ave= 

0.00231643848715103	-0.999947917155666	0.00744424256810242	0.563590354821264 

0.999885254700563	0.00241546865174744	0.0134072801517942	-0.0203498147004006 

-0.0134229208930993	0.00740600442133213	0.999881546888257	0.0495995695439080 

0	0	0	1 

 

Linear least square method (X=A\B) 

Hft =  

0.0222167813068605	-1.02295482275761	-0.0259576856998971	0.567057356690978 

0.972480795562229	0.00228203566573079	0.0252472279841953	-0.0202496361027587 

3.20720621080189e-05	0.00397624527625073	0.937355775921548	0.0575258132287725 

0	0	0	1 


Using 9 points from table surface 

Lsqnonlin (optimization) 

Solve nonlinear least-squares (nonlinear data-fitting) problems 

Tft =	-0.00345879438587434	-0.999976058380228	0.00766405186868026	0.563933079917427 

0.999992366877681	-0.00342962190517934	-0.00265771869316834	-0.0181361011819294 

0.00268165410250359	0.00765809230506731	0.999967435616801	0.0499975044762943 

0	0	0	1 

 

3-point method 

H_ft = 

-0.00432829794796358	-0.999980171828400	0.00457403398725983	0.564423639143585 

0.999980119559747	-0.00434918154664971	-0.00456564400158468	-0.0191393574396221 

0.00458544677742310	0.00455418158588725	0.999979116335903	0.0496135961800502 

0	0	0	1 

H_ft_ave= 

-0.00367772026582749	-0.999958275612337	0.00765481618264748	0.564170544315878 

0.999983176933111	-0.00370188886497404	-0.00315525112173575	-0.0183367476620569 

0.00318345802455757	0.00764304601091306	0.999962437372146	0.0499635315702305 

0	0	0	1 

 

Linear least square method (X=A\B) 

Hft =  

-0.00566827935374654	-1.00197106911225	0.00583994313380850	0.563968807113787 

1.00047521857898	-0.00101023049574232	-0.0100321979373340	-0.0179916638120663 

0.00253523752417072	0.00775890184253339	1.00237067475728	0.0499504343674623 

0	0	0	1 
