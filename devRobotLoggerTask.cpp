/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
Author(s):	Marcin Balicki
Created on:   2008-09-03

(C) Copyright 2008 Johns Hopkins University (JHU), All Rights
Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifdef _MSC_VER
#pragma warning(disable:4786)
#pragma warning(disable:4996)
#endif

#include "devRobotLoggerTask.h"


#include <math.h>
#include <iomanip>
#include <time.h>
#include <queue>
#include <time.h>





//std::queue<Mat> video_buffer;
devRobotLoggerTask::devRobotLoggerTask(const std::string & taskName,
                                       double period)
{

    Delim= ',';
    FilePath = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data";
    //to access the whole vector use .Data
    //this gets resized by the sensor class anyway.

    FileNameBase=std::string("");
    FullFileName  = std::string("ASDF");

    LogEnabled=false;

}
devRobotLoggerTask::~devRobotLoggerTask ( ){
    void * result;
//    pthread_join(video_t,&result);
    CloseFiles();
}

void  devRobotLoggerTask::SetLogEnabled(const bool &enable){

    //if we not already logging then we should reset the latest and start.
    LogEnabled=enable;
}


void devRobotLoggerTask::Startup(void)
{
    //this zeros the state index...it might miss a few initial values but
    //generally that is ok.
    //LastStateIndex.set=0;
//    Robot.GetStateIndex(LastStateIndex);
    //Robot.GetRobotState(this->RobotState);

    //get the values for the wavelength here.
    // open all files and set some initial data.
    //    if(!OpenFiles(FileNameBase)){
    //        CMN_LOG_CLASS_INIT_ERROR << "Startup: can't open files" << std::endl;
    //        //exit(-1);
    //    }

}

void devRobotLoggerTask::Record(void)
{
    //default
    if (!LogFileAllData.is_open()) {
        std::cerr<<" Log file is not open, creating a default file"<<std::endl;
        if (!OpenFiles("Default"))
            std::cerr<<" Log file failed to open"<<std::endl;
    }

            LogFileAllData<<std::setprecision(0)<<tempIndex.Ticks()
                        <<Delim<<std::setiosflags(std::ios::fixed)
                       <<std::setprecision(6)<<currentTime.elapsed()/1000.0;
    // this part is modified, to print ForceTip
        /*    LogFileForces<<Delim<<std::setprecision(0)<<TipForces.Valid()
                        <<std::setprecision(7);
            unsigned int i = 0;
            for (i = 0;i < TipForces.size();i++)
                LogFileForces<<Delim<<TipForces[i];

            double ScleraForceNorm = sqrt(pow(TipForces[2],2) + pow(TipForces[3],2));
            LogFileForces<<Delim<<TipForceNorm.Data;
            LogFileForces<<Delim<<FBGForceScleraNorm.Data;*/

            LogFileAllData<<Delim<<std::setprecision(0)<<ForcesTip.Valid()
                          <<std::setprecision(7);
           unsigned int i = 0;
           for (i = 0;i < ForcesTip.size();i++)
                LogFileAllData<<Delim<<ForcesTip[i];


           /*LogFileForces<<Delim<<std::setprecision(0)<<ForcesSclera.Valid()
                       <<std::setprecision(7);*/
           for (i = 0;i < ForcesSclera.size();i++)
                LogFileAllData<<Delim<<ForcesSclera[i];

           LogFileAllData<<Delim<<ForcesTipNorm.Data;
           LogFileAllData<<Delim<<ForcesScleraNorm.Data;


            LogFileAllData<<std::endl;
            NumWritten.Data++;

            //LogRawData.setf(ios::fixed); //Print floating point numbers using fixed point notation.
            ////ios::showpoint  Print a decimal point for all floating point numbers, even when
            ////it's not needed (e.g. the number is exactly an integer).
            ////The precision of numbers can be changed as follows. You can also set the width,
            ////i.e. the minimum number of spaces used to print the next.
            ////These featuers are used, for example, to make output items line up in columns
            ////when printed. Both of these features require that you include the iomanip header file.
            //LogRawData.precision(0);  // print 0 digits after decimal point


}

bool devRobotLoggerTask::OpenFiles(const std::string &fileNameBase){
    //Robot.GetRobotControlModeQR(LastStateIndex,          ControlMode);
    std::cout << "Opening new LogFile in Control Mode: " <<std::endl;
    CloseFiles();

    LogFileAllData.clear();

    //LogFileReadme.clear();
    FilePath = "C:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/Data";

    std::string dateTime;
//    osaGetDateTimeString(dateTime);

    // Find time to create a unique filename for saving
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

//        strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo); // Filename cannot contain ":" symbol
    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H-%M-%S",timeinfo);
    std::string temp(buffer);
//    dateTime = QString::fromStdString(temp);
    dateTime = temp;
//    filename = DATA_SAVE_PATH + filename + ".txt"; // This file will not be changed until a new recording is started


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

            LogFileAllData<<"TimeStamp(s)"<<Delim;


            unsigned int i;
            for (i = 0; i < 3; i++)
                LogFileAllData<<"ProbeReading_mT_"<<i<<Delim;

            for (i = 0; i < 3; i++)
                LogFileAllData<<"ProbePos_mm_"<<i<<Delim;


            for (i = 0;i < 8; i++)
                LogFileAllData<<"cmdCoilCurrent_A_"<<i<<Delim;

            for (i = 0;i < 8; i++)
                LogFileAllData<<"mrdCoilCurrent_A_"<<i<<Delim;

            for (i = 0;i < 3; i++)
                LogFileAllData<<"Daqraw_v"<<i<<Delim;


        std::cout << "Log files opened"<<std::endl;
    }
    return true;
}

void devRobotLoggerTask::SetControlMode(const int & controlMode)
{
    ControlMode = controlMode;
}

void devRobotLoggerTask::SetFileName(const std::string & fileNameBase)
{
    FileNameBase = fileNameBase;
    LogEnabled = false;
    CloseFiles();
    OpenFiles(FileNameBase);
}


void devRobotLoggerTask::CloseFiles(void){
    LogFileAllData.close();

    //LogFileReadme.close();
    NumWritten=0;
}


//void devRobotLoggerTask::SetFilePath(const std::string& path){
//    CMN_LOG_CLASS_RUN_VERBOSE<<"Set file path : "<<path<<std::endl;
//    FilePath = path;
//}
