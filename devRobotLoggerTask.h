// $Id

#ifndef _devRobotLoggerTask_h
#define _devRobotLoggerTask_h

#include <string>
#include <iostream>
#include <fstream>
#include <QString>
#include <QElapsedTimer>

//add timestamp
class devRobotLoggerTask {

protected:   

    // these are for the new dual force sensing tool with three FBG array.
    //void addNote(string s);
    bool OpenFiles(const std::string &fileNameBase);
    void CloseFiles(void);

    //flags set by the user

    //functions that can be accessed via the command pattern
    void EnableLog();
    void DisableLog();

    std::ofstream  LogFileAllData;


    std::string     FileNameBase;
    std::string     FullFileName;
    std::string     FilePath;
    bool            LogEnabled;

    char			Delim;
    int             NumWritten;
    int             ControlMode;
    QElapsedTimer   currentTime;

public:
    devRobotLoggerTask(const std::string & taskName, double period);
    ~devRobotLoggerTask();

    void IsLogging(bool &isLogging) const;
    void SetFileName(const std::string & fileNameBase);
    void SetLogEnabled(const bool &enable);
    void SetFilePath(const std::string& path);
    void SetControlMode(const int & controlMode);

    void Configure(const std::string & filename);
    void Startup(void);
    void Record(void);
    void Cleanup(void) {}

};

#endif // _devRobotLoggerTask_h
