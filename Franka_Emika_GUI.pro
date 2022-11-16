QT       += core gui gamepad

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


TARGET = Franka_Emika_GUI
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

CONFIG += c++11
#CONFIG += c++17
#-std:c++17

QMAKE_CXXFLAGS += -bigobj

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    callbacks.cpp \
    frankathread.cpp \
    main.cpp \
    mainwindow.cpp \
    examples_common.cpp \
    daq.cpp \
    s826.cpp \
    gamepadmonitor.cpp \
    magneticmathfunctions.cpp \
    electromagnet_calibration.cpp \
    scalorPotential.cpp


HEADERS += \
    callbacks.h \
    frankathread.h \
    mainwindow.h \
    examples_common.h \
    826api.h \
    daq.h \
    robotstatus.h \
    s826.h \
    gamepadmonitor.h \
    magneticmathfunctions.h \
    EigenToYAML.h \
    electromagnet_calibration.h \
    scalorPotential.h

FORMS += \
    mainwindow.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


## These are important for finding the libraries of the Franka robot that are in the /libfranka/ part of the project.
win32:CONFIG(release, debug|release): LIBS += -LC:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/libfranka/lib/ -lfranka
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/libfranka/lib/ -lfranka
DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\poco\cmake-build\lib\Debug"
DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\poco\cmake-build\bin\Debug"
DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\Eigen3"

#LIBS += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\liborl-master\build\Debug\orl.lib"
#INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\liborl-master\include"

LIBS += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\liborl-master - modified\build\Debug\orl.lib"
INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\liborl-master - modified\include"



#win32:CONFIG(release, debug|release): LIBS += -LC:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/yaml-cpp-master/build/Debug/ -lyaml-cppd
#else:win32:CONFIG(debug, debug|release): LIBS += -LC:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/yaml-cpp-master/build/Debug/ -lyaml-cppd

#LIBS +="C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\yaml-cpp-master\build\Debug\yaml-cppd.lib"
#LIBS +="C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\yaml-cpp-yaml-cpp-0.5.3\build\Debug\yaml-cpp.lib"
LIBS +="C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\yaml-cpp-yaml-cpp-0.5.3\build\Debug\libyaml-cppmdd.lib"
INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\yaml-cpp-yaml-cpp-0.5.3\include"
#DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\yaml-cpp-master\build\Debug"


INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\libfranka\include"
INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\Eigen3"

INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\includes-fdeep"

INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\lib\boost_1_80_0"

#INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\liborl-master\include"
#INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\boost_1_79_0\boost_1_79_0"

## These are important for finding the libraries of the s826 board that are in the /lib/ part of the project.
# We were using LIB instead of LIBS so it didn't work at first.
LIBS += $$PWD/../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64/s826.lib
INCLUDEPATH += $$PWD/../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64


# NI DAQ Libraries
# For 32bit compiling
#LIBS += "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/lib/msvc/NIDAQmx.lib"
#INCLUDEPATH += "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/include"
# For 64 bit compiling
INCLUDEPATH +=  "C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\include"
LIBS += "C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc/NIDAQmx.lib"


# Information for the OpenCV Libraries from System Directories
win32:CONFIG(release, debug|release): LIBS += -LC:/OpenCV-4.5.1/opencv/build/x64/vc15/lib/ -lopencv_world451
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/OpenCV-4.5.1/opencv/build/x64/vc15/lib/ -lopencv_world451d

INCLUDEPATH += C:/OpenCV-4.5.1/opencv/build/include
DEPENDPATH += C:/OpenCV-4.5.1/opencv/build/include

LIBS += -L"C:/OpenCV-4.5.1/opencv/build/x64/vc15/bin/" \
    -lopencv_world451 \
    -lopencv_world451d
