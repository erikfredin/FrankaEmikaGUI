QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


TARGET = Franka_Emika_GUI
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    callbacks.cpp \
    main.cpp \
    mainwindow.cpp \
    examples_common.cpp

HEADERS += \
    callbacks.h \
    mainwindow.h \
    examples_common.h

FORMS += \
    mainwindow.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


## These are important for finding the libraries of the s826 board that are in the /lib/ part of the project.
win32:CONFIG(release, debug|release): LIBS += -LC:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/libfranka/lib/ -lfranka
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/Users/MicroRoboticsLab/Documents/Franka_Emika_Console/libfranka/lib/ -lfranka
DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\poco\cmake-build\lib\Debug"
DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\poco\cmake-build\bin\Debug"
DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\Eigen3"





# We were using LIB instead of LIBS so it didn't work at first.
#LIBS += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\libfranka\lib\franka.lib"
#DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\poco\cmake-build\lib\Debug"
#DEPENDPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\Eigen3"


#INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\libfranka\lib"
INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\Franka_Emika_Console\libfranka\include"
INCLUDEPATH += "C:\Users\MicroRoboticsLab\Documents\libfranka\Eigen3"
