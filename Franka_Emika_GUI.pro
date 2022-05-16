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
    mainwindow.cpp

HEADERS += \
    callbacks.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


# We were using LIB instead of LIBS so it didn't work at first.
#LIBS += $$PWD/../libfranka
INCLUDEPATH += $$PWD/../libfranka/include/franka

#INCLUDEPATH += $$PWD/../../vcpkg/vcpkg/installed/x64-windows/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../vcpkg/vcpkg/installed/x64-windows/lib/ -llibexpat
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../vcpkg/vcpkg/installed/x64-windows/lib/ -llibexpatd
else:unix: LIBS += -L$$PWD/../../vcpkg/vcpkg/installed/x64-windows/lib/ -llibexpat

INCLUDEPATH += $$PWD/../../vcpkg/vcpkg/installed/x64-windows/include
DEPENDPATH += $$PWD/../../vcpkg/vcpkg/installed/x64-windows/include
