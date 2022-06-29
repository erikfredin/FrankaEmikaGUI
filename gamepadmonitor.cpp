/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Gamepad module
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "gamepadmonitor.h"

GamepadMonitor::GamepadMonitor(QObject *parent) : QObject(parent) , m_gamepad(0)
{

      //// Default Gamepad configuration code (Worked on Linux?)
//    auto gamepads = QGamepadManager::instance()->connectedGamepads();
//    qDebug() << gamepads;
//    if (gamepads.isEmpty()) {
//        qDebug() << "No Controller connection detected. Retry connection and restart App";
//        return;
//    }
//    // Create Gamepad object
//    m_gamepad = new QGamepad(*gamepads.begin(), this);
//    // Connect signals to slots:

    // Connection to the gamepad controller has been moved to a function for simplicity.
//    reconnectController();

//    qInfo() << " ";
//    qInfo() << "Attempting to establish game controller connection:";

//    // Gamepad Configuration coded by Sumanth Kandala to work on Windows OS
//    QGamepadManager* gamepad_manager = QGamepadManager::instance();
//    QList<int> gamepads;
//    int i = 0;
//    qInfo() << "Checking connected ports for connected game controllers (Max. " << connectionAttempts << " trials)";
//    while (i < connectionAttempts)
//    {
//        QApplication::processEvents();
//        qInfo() << "Searching for connected gamepads iteration number : " << i+1;
//        gamepads = gamepad_manager->connectedGamepads();
//        if(!gamepads.isEmpty())
//        {
//            // If found a controller then exit out early
//            i = connectionAttempts;
//        } else
//        {
//            qWarning() << "No Controller connection detected ...";
//        }
//        i++;
//    }
//    qInfo() << "Number of connected gamepads : " << gamepads.size();
//    if (gamepads.isEmpty())
//    {
//        qWarning() << "No Controller connection detected. Plug in controller before attempting to reconnect.";
//        qInfo() << "Check that the controller is responding by going to Control Panel\Hardware and Sound\Devices and Printers\Game Controller Settings.";
//        return;
//    }
//    m_gamepad = new QGamepad(*gamepads.begin(), this);



    // Finish setup constructor
    // this is the callback timer -> REMOVE THIS
//    QTimer *changeFieldTimer = new QTimer(this);
//    connect(changeFieldTimer, SIGNAL(timeout()), this, SLOT(increaseBfield()));
//    changeFieldTimer->start(20); //Increase 50 times per second


//    enableController(); // Comment this out after testing purposes to default off on startup


}

GamepadMonitor::~GamepadMonitor()
{
    qInfo() << "Disconnecting game controller...";
    delete m_gamepad;
}

//void GamepadMonitor::associateWith(MainWindow* window)
//{
//    connectedWindow = window;
//    connect(connectedWindow, &MainWindow::controllerStateSignal,  this, &GamepadMonitor::setControllerState);
//    connect( this, SIGNAL(changeRollSignal(int)), connectedWindow, SLOT(changeRoll(int)) );
//    connect( this, SIGNAL(changeMagnetsSignal()), connectedWindow, SLOT(updateMotors()) );
////    connect(connectedWindow->ui->comboBox_controllerMode,SIGNAL(currentIndexChanged (int)),this,SLOT( increaseBfield() ) );

//}

void GamepadMonitor::enableController(void)
{
    // Unused buttons are commented out here and in disableController

    connect(m_gamepad, &QGamepad::axisLeftXChanged,  this, &GamepadMonitor::leftXchanged);
    connect(m_gamepad, &QGamepad::axisLeftYChanged,  this, &GamepadMonitor::leftYchanged);
    connect(m_gamepad, &QGamepad::axisRightXChanged, this, &GamepadMonitor::rightXchanged);
    connect(m_gamepad, &QGamepad::axisRightYChanged, this, &GamepadMonitor::rightYchanged);
    connect(m_gamepad, &QGamepad::buttonAChanged,  this, &GamepadMonitor::buttonAchanged);
    connect(m_gamepad, &QGamepad::buttonBChanged,  this, &GamepadMonitor::buttonBchanged);
    connect(m_gamepad, &QGamepad::buttonXChanged,  this, &GamepadMonitor::buttonXchanged);
    connect(m_gamepad, &QGamepad::buttonYChanged,  this, &GamepadMonitor::buttonYchanged);
    connect(m_gamepad, &QGamepad::buttonL1Changed, this, &GamepadMonitor::buttonL1changed);
    connect(m_gamepad, &QGamepad::buttonR1Changed, this, &GamepadMonitor::buttonR1changed);
    connect(m_gamepad, &QGamepad::buttonL2Changed, this, &GamepadMonitor::buttonL2changed);
    connect(m_gamepad, &QGamepad::buttonR2Changed, this, &GamepadMonitor::buttonR2changed);
//    connect(m_gamepad, &QGamepad::buttonSelectChanged, this, &GamepadMonitor::buttonSelectchanged);
    connect(m_gamepad, &QGamepad::buttonStartChanged,  this, &GamepadMonitor::buttonStartchanged);
//    connect(m_gamepad, &QGamepad::buttonGuideChanged,  this, &GamepadMonitor::buttonGuidechanged);

    enabled = true;
}
void GamepadMonitor::disableController(void)
{
    disconnect(m_gamepad, &QGamepad::axisLeftXChanged,  this, &GamepadMonitor::leftXchanged);
    disconnect(m_gamepad, &QGamepad::axisLeftYChanged,  this, &GamepadMonitor::leftYchanged);
    disconnect(m_gamepad, &QGamepad::axisRightXChanged, this, &GamepadMonitor::rightXchanged);
    disconnect(m_gamepad, &QGamepad::axisRightYChanged, this, &GamepadMonitor::rightYchanged);
    disconnect(m_gamepad, &QGamepad::buttonAChanged,  this, &GamepadMonitor::buttonAchanged);
    disconnect(m_gamepad, &QGamepad::buttonBChanged,  this, &GamepadMonitor::buttonBchanged);
    disconnect(m_gamepad, &QGamepad::buttonXChanged,  this, &GamepadMonitor::buttonXchanged);
    //disconnect(m_gamepad, &QGamepad::buttonYChanged,  this, &GamepadMonitor::buttonYchanged);
    disconnect(m_gamepad, &QGamepad::buttonL1Changed, this, &GamepadMonitor::buttonL1changed);
    disconnect(m_gamepad, &QGamepad::buttonR1Changed, this, &GamepadMonitor::buttonR1changed);
    disconnect(m_gamepad, &QGamepad::buttonL2Changed, this, &GamepadMonitor::buttonL2changed);
    disconnect(m_gamepad, &QGamepad::buttonR2Changed, this, &GamepadMonitor::buttonR2changed);
    //disconnect(m_gamepad, &QGamepad::buttonSelectChanged, this, &GamepadMonitor::buttonSelectchanged);
    //disconnect(m_gamepad, &QGamepad::buttonStartChanged,  this, &GamepadMonitor::buttonStartchanged);
    //disconnect(m_gamepad, &QGamepad::buttonGuideChanged,  this, &GamepadMonitor::buttonGuidechanged);

    enabled = false;
    // For a disabled controller, all joystick values are defaulted to zero.
    for (int i=0; i<numAxes; i++)
    {
        joystickValues[i] = 0.0;
    }
}

void GamepadMonitor::reconnectController(void)
{
    qInfo() << " ";
    qInfo() << "Attempting to establish game controller connection:";

    // Gamepad Configuration coded by Sumanth Kandala to work on Windows OS
    QGamepadManager* gamepad_manager = QGamepadManager::instance();
    QList<int> gamepads;
    int i = 0;
    qInfo() << "Checking connected ports for connected game controllers (Max. " << connectionAttempts << " trials)";
    while (i < connectionAttempts)
    {
        QApplication::processEvents();
        qInfo() << "Searching for connected gamepads iteration number : " << i+1;
        gamepads = gamepad_manager->connectedGamepads();
        if(!gamepads.isEmpty())
        {
            // If found a controller then exit out early
            i = connectionAttempts;
        } else
        {
            qWarning() << "No Controller connection detected ...";
        }
        i++;
    }
    qInfo() << "Number of connected gamepads : " << gamepads.size();
    if (gamepads.isEmpty())
    {
        qWarning() << "No Controller connection detected. Plug in controller before attempting to reconnect.";
        qInfo() << "Check that the controller is responding by going to Control Panel/Hardware and Sound/Devices and Printers/Game Controller Settings.";
        return;
    }
    m_gamepad = new QGamepad(*gamepads.begin(), this);
}

// ------------------------------------- SLOTS -------------------------------------//

void GamepadMonitor::leftXchanged( double value )
{
//    qDebug() << "Left X" << value;
    joystickValues[0] = value;
}

void GamepadMonitor::leftYchanged( double value )
{
//    qDebug() << "Left Y" << value;
    joystickValues[1] = value;
}

void GamepadMonitor::rightXchanged( double value )
{
//    qDebug() << "Right X" << value;
    joystickValues[2] = value;
}

void GamepadMonitor::rightYchanged( double value )
{
//    qDebug() << "Right Y" << value;
    joystickValues[3] = value;
}

void GamepadMonitor::buttonAchanged( bool pressed )
{
//    qDebug() << "Button A" << pressed;
    if (pressed)
    {
//        connectedWindow->ToolOUT_press_Slot();
    }
    else
    {
//        connectedWindow->ToolINOUT_rel_Slot();
    }

}

void GamepadMonitor::buttonBchanged( bool pressed )
{
    qDebug() << "Button B" << pressed;
}

void GamepadMonitor::buttonXchanged( bool pressed )
{
    // Tool in fast
    qDebug() << "Button X" << pressed;
    if (pressed)
    {
//        connectedWindow->ToolIN_press_Slot();
    }
    else
    {
//        connectedWindow->ToolINOUT_rel_Slot();
    }
}

void GamepadMonitor::buttonYchanged( bool pressed )
{
    qDebug() << "Button Y" << pressed;
    // Show the current controller state in terminal
//    qDebug() << connectedWindow->currentControllerMode;
}

void GamepadMonitor::buttonL1changed( bool pressed )
{
    qDebug() << "Button L1" << pressed;
    if ( pressed )
    {
//        connectedWindow->roll_dir = -1;
//        emit changeRollSignal(-1);
    }
    else
    {
//        connectedWindow->roll_dir = 0;
//        emit changeRollSignal(0);
    }

}

void GamepadMonitor::buttonR1changed( bool pressed )
{
    qDebug() << "Button R1" << pressed;
    if ( pressed )
    {
//        connectedWindow->roll_dir = 1;
//        emit changeRollSignal(1);
    }
    else
    {
//        connectedWindow->roll_dir = 0;
//        emit changeRollSignal(0);
    }

}

void GamepadMonitor::buttonL2changed( double value )
{
//    qDebug() << "Button L2: " << value;
    joystickValues[4] = value;
}

void GamepadMonitor::buttonR2changed( double value )
{
//    qDebug() << "Button R2: " << value;
    joystickValues[5] = value;
}

void GamepadMonitor::buttonStartchanged( bool pressed )
{
    qDebug() << "Button Start" << pressed;

    if (pressed)
    {
    // Testing math functions here
//    const int r = 3;
//    const int c = 3;
    const size_t r = 3;
    const size_t c = 4;
    double AA[r][c] = {{-3.0, 4.0, -3.0, 1},
                       {0.0, 1.0, 6.0, 0},
                       {-12.0, 0.0, 10.0, -5}};
    double AAinv[c][r] = {0.0};

    // This is the golden command.
    cv::invert(cv::Mat(r,c,CV_64F,AA),cv::Mat(c,r,CV_64F,AAinv),cv::DECOMP_SVD); // Singular Value Decomposition is slowest but it can handle nonsquare



//    invertMatrix<r,c>(AA, AAinv);

//    // Using OpenCV to invert the matrix:
//    // A_Dagger, the inverse of an MxM matrix, will also be MxM
//    // An NxM matrix A will have a psuedoinverse of size MxN, A_dagger
//    cv::Mat B = cv::Mat(r, c, CV_64F, AA); // CV_32F 32bit float  to CV_32S 32bit signed int
//    qInfo() << "Should be: " << AA[1][2] << " Is Actually: " << B.at<double>(1,2);
////    std::cout << B;
//    cv::Mat B_dagger = cv::Mat(r, c, CV_64F, AA).inv();
////    cv::_InputArray C = B;
//    cv::invert(B,B_dagger);
//    // Convert back to an array
//     qInfo() << "B Matrix: ";
//    // rows are colums and columns are rows for the inverted matrix
//    for ( int i = 0; i < r; i++ )
//    {
//        for ( int j = 0; j < c; j++ )
//        {
//            // only need to return the inverse so A is not needed
////            A[i][j] = B.at<double>(i,j);
//            // Matrix index to array index [i][j] - > [i*cols + j]
//            // (but inverse matrix is transposed)

//            AAinv[j][i] = B_dagger.at<double>(j,i);
////            qInfo() << B_dagger.at<double>(j,i);
//        }

//        qInfo() << B.at<double>(i,0) << B.at<double>(i,1) << B.at<double>(i,2);

//    }

//    std::cout<<B;
    qInfo() << "Inverse of the matrix after passing is:";
    for ( int i = 0; i < c; i++ )
    {

        qInfo() << AAinv[i][0] << AAinv[i][1] << AAinv[i][2];
    }

    }
//    invertMatrix(3,3,AA,AAinv);



    //    double m[3] = {1.0,2.0,-2.0};
    //    double p[3] = {0.0,1.0,-10.0};
    //    double B[3];
    //    double G[5];
    //    if (pressed)
    //    {
    //        dipoleField(p,m,B);
    //        qInfo() << "Magnetic Field is: ";
    //        qInfo() << B[0];
    //        qInfo() << B[1];
    //        qInfo() << B[2];
    //        dipoleGradient(p,m,G);
    //        qInfo() << "Magnetic Gradients are: ";
    //        qInfo() << G[0];
    //        qInfo() << G[1];
    //        qInfo() << G[2];
    //        qInfo() << G[3];
    //        qInfo() << G[4];
    //    }



    // This was an attempt to automatically open the windows gaming center even if using a PS3 controller
//    if (pressed)
//    {
//        openGameBar = true;
//    }
//    else
//    {
//        openGameBar = false;
//    }

//    QKeyEvent *event_Meta = new QKeyEvent( QEvent::KeyPress, Qt::Key_Meta, Qt::NoModifier);
//    QCoreApplication::postEvent(this, event_Meta);
//    QKeyEvent *event_G = new QKeyEvent( QEvent::KeyPress, Qt::Key_G, Qt::NoModifier);
//    QCoreApplication::postEvent(this, event_G);
//    QKeyEvent *event_Enter = new QKeyEvent( QEvent::KeyPress, Qt::Key_Enter, Qt::NoModifier);
//    QCoreApplication::postEvent(this, event_Enter);

//    delete event_Meta;
//    delete event_G;
//    delete event_Enter;

}
void GamepadMonitor::buttonSelectchanged( bool pressed )
{
    qDebug() << "Button Select" << pressed;
}

void GamepadMonitor::buttonGuidechanged( bool pressed )
{
    qDebug() << "Button Guide" << pressed;
}



void GamepadMonitor::setControllerState(bool state)
{
    if (state)
    {
        enableController();
    }
    else
    {
        disableController();
    }
}

void GamepadMonitor::increaseBfield(void) // Callback for updating motor position
{
        if (enabled)
        {
//            qDebug() << joystickValues[1];
        }
}


// Callbacks
//void GamepadMonitor::increaseBfield(void) // Callback for updating motor position
//{

//    if (connectedWindow->currentControllerMode == 0 && enabled) // Update DIRECT in B-field
//    {
//        qDebug() << "Working";
//        // Set desired B local field
//        connectedWindow->B_Desired[0] = -this->maxDesiredB*joystickValues[0] ; // Increase nonlinearly to add sensitivity and control
//        connectedWindow->B_Desired[1] =  this->maxDesiredB*joystickValues[1] ; // Increase nonlinearly to add sensitivity and control
//        connectedWindow->B_Desired[2] = -this->maxDesiredB*joystickValues[2] + this->maxDesiredB*joystickValues[3]; // Increase nonlinearly to add sensitivity and control
//        // Set desired global field
//        tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
//        // Find best motor angle solution to desired field
//        find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
//        // Determine the resulting actual field
//        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
//        // Write angles to motors
//        emit changeMagnetsSignal();
//        connectedWindow->updateAngles();
//        qDebug() << "Printing Angles";
//    }
//    else if (connectedWindow->currentControllerMode == 1 && enabled) // Update Change in B-field
//    {
//        // Set desired Bz local field
//        if (abs(joystickValues[0]) > 0.075) // threshold because of the noisy controller in x
//        {
//            connectedWindow->B_Desired[0] = connectedWindow->B_Desired[0] + -this->maxDesiredB*joystickValues[0]/100 ; // Increase nonlinearly to add sensitivity and control
//        }
//        connectedWindow->B_Desired[1] = connectedWindow->B_Desired[1] + this->maxDesiredB*joystickValues[1]/100 ; // Increase nonlinearly to add sensitivity and control
//        connectedWindow->B_Desired[2] = connectedWindow->B_Desired[2] - this->maxDesiredB*joystickValues[2]/100 + this->maxDesiredB*joystickValues[3]/100; // Increase nonlinearly to add sensitivity and control

//        // Threshold the max Bfield so it doesn't increase to an absurd number
//        for (int i=0;i<3;i++)
//        {
//            if (abs(connectedWindow->B_Desired[i]) > this->maxDesiredB)
//            {
//                connectedWindow->B_Desired[i] = this->maxDesiredB * connectedWindow->B_Desired[i]/abs(connectedWindow->B_Desired[i]);
//            }
//        }
//        // Set desired global field
//        tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
//        // Find best motor angle solution to desired field
//        find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
//        // Determine the resulting actual field
//        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
//        // Write angles to motors
//        emit changeMagnetsSignal();
//        connectedWindow->updateAngles();
//    }
//    else if (connectedWindow->currentControllerMode == 2 && enabled) // Update Change in Polar Direction
//    {
//        if (abs(joystickValues[0]) > 0.075) // threshold because of the noisy controller in x
//        {
//            connectedWindow->angle2   = connectedWindow->angle2   + joystickValues[0] / 25; //phi
//        }
//        connectedWindow->angle1 = connectedWindow->angle1 + joystickValues[1] / 25;
//        connectedWindow->B_mag = connectedWindow->B_mag - joystickValues[2] / 5000 + joystickValues[3] / 5000;
//        // Threshold the max Bfield so it doesn't increase to an absurd number
//        if ( abs(connectedWindow->B_mag) > this->maxDesiredB)
//        {
//            connectedWindow->B_mag = this->maxDesiredB * connectedWindow->B_mag/abs(connectedWindow->B_mag); // x/abs(x) = sign(x);
//        }
//        connectedWindow->B_Desired[0] = connectedWindow->B_mag*sin(connectedWindow->angle2)*cos(connectedWindow->angle1); // use phi and theta, angles 1 and 2
//        connectedWindow->B_Desired[1] = connectedWindow->B_mag*sin(connectedWindow->angle2)*sin(connectedWindow->angle1); // use phi and theta, angles 1 and 2
//        connectedWindow->B_Desired[2] = connectedWindow->B_mag*cos(connectedWindow->angle2); //
//        // Set desired global field
//        tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
//        // Find best motor angle solution to desired field
//        find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
//        // Determine the resulting actual field
//        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
//        // Write angles to motors
//        emit changeMagnetsSignal();
//        connectedWindow->updateAngles();
//    }
//    else if (connectedWindow->currentControllerMode == 3 && enabled) // Surgeon Simulator
//    {
//        // Set desired In/Out Speed of tool
//        if (abs(joystickValues[0]) > 0.075) // threshold because of the noisy controller in x
//        {
////            joystickValues[0] = (abs(joystickValues[0]) - 0.075) * (joystickValues[0]/abs(joystickValues[0])); // subtract the dead zone
//            connectedWindow->ToolINOUT_speed_Slot(int(-100*sqrt(abs(joystickValues[0])) * (joystickValues[0]/abs(joystickValues[0]))) + 110); // Value -1 to 1 shifted to 10 to 210
//        }
//        else
//        {
//            connectedWindow->ToolINOUT_rel_Slot(); // sends a stop command
//            // This doesn't allow the a faster in/out motion with the buttons
//        }

//        // Check If Field control or Angle control is Enabled
//        if (connectedWindow->enableAngleControlState)
//        {
//            connectedWindow->angle1   = connectedWindow->angle1   - joystickValues[5] / 60; // change - to + to invert direction  of joystick
//            connectedWindow->angle2   = connectedWindow->angle2   + joystickValues[1] / 60;
//            connectedWindow->B_mag = connectedWindow->B_mag - this->maxDesiredB*joystickValues[2]/100 + this->maxDesiredB*joystickValues[3]/100; // Increase nonlinearly to add sensitivity and control
//            // Check bounds
//            if (abs(connectedWindow->angle1) > M_PI/2)
//            {
//                connectedWindow->angle1 = M_PI/2 * connectedWindow->angle1/abs(connectedWindow->angle1); // (keeps the polarity of the angle)
//            }
//            if (abs(connectedWindow->angle2) > M_PI/2)
//            {
//                connectedWindow->angle2 = M_PI/2 * connectedWindow->angle2/abs(connectedWindow->angle2); // (keeps the polarity of the angle)
//            }

//            connectedWindow->calculateTheta();
//            connectedWindow->findB_SurgeonSimulator(connectedWindow->B_Desired);
//        }
//        else
//        {
//            connectedWindow->angle1 = 0.0;
//            connectedWindow->angle2 = 0.0;
//            connectedWindow->B_mag = 0.0;
//            connectedWindow->B_Desired[0] = connectedWindow->B_Desired[0] + -this->maxDesiredB*joystickValues[5]/100 ; // Increase nonlinearly to add sensitivity and control
//            connectedWindow->B_Desired[1] = connectedWindow->B_Desired[1] + this->maxDesiredB*joystickValues[1]/100 ; // Increase nonlinearly to add sensitivity and control
//            connectedWindow->B_Desired[2] = connectedWindow->B_Desired[2] - this->maxDesiredB*joystickValues[2]/100 + this->maxDesiredB*joystickValues[3]/100; // Increase nonlinearly to add sensitivity and control
//        }

//        // Threshold the max Bfield so it doesn't increase to an absurd number
//        for (int i=0;i<3;i++)
//        {
//            if (abs(connectedWindow->B_Desired[i]) > this->maxDesiredB)
//            {
//                connectedWindow->B_Desired[i] = this->maxDesiredB * connectedWindow->B_Desired[i]/abs(connectedWindow->B_Desired[i]);
//            }
//        }
//        // Set desired global field
//        tip2global(connectedWindow->tilt, connectedWindow->roll, connectedWindow->B_Desired, connectedWindow->B_Global_Desired);
//        // Find best motor angle solution to desired field
//        find_angles_3(connectedWindow->pAct, connectedWindow->RzyAct, connectedWindow->mAct, connectedWindow->pTool, connectedWindow->B_Global_Desired, connectedWindow->K, connectedWindow->init_Angles, connectedWindow->next_Angles);
//        // Determine the resulting actual field
//        connectedWindow->find_Field(connectedWindow->pAct, connectedWindow->mAct, connectedWindow->RzyAct, connectedWindow->init_Angles, connectedWindow->B_Global_Output);
//        // Write angles to motors
//        emit changeMagnetsSignal();
//        connectedWindow->updateAngles();
//    }

//}
