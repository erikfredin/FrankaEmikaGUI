#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // Magnetic serial robot setup
    Eigen::Matrix4d T;

    // Robot Denavit-Hartenberg parameters
    int numLinks = 2;
    double linkLength[2] = {7.22e-3, 7.77e-3}; //meters
    double linkTwist[2] = {1.571, 0.0}; //radians
    double linkOffset[2] = {0.0, 0.0}; //meters
    double jointAngle[2] = {0.0, 0.0}; //radians
    int jointType[2] = {JOINTREV, JOINTREV}; //0 for revolute, 1 for prismatic
    // Magnet dipole vectors and positions in local link coordinates
    Eigen::Vector3d magnetLocal[3];
    Eigen::Vector3d magnetPosLocal[3];
    magnetLocal[0] <<  0, 0, 0; // A.m^2
    magnetLocal[1] << 35.859e-3, 0, 0; // A.m^2
    magnetLocal[2] << -16.088e-3, 0, 0; // A.m^2
    magnetPosLocal[0] << -3.2e-3, 0, 0; // m
    magnetPosLocal[1] << -3.72e-3, 0, 0; // m
    magnetPosLocal[2] << -4.01e-3, 0.94e-3, 0; // m
    // Transformation matrix from base coords to global coords
    T << 0,-1, 0, 0,
         1, 0, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    QApplication a(argc, argv);
    MainWindow w(NULL, numLinks, linkLength, linkTwist, linkOffset, jointAngle, jointType, magnetLocal, magnetPosLocal);

    // Joint limits
    Eigen::MatrixXd qRange(2,2);
    qRange << -85.0/180.0*M_PI, 85.0/180.0*M_PI, 0.0, M_PI/3; // rad

    //window.myMagbot.setTbase(T);
    Eigen::Vector2d q_init {0, 0};
    w.magbot.m_set_q(q_init);

    w.magbot.m_change_DH_params(linkLength, linkTwist, linkOffset, jointAngle, jointType);
    // Set the magnetic properties of the links (dipole orientation and location
    // in local link frame coords).
    w.magbot.m_change_magnets(magnetLocal, magnetPosLocal);
    // Set the coil matrix.
    // Set the joint limits.
    // Set the base to global transformation.
    w.magbot.m_set_Tbase(T);

    w.show();
    return a.exec();
}



