#ifndef MAGNETICMATHFUNCTIONS_H
#define MAGNETICMATHFUNCTIONS_H


#include <math.h>
#include <tuple>

// This OpenCV header includes all the OpenCV headers
#include <opencv2/opencv.hpp>

#include <QDebug>

// Is this really necessary?
#define M_PI 3.14159265358979323846264338327950288

const double mu_0 = M_PI * 4e-7;

// GET RID OF THESE PARAMETERS
//const double Br = 1.457; // For Gripper experiments: 1.47; // [T]
//const double M = 1/mu_0*Br;
//const double vAct = 0.0508*0.0508*0.0508; // [m^3]

// remove this temporary fix or change to 8
//const int numAct0 = 4;

// PUBLIC GENERAL MATHEMATICAL FUNCTIONS

void MatrixMultVect(double C[3][3], double D[3], double E[3]); // DONE
void MatrixMultVect6(double C[6][6], double D[6], double E[6]);
void MatrixMult3(double C[3][3], double D[3][3], double E[3][3]); // DONE
void MatrixMult4(double C[4][4], double D[4][4], double E[4][4]); // DONE
void dipoleField(double p[3], double m[3], double B[3]);
void dipoleGradient(double p[3], double m[3], double G[5]);
void calcAnisotropicControlMatrix(double pAct[3][8], double mAct[3][8], double M[3][8]);
void calcIsotropicControlMatrix(double pAct[3][8], double mAct[3][8], double N[8][8]);
void maxAnisotropicField( double pAct[3][8],double mAct[3][8], double maxB[3]);
void maxIsotropicField( double pAct[3][8], double mAct[3][8], double maxB[3]);

double norm(double A[12]); // DONE

// Don't invert matrices this way:
void MatrixInvert(double H[12][12], double result[12][12]); // DONE

//void invertMatrix(int rows, int columns, double A[], double A_dagger[]);
//template <size_t rows, size_t columns>
//void invertMatrix(double (&A)[rows][columns], double (&A_dagger)[columns][rows]);


//// PUBLIC OPTIMIZATION FUNCTIONS
double calculateTheta(double angle1, double angle2);
//void findB_SurgeonSimulator(double B_desired_local[3]);

// New functions for Electromagnetic Actuation System
void local2global(double tool_tilt, double tool_roll, double B_local[3], double B_global[3]);
void global2local(double tool_tilt, double tool_roll, double B_global[3], double B_local[3]);
void calcFieldInSphericalCoords(double theta, double phi, double B_global[3], double B_sph[3]);


#endif // MAGNETICMATHFUNCTIONS_H
