#include "magneticmathfunctions.h"

///------------Copied from Adam's project EM_System_App
///---------------2022-07-01----------------------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MATH FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// ANDREWS MATH FUNCTIONS
void MatrixMultVect(double C[3][3], double D[3], double E[3])
{
    // This function returns the matrix multiplication of C*D=E
    // Where C is a 3x3 matrix and D is a 3x1 vector and thus E is
    // the resulting 3x1 vector.
    // The vector E is a buffer array that is passed by reference and editted
    // to receive the result of the matrix multiplication.

    // There is probably a built in C++ math function for
    // this, but maybe this explicit code is faster?

    double num;

    for ( int i = 0; i < 3; i++ )
    {
        num = 0.0;
        for ( int k = 0; k < 3; k++ )
        {
            num = C[i][k]*D[k] + num;
        }
        E[i] = num;
    }
}

void MatrixMultVect6(double C[6][6], double D[6], double E[6])
{
    // This function returns the matrix multiplication of C*D=E
    // Where C is a 6x6 matrix and D is a 6x1 vector and thus E is
    // the resulting 6x1 vector.
    // The vector E is a buffer array that is passed by reference and editted
    // to receive the result of the matrix multiplication.

    // There is probably a built in C++ math function for
    // this, but maybe this explicit code is faster?

    double num;

    for ( int i = 0; i < 6; i++ )
    {
        num = 0.0;
        for ( int k = 0; k < 6; k++ )
        {
            num = C[i][k]*D[k] + num;
        }
        E[i] = num;
    }
}

void MatrixMult4(double C[4][4], double D[4][4], double E[4][4])
{
    // This function returns the matrix multiplication of C*D=E
    // Where both C and D are 4x4 matrices and thus E is also a 4x4 matrix
    // The matrix E is a buffer array that is passed by reference and editted
    // to receive the result of the matrix multiplication.

    // There is probably a built in C++ math function for
    // this, but maybe this explicit code is faster?

    double num;

    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
        {
            num = 0.0;
            for ( int k = 0; k < 4; k++ )
            {
                num = C[i][k]*D[k][j] + num;
            }
            E[i][j] = num;
         }
    }
}

void MatrixMult3(double C[3][3], double D[3][3], double E[3][3])
{
    // This function returns the matrix multiplication of C*D=E
    // Where both C and D are 3x3 matrices and thus E is also a 3x3 matrix
    // The matrix E is a buffer array that is passed by reference and editted
    // to receive the result of the matrix multiplication.

    // There is probably a built in C++ math function for
    // this, but maybe this explicit code is faster?

    double num;

    for ( int i = 0; i < 3; i++ )
    {
        for ( int j = 0; j < 3; j++ )
        {
            num = 0.0;
            for ( int k = 0; k < 3; k++ )
            {
                num = C[i][k]*D[k][j] + num;
            }
            E[i][j] = num;
         }
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// ADAMS MATH FUNCTIONS

void dipoleField(double p[3], double m[3], double B[3])
{
    //INPUTS
    // p :  position   -  A vector of length 3
    // m :  mag_moment -  A vector of length 3
    //
    //RETURNED OUTPUTS
    // B  - The 3 x 1 vector containing Bx, By, and Bz field components (passed by reference)

    double p_norm = sqrt( pow(p[0],2) + pow(p[1],2) + pow(p[2],2) );
    double p_hat[3];
    for (int j = 0; j < 3; j++)
    {
        p_hat[j] = p[j]/p_norm;
    }
//    double eye[3][3] = { {1.0, 0.0, 0.0},
//                         {0.0, 1.0, 0.0},
//                         {0.0, 0.0, 1.0} };

    // B = 1e-7/(norm(p)^3)*(3*p_hat*p_hat'-eye(3))*m;

    double f = 1.0e-7 / pow(p_norm,3);
    double Bm[3][3] = { {3.0*p_hat[0]*p_hat[0] - 1.0,       3.0*p_hat[0]*p_hat[1],       3.0*p_hat[0]*p_hat[2]},
                        {      3.0*p_hat[1]*p_hat[0], 3.0*p_hat[1]*p_hat[1] - 1.0,       3.0*p_hat[1]*p_hat[2]},
                        {      3.0*p_hat[2]*p_hat[0],       3.0*p_hat[2]*p_hat[1], 3.0*p_hat[2]*p_hat[2] - 1.0} };

    MatrixMultVect(Bm, m, B);
    B[0] *= f;
    B[1] *= f;
    B[2] *= f;
    // Returns B implicitly
}


void dipoleGradient(double p[3], double m[3], double G[5])
{
    //  ABOUT:    Finds the 5 unique gradient terms of a magnet with magnetic
    // moment mag_moment at position assuming the dipole model is valid
    //
    // INPUTS
    // p :  position   -  A vector of length 3
    // m :  mag_moment -  A vector of length 3
    // RETURNED OUTPUTS
    // G  - The 5 x 1 vector of field gradient components including:
    //               dBxdx, dBxdy dBxdz dBydy dBydz

    //%dipole pointing in z?
    //% position and mag_moment should both be 3x1 vectors
    //% constant out front, alpha, is mu_0 / (4 pi) , but mu_0 is 4pi e-7 so:
    double alpha = 1.0e-7;
    double p_norm = sqrt( pow(p[0],2) + pow(p[1],2) + pow(p[2],2) );
    double p_hat[3];
    for (int j = 0; j < 3; j++)
    {
        p_hat[j] = p[j]/p_norm;
    }

//    double coeff_x[3] = {4.0, 3.0, 3.0};
//    double coeff_y[3] = {3.0, 4.0, 3.0};

    double Bm[3][3] = { {3.0*p[0]*p[0] - pow(p_norm,2),                 3.0*p[0]*p[1],                 3.0*p[0]*p[2]},
                        {                3.0*p[1]*p[0], 3.0*p[1]*p[1] - pow(p_norm,2),                 3.0*p[1]*p[2]},
                        {                3.0*p[2]*p[0],                 3.0*p[2]*p[1], 3.0*p[2]*p[2] - pow(p_norm,2)} };

    double B_submatrix[3]; // = (3*p*p'-norm(p)^2*eye(3))*m; or Bm*m
    MatrixMultVect(Bm, m, B_submatrix);
    // Derived Derivatives of B Field:
    // dBxdx
    G[0] = alpha * ( (4.0*p[0]*m[0] + 3.0*p[1]*m[1] + 3.0*p[2]*m[2])/pow(p_norm,5) + (-5.0)*p[0]*B_submatrix[0]/pow(p_norm,7) );
    // dBxdy
    G[1] = alpha * ( (3.0*p[0]*m[1] - 2.0*m[0]*p[1])/pow(p_norm,5) + (-5.0)*p[1]*B_submatrix[0]/pow(p_norm,7) );
    // dBxdz
    G[2] = alpha * ( (3.0*p[0]*m[2] - 2.0*m[0]*p[2])/pow(p_norm,5) + (-5.0)*p[2]*B_submatrix[0]/pow(p_norm,7) );
    // dBydy
    G[3] = alpha * ( (3.0*p[0]*m[0] + 4.0*p[1]*m[1] + 3.0*p[2]*m[2])/pow(p_norm,5) + (-5.0)*p[1]*B_submatrix[1]/pow(p_norm,7) );
    // dBydz
    G[4] = alpha * ( (3.0*p[1]*m[2] - 2.0*m[1]*p[2])/pow(p_norm,5) + (-5.0)*p[2]*B_submatrix[1]/pow(p_norm,7) );

//    G = [dBxdx dBxdy dBxdz dBydy dBydz]';

}

void local2global(double tool_tilt, double tool_roll, double B_local[3], double B_global[3])
{
    // Written by Adam Schonewille, based on function written by Andrew Lim
    // Inputs for roll and tilt must be in radians
    // This function only supports setting the magnetic field magnitude components

    // NOTE: This function MODIFIES the B_global variable

    // Employs a ZY Euler rotation of the frame from local orientation to global orientation
    // Tilt rotation is the rotation about the y axis
    // The factor of PI comes from the basis of the permanent magnet system being 180s
    double roty_tilt [3][3]= {{     cos(tool_tilt + M_PI),                 0.0, sin(tool_tilt + M_PI)},
                              {	  			 	      0.0,                 1.0,                   0.0},
                              {-1.0*sin(tool_tilt + M_PI),                 0.0, cos(tool_tilt + M_PI)}};

    double rotz_roll [3][3]= {{            cos(tool_roll), -1.0*sin(tool_roll),                   0.0},
                              {            sin(tool_roll),      cos(tool_roll),                   0.0},
                              {                       0.0,                 0.0,                   1.0}};
    double rot_total[3][3];
    MatrixMult3(roty_tilt,rotz_roll,rot_total);
    // This code assumes the origin of the tool frame is 0 with respect to the
    // global frame - otherwise, further matrix transformation is required
    MatrixMultVect(rot_total,B_local,B_global);
}


void global2local(double tool_tilt, double tool_roll, double B_global[3], double B_local[3])
{
    // Written by Adam Schonewille, based on function written by Andrew Lim
    // Inputs for roll and tilt must be in radians
    // This function only supports setting the magnetic field magnitude components

    // NOTE: This function MODIFIES the B_local variable

    // Employs a ZY Euler rotation of the frame from local orientation to global orientation
    // Finds the inverse of that to get the transformation in the opposite direction (global to local)
    // This function should be identical to the local2global function be with an inverse matrix found.

    // Tilt rotation is the rotation about the y axis
    // The factor of PI comes from the basis of the permanent magnet system being 180s
    double roty_tilt [3][3]= {{     cos(tool_tilt + M_PI),                 0.0, sin(tool_tilt + M_PI)},
                              {	  			 	      0.0,                 1.0,                   0.0},
                              {-1.0*sin(tool_tilt + M_PI),                 0.0, cos(tool_tilt + M_PI)}};

    double rotz_roll [3][3]= {{            cos(tool_roll), -1.0*sin(tool_roll),                   0.0},
                              {            sin(tool_roll),      cos(tool_roll),                   0.0},
                              {                       0.0,                 0.0,                   1.0}};
    double rot_local2global[3][3];
    MatrixMult3(roty_tilt,rotz_roll,rot_local2global);

    // This new transformation matrix describes the transformation from local frame to global frame,
    // The inverse will describe the transformation from the global to the local frame.
    double rot_global2local[3][3];
    cv::invert(cv::Mat(3,3,CV_64F,rot_local2global),cv::Mat(3,3,CV_64F,rot_global2local),cv::DECOMP_SVD);

    // This code assumes the origin of the tool frame is 0 with respect to the
    // global frame - otherwise, further matrix transformation is required
    MatrixMultVect(rot_global2local,B_global,B_local);
}

void calcFieldInSphericalCoords(double theta, double phi, double B_global[3], double B_sph[3])
{
    // Written by Adam Schonewille
    // Inputs for roll and tilt must be in radians
    // This function only supports setting the magnetic field magnitude components

    // NOTE: This function MODIFIES the B_sph variable

    // Employs a ZY Euler rotation of the frame from local orientation to global orientation
    // Finds the inverse of that to get the transformation in the opposite direction (global to local)

    // Phi rotation is the rotation about the y-axis
    double roty_phi [3][3]=  {{      cos(phi),   0.0,   sin(phi)},
                              {	  	      0.0,   1.0,        0.0},
                              { -1.0*sin(phi),   0.0,   cos(phi)}};
    // Theta rotation is the rotation about z-axis
    double rotz_theta [3][3]= {{  cos(theta), -1.0*sin(theta),   0.0},
                              {   sin(theta),      cos(theta),   0.0},
                              {          0.0,             0.0,   1.0}};

    double rot_sph[3][3];
    MatrixMult3(rotz_theta,roty_phi,rot_sph);
    MatrixMultVect(rot_sph, B_global, B_sph);
}



void calcIsotropicControlMatrix(double pAct[3][8], double mAct[3][8], double N[8][8])
{
    // INPUTS
    // pAct :   The cartesian X, Y, and Z coords of each of the 8 electromagnets
    // mAct :   The cartesian vector describing the magnitude and direction of each of the 8 electromagnets magnetic moment
    // N    :   A blank 8x8 matrix to store the output. (Any values will be overwritten)
    // OUTPUTS
    // N    :   Rewritten values to include the field per unit current for control.
    // Assumes that the position and orientation of the electromagnets are in cartesian coords


    for (int i = 0; i < 8; i++)
    {
        // Declare temporary variable for extracting and calculating
        double B[3] = {0.0};
        double G[5] = {0.0};
        double p[3] = {0.0};
        double m[3] = {0.0};
        for (int j = 0; j < 3; j++)
        {
            p[j] = pAct[j][i];
            m[j] = mAct[j][i];
        }
        // Now that temporary position and magnetic moment are acquired
        // get the B and G field components
        dipoleField(p,m,B);
        dipoleGradient(p,m,G);
        // Now write the calculated B and G to the control matrix
        for (int k = 0; k < 5; k++)
        {
            if (k < 3)
            {
                N[k][i] = B[k];
            }
            N[k+3][i] = G[k];

        }
    }

    // N should be filled up now
}

void calcAnisotropicControlMatrix(double pAct[3][8], double mAct[3][8], double M[3][8])
{
    // INPUTS
    // pAct :   The cartesian X, Y, and Z coords of each of the 8 electromagnets
    // mAct :   The cartesian vector describing the magnitude and direction of each of the 8 electromagnets magnetic moment
    // M    :   A blank 3x8 matrix to store the output. (Any values will be overwritten)
    // OUTPUTS
    // N    :   Rewritten values to include the field per unit current for control.
    // Assumes that the position and orientation of the electromagnets are in cartesian coords


    for (int i = 0; i < 8; i++)
    {
        // Declare temporary variable for extracting and calculating
        double B[3] = {0.0};
        double p[3] = {0.0};
        double m[3] = {0.0};
        for (int j = 0; j < 3; j++)
        {
            p[j] = pAct[j][i];
            m[j] = mAct[j][i];
        }
        // Now that temporary position and magnetic moment are acquired
        // get the B field components
        dipoleField(p,m,B);
        // Now write the calculated B to the control matrix
        for (int k = 0; k < 3; k++)
        {
            M[k][i] = B[k];
        }
    }
    // M should be filled up now
}

void maxIsotropicField( double pAct[3][8], double mAct[3][8], double maxB[3])
{
    //function [ maxB, maxF, maxG ] = max_field_force_gradient( pAct, mAct, mTool_mag )
    // % Determine the maximum field, force, and field gradient magnitude that can be produced
    // % if all the magnetic volume is concentrated at a single point at R dist from the workspace

    // % These values are useful for scaling the relative field, force, and gradient terms

    double N[8][8] = {0.0};
    double invN[8][8] = {0.0};
    calcIsotropicControlMatrix(pAct, mAct, N);
    cv::invert(cv::Mat(8,8,CV_64F,N),cv::Mat(8,8,CV_64F,invN),cv::DECOMP_SVD);

    double Bx_des[3] = {1.0, 0, 0};
    double By_des[3] = {0, 1.0, 0};
    double Bz_des[3] = {0, 0, 1.0};

    double current_x[8] = {0.0};
    double current_y[8] = {0.0};
    double current_z[8] = {0.0};
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;

    for ( int i = 0; i < 8; i++ )
    {
        double sumx = 0.0;
        double sumy = 0.0;
        double sumz = 0.0;
        for ( int j = 0; j < 3; j++ )
        {
            sumx += Bx_des[j]*invN[i][j];
            sumy += By_des[j]*invN[i][j];
            sumz += Bz_des[j]*invN[i][j];
        }
        current_x[i] = sumx;
        if ( abs(current_x[i]) > max_x)
        {
            max_x = abs(current_x[i]);
        }
        current_y[i] = sumy;
        if ( abs(current_y[i]) > max_y)
        {
            max_y = abs(current_y[i]);
        }
        current_z[i] = sumz;
        if ( abs(current_z[i]) > max_z)
        {
            max_z = abs(current_z[i]);
        }

    }
    // All currents for desired fields should have been calculated now
    // Now scale all currents to max value through normalized currents
    double sumx = 0.0;
    double sumy = 0.0;
    double sumz = 0.0;
    for ( int i = 0; i < 8; i++ )
    {
        current_x[i] /= max_x;
        current_y[i] /= max_y;
        current_z[i] /= max_z;
        // Calculate resulting B field too (only corresponding row from control matrix matters)
        sumx += current_x[i]*N[0][i];
        sumy += current_y[i]*N[1][i];
        sumz += current_z[i]*N[2][i];
    }
    maxB[0] = sumx;
    maxB[1] = sumy;
    maxB[2] = sumz;

}

void maxAnisotropicField( double pAct[3][8], double mAct[3][8], double maxB[3])
{
    //function [ maxB, maxF, maxG ] = max_field_force_gradient( pAct, mAct, mTool_mag )
    // % Determine the maximum field, force, and field gradient magnitude that can be produced
    // % if all the magnetic volume is concentrated at a single point at R dist from the workspace

    // % These values are useful for scaling the relative field, force, and gradient terms

    double M[3][8] = {0.0};
    double pseudoinvM[8][3] = {0.0};
    calcAnisotropicControlMatrix(pAct, mAct, M);
    cv::invert(cv::Mat(3,8,CV_64F,M),cv::Mat(8,3,CV_64F,pseudoinvM),cv::DECOMP_SVD);

    double Bx_des[3] = {1.0, 0, 0};
    double By_des[3] = {0, 1.0, 0};
    double Bz_des[3] = {0, 0, 1.0};

    double current_x[8] = {0.0};
    double current_y[8] = {0.0};
    double current_z[8] = {0.0};
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;

    for ( int i = 0; i < 8; i++ )
    {
        double sumx = 0.0;
        double sumy = 0.0;
        double sumz = 0.0;
        for ( int j = 0; j < 3; j++ )
        {
            sumx += Bx_des[j]*pseudoinvM[i][j];
            sumy += By_des[j]*pseudoinvM[i][j];
            sumz += Bz_des[j]*pseudoinvM[i][j];
        }
        current_x[i] = sumx;
        if ( abs(current_x[i]) > max_x)
        {
            max_x = abs(current_x[i]);
        }
        current_y[i] = sumy;
        if ( abs(current_y[i]) > max_y)
        {
            max_y = abs(current_y[i]);
        }
        current_z[i] = sumz;
        if ( abs(current_z[i]) > max_z)
        {
            max_z = abs(current_z[i]);
        }

    }
    // All currents for desired fields should have been calculated now
    // Now scale all currents to max value through normalized currents
    double sumx = 0.0;
    double sumy = 0.0;
    double sumz = 0.0;
    for ( int i = 0; i < 8; i++ )
    {
        current_x[i] /= max_x;
        current_y[i] /= max_y;
        current_z[i] /= max_z;
        // Calculate resulting B field too
        sumx += current_x[i]*M[0][i];
        sumy += current_y[i]*M[1][i];
        sumz += current_z[i]*M[2][i];
    }
    maxB[0] = sumx;
    maxB[1] = sumy;
    maxB[2] = sumz;

}


// TODO: REMOVE
//template <size_t rows, size_t columns>
//void invertMatrix(double (&A)[rows][columns], double (&A_dagger)[columns][rows]) //int rows, int columns,    // Passing by reference not pointers
//{
////    // Using OpenCV to invert the matrix:
////    // A_Dagger, the inverse of an MxM matrix, will also be MxM
////    // An NxM matrix A will have a psuedoinverse of size MxN, A_dagger
////    cv::Mat B = cv::Mat(rows, columns, CV_32F, A);
////    cv::Mat B_dagger = cv::Mat(columns, rows, CV_32F, A_dagger);
//////    cv::_InputArray C = B;
////    cv::invert(B,B_dagger);
////    // Convert back to an array

////    // rows are colums and columns are rows for the inverted matrix
////    for ( int i = 0; i < rows; i++ )
////    {
////        for ( int j = 0; j < columns; j++ )
////        {
////            // only need to return the inverse so A is not needed
//////            A[i][j] = B.at<double>(i,j);
////            // Matrix index to array index [i][j] - > [i*cols + j]
////            // (but inverse matrix is transposed)
////            A_dagger[j*rows+i] = B_dagger.at<double>(j,i);
////        }
////    }

//    // Testing math functions here
////    const int r = 3;
////    const int c = 3;
////    double AA[r][c] = {{-3.0, 4.0, -3.0},
////                       {0.0, 1.0, 6.0},
////                       {-12.0, 0.0, 10.0}};
////    double AAinv[c][r] = {0.0};

//    // recreate Matrix from pointer

////    double AA[rows][columns];
////    double AAinv[columns][rows];
////    for ( int i = 0; i < rows; i++ )
////    {
////        for (int j = 0; j < columns; j++)
////        {
////            // Access the same memory from the pointer
////            AA[i][j] = *(A+i*columns+j);
////            // Access the same memory and
////            AAinv[j][i] = 0.0;
////        }
////    }

//    cv::invert(cv::Mat(rows,columns,CV_64F,A),cv::Mat(columns,rows,CV_64F,A_dagger));

////    // Using OpenCV to invert the matrix:
////    // A_Dagger, the inverse of an MxM matrix, will also be MxM
////    // An NxM matrix A will have a psuedoinverse of size MxN, A_dagger
////    cv::Mat B = cv::Mat(rows, columns, CV_64F, A); // CV_32F 32bit float  to CV_32S 32bit signed int
////    qInfo() << "Should be: " << A[1][2] << " Is Actually: " << B.at<double>(1,2);
//////    std::cout << B;
////    cv::Mat B_dagger = cv::Mat(columns, rows, CV_64F, A_dagger);
//////    cv::_InputArray C = B;
////    cv::invert(B,B_dagger);
////    // Convert back to an array
////     qInfo() << "B Matrix: ";
////    // rows are colums and columns are rows for the inverted matrix
////    for ( size_t i = 0; i < rows; i++ )
////    {
////        for ( size_t j = 0; j < columns; j++ )
////        {
////            // only need to return the inverse so A is not needed
//////            A[i][j] = B.at<double>(i,j);
////            // Matrix index to array index [i][j] - > [i*cols + j]
////            // (but inverse matrix is transposed)

////            A_dagger[j][i] = B_dagger.at<double>(j,i);
//////            qInfo() << B_dagger.at<double>(j,i);
////        }

////        qInfo() << B.at<double>(i,0) << B.at<double>(i,1) << B.at<double>(i,2);
////        qInfo() << B_dagger.at<double>(i,0) << B_dagger.at<double>(i,1) << B_dagger.at<double>(i,2);

////    }

//}

double norm(double A[12])
{
    // this looks just wrong
    // why 8 and not 12?

    double result = 0;
    for (int n=0; n<8; n++)
    {
        result += A[n]*A[n];
    }
    result = sqrt(result);
    return result;
}

void MatrixInvert(double H[12][12], double result[12][12])
{
  // This function finds the inverse of a 12 x 12 matrix using Guassian Elimination,
  // or Reduced-row Echelon form.
  // Create a 12 x 24 Matrix of the form [ H | I ] where I is the identity matrix
  double invert[12][24] = {0};
  for (int i=0; i<12; i++)
  {
    for (int j=0; j<12; j++)
    {
      invert[i][j] = H[i][j];
    }
    invert[i][i+12] = 1;
  }
  // Matrix now looks like [ H | I ] Matrix
  // Perform reduced row-echelon algorithm.
  for (int i=0; i<12-1; i++)
  {
    double factor = invert[i][i]; //diagonal value on left side
    // Normalize rows so leading value is 1
    for (int j=0; j<24; j++)
    {
      invert[i][j] = invert[i][j]/factor;
    }
    // leading row should be 1 now
    // Subtract row from lower rows
    for (int j=i+1; j<12; j++)
    {
      factor = invert[j][i]; //value below the leading 1
      for (int k=0; k<24; k++)
      {
        invert[j][k] = invert[j][k] - invert[i][k]*factor; // top row above multiplied by correction factor subtracted from current row
      }
    }

  }
  // Normalizing last row
  double factor = invert[11][11]; //diagonal value
  // Normalize final row
    // if( abs(factor) > 1e-16 )
    // {
        for (int j=0; j<24; j++)
    {
        invert[11][j] = invert[11][j]/factor;
    }
    // }
  // Should look like:
  // {1 a b c d | k l m n o}
  // {0 1 e f g | p q r s t}
  // {0 0 1 h i | u v w x y}
  // {0 0 0 1 j | z .......}
  // {0 0 0 0 1 | .........}
  // Now work upwards,
  for (int i=12-1; i>=0+1; i--)
  {
    for (int j=i-1; j>=0; j--)
    {
      factor = invert[j][i]; // value above diagonal
      for (int k=0; k<24; k++)
      {
        invert[j][k] = invert[j][k] - invert[i][k]*factor; // bottom row below multiplied by correction factor subtracted from current row
      }
    }
  }
  // record result
  for (int i = 0; i < 12; i++)
  {
      for (int j = 0; j < 12; j++)
      {
        result[i][j] = invert[i][j+12];
      }
  }
}


double calculateTheta(double angle1, double angle2)
{
    // For Surgeon Simulator control mode the angles are as follows:
    // angle3 : Theta, the angle between the z-axis and the desired heading of the gripper
    // angle1 : Gamma, the x plane rotation angle
    // angle2 : Beta, the out of plane rotation angle

//    double test = atan( sqrt( pow(sin(angle2),2)+pow(cos(angle2),2)*pow(sin(angle1),2) ) / (cos(angle1)*cos(angle2)) );
//    qDebug() << test;

    double angle3 = atan( sqrt( pow(sin(angle2),2)+pow(cos(angle2),2)*pow(sin(angle1),2) ) / (cos(angle1)*cos(angle2)) );
    return angle3;
    // This function is not called anywhere in the code?
    // Yes it was in Gamepad and now in callbacks for polar angle control
}


// TODO: REMOVE

//void setup_parameters(double pAct_sph[3][8], double rAct_sph[2][8], double pAct[3][8], double RzyAct[3][2][8], double axesAct[3][8] )
//{
//    // function [pAct,RzyAct,axesAct] = setup_parameters(pAct_sph, rAct_sph)
//    // % Inputs:
//    // % pAct_sph = actuator magnet positions in spherical coordinates [radius(m); azimuth(deg); inclination (deg)];
//    // % rAct_sph = actuator magnet rotational axes in spherical coordinates [azimuth(deg); inclination (deg)];
//    // % Outputs:
//    // % pAct: xyz coordinates of actuator magnet centers [m]
//    // % RzyAct: ZY Euler angle rotation matrix for each actuator magnet
//    // % axesAct: direction of axis of rotation in xyz
//    // % NOTE: These values are returned via pass-by-reference call to the function. The input arrays are directly editted.

//    // % Author: Patrick Ryan
//    // % Adapted by: Adam Schonewille
//    // % Rewritten in C++ from MATLAB by Adam Schonewille

//    // Purpose of this function is to convert from spherical basis to cartesian and produce rotation matrix for permanent magnets.

//    // Magnet locations
//    for (int n=0; n<numAct0; n++)
//    {
//        pAct[0][n] = pAct_sph[0][n]*sin(pAct_sph[2][n])*cos(pAct_sph[1][n]);
//        pAct[1][n] = pAct_sph[0][n]*sin(pAct_sph[2][n])*sin(pAct_sph[1][n]);
//        pAct[2][n] = pAct_sph[0][n]*cos(pAct_sph[2][n]);
//    }
//    // Axes & constants
//    // Omega of revolution
//    for (int n=0; n<numAct0; n++)
//    {
//      axesAct[0][n] = sin(rAct_sph[1][n])*cos(rAct_sph[0][n]);
//      axesAct[1][n] = sin(rAct_sph[1][n])*sin(rAct_sph[0][n]);
//      axesAct[2][n] = cos(rAct_sph[1][n]);
//    }
//    for (int i=0; i<numAct0; i++)
//    {
//        RzyAct[0][0][i] =      cos(rAct_sph[1][i])*cos(rAct_sph[0][i]);
//        RzyAct[0][1][i] = -1.0*sin(rAct_sph[0][i]);
//        RzyAct[1][0][i] =      cos(rAct_sph[1][i])*sin(rAct_sph[0][i]);
//        RzyAct[1][1][i] =      cos(rAct_sph[0][i]);
//        RzyAct[2][0][i] = -1.0*sin(rAct_sph[1][i]);
//        RzyAct[2][1][i] =  0.0;
//    }
//}

