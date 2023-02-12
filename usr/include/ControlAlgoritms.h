#ifndef CONTROLALGORITMS_H
#define CONTROLALGORITMS_H

#include <iostream>
#include <list>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <complex>
#include <math.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "matrix_utils.h"
#include "statistics.h"

using namespace std;
using namespace Eigen;


// Purpose: The calculation of the range and null space matrices of an
//          mxn matrix A



// Calculate the feedback of the controller, for a given set of eigenvalues
MatrixXd place(MatrixXd A, MatrixXd B, MatrixXcd P);

// Calculate the feedback of the controller
MatrixXd dlqr(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);

// Calculate the feedback of the observer
MatrixXd dlqe(MatrixXd A, MatrixXd G, MatrixXd C, MatrixXd Q, MatrixXd R);

//Solve Sylvester matrix equation
// A*X + X*B + C = 0
MatrixXcd sylvester(MatrixXcd A, MatrixXcd B, MatrixXcd C );

//Solve Sylvester matrix equation
// A*X + X*A' + C = 0
// MatrixXcd sylvester(MatrixXcd A,MatrixXcd C );


MatrixXd lyapunov( MatrixXd A, MatrixXd B);
MatrixXd lyapunov( MatrixXd A, MatrixXd B, MatrixXd C);

// Solve discrete lyaponov equation
// AXA' - X + B = 0
MatrixXd discrete_lyapunov( MatrixXd A, MatrixXd B);


MatrixXcd eigenvalues( MatrixXd A );
MatrixXcd eigenvectors( MatrixXd A );

MatrixXd characteristicPolynomial( MatrixXd A );

/**
 * return controlability matrix
 */
MatrixXd ctrb( MatrixXd A, MatrixXd B);
MatrixXd obsv( MatrixXd A, MatrixXd C);

bool is_ctrb( MatrixXd A, MatrixXd B );
bool is_obsv( MatrixXd A, MatrixXd C );

MatrixXd gramian( MatrixXd, MatrixXd );

bool ResetModelSamplingRate( MatrixXd A, MatrixXd B, time_t old_sampling_rate, MatrixXd& A_new, MatrixXd& B_new, time_t new_sampling_rate);

//These calls are used to generate a state-space model from my own least square model
// Y(n+1) = A_ Y(n) + B_ U(n) + C_
MatrixXd derive_Psi( list<MatrixXd> A, const unsigned order );
MatrixXd fir2ss_Psi( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2ss_Gamma( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );

MatrixXd fir2ss_C(MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2ss_Fixedpoint( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2ss_C1(MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2ss_Fixedpoint1( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2ss_C2(MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2ss_Fixedpoint2( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );
MatrixXd fir2fir_C( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order );


/**
 * Check if model's matrices are of the correct dimensions for operation
 */
bool is_model_valid( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_);


// General functions for optimal observabilty.
MatrixXd ss_measurement_noise_matrix( MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y );
MatrixXd ss_process_noise_matrix( MatrixXd A, MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y);
MatrixXd ss_measurement_noise_cov( MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y );
MatrixXd ss_process_noise_cov( MatrixXd A, MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y);
#endif // CONTROLALGORITMS_H
