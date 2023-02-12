#include <iostream>
#include <float.h>
#include <Eigen/Dense>
#include <random>
#include <chrono>

using namespace std;
using namespace Eigen;

#ifndef STATS_
#define STATS_

#define PI 3.1415926536

//static std::default_random_engine generator;

double AWGN_generator(void);
double kahan_sum( VectorXd& x );
double calc_mean( VectorXd& x );
double calc_variance( VectorXd& x );
double calc_covariance( VectorXd& x, VectorXd& y );
MatrixXd calc_matrix_covariance(MatrixXd& X);
double calc_correlation( VectorXd& x, VectorXd& y );

double calc_auto_covariance( VectorXd& y, int i = 0 );
double calc_auto_correlation( VectorXd& y,  int i = 0 );
double calc_cross_covariance( VectorXd& u, VectorXd& y, int i = 0 );
double calc_cross_correlation( VectorXd& u, VectorXd& y, int i = 0 );


//generate an impulse reponse over horison n
MatrixXd impulse_response( VectorXd& u, VectorXd& y, int n );
MatrixXd step_response( VectorXd& u, VectorXd& y, int n );
MatrixXd generate_cross_correlation_vector( VectorXd& x, VectorXd& y, int n );

MatrixXd generate_randn_matrix( const int rows, const int cols, const double mean = 0, const double std_deviation = 1);

#endif
