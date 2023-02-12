#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <climits>
#include <cfloat>
#include <complex>


#ifndef MATRIX_UTILS
#define MATRIX_UTILS

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

//#define MAX_ERROR 1e-12
#define tol 2.2204e-16

int get_sum_of_columns( list<MatrixXd> A);
int get_sum_of_columns( list<MatrixXcd> A);
MatrixXd concat_matrix_list( list<MatrixXd> l );
MatrixXcd concat_MatrixXcd_list( list<MatrixXcd> l );
MatrixXd stack_matrix_list( list<MatrixXd> l );
MatrixXd concat_matrix_vector( vector<MatrixXd> l );
MatrixXcd to_complex( MatrixXd A );

list<MatrixXd> expand_matrix( MatrixXd & M, const int o );  //expand matrix into its subcomponents

vector<MatrixXd> list2vector( list<MatrixXd> );

MatrixXd get_sum( list<MatrixXd> l );
bool is_invertible( MatrixXd A );
MatrixXd pinv( MatrixXd A );

long Rank( MatrixXd A );
MatrixXd Range( MatrixXd A );
MatrixXd Nullity( MatrixXd A );
bool NullRangeSpace ( MatrixXd A, MatrixXd& N, MatrixXd& R);
bool INOU( MatrixXd R, MatrixXd Q, MatrixXd& Qr, MatrixXd Qou );
// purpose: To decompose the matrix Q into two subspaces: Qr - projection of Q into range space of R,
// and Qou, the subspace outside the range space R

MatrixXd clamp( MatrixXd M, MatrixXd Min, MatrixXd Max);
bool is_nan( MatrixXd M);
bool is_finite( MatrixXd M);

#endif

