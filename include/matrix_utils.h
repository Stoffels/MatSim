#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <climits>
#include <cfloat>


#ifndef MATRIX_UTILS
#define MATRIX_UTILS

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

MatrixXd concat_matrix_list( list<MatrixXd> l );
MatrixXd concat_matrix_vector( vector<MatrixXd> l );
list<MatrixXd> expand_matrix( MatrixXd & M, const int o );  //expand matrix into its subcomponents

vector<MatrixXd> list2vector( list<MatrixXd> );

MatrixXd get_sum( list<MatrixXd> l );
MatrixXd pinv( MatrixXd A );

#endif

