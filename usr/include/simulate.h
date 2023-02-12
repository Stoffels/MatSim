#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <climits>
#include <cfloat>

#include <Eigen/Dense>
#include <string>


#ifdef __BORLANDC__
	#pragma hdrstop
#endif

//#ifndef WX_PRECOMP
//	#include <wx/wx.h>
//	#include <wx/frame.h>
//#else
//	#include <wx/wxprec.h>
//#endif


#ifndef ARX_SIM
#define ARX_SIM

#include "history.h"
#include "identifier.h"
#include "matrix_utils.h"

using namespace std;
using namespace Eigen;

class identifier_table;

//this is to simulate single input-output models
class arx_siso_simulate{
public:
    arx_siso_simulate( const MatrixXd, const MatrixXd, const MatrixXd );
    ~arx_siso_simulate( void );

    double * step_response( const long len );
    double * impulse_response( const long len );
    double * general_response( double * mv, const long len, double * init = NULL );
    double fixed_point( double mv = 0.0 );
private:
    MatrixXd A,B,C;
    MatrixXd calc_fixed_point( MatrixXd A_, MatrixXd f );
};


//simulate multi input, single output models
class arx_simulate{
public:
    arx_simulate( const string model_tag, historian_table * h, identifier_table * i );
    ~arx_simulate( void );

    MatrixXd general_response( void );

private:
    string model_tagname;
    historian_table * pHistorian;
    identifier_table * pIdentifier;

    vector<MatrixXd> get_Un_vector( void );
    MatrixXd get_historian_vector( list<string>, const unsigned index );
    unsigned get_minimum_historian_vector_length(list<string>);
    MatrixXd calc_fixed_point( MatrixXd U_0 );
};



#endif
