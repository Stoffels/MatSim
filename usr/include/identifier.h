/* Written by Herman
 * 18/5/2013 - added decomposing
 */

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

#ifndef IDENTIFIER
#define IDENTIFIER

#include "utilities.h"
#include "history.h"
#include "matrix_utils.h"
#include "ControlAlgoritms.h"
#include "statistics.h"
#include "simulate.h"

using namespace std;
using namespace Eigen;

#ifdef __BORLANDC__
#pragma hdrstop
#endif

//#ifndef WX_PRECOMP
//	#include <wx/wx.h>
//	#include <wx/frame.h>
//#else
//	#include <wx/wxprec.h>
//#endif



class identifier;

class identifier_table
{
public:
    identifier_table( historian_table * h );
    ~identifier_table( void );

    bool contains( const string tag );

    bool add( const string tag, const int order = 1, time_t sampling_rate = 1 );
    bool add( const string tag, const identifier );
    bool remove( const string tag );
    identifier get_identifier( const string tag );

    bool add_cv( const string tag, const string cv );
    bool add_cv( const string tag, list<string> cvs );
    bool contains_cv( const string tag, const string cv );

    bool add_mv( const string tag, const string mv );
    bool add_mv( const string tag, list<string> mvs );
    bool contains_mv( const string tag, const string mv );

    bool get_previous_sqrt_error( const string tag, double & err ); //can only call after identification has been done
    bool get_model_correlation( const string tag, double &r, const int deadtime = 0 );
    bool get_noise_variance( const string tag, double &s, const int deadtime = 0 );
    bool find_model_deadtime( const string tag, int & deadtime );
    bool return_sample_rate( const string tag, time_t& sample_rate );
    bool identify_model( const string tag, const int deadtime = 0 );
    bool get_system_parameters( const string tag, MatrixXd& A, MatrixXd& B, MatrixXd& C );
    bool get_unity_steadystate_system( const string tag, MatrixXd& A, MatrixXd& B, MatrixXd& C );

    bool get_Kp(const string tag, MatrixXd& Kp ); //calculate the gain
    bool get_RGA(const string tag, MatrixXd& RGA);
    bool get_RGA_inverse(const string tag, MatrixXd& iRGA );
    bool get_condition_number(const string tag, double& cn );
    bool get_SV_decomposision( const string tag, MatrixXd& U, MatrixXd& S, MatrixXd& V );
    bool decompose_variables(const string tag);

    bool is_identified( const string tag );

    bool getCVs(const string tag, list <string> & cvs );
    bool getMVs(const string tag, list <string> & mvs );

    int get_CV_count( const string tag );
    int get_MV_count( const string tag );

    bool get_order( const string tag, int& order );

    list<string> get_model_tags(void);



private:
    struct ltstr
    {
        bool operator()(const string x, const string y) const
        {
            return (x < y);
        }
    };
    map< const string, identifier, ltstr > table;
    historian_table * historian;
};


//Hierdie object is enigste taak is om 'n model te identifiseeer. Die model is:
// y_{n+1} = \sum_{i=0}{n}(a_i y_i) + \sum{i=0}{n} (b_i u_i) + c;
// 3 vektore word teruggegee nl: A, B, C;
class identifier
{
public:
    identifier( historian_table * h , const int o, time_t sr );
    identifier( const identifier& );

    ~identifier(void);

    //constructors
    identifier & operator=( const identifier& );

    //add control variables
    bool add_cv( const string cv );
    bool contains_cv( const string cv );


    //add manipulated variables
    bool add_mv( const string mv );
    bool contains_mv( const string cv );

    bool get_system_parameters( MatrixXd& A, MatrixXd& B, MatrixXd& C );
    bool get_unity_steadystate_system( MatrixXd& A_, MatrixXd& B_, MatrixXd& C_ );
    MatrixXd get_Kp(void); //calculate the gain
    MatrixXd get_RGA(void);
    MatrixXd get_RGA_inverse(void);


    double get_condition_number(void);
    bool get_SV_decomposision( MatrixXd& U, MatrixXd& S, MatrixXd& V );
    bool decompose_variables(void);
    MatrixXd get_CV_data_array(void);
    MatrixXd get_MV_data_array(void);

    double get_previous_sqrt_error( void );
    int find_model_deadtime( void );

    time_t return_sample_rate( void )
    {
        return sampling_rate;
    }

    list <string> getCVs(void)
    {
        return CVs;
    }
    list <string> getMVs(void)
    {
        return MVs;
    }

    int get_order( void )
    {
        return order;
    }

    bool is_identified( void )
    {
        return identified;
    }

    double identify_model( const int deadtime = 0 );  //identify model and return sqrt_error

    int get_CV_count( void )
    {
        return CVs.size();
    }
    int get_MV_count( void )
    {
        return MVs.size();
    }


private:
    historian_table * historian;

    list<string> CVs;
    list<string> MVs;

    bool identified; //set flag if system has been identified;
    MatrixXd b;

    int order;
    time_t sampling_rate;
    double sqrt_error;

private:
    double calc_sqrt_error( const int deadtime = 0 );
};


#endif
