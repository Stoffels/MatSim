//Rev 1.1
// 28 Aug 2017 - added return_history_data_array call, for returning data in a double array
// 1 May 2012 - Update historian to work with Eigen matrices
// 31 August 2012 - add some statistics for fun
// 31 Jan 2017 - Added get_last_logged_value() function to return the last logged value

#include <iostream>
#include <deque>
#include <map>
#include <list>
#include <vector>
#include <string>
#include <ctime>
#include <float.h>

#include "statistics.h"

#include <Eigen/Dense>



using namespace std;
using namespace Eigen;


#ifndef HISTORY
#define HISTORY


class timestamped_variable;
class historian_variable;


class historian_table
{
public:
    historian_table(const long = 10000 );
    ~historian_table();

    historian_table( const historian_table& );
    historian_table& operator=(const historian_table& );

    bool add( const string tag_name ,
              const string eu = "%",
              const string desc = "" );
    bool remove( const string tag_name );
    bool clear( void );

    bool contains( const string tag_name );

    bool log( const string tag_name, const double value, const time_t = -1 );
    bool clear_variable_history( void );
    bool size( const string tag_name, unsigned& len );

    bool get_snapshot_value(
        const string tag_name,
        const unsigned i,
        double& value );
    bool get_snapshot_value(
        const string tag_name,
        const time_t i,
        double& value );

    bool get_normalized_snapshot_value( const string tag_name, const unsigned i, double & val );
    bool get_normalized_snapshot_value( const string tag_name, const time_t t, double & val );


    bool get_snapshot_time(
        const string tag_name,
        const unsigned i,
        time_t& the_time );

    bool return_history_vector( const string tag_name, VectorXd& values );
    bool return_history_vector( const string tag_name, VectorXd& values, const time_t sampling_rate );

    bool return_normalized_history_vector( const string tag_name, VectorXd& values );
    bool return_normalized_history_vector( const string tag_name, VectorXd& values, const time_t sampling_rate );
    bool return_normalized_history_matrix( list<string> tagnames, MatrixXd& values );





    bool get_log_time_interval( const string tag_name, time_t & dT );
    bool get_start_time( const string tag_name, time_t & T );
    bool get_end_time( const string tag_name, time_t & T );

    bool get_max_value( const string tag_name, double & val );
    bool get_min_value( const string tag_name, double & val );
    bool get_last_logged_value( const string tag_name, double & val);

    list<string> return_tagnames(void);
    int return_tag_count( void )
    {
        return table.size();
    }

    string get_engineering_units(const string tag_name );
    string get_description( const string tag_name );

    //kry die lengte van dit tags wat die kortste is in terme van logging
    bool get_min_length( list <string> tag_names, unsigned & len );

    bool set_LowerRangeValue( const string tag_name, const double val );
    bool get_LowerRangeValue( const string tag_name, double & val );
    bool set_UpperRangeValue( const string tag_name, const double val  );
    bool get_UpperRangeValue( const string tag_name, double & val );

    MatrixXd get_correlation_table( void );
    MatrixXd get_correlation_table( list<string> tag_names );
    MatrixXd calc_covariance_table( void );
    MatrixXd calc_covariance_table( list<string> tag_names );

    double get_mean( const string tag_name );
    double get_variance( const string tag_name );

private:
    struct ltstr
    {
        bool operator()(const string x, const string y) const
        {
            return (x < y);
        }
    };
    map< const string, historian_variable, ltstr > table;
    unsigned long max_length;
};




class historian_variable
{
public:
    historian_variable(
        const long = 1000,
        const string = "",
        const string = ""
    );
    historian_variable( const historian_variable& );
    ~historian_variable();

    historian_variable& operator=(const historian_variable& );

    void log( const double value, const time_t = -1 );
    void clear_variable_history(void); //clear all the history of this tag.

    double get_snapshot_value( const unsigned )const;
    double get_snapshot_value( const time_t ) const;
    double get_normalized_snapshot_value( const unsigned )const;
    double get_normalized_snapshot_value( const time_t ) const;


    time_t get_snapshot_time( const unsigned )const;
    double  operator[](unsigned)const;    //index RO value of variable

    string get_engineering_units(void)
    {
        return engineering_units;
    }
    string get_description(void)
    {
        return description;
    }

    //this is for the normalization of the variables.
    void set_LowerRangeValue( const double l ){ LRV = l; };
    double get_LowerRangeValue( void ){ return LRV; }
    void set_UpperRangeValue( const double u ){ URV = u; };
    double get_UpperRangeValue( void ){ return URV; }


    VectorXd get_raw_history_vector( void );
    VectorXd get_normalized_history_vector( void );


    unsigned size( void );


    time_t get_log_interval( void );
    time_t get_start_time( void );
    time_t get_end_time( void );

    double get_max_value( void );
    double get_min_value( void );
    double get_last_logged_value( void);

private:
    string engineering_units;
    string description;
    deque< timestamped_variable > history;
    unsigned long maximum_length;
    double min_value, max_value;
    double last_logged_value;
    double LRV, URV; //lower and upper range values
};





//This object keep track of the time a logging is done.
class timestamped_variable
{
    friend int operator==(const timestamped_variable&, const timestamped_variable&);
    friend int operator!=(const timestamped_variable&, const timestamped_variable&);
    friend int operator< (const timestamped_variable&, const timestamped_variable&);
    friend int operator<=(const timestamped_variable&, const timestamped_variable&);
    friend int operator> (const timestamped_variable&, const timestamped_variable&);
    friend int operator>=(const timestamped_variable&, const timestamped_variable&);
    friend ostream& operator<<(ostream&, const timestamped_variable& );
public:
    timestamped_variable( const double value = 0.0, const time_t = -1 );
    timestamped_variable( const timestamped_variable& );
    timestamped_variable & operator=( const timestamped_variable& );
    time_t get_snapshot_time( void )const;
    double get_snapshot_value(void )const;
    ~timestamped_variable();

private:
    double snapshot_value;
    time_t snapshot_time;   //long
};

#endif
