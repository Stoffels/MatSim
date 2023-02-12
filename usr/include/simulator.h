#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <list>
#include <ctime>
#include <cstdlib>

#include "matrix_utils.h"
#include "ControlAlgoritms.h"

using namespace std;
using namespace Eigen;


class fir_simulator
{
public:
    fir_simulator();
    fir_simulator(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd X0, time_t sr = 1);
    virtual ~fir_simulator();
    fir_simulator(const fir_simulator& other);
    fir_simulator& operator=(const fir_simulator& other);

    bool load_model(MatrixXd,MatrixXd,MatrixXd);
    bool load_model(MatrixXd,MatrixXd,MatrixXd, MatrixXd X0, time_t sr = 1);
    bool read_model(MatrixXd&, MatrixXd&, MatrixXd&, MatrixXd&, time_t& );

    bool set_sampling_rate( time_t );
    bool set_initial_state( MatrixXd Y_inf );

    //writing to the system
    bool set_input( MatrixXd U );
    MatrixXd get_input( void );

    //reading from the system and states
    bool set_output( MatrixXd Y_inf);
    MatrixXd get_output( void );


    void start( void );
    void stop(void);

    int get_process_variable_count( void );
    int get_control_variable_count( void );
    int get_system_order( void );
    time_t get_sampling_rate( void );

    bool generate_random_model( int pv_count, int cv_count, int ordr, time_t s_rate);
    bool generate_stable_random_model( int pv_count, int cv_count, int ordr, time_t s_rate);

    bool is_initialized( void )
    {
        return initialized;
    };

    bool is_running( void )
    {
        return running;
    };

protected:

private:
    thread th;
    std::mutex calc_mutex;

    bool running;

    //Model information
    vector <MatrixXd> A_n;
    vector <MatrixXd> B_n;
    MatrixXd A,B,C;

    //Dynamic variable information
    vector <MatrixXd> Y_n;
    vector <MatrixXd> U_n;

    //Current inputs and outputs
    MatrixXd U_now; //The current input to the system
    MatrixXd Y_now; //the current output

    //noise matrices

    time_t sampling_rate = 1;

    //helper methods
    MatrixXd calc_steady_state_input( MatrixXd Y_inf );
    MatrixXd calc_steady_state_output( MatrixXd U_inf );
    bool get_time_normalized_matrices( MatrixXd &A_new, MatrixXd &B_new, MatrixXd &C_new, time_t &sr_new);
    void update_simulation( void );
    int get_order( MatrixXd&  );
    bool initialized = false;

};

class ss_simulator
{
public:
    ss_simulator();
    ss_simulator(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_, time_t sr = 1);
    virtual ~ss_simulator();
    ss_simulator(const ss_simulator& other);
    ss_simulator& operator=(const ss_simulator& other);

    bool load_model(MatrixXd,MatrixXd,MatrixXd, MatrixXd X0, time_t sr = 1);
    bool read_model(MatrixXd&, MatrixXd&, MatrixXd&, MatrixXd&, time_t& );

    bool set_initial_state( MatrixXd Y_inf );

    // writing to the system
    bool set_input( MatrixXd U );
    MatrixXd get_input( void );

    bool set_state( MatrixXd X_);
    MatrixXd get_state( void );

    bool set_process_noise( const double std_dev);
    bool set_measurement_noise( const double std_dev);

    //reading from the system and states
    bool set_output( MatrixXd Y_inf);
    MatrixXd get_output( void );

    int get_process_variable_count( void );
    int get_control_variable_count( void );

    time_t get_sampling_rate( void );


    void start( void );
    void stop(void);

    bool is_initialized( void );
    bool is_running( void );

    bool generate_random_model( int pv_count, int cv_count, int ordr = 1, time_t s_rate = 1);
    bool generate_stable_random_model( int pv_count, int cv_count, int ordr = 1, time_t s_rate = 1);

protected:

private:
    thread th;
    std::mutex calc_mutex;
    void update_simulation( void );

    MatrixXd A,B,C,D;
    time_t sampling_rate;

    MatrixXd U_now; //input vector;
    MatrixXd X; //current state

    //noise matrices
    MatrixXd v_n;
    double v_std;
    MatrixXd w_n;
    double w_std;

    bool running;
    bool initialized = false;

};



#endif // SIMULATOR_H
