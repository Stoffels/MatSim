#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <list>


#include "matrix_utils.h"

using namespace std;
using namespace Eigen;


class simulator
{
    public:
        simulator();
        simulator(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd X0, time_t sr = 1);
        virtual ~simulator();
        simulator(const simulator& other);
        simulator& operator=(const simulator& other);

        bool load_model(MatrixXd,MatrixXd,MatrixXd);
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

    protected:

    private:
        thread th;
        std::mutex calc_mutex;

        bool running;

        //Model information
        vector <MatrixXd> A_n;
        vector <MatrixXd> B_n;
        MatrixXd C;

        //Dynamic variable information
        vector <MatrixXd> Y_n;
        vector <MatrixXd> U_n;

        //Current inputs and outputs
        MatrixXd U_now; //The current input to the system
        MatrixXd Y_now; //the current output

        time_t sampling_rate;

        //helper methods
        MatrixXd calc_steady_state_input( MatrixXd Y_inf );
        MatrixXd calc_steady_state_output( MatrixXd U_inf );
        void convert_to_first_order( void );        //if order > 1, convert to 1st order
        void update_simulation( void );
        int get_order( MatrixXd&  );
};

#endif // SIMULATOR_H
