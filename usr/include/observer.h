#ifndef OBSERVER_H
#define OBSERVER_H

#include <mutex>
#include <list>
#include <Eigen/Dense>
#include "statistics.h"
#include "ControlAlgoritms.h"

const int MAX_W_SIZE = 3600; // The maximum lenght of the process noise queue
const int MIN_W_SIZE = 300;  // when to start with optimally calculating Q and R

class observer
{
    public:
        //observer(mbTcpClient * mb, int pv_block_address, int mv_block_address, unsigned order = 1, time_t sr = 1);
        observer( MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_,unsigned order = 1);
        observer( unsigned order = 1);

        virtual ~observer();
        observer(const observer& other);
        observer& operator=(const observer& other);

        bool update_model( MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_);
        bool get_model( MatrixXd &A_, MatrixXd &B_, MatrixXd &C_, MatrixXd D_);

        unsigned get_B_cols(void){ return B.cols();}
        unsigned get_C_rows(void){ return C.rows();}

        unsigned get_order(void);

        bool update_filter_parms( MatrixXd Q, MatrixXd R);
        bool is_filter_parms_valid( MatrixXd Q, MatrixXd R);

        MatrixXd get_X( void );


        bool update_state(  MatrixXd U , MatrixXd Y );

        //void start();
        //void stop();
        //bool running(){ return th.joinable();}
    protected:

    private:
        // Communication interface
        //mbTcpClient * mb_server;
        //int mb_pv_start_address;
        //int mb_mv_start_address;

        unsigned system_order; // dui aan hoeveel data die observer moet stoor
        //time_t sampling_rate;


        MatrixXd X; //State matrix;
        list<MatrixXd>Xs; //historical data for calculating the process covariance
        list<MatrixXd>Ys; //historical data for calculating the measurement covariance.
        MatrixXd Y_curr_err,Y_prev_err;

        MatrixXd Lp; //
        MatrixXd A,B,C,D; //Model of plant
        //MatrixXd Q,R; // For optimizing the kalman filter

        bool initialized;

        //for updating
        mutex mtx;
        //thread th;

        //bool stop_updating_thread;
        //void update_thread(void);
        bool update_prediction_estimator(MatrixXd, MatrixXd);
        bool auto_update_filter_parms(void);
        bool auto_set_default_filter_parms(void);
        //bool is_model_valid( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_);
        bool is_model_consistent( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_);

};

#endif // OBSERVER_H
