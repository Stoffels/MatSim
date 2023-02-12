#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mutex>
#include <list>
#include <Eigen/Dense>
#include "statistics.h"
#include "ControlAlgoritms.h"

using namespace std;
using namespace Eigen;

enum controller_type {
    OBSV,
    OBSV_INTEGRATOR
};


class controller
{
    public:
        controller(const controller_type ct, unsigned order = 1);
        controller(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_, const controller_type ct, unsigned order = 1);
        virtual ~controller();
        controller(const controller& other);
        controller& operator=(const controller& other);

        unsigned get_order(void);

        bool update_model( MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_);
        bool get_model( MatrixXd &A_, MatrixXd &B_, MatrixXd &C_, MatrixXd D_);
        bool update_QR_matrices( MatrixXd Q_, MatrixXd R_);

        MatrixXd calc_output( MatrixXd Yr, MatrixXd X);
        MatrixXd calc_output( MatrixXd Yr, MatrixXd X, MatrixXd Y);

        MatrixXd calc_state_pv( MatrixXd X ); //calculate the pv from the given state

    protected:
        MatrixXd A,B,C,D;       //Model of plant
        controller_type c_type;

        unsigned system_order;  //Used to remember its ordered...thats it...better to move to parent

        MatrixXd Q, R;
        MatrixXd K, K1;         // Matrices for optimal feedback control
        MatrixXd N;             // Dimensinoing matrix;
        bool initialized;
        MatrixXd X_int;

        //for updating
        mutex mtx;

        //private methods
        bool is_model_consistent( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_);
        MatrixXd calc_feedback_matrix(MatrixXd, MatrixXd);
        MatrixXd rscale( MatrixXd K_);
        bool update_NKK1_matrices( MatrixXd Q_, MatrixXd R_);
    private:
};

#endif // CONTROLLER_H
