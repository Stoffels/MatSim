#include "controller.h"

controller::controller(const controller_type ct, unsigned order):
    c_type(ct),system_order(order)
{
    //ctor
    initialized = false;
}

/* Pre: Model is valid */
controller::controller(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_, const controller_type ct, unsigned order)
    :A(A_),B(B_),C(C_),D(D_), c_type(ct),system_order(order)
{

    update_model(A_,B_,C_,D_);

}

controller::~controller()
{
    //dtor
}

controller::controller(const controller& other)
{
    //copy ctor
    A = other.A;
    B = other.B;
    C = other.C;
    D = other.D;
    c_type = other.c_type;

    system_order = other.system_order;
    K = other.K;
    K1 = other.K1;
    N = other.N;
    initialized = other.initialized;

}

controller& controller::operator=(const controller& rhs)
{
    if (this == &rhs)
        return *this; // handle self assignment
    //assignment operator
    A = rhs.A;
    B = rhs.B;
    C = rhs.C;
    D = rhs.D;
    c_type = rhs.c_type;
    system_order = rhs.system_order;
    K = rhs.K;
    K1 = rhs.K1;
    N = rhs.N;
    initialized = rhs.initialized;
    return *this;
}

/**
 * Ensure new model is of same dimension as is current model, for replacement purposes
 */
bool controller::is_model_consistent( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_)
{
    return A.rows() == A_.rows() and A.cols() == A_.cols() and
           B.rows() == B_.rows() and B.cols() == B_.cols() and
           C.rows() == C_.rows() and C.cols() == C_.cols() and
           D.rows() == D_.rows() and D.cols() == D_.cols();
}

unsigned controller::get_order(void)
{
    return system_order;
}

bool controller::get_model( MatrixXd &A_, MatrixXd &B_, MatrixXd &C_, MatrixXd D_)
{
    std::lock_guard<std::mutex> lk(mtx);
    if ( initialized)
    {
        A_ = A;
        B_ = B;
        C_ = C;
        D_ = D;
        return true;
    }
    else
        return false;
}

bool controller::update_model( MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_)
{
    if (is_ctrb(A_,B_))
    {
        std::lock_guard<std::mutex> lk(mtx);
        if( not initialized)
        {
            if( is_model_valid(A_,B_,C_,D_))
            {
                A = A_;
                B = B_;
                C = C_;
                D = D_;

                X_int = MatrixXd::Zero(C.rows(),1);

                Q = MatrixXd::Identity(A.rows()+C.rows(),A.cols()+C.rows());
                R = MatrixXd::Identity(B.cols(),B.cols());

                initialized = true;

                return update_NKK1_matrices(Q,R);
            }
            else
                return false;
        }
        else
        {
            if( is_model_consistent(A_,B_,C_,D_))
            {
                A = A_;
                B = B_;
                C = C_;
                D = D_;

                return update_NKK1_matrices(Q,R);
            }
            else
                return false;
        }
    }
    else
        return false;
}

MatrixXd controller::calc_output( MatrixXd Yr, MatrixXd X)
{
    std::lock_guard<std::mutex> lk(mtx);
    MatrixXd U = MatrixXd::Zero(B.cols(),1);

    if ( initialized )
    {
        MatrixXd Zr = Yr - D;
        //update the integral value
        X_int = X_int + C*X - Zr;
        U = -K1*X_int - K*X + N*Zr;
    }
    return U;
}


MatrixXd controller::calc_output( MatrixXd Yr, MatrixXd X, MatrixXd Y)
{
    std::lock_guard<std::mutex> lk(mtx);
    MatrixXd U = MatrixXd::Zero(B.cols(),1);


    if ( initialized )
    {
        MatrixXd Zr = Yr - D;
        X_int = X_int + Y - Yr; //verander moontlik na trapezoidal rule or simsons
        U = -K1*X_int - K*X + N*Zr;
    }
    return U;
}

MatrixXd controller::calc_state_pv( MatrixXd X )
{
    std::lock_guard<std::mutex> lk(mtx);
    return C*X+D;
}

//update system with updated Q,R matrices
bool controller::update_QR_matrices( MatrixXd Q_, MatrixXd R_)
{
    std::lock_guard<std::mutex> lk(mtx);
    Q = Q_;
    R = R_;
    return update_NKK1_matrices(Q,R);
}

/**
 * update_NKK1 matrices
 * Function: Update the N, K and K1 matrices.
 *           To be executed atomically
 */
bool controller::update_NKK1_matrices( MatrixXd Q_, MatrixXd R_)
{
    if ( initialized)
    {
        MatrixXd KK = calc_feedback_matrix(Q_,R_);
        if ( not is_nan(KK))
        { //ensure that at least KK is OK, else keep all as it was
            K1 = KK.block(0,0,KK.rows(),C.rows());
            K = KK.block(0,C.rows(),B.cols(),A.rows());
            N = rscale(K);
        }
        return true;
    }
    else
        return false;
}

/** to be excecuted atomically
 * function calc_feedback matrix
 * PRE: system has been initialized
 * POST: Calculate the feedback matrix
 * where
 *      Q(nxn), R(mxm) matrics, n=A.rows()+C.rows(), m=B.cols()
 * solve for K in :
 * [x1(k+1)]   [ I  C ][x1(k)]   [O]
 * [       ] = [      ][     ] + [ ]U(k)
 * [X (k+1)]   [ O  A ][X (k)]   [B]
 */
MatrixXd controller::calc_feedback_matrix(MatrixXd Q_, MatrixXd R_)
{

    MatrixXd K_;
    MatrixXd AA = MatrixXd::Identity(C.rows()+A.rows(),A.cols()+C.rows());
    MatrixXd BB = MatrixXd::Zero(C.rows()+B.rows(), B.cols());
    AA.block(0,C.rows(),C.rows(),C.cols()) = C;
    AA.block(C.rows(),C.rows(),A.rows(),A.cols()) = A;
    BB.block(C.rows(),0,B.rows(),B.cols()) = B;
    K_= dlqr(AA,BB,Q_,R_);
    IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    cout << "K = " << endl << K.format(OctaveFmt) << endl;
    return K_;
}


/** to be excecuted atomically
 * function rscale
 * PRE: system has been initialized
 * POST: Calculate the state command matrix
 * where
 * Nbar = 1/B * (I - A + B*K ) * 1/C
 */
MatrixXd controller::rscale( MatrixXd K_)
{
    return pinv(B) * ( MatrixXd::Identity(A.rows(),A.cols()) - A + B*K_ ) * pinv(C);
}

