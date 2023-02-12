#include "observer.h"


observer::observer( MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_,unsigned order):
    system_order(order)
{
    initialized = update_model(A_,B_,C_,D_);
}


observer::observer( unsigned order):
    system_order(order)
{
    initialized = false;
}


observer::~observer()
{

}

observer::observer(const observer& other)
{
    //copy ctor
    Lp = other.Lp;
    A = other.A;
    B = other.B;
    C = other.C;
    D = other.D;
    Xs = other.Xs;
    Ys = other.Ys;
    system_order = other.system_order;
    initialized = other.initialized;
}

observer& observer::operator=(const observer& rhs)
{
    if (this == &rhs)
        return *this; // handle self assignment
    Lp = rhs.Lp;
    A = rhs.A;
    B = rhs.B;
    C = rhs.C;
    D = rhs.D;
    Xs = rhs.Xs;
    Ys = rhs.Ys;
    system_order = rhs.system_order;
    initialized = rhs.initialized;

    return *this;
}

bool observer::update_model( MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_)
{
    if (is_obsv(A_,C_))
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
                //for calculating process noize
                auto_set_default_filter_parms();

                Y_curr_err = MatrixXd::Zero(D.rows(), 1 ); //D.cols());
                X = MatrixXd::Zero(A.rows(),1);
                initialized = true;
                return true;
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
                if ( not auto_update_filter_parms())
                    auto_set_default_filter_parms();
                return true;
            }
            else
                return false;
        }
    }
    else
        return false;
}

/**
 * Check if model's matrices are of the correct dimensions for operation
 */
//bool observer::is_model_valid( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_)
//{
//    return A_.rows() == A_.cols() and A_.rows() == B_.rows() and A_.cols() == C_.cols() and C_.rows() == D_.rows();
//}

/**
 * Ensure new model is of same dimension as is current model, for replacement purposes
 */
bool observer::is_model_consistent( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_)
{
    return A.rows() == A_.rows() and A.cols() == A_.cols() and
           B.rows() == B_.rows() and B.cols() == B_.cols() and
           C.rows() == C_.rows() and C.cols() == C_.cols() and
           D.rows() == D_.rows() and D.cols() == D_.cols();
}


bool observer::get_model( MatrixXd &A_, MatrixXd &B_, MatrixXd &C_, MatrixXd D_)
{
    std::lock_guard<std::mutex> lk(mtx);
    A_ = A;
    B_ = B;
    C_ = C;
    D_ = D;
    return true;
}


unsigned observer::get_order(void){
    return system_order;
}

/**
 * Pre: The Q and R matrices has been correctly initialized
 * Post: The prediction estimator Lp has been updated
 */
bool observer::update_prediction_estimator(MatrixXd Q, MatrixXd R)
{
    IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    MatrixXd G = MatrixXd::Identity(A.rows(),A.cols());

    Lp = A*dlqe(A,G,C,Q,R); //this call calculate the current estimator
    //Lp = dlqe(A,G,C,Q,R);

    cout << "Lp = " << endl << Lp.format(OctaveFmt) << endl;

    if (is_nan(Lp))
        return false;
    else
        return true;
}


bool observer::auto_set_default_filter_parms(void)
{
    if( not initialized )
    {
        MatrixXd Q = MatrixXd::Identity(A.rows(),A.cols());
        MatrixXd R = MatrixXd::Identity(C.rows(),C.rows());

        if ( is_filter_parms_valid(Q,R))
            return update_prediction_estimator(Q,R);
        else
            return false;
    }
    else
        return false;
}

bool observer::auto_update_filter_parms(void)
{
    if( initialized and (Xs.size() > MIN_W_SIZE) and (Xs.size() == Ys.size()))
    {
        MatrixXd X_ = concat_matrix_list(Xs);
        //cout << "Xs(" << X_.rows() << ", " << X_.cols() << ") has been generated" << endl;

        MatrixXd Y_ = concat_matrix_list(Ys);
        //cout << "Ys(" << Y_.rows() << ", " << Y_.cols() << ") has been generated" << endl;


        MatrixXd R  = ss_measurement_noise_cov( C, D, X_, Y_ );
        //cout << "R(" << R.rows() << ", " << R.cols() << ") has been generated" << endl;

        MatrixXd Q  = ss_process_noise_cov( A, C, D, X_, Y_);
        //cout << "Q(" << Q.rows() << ", " << Q.cols() << ") has been generated" << endl;
        IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        cout << "Q = " << endl << Q.format(OctaveFmt) << endl;
        cout << "R = " << endl << R.format(OctaveFmt) << endl;

        if (is_filter_parms_valid(Q,R))
            return update_prediction_estimator(Q,R);
        else
            return false;
    }
    else
        return false;
}

bool observer::update_filter_parms( MatrixXd Q, MatrixXd R)
{

    if ( initialized )
    {
        if(is_filter_parms_valid(Q,R))
        {
            std::lock_guard<std::mutex> lk(mtx);
            return update_prediction_estimator(Q,R);
        }
        else
            return false;
    }
    else
        return false;

}

MatrixXd observer::get_X( void)
{
    std::lock_guard<std::mutex> lk(mtx);
    return X;
}

bool observer::is_filter_parms_valid( MatrixXd Q, MatrixXd R)
{
    return Q.rows() == A.rows() and Q.rows() == Q.cols() and R.rows() == C.rows() and R.cols() == R.cols();
}


bool observer::update_state(  MatrixXd U, MatrixXd Y )
{
    //cout << "observer: Update state" << endl;
    if( initialized)
    {
        std::lock_guard<std::mutex> lk(mtx);

        //X = (A-Lp*C)*X + B*U + Lp*Y - Lp*D;
        X = (A-Lp*C)*X + B*U + Lp*Y - Lp*D;

        //Y_prev_err = Y_curr_err;
        //Y_curr_err = Y - C * X - D;

        //W.push_back(pinv(C)*Y_curr_err - A*pinv(C)*Y_prev_err);
        //while (W.size() > MAX_W_SIZE )
        //    W.pop_front();
        Xs.push_back(X);    //for calculating Q and R
        Ys.push_back(Y);    //for claculating Q and R
        while (Xs.size() > MAX_W_SIZE )
            Xs.pop_front();
        while (Ys.size() > MAX_W_SIZE )
            Ys.pop_front();

        return true;
    }
    else
        return false;
}

