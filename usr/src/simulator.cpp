#include "simulator.h"


fir_simulator::fir_simulator()
{
    running = false;
    initialized = false;
}

// A(n,n), B(n,m), C(n,1)
fir_simulator::fir_simulator(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd X0,time_t sr)
    :A(A_),B(B_),C(C_),sampling_rate(sr)
{
    //ctor
    //calc_mutex.unlock();
    if (sr > 1)
    {
        //reset the sampling rate to 1 sek by default. Else the thing will be to slow
        MatrixXd A_new,B_new,C_new;
        time_t sr_new;
        get_time_normalized_matrices( A_new, B_new, C_new, sr_new);
        A = A_new;
        B = B_new;
        C = C_new;
        sampling_rate = sr_new;
    }


    A_n = list2vector( expand_matrix( A, get_order(A) ) );
    B_n = list2vector( expand_matrix( B, get_order(B) ) );

    U_now = calc_steady_state_input(X0);

    running = false;
    set_initial_state(X0);
    initialized = true;
}

fir_simulator::~fir_simulator()
{
    //dtor
    if (running)
        th.join();
}

fir_simulator::fir_simulator(const fir_simulator& other)
{
    //copy ctor
    A_n = other.A_n;
    B_n = other.B_n;
    A = other.A;
    B = other.B;
    C = other.C;
    Y_n = other.Y_n;
    U_n = other.U_n;
    U_now = other.U_now;
    Y_now = other.Y_now;

    sampling_rate = other.sampling_rate;
    initialized  = other.initialized;
    running = false;

}

fir_simulator& fir_simulator::operator=(const fir_simulator& rhs)
{
    if (this == &rhs) return *this; // handle self assignment
    //assignment operator
    A_n = rhs.A_n;
    B_n = rhs.B_n;
    A = rhs.A;
    B = rhs.B;
    C = rhs.C;
    Y_n = rhs.Y_n;
    U_n = rhs.U_n;
    U_now = rhs.U_now;
    Y_now = rhs.Y_now;

     sampling_rate = rhs.sampling_rate;
    initialized = rhs.initialized;
    running = false;
    return *this;
}



// Pre: System is not running
// A(n,n), B(n,m), C(n,1)
// Assuming that U_0 = 0
bool fir_simulator::load_model(MatrixXd a,MatrixXd b,MatrixXd c)
{
    if ( not running )
    {
        //cout << get_order(a) << endl;
        int order = get_order(a);
        A_n = list2vector( expand_matrix( a, order ) );
        B_n = list2vector( expand_matrix( b, order ) );
        A = a;
        B = b;
        C = c;

        U_now = MatrixXd::Zero(B_n[0].cols(),1);

        Y_now = calc_steady_state_output( U_now );

        Y_n.clear();
        for( unsigned i = 0 ; i < A_n.size(); i++ )
            Y_n.push_back( Y_now );

        U_n.clear();
        for( unsigned i = 0 ; i < B_n.size(); i++ )
            U_n.push_back( U_now );
        initialized = true;
        return true;
    }
    else
        return false;

}

bool fir_simulator::load_model(MatrixXd a,MatrixXd b, MatrixXd c, MatrixXd X0, time_t sr)
{
    if ( not running )
    {
        //cout << get_order(a) << endl;
        int order = get_order(a);
        A = a;
        B = b;
        C = c;
        sampling_rate = sr;

        if (sr > 1)
        {
            //reset the sampling rate to 1 sek by default. Else the thing will be to slow
            MatrixXd A_new,B_new,C_new;
            time_t sr_new;
            get_time_normalized_matrices( A_new, B_new, C_new, sr_new);
            A = A_new;
            B = B_new;
            C = C_new;
            sampling_rate = sr_new;
        }



        A_n = list2vector( expand_matrix( a, order ) );
        B_n = list2vector( expand_matrix( b, order ) );


        U_now = calc_steady_state_input(X0);

        Y_now = calc_steady_state_output( U_now );

        Y_n.clear();
        for( unsigned i = 0 ; i < A_n.size(); i++ )
            Y_n.push_back( Y_now );

        U_n.clear();
        for( unsigned i = 0 ; i < B_n.size(); i++ )
            U_n.push_back( U_now );
        initialized = true;
        return true;
    }
    else
        return false;

}

bool fir_simulator::read_model(MatrixXd& a, MatrixXd& b, MatrixXd& c, MatrixXd& x0, time_t& sr)
{
    if (is_initialized())
    {
        a = A;
        b = B;
        c = C;
        x0 = calc_steady_state_output( U_now );
        sr = sampling_rate;
        return true;
    }
    else
        return false;
}

// Pre: System is not running
bool fir_simulator::set_sampling_rate( time_t sr )
{
    if (not running)
    {
        sampling_rate = sr;
        return true;
    }
    else
        return false;

}

// Pre: System is not running
// Pre: Model has been loaded
// use initial state to calculate what U_n should be;
bool fir_simulator::set_initial_state( MatrixXd Y_inf )
{
    if (not running )
    {

        Y_now = Y_inf;
        Y_n.clear();
        for( unsigned i = 0 ; i < A_n.size(); i++ )
            Y_n.push_back( Y_now );

        U_now = calc_steady_state_input(Y_now);

        U_n.clear();
        for( unsigned i = 0; i < B_n.size(); i++ )
            U_n.push_back(U_now);
        return true;
    }
    else
        return false;
}

//will use semaphores to update this
bool fir_simulator::set_input( MatrixXd U )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    if( (U.rows() == U_now.rows()) and (U.cols() == U_now.cols()))
    {
        U_now = U;
        return true;
    }
    else
        return false;
}

MatrixXd fir_simulator::get_input( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    return U_now;
}

//will use semaphores to update this
MatrixXd fir_simulator::get_output( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    return Y_now;
}

bool fir_simulator::set_output( MatrixXd Y_inf)
{
    return set_initial_state(Y_inf);
}


void fir_simulator::start( void )
{
    if (not running)
    {
        running = true;
        th = thread([=]()
        {
            while (running == true)
            {
                this_thread::sleep_for(chrono::seconds(sampling_rate));
                update_simulation();
                // Update the dynamic simulation
            }
        });
    }
}

void fir_simulator::stop(void)
{
    if ( running )
    {
        running = false;
        th.join();
    }
}


void fir_simulator::update_simulation( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    //update the U_n vector before the calculation
    U_n.insert(U_n.begin(),U_now);
    U_n.pop_back();

    //Update the simulation
    MatrixXd Y_i = MatrixXd::Zero(C.rows(),C.cols());
    for ( unsigned i = 0; i < A_n.size(); i++ )
        Y_i = Y_i + A_n[i]* Y_n[i];

    MatrixXd B_i = MatrixXd::Zero(C.rows(),C.cols());
    for ( unsigned i = 0; i < B_n.size(); i++ )
        B_i = B_i + B_n[i]* U_n[i];

    Y_now = Y_i + B_i + C;

    //cout << Y_now << endl << endl;
    //update the Y_n vector after the calculation
    Y_n.insert(Y_n.begin(),Y_now);
    Y_n.pop_back();

}


int fir_simulator::get_order( MatrixXd& X )
{
    if ( X.rows() != 0 )
        return X.cols() / X.rows(); // or we could use B.cols()/B.cols();
    else
        return 0;
}

//for a given X_0 value, calculate the U_inf
// U_inf = pinv(B)*(I-A)X_0 - pinv(B)*C;
// PRE: B has a pseudo inverse
MatrixXd fir_simulator::calc_steady_state_input( MatrixXd Y_inf )
{
    MatrixXd At = MatrixXd::Zero( A_n[0].rows(),A_n[0].cols());
    for( unsigned i = 0; i < A_n.size(); i++)
        At = At + A_n[i];

    MatrixXd Bt = MatrixXd::Zero( B_n[0].rows(), B_n[0].cols());
    for( unsigned i = 0; i < B_n.size(); i++ )
        Bt = Bt + B_n[i];

    MatrixXd I = MatrixXd::Identity(At.rows(),At.cols());
    return pinv(Bt)*((I-At)*Y_inf - C );
}

// For a given U_inf input, calcute the steady state Y_n value
// PRE: (I-A) has an pseudo inverse
MatrixXd fir_simulator::calc_steady_state_output( MatrixXd U_inf )
{
    MatrixXd At = MatrixXd::Zero( A_n[0].rows(),A_n[0].cols());
    for( unsigned i = 0; i < A_n.size(); i++)
        At = At + A_n[i];

    MatrixXd Bt = MatrixXd::Zero( B_n[0].rows(), B_n[0].cols());
    for( unsigned i = 0; i < B_n.size(); i++ )
        Bt = Bt + B_n[i];

    MatrixXd I = MatrixXd::Identity(At.rows(),At.cols());
    return pinv(I-At)*(Bt*U_inf + C );
}



int fir_simulator::get_process_variable_count( void )
{
    if ( A_n.size() > 0 )
    {
        return A_n[0].cols(); //[0].cols();
    }
    else
        return 0;
}

int fir_simulator::get_control_variable_count( void )
{
    if ( B_n.size() > 0 )
    {
        return B_n[0].cols(); //B.cols();
    }
    else
        return 0;
}


int fir_simulator::get_system_order( void )
{
    if ( A_n.size() > 0 )
    {
        return A.cols() / A.rows();
    }
    else
        return 0;
}


time_t fir_simulator::get_sampling_rate( void )
{
    return sampling_rate;
}

bool fir_simulator::generate_random_model( int pv_count, int cv_count, int ordr, time_t s_rate)
{
    if (not is_initialized())
    {
        srand (time(NULL));

        MatrixXd a = MatrixXd::Zero( pv_count, pv_count*ordr);
        for( int i = 0; i < a.rows(); i++ )
            for( int j = 0; j < a.cols(); j++ )
                a(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;


        MatrixXd b = MatrixXd::Zero( pv_count, cv_count*ordr);
        for( int i = 0; i < b.rows(); i++ )
            for( int j = 0; j < b.cols(); j++ )
                b(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;

        MatrixXd c = MatrixXd::Zero( pv_count, 1 );
        for( int i = 0; i < c.rows(); i++ )
            for( int j = 0; j < c.cols(); j++ )
                c(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;

        MatrixXd x0 = MatrixXd::Zero( pv_count, 1 );
        for( int i = 0; i < x0.rows(); i++ )
            for( int j = 0; j < x0.cols(); j++ )
                x0(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;
        load_model(a,b,c,x0,s_rate);
        return true;
    }
    else
    {
        return false;
    }
}

bool fir_simulator::generate_stable_random_model( int pv_count, int cv_count, int ordr, time_t s_rate)
{
    //generate_random_model(pv_count,cv_count,ordr,s_rate);
    if (not is_initialized())
    {
        srand (time(NULL));

        MatrixXd a = MatrixXd::Zero( pv_count, pv_count);
        for( int i = 0; i < a.rows(); i++ )
            for( int j = 0; j < a.cols(); j++ )
                a(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;



        MatrixXd b = MatrixXd::Zero( pv_count, cv_count);
        for( int i = 0; i < b.rows(); i++ )
            for( int j = 0; j < b.cols(); j++ )
                b(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;



        MatrixXd K = dlqr(a, b, MatrixXd::Identity(a.rows(),a.rows()), MatrixXd::Identity(b.cols(),b.cols()));


        a = a - b*K;



        MatrixXd c = MatrixXd::Zero( pv_count, 1 );
        for( int i = 0; i < c.rows(); i++ )
            for( int j = 0; j < c.cols(); j++ )
                c(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;



        MatrixXd x0 = MatrixXd::Zero( pv_count, 1 );
        for( int i = 0; i < x0.rows(); i++ )
            for( int j = 0; j < x0.cols(); j++ )
                x0(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;


        load_model(a,b,c,x0,s_rate);
        return true;
    }
    else
    {
        return false;
    }
}

bool fir_simulator::get_time_normalized_matrices( MatrixXd &A_new, MatrixXd &B_new, MatrixXd &C_new, time_t &sr_new)
{
    if ( sampling_rate > 1)
    {
        ResetModelSamplingRate(A,B,sampling_rate,A_new,B_new,(time_t)1);
        MatrixXd FP_0 = pinv(MatrixXd::Identity(A.rows(),A.cols()) - A)*C;
        C_new = (MatrixXd::Identity(A_new.rows(),A_new.cols()) - A_new)*FP_0;
        sr_new = (time_t) 1;
    }
    else
    {
        A_new = A;
        B_new = B;
        C_new = C;
        sr_new = sampling_rate;
    }
    return true;
}


/**
 *  Implementation of the Statespace simulator
 */
ss_simulator::ss_simulator()
{
    running = false;
    initialized = false;
    v_std = 0.0;
    w_std = 0.0;
}

ss_simulator::ss_simulator(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd D_, time_t sr):
    A(A_),B(B_),C(C_),D(D_),sampling_rate(sr)
{


    X = MatrixXd::Zero(A.cols(),1);
    U_now = MatrixXd::Zero(B_.cols(),1);

    running = false;
    initialized = true;
}

ss_simulator::~ss_simulator()
{
    stop();
}

ss_simulator::ss_simulator(const ss_simulator& other)
{
    A = other.A;
    B = other.B;
    C = other.C;
    D = other.D;
    X = other.X;
    U_now = other.U_now;
    sampling_rate = other.sampling_rate;
    initialized = other.initialized;
    running = false;

    v_std = other.v_std;
    w_std = other.w_std;
}


ss_simulator& ss_simulator::operator=(const ss_simulator& other)
{
    if (this == &other) return *this; // handle self assignment
    //assignment operator
    A = other.A;
    B = other.B;
    C = other.C;
    D = other.D;
    X = other.X;
    U_now = other.U_now;
    sampling_rate = other.sampling_rate;
    initialized = other.initialized;
    running = false;

    v_std = other.v_std;
    w_std = other.w_std;
    return *this;
}

/**
 * Load the model on the fly. Can be updated while system is running, but i have to properly
 * test it.
 */
bool ss_simulator::load_model(MatrixXd A_,MatrixXd B_,MatrixXd C_, MatrixXd D_, time_t sr)
{
    //allow to set the model up only once
    if( not initialized)
    {
        if ( sr < 1)
            return false;

        sampling_rate = sr;
        //define the state matix
        X = MatrixXd::Zero(A_.cols(),1);
        U_now = MatrixXd::Zero(B_.cols(),1);
        // convert the model to 1s update rate
        if ( sr > 1 )
        {

            A = A_;
            B = B_;
            C = C_;
            D = D_;
            initialized = true;

            return true;
        }
        else
        {
            A = A_;
            B = B_;
            C = C_;
            D = D_;
            initialized = true;
            return true;
        }

    }
    else
        return false;
}

bool ss_simulator::read_model(MatrixXd& A_, MatrixXd& B_, MatrixXd& C_, MatrixXd& D_, time_t& sr)
{
    A_ = A;
    B_ = B;
    C_ = C;
    D_ = D;
    sr = sampling_rate;
    return true;
}

bool ss_simulator::set_initial_state( MatrixXd Y_inf )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    if( initialized)
    {
        MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
        U_now = pinv(B)*(I-A)*pinv(C)*(Y_inf - D );
        X = pinv(I-A)*B*U_now;
        return true;
    }
    else
        return false;
}


bool ss_simulator::set_input( MatrixXd U )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    if( U.rows() == U_now.rows() and U.cols() == U_now.cols())
    {
        U_now = U;
        return true;
    }
    else
        return false;
}

MatrixXd ss_simulator::get_input( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    return U_now;
}

//reading from the system and states
bool ss_simulator::set_output( MatrixXd Y_inf)
{
    return set_initial_state(Y_inf);
}

MatrixXd ss_simulator::get_output( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    MatrixXd W = generate_randn_matrix(D.rows(), D.cols(), 0.0, w_std);
    return C*X+D + W;
}


bool ss_simulator::set_state( MatrixXd X_)
{
    if( X_.rows() == X.rows() and X_.cols() == X.cols())
    {
        std::lock_guard<std::mutex> lk(calc_mutex);
        X = X_;
        return true;
    }
    else
        return false;
}

MatrixXd ss_simulator::get_state( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    return X;
}

int ss_simulator::get_process_variable_count( void )
{
    return C.rows();
}

int ss_simulator::get_control_variable_count( void )
{
    return B.cols();
}

time_t ss_simulator::get_sampling_rate( void )
{
    return sampling_rate;
}

bool ss_simulator::set_process_noise( const double std_dev){

    v_std = std_dev;
    return true;
}

bool ss_simulator::set_measurement_noise( const double std_dev){

    w_std = std_dev;
    return true;
}

bool ss_simulator::is_initialized( void )
{
    return initialized;
}

bool ss_simulator::is_running( void )
{
    return running;
}


bool ss_simulator::generate_random_model( int pv_count, int cv_count, int order, time_t s_rate)
{
    if (not is_initialized())
    {
        srand (time(NULL));

        MatrixXd a = MatrixXd::Zero( pv_count, pv_count*order);
        for( int i = 0; i < a.rows(); i++ )
            for( int j = 0; j < a.cols(); j++ )
                a(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;


        MatrixXd b = MatrixXd::Zero( pv_count, cv_count*order);
        for( int i = 0; i < b.rows(); i++ )
            for( int j = 0; j < b.cols(); j++ )
                b(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;

        MatrixXd c = MatrixXd::Zero( pv_count, 1 );
        for( int i = 0; i < c.rows(); i++ )
            for( int j = 0; j < c.cols(); j++ )
                c(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;


        MatrixXd A_ = fir2ss_Psi( a,b,c,order );
        MatrixXd B_ = fir2ss_Gamma( a, b, c, order );
        MatrixXd C_ = fir2ss_C(a,b,c,order);
        MatrixXd D_ = fir2ss_Fixedpoint( a,b,c,order);

        load_model(A_,B_,C_,D_,s_rate);
        return true;
    }
    else
    {
        return false;
    }
}

bool ss_simulator::generate_stable_random_model( int pv_count, int cv_count, int order, time_t s_rate)
{
    if (not is_initialized())
    {
        srand (time(NULL));

        //generate random FIR model
        MatrixXd a = MatrixXd::Zero( pv_count, pv_count*order);
        for( int i = 0; i < a.rows(); i++ )
            for( int j = 0; j < a.cols(); j++ )
                a(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;


        MatrixXd b = MatrixXd::Zero( pv_count, cv_count*order);
        for( int i = 0; i < b.rows(); i++ )
            for( int j = 0; j < b.cols(); j++ )
                b(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;

        MatrixXd c = MatrixXd::Zero( pv_count, 1 );
        for( int i = 0; i < c.rows(); i++ )
            for( int j = 0; j < c.cols(); j++ )
                c(i,j) = static_cast <double> ( rand()) / static_cast <double> (RAND_MAX) - 0.5;

        //convert to state space model
        MatrixXd A_ = fir2ss_Psi( a,b,c,order );
        MatrixXd B_ = fir2ss_Gamma( a, b, c, order );

        MatrixXd K = dlqr(A_, B_,  MatrixXd::Identity(A_.rows(),A_.rows()), MatrixXd::Identity(B_.cols(),B_.cols()));

        A_ = A_ - B_* K;

        MatrixXd C_ = fir2ss_C(a,b,c,order);
        MatrixXd D_ = fir2ss_Fixedpoint( a,b,c,order);
        load_model(A_,B_,C_,D_,s_rate);
        return true;
    }
    else
    {
        return false;
    }
}

void ss_simulator::update_simulation( void )
{
    std::lock_guard<std::mutex> lk(calc_mutex);
    //update the U_n vector before the calculation
    MatrixXd V = generate_randn_matrix(X.rows(),X.cols(), 0.0, v_std);
    X = A*X + B*U_now + V;
}

void ss_simulator::start( void )
{
    if (not running)
    {
        running = true;
        th = thread([=]()
        {
            while (running == true)
            {
                this_thread::sleep_for(chrono::seconds(sampling_rate));
                update_simulation();
                // Update the dynamic simulation
            }
        });
    }
}

void ss_simulator::stop(void)
{
    if (th.joinable())
    {
        running = false;
        th.join();
    }
}

