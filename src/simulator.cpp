#include "simulator.h"


simulator::simulator()
{
    running = false;
}

// A(n,n), B(n,m), C(n,1)
simulator::simulator(MatrixXd A_, MatrixXd B_, MatrixXd C_, MatrixXd X0,time_t sr)
    :C(C_),sampling_rate(sr)
{
    //ctor
    //calc_mutex.unlock();
    A_n = list2vector( expand_matrix( A_, get_order(A_) ) );
    B_n = list2vector( expand_matrix( B_, get_order(B_) ) );

    U_now = calc_steady_state_input(X0);

    running = false;
    set_initial_state(X0);
}

simulator::~simulator()
{
    //dtor
    if (running)
        th.join();
}

simulator::simulator(const simulator& other)
{
    //copy ctor
    A_n = other.A_n;
    B_n = other.B_n;
    C = other.C;
    Y_n = other.Y_n;
    U_n = other.U_n;
    U_now = other.U_now;
    Y_now = other.Y_now;
    sampling_rate = other.sampling_rate;
    running = false;

}

simulator& simulator::operator=(const simulator& rhs)
{
    if (this == &rhs) return *this; // handle self assignment
    //assignment operator
    A_n = rhs.A_n;
    B_n = rhs.B_n;
    C = rhs.C;
    Y_n = rhs.Y_n;
    U_n = rhs.U_n;
    U_now = rhs.U_now;
    Y_now = rhs.Y_now;
    sampling_rate = rhs.sampling_rate;
    running = false;
    return *this;
}


// Pre: System is not running
// A(n,n), B(n,m), C(n,1)
// Assuming that U_0 = 0
bool simulator::load_model(MatrixXd a,MatrixXd b,MatrixXd c)
{
    if ( not running )
    {
        //cout << get_order(a) << endl;
        int order = get_order(a);
        A_n = list2vector( expand_matrix( a, order ) );
        B_n = list2vector( expand_matrix( b, order ) );
        C = c;

        U_now = MatrixXd::Zero(B_n[0].cols(),1);

        Y_now = calc_steady_state_output( U_now );

        Y_n.clear();
        for( unsigned i = 0 ; i < A_n.size(); i++ )
            Y_n.push_back( Y_now );

        U_n.clear();
        for( unsigned i = 0 ; i < B_n.size(); i++ )
            U_n.push_back( U_now );
        return true;
    }
    else
        return false;

}

// Pre: System is not running
bool simulator::set_sampling_rate( time_t sr )
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
bool simulator::set_initial_state( MatrixXd Y_inf )
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
bool simulator::set_input( MatrixXd U ){
    std::lock_guard<std::mutex> lk(calc_mutex);
    if( (U.rows() == U_now.rows()) and (U.cols() == U_now.cols())){
        U_now = U;
        return true;
    }else
        return false;
}

MatrixXd simulator::get_input( void ){
    return U_now;
}

//will use semaphores to update this
MatrixXd simulator::get_output( void ){
    std::lock_guard<std::mutex> lk(calc_mutex);
    return Y_now;
}

bool simulator::set_output( MatrixXd Y_inf){
    return set_initial_state(Y_inf);
}


void simulator::start( void )
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

void simulator::stop(void)
{
    if ( running ){
        running = false;
        th.join();
    }
}


void simulator::update_simulation( void )
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


int simulator::get_order( MatrixXd& X )
{
    if ( X.rows() != 0 )
        return X.cols() / X.rows(); // or we could use B.cols()/B.cols();
    else
        return 0;
}

//for a given X_0 value, calculate the U_inf
// U_inf = pinv(B)*(I-A)X_0 - pinv(B)*C;
// PRE: B has a pseudo inverse
MatrixXd simulator::calc_steady_state_input( MatrixXd Y_inf )
{
    MatrixXd A = MatrixXd::Zero( A_n[0].rows(),A_n[0].cols());
    for( unsigned i = 0; i < A_n.size(); i++)
        A = A + A_n[i];

    MatrixXd B = MatrixXd::Zero( B_n[0].rows(), B_n[0].cols());
    for( unsigned i = 0; i < B_n.size(); i++ )
        B = B + B_n[i];

    MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
    return pinv(B)*((I-A)*Y_inf - C );
}

// For a given U_inf input, calcute the steady state Y_n value
// PRE: (I-A) has an pseudo inverse
MatrixXd simulator::calc_steady_state_output( MatrixXd U_inf ){
    MatrixXd A = MatrixXd::Zero( A_n[0].rows(),A_n[0].cols());
    for( unsigned i = 0; i < A_n.size(); i++)
        A = A + A_n[i];

    MatrixXd B = MatrixXd::Zero( B_n[0].rows(), B_n[0].cols());
    for( unsigned i = 0; i < B_n.size(); i++ )
        B = B + B_n[i];

    MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
    return pinv(I-A)*(B*U_inf + C );
}

//void simulator::convert_to_first_order( void ){


//}
