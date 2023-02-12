#include "simulate.h"

/*------------------------------------------------------------------------
 * Pre: dimension of a: (1,m), b: (1,n), c: (1,1)
 *-----------------------------------------------------------------------*/
arx_siso_simulate
::arx_siso_simulate( const MatrixXd a, const MatrixXd b, const MatrixXd c )
    :A(a),B(b),C(c)
{

}


arx_siso_simulate
::~arx_siso_simulate( void )
{

}

//* we assume that the system was stable before the step response
double * arx_siso_simulate
::step_response( const long len )
{
    double * y_n = new double[len];

    double u_n = 1.0;
    //y_n[0] = fixed_point(0.0);
    y_n[0] = 0.0;

    for( int i = 1; i < len; i++ )
    {
        y_n[i] = 0.0;
        for( int j = 0; j < A.cols(); j++ )
            if ( i-j < 1 )
                y_n[i] += A(0,j)*y_n[0];
            else
                y_n[i] += A(0,j)* y_n[i-j-1];

        for( int j = 0; j < B.cols(); j++ )
            y_n[i]+= B(0,j) * u_n;

        //y_n[i] += C(0,0);
    }

    return y_n;
}

double * arx_siso_simulate
::impulse_response( const long len )
{
    double * y_n = new double[len];

    double u_n = 1.0;
    //y_n[0] = fixed_point(0.0);
    y_n[0] = 0.0;

    for( int i = 1; i < len; i++ )
    {
        y_n[i] = 0.0;

        for( int j = 0; j < A.cols(); j++ )
            if ( i-j < 1 )
                y_n[i] += A(0,j)*y_n[0];
            else
                y_n[i] += A(0,j)* y_n[i-j-1];

        if( i <= B.cols())
            y_n[i] += B(0,i-1) * u_n;

        //y_n[i] += C(0,0);
    }

    return y_n;
}


//make sure the init array at least contain the same number of variables
//as the number of columns of A
double * arx_siso_simulate
::general_response( double * mv, const long len, double * init  )
{
    double * y_n = new double[len];
    int y_i;
    //set intial values of y_n;
    if ( init )
    {
        y_i = A.cols();
        for( int i = 0; i < A.cols(); i++ )
            y_n[i] = init[i];
    }
    else
    {
        //no init array, assume that mv[i<0] = mv[0], and system is in steady state;
        y_i = 1;
        y_n[0] = fixed_point(mv[0]);
    }

    //fill the array with data
    for( int i = y_i; i < len; i++ )
    {
        y_n[i] = 0.0;
        for( int j = 0; j < A.cols(); j++ )
            if ( i-j < 1 )
                y_n[i] += A(0,j)*y_n[0];
            else
                y_n[i] += A(0,j)* y_n[i-j-1];

        for( int j = 0; j < B.cols(); j++ )
            if ( i-j < 1 )
                y_n[i] += B(0,j) * mv[0];
            else
                y_n[i] += B(0,j) * mv[i-j-1];

        y_n[i] += C(0,0);
    }

    return y_n;
}


double arx_siso_simulate
::fixed_point( double mv )
{
    MatrixXd B_ = B * MatrixXd::Constant(B.cols(),1, mv );
    MatrixXd f = MatrixXd::Zero( A.cols(), 1 );
    f(0,0) = B_(0,0) + C(0,0);

    MatrixXd A_ = MatrixXd::Zero( A.cols(), A.cols() );
    for( int i = 0; i < A_.rows(); i++ )
        for( int j = 0; j < A_.cols(); j++ )
        {
            if( i == 0 )
                A_(i, j) = A(i,j);
            else
                if( i - j == 1 )
                    A_(i,j) = 1.0;
        }


    MatrixXd fpoint = calc_fixed_point(A_,f);

    return fpoint(0,0);
}


MatrixXd arx_siso_simulate
::calc_fixed_point( MatrixXd A_, MatrixXd f )
{
    if( A_.cols() == f.rows() )
    {
        MatrixXd I = MatrixXd::Identity( A_.rows(), A_.cols());
        return (I - A_).inverse() * f;
    }
    else
        return MatrixXd::Zero(f.rows(),f.cols());
}


//----------------------------------------------------------------------------
// multi input multi output simulation
//----------------------------------------------------------------------------

arx_simulate
::arx_simulate( const string model_tag, historian_table * h, identifier_table * i )
:model_tagname(model_tag), pHistorian(h), pIdentifier(i)
{

}


arx_simulate
::~arx_simulate( void )
{
}

MatrixXd arx_simulate
::general_response( void )
{
    MatrixXd A,B,C;

    vector<MatrixXd> A_n, B_n, Y_n, U_n;
    list<string> CVs, MVs;
    int order;

    pIdentifier->getCVs(model_tagname, CVs );
    pIdentifier->getMVs(model_tagname, MVs );

    if( not pIdentifier->is_identified(model_tagname))
        pIdentifier->identify_model( model_tagname );

    pIdentifier->get_system_parameters( model_tagname, A, B, C );
    pIdentifier->get_order(model_tagname, order );

    A_n = list2vector( expand_matrix( A, order ) );
    B_n = list2vector( expand_matrix( B, order ) );


    U_n = get_Un_vector();

    for( unsigned i = 0 ; i < A_n.size(); i++ )
        Y_n.push_back( get_historian_vector( CVs , i) );

    int len = U_n.size();

    //We now have Y_n+1 = A_0 Y_n + A_1 Y_n-1 + A_k Y_n-1+ ... + B_0 U_0 + ... * B_k U_n-k + C, but we still need to
    //fill the Y_n vector array with the simulation data.

    for( int i = Y_n.size(); i < len; i++ )
    {
        MatrixXd Y_i = MatrixXd::Zero( CVs.size(), 1);
        for( int j = 0; j < (int)A_n.size(); j++ )
            Y_i += A_n[j]*Y_n[i-j-1];
        for( int j = 0; j < (int)B_n.size(); j++ )
            Y_i += B_n[j]*U_n[i-j-1];

        Y_i += C;
        Y_n.push_back( Y_i );
    }

    return concat_matrix_vector( Y_n );

}


vector<MatrixXd> arx_simulate
::get_Un_vector( void )
{
    list< string > MVs;
    pIdentifier->getMVs( model_tagname, MVs );
    list< string >::iterator i;

    int horison  = get_minimum_historian_vector_length(MVs);

    MatrixXd U = MatrixXd::Zero( MVs.size(), horison );

    int j;
    for( i = MVs.begin(),j = 0; i != MVs.end(); i++, j++ )
    {
        VectorXd i_MV_values;
        pHistorian->return_history_vector( *i, i_MV_values );
        for( int k = 0; k < horison; k++ )
            U(j,k) = i_MV_values(k);
    }


    //dump all to a vector<MatrixXd>
    vector<MatrixXd> U_n;
    for( int i = 0; i < U.cols(); i++ )
        U_n.push_back( U.block(0,i, MVs.size(), 1 ));


    return U_n;
}


unsigned arx_simulate
::get_minimum_historian_vector_length(list<string> v)
{
    if( v.size() > 0 )
    {
        unsigned min_len;
        list<string>::iterator i = v.begin();
        pHistorian->size( *i , min_len );

        for(  i = v.begin(); i != v.end(); i++ )
        {
            unsigned tag_len;
            pHistorian->size( *i , tag_len );
            if (tag_len < min_len)
                min_len = tag_len;
        }

        return min_len;
    }
    else
        return 0;
}





// U_0.dimension() = (A.rows(),1)
MatrixXd arx_simulate
::calc_fixed_point( MatrixXd U_0 )
{
    MatrixXd A, B, C;
    int order;

    pIdentifier->get_system_parameters( model_tagname, A, B, C );
    pIdentifier->get_order(model_tagname, order );

    list<MatrixXd> A_n = expand_matrix( A, order);
    list<MatrixXd> B_n = expand_matrix( B, order);
    return ( MatrixXd::Identity(A.rows(),A.rows()) - get_sum(A_n) ).colPivHouseholderQr().solve(get_sum(B_n) + C );
}


MatrixXd arx_simulate
::get_historian_vector( list<string> V, const unsigned index )
{
    MatrixXd Vect = MatrixXd::Zero( V.size(), 1 );


    if ( (V.size() > 0)  and (index < get_minimum_historian_vector_length(V)))
    {
        list<string>::iterator i;
        int j;
        for(  i = V.begin(), j = 0; i != V.end(); i++, j++ )
        {
            double val;
            pHistorian->get_snapshot_value( *i, index, val );
            Vect(j,0) = val;
        }

    }
    return Vect;


}






