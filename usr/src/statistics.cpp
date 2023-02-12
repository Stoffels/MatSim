#include "statistics.h"


//The procedure is to the credit of William Kahan. And it works excellently
double kahan_sum( VectorXd& x )
{
    double sum = 0.0;
    double c = 0.0;
    for( int i = 0; i < x.size(); i++ )
    {
        double y = x(i) - c;
        double t = sum + y;
        c = ( t - sum ) - y;
        sum = t;
    }
    return sum;
}

double calc_mean( VectorXd& x )
{
    return kahan_sum(x) / x.size();
}

//use the Kahan summation algorithm  to calculate variance
double calc_variance( VectorXd& x )
{
    int n = x.size();
    if( n > 1 )
    {
        double mean = calc_mean(x);
        VectorXd C = VectorXd::Zero(n);

        for( int i = 0; i < n; i++ )
            C(i) = ( x(i) - mean )*( x(i) - mean );
        return kahan_sum(C) / (double) n;
    }
    else
        return 0.0;
}


//use the Kahan summation algorithm  to calculate co variance
double calc_covariance( VectorXd& x, VectorXd& z )
{
    unsigned n = x.size();

    if( x.size() > 1 )
        if( x.size() == z.size() )
        {


            double mean_x = x.mean();
            double mean_z = z.mean();
            VectorXd C = VectorXd::Zero(n);
            for( unsigned i = 0; i < n; i++ )
                C(i) = (x(i) - mean_x)*(z(i) - mean_z);
            return kahan_sum(C) / (double)n;
        }
        else
            return 0.0;
    else
        return 0.0;
}

//martrix is long
MatrixXd calc_matrix_covariance(MatrixXd& X)
{
    MatrixXd Cov = MatrixXd::Zero(X.cols(),X.cols());



    for( int i = 0; i < Cov.rows(); i++ )
        for ( int j = 0; j < Cov.cols(); j++ )
        {
            VectorXd x = X.block(0,i,X.rows(),1);

            VectorXd z = X.block(0,j,X.rows(),1);

            Cov(i,j)=calc_covariance(x,z);
        }
    return Cov;
}


double calc_correlation( VectorXd& x, VectorXd& y )
{
    return calc_covariance( x, y ) / sqrt( calc_variance(x) * calc_variance(y));
}


//use the Kahan summation algorithm  to calculate  auto variance
//c(i)
double calc_auto_covariance( VectorXd& y, int i )
{
    int n = y.size();

    if ( n > 1 )
    {
        double mean = calc_mean(y);
        VectorXd C = VectorXd::Zero(n-i);
        for( int k = 0; k < n - i; k++ )
            C(k) = (y(k) - mean )*(y(k+i) - mean );
        return kahan_sum(C) / (double) n;
    }
    else
        return 0.0;

}

double calc_auto_correlation( VectorXd& y, int i )
{
    return calc_auto_covariance( y,i ) / calc_auto_covariance( y, 0);
}



double calc_cross_covariance( VectorXd& u, VectorXd& y, int i )
{
    unsigned n = u.size();
    if ( n > 1 )
        if ( u.size() == y.size())
        {
            double u_mean = calc_mean( u );
            double y_mean = calc_mean( y );
            VectorXd Cuy = VectorXd::Zero(n-i);

            for( unsigned k = 0; k < n - i; k++ )
                Cuy(k) = (u(k) - u_mean)*(y(k+i) - y_mean );
            return kahan_sum(Cuy) / (double) n;
        }
        else
            return 0.0;
    else
        return 0.0;

}


double calc_cross_correlation( VectorXd& u, VectorXd& y, int i )
{
    return  calc_cross_covariance(u,y,i) / sqrt(calc_cross_covariance(u,u,0)*calc_cross_covariance(y,y,0));
}

MatrixXd impulse_response( VectorXd& u, VectorXd& y, int n )
{
    MatrixXd C_uu = MatrixXd::Zero(n,n);
    for( int i = 0; i < n; i++ )
        for( int j = 0; j < n; j++ )
            C_uu(i,j) = calc_cross_covariance( u, u, abs(i - j) );

    MatrixXd C_uy = MatrixXd::Zero(n,1);
    for( int i = 0; i < n; i++ )
        C_uy(i,0) = calc_cross_covariance( u, y, i );


    return C_uu.colPivHouseholderQr().solve( C_uy );
}

MatrixXd step_response( VectorXd& u, VectorXd& y, int n )
{
    MatrixXd V = impulse_response( u,y, n );
    MatrixXd S = MatrixXd::Zero( V.rows(), V.cols() );
    double sum = 0.0;
    for( int i = 0; i < V.rows(); i++ )
    {
        S(i,0) = sum;
        sum += V(i,0 );
    }
    return S;
}

MatrixXd generate_cross_correlation_vector( VectorXd& x, VectorXd& y, int n )
{
    MatrixXd V = MatrixXd::Zero(n,1);
    for( int i = 0; i < n; i++ )
        V(i,0) = calc_cross_correlation(x,y,i);
    return V;
}

//generate gausian noise with stdev=0 and mean = 0
double AWGN_generator(void)
{
    double temp1,temp2;
    int p = 1.0;
    while ( p > 0.0 )
    {
        temp2 = (rand()/((double)RAND_MAX));
        if (temp2 == 0.0)
            p = 1.0;
        else
            p = -1.0;
    }
    temp1 = cos((2.0*(double)PI)*rand()/((double)RAND_MAX));
    return sqrt(-2.0*log(temp2))*temp1;
}


MatrixXd generate_randn_matrix( const int nrows, const int ncols, const double mean, const double std_deviation){
    MatrixXd M = MatrixXd::Zero(nrows,ncols);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    std::normal_distribution<double> distribution(mean,std_deviation);

    for( int i = 0; i < nrows; i++)
    for( int j = 0; j < ncols; j++ ){
        M(i,j) =  distribution(generator);
    }

    //cout << M << endl;

    return M;
}
