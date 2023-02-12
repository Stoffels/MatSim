#include "matrix_utils.h"


list<MatrixXd> expand_matrix( MatrixXd & M, const int order )
{
    list<MatrixXd> result;

    int variables = M.cols()/order;
    for( int i = 0; i < order ; i++ )
    {
        MatrixXd m = MatrixXd::Zero( M.rows(), variables );
        int j,k;

        for ( j = i, k = 0 ; j < variables*order; j += order,k++ )
            m.block(0,k,M.rows(),1) = M.block(0,j,M.rows(),1);
        result.push_back(m);
    }
    return result;
}



MatrixXd concat_matrix_list( list<MatrixXd> l )
{
    list<MatrixXd>::iterator i = l.begin();
    if( l.size() > 0 )
    {
        MatrixXd M = MatrixXd::Zero( (*i).rows(), l.size() * (*i).cols());
        int col_step = (*i).cols();
        int col;
        for( i = l.begin(), col = 0; i != l.end(); col += col_step, i++ )
            M.block(0,col, (*i).rows(), (*i).cols()) = *i;
        return M;
    }
    else
        return MatrixXd::Zero(1,1);

}



MatrixXd concat_matrix_vector( vector<MatrixXd> l )
{
    vector<MatrixXd>::iterator i = l.begin();
    if( l.size() > 0 )
    {
        MatrixXd M = MatrixXd::Zero( (*i).rows(), l.size() * (*i).cols());
        int col_step = (*i).cols();
        int col;
        for( i = l.begin(), col = 0; i != l.end(); col += col_step, i++ )
            M.block(0,col, (*i).rows(), (*i).cols()) = *i;
        return M;
    }
    else
        return MatrixXd::Zero(1,1);

}



MatrixXd  get_sum( list<MatrixXd> l )
{
    if( l.size() > 0 )
    {
        list<MatrixXd>::iterator i=l.begin();
        MatrixXd SUM = MatrixXd::Zero( (*i).rows(), (*i).cols() );
        for( i = l.begin(); i != l.end(); i++ )
            SUM += *i;
        return SUM;
    }
    else
        return MatrixXd::Zero(1,1);
}


vector<MatrixXd> list2vector( list<MatrixXd> x)
{
    list<MatrixXd>::iterator i;

    vector<MatrixXd> y;

    for( i = x.begin(); i != x.end(); i++ )
        y.push_back( *i );

    return y;

}

MatrixXd pinv( MatrixXd A )
{
    JacobiSVD<MatrixXd> svd( A, ComputeThinU|ComputeThinV);
    MatrixXd U = svd.matrixU();

    MatrixXd L = svd.singularValues();
    MatrixXd lambda = MatrixXd::Identity(L.rows(),L.rows());
    for ( int i = 0; i < L.rows(); i++ )
        lambda(i,i) = L(i,0);
    MatrixXd V = svd.matrixV();

    return V* lambda.inverse()*U.transpose();
}




