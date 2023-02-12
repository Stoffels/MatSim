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

int get_sum_of_columns( list<MatrixXd> A)
{
    int cols = 0;
    list<MatrixXd>::iterator i;
    for ( i = A.begin(); i!= A.end(); ++i)
        cols += (*i).cols();
    return cols;
}

int get_sum_of_columns( list<MatrixXcd> A)
{
    int cols = 0;
    list<MatrixXcd>::iterator i;
    for ( i = A.begin(); i!= A.end(); ++i)
        cols += (*i).cols();
    return cols;
}

MatrixXd concat_matrix_list( list<MatrixXd> l )
{
    list<MatrixXd>::iterator i = l.begin();
    if( l.size() > 0 )
    {
        //MatrixXd M = MatrixXd::Zero( (*i).rows(), l.size() * (*i).cols());
        MatrixXd M = MatrixXd::Zero( (*i).rows(), get_sum_of_columns(l));
        int col_step = (*i).cols();
        int col;
        for( i = l.begin(), col = 0; i != l.end(); col += col_step, i++ )
            M.block(0,col, (*i).rows(), (*i).cols()) = *i;
        return M;
    }
    else
        return MatrixXd::Zero(1,1);

}


MatrixXcd concat_MatrixXcd_list( list<MatrixXcd> l ){
    list<MatrixXcd>::iterator i = l.begin();
    if( l.size() > 0 )
    {
        //MatrixXd M = MatrixXd::Zero( (*i).rows(), l.size() * (*i).cols());
        MatrixXcd M = MatrixXcd::Zero( (*i).rows(), get_sum_of_columns(l));
        int col_step = (*i).cols();
        int col;
        for( i = l.begin(), col = 0; i != l.end(); col += col_step, i++ )
            M.block(0,col, (*i).rows(), (*i).cols()) = *i;
        return M;
    }
    else
        return MatrixXcd::Zero(1,1);
}

//all columns and rows are or the same size in list entry
MatrixXd stack_matrix_list( list<MatrixXd> l )
{

    if ( l.size() > 0 )
    {
        list<MatrixXd>::iterator i = l.begin();
        MatrixXd T = MatrixXd::Zero( (*i).rows() * l.size(), (*i).cols());
        int j = 0;

        for( i = l.begin(), j= 0; i != l.end(); i++,j++)
            T.block(j * (*i).rows(),0, (*i).rows(),(*i).cols()) = *i;

        return T;
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


MatrixXcd to_complex( MatrixXd A )
{
    MatrixXcd T = MatrixXcd::Zero(A.rows(),A.cols());
    for (int i = 0; i < A.rows(); i++)
        for( int j = 0; j < A.cols(); j++)
            T(i,j) = A(i,j);
    return T;
}

//
MatrixXd get_real_part( MatrixXcd A )
{
    MatrixXd T = MatrixXd::Zero( A.rows(), A.cols());
    for ( int i = 0; i < A.rows(); i++ )
        for ( int j = 0; j < A.cols(); j++)
            T(i,j) = A(i,j).real();
    return T;
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



bool is_invertible( MatrixXd A )
{
    if ( A.rows() == A.cols())
    {
        JacobiSVD<MatrixXd> svd(A);
        return svd.rank() == A.rows();
    }
    else
        return false;
}

long Rank( MatrixXd A )
{
    JacobiSVD<MatrixXd> svd( A, ComputeFullU | ComputeFullV);
    return svd.rank();
}

MatrixXd Range( MatrixXd A )
{
    JacobiSVD<MatrixXd> svd( A, ComputeFullU | ComputeFullV);
    MatrixXd U = svd.matrixU();
    long r = svd.rank();
    return U.block(0,0,U.rows(),r);
}


MatrixXd Nullity( MatrixXd A )
{
    JacobiSVD<MatrixXd> svd( A, ComputeFullU | ComputeFullV);
    MatrixXd V = svd.matrixV();
    long r = svd.rank();
    return V.block(0,r,V.rows(),V.cols()-r);
}

// Purpose: The calculation of the range and null space matrices of an
//          mxn matrix A
bool NullRangeSpace ( MatrixXd A, MatrixXd& N, MatrixXd& R )
{
    JacobiSVD<MatrixXd> svd( A, ComputeFullU | ComputeFullV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    long r = svd.rank();
    N = V.block(0,r,V.rows(),V.cols()-r);
    R = U.block(0,0,U.rows(),r);
    return true;
}

// purpose: To decompose the matrix Q into two subspaces: Qr - projection of Q into range space of R,
// and Qou, the subspace outside the range space R
// R is an nxm matrx, Q is an nxk matrix
bool INOU( MatrixXd Q, MatrixXd R, MatrixXd& Qr, MatrixXd Qou )
{
    long k = Q.cols();

    // Let QR = [Q|R]
    MatrixXd QR = MatrixXd::Zero(Q.rows(), Q.cols()+R.cols());
    QR.block(0,0,Q.rows(),Q.cols()) = Q;
    QR.block(0,Q.cols(),R.rows(),R.cols()) = R;

    MatrixXd Nqr = Nullity(QR);
    MatrixXd Nq  = Nqr.block( 0, 0, k, Nqr.cols());

    MatrixXd Nqn = Nullity(Nq.transpose());

    Qr = Q*Nq;
    Qou = Q*Nqn;

    return true;
}


MatrixXd clamp( MatrixXd M, MatrixXd Min, MatrixXd Max)
{
    if ( M.rows() == Min.rows() and M.cols() == Min.cols()  and
            M.rows() == Max.rows() and M.cols() == Max.cols())
    {
        for( int i = 0; i < M.rows(); i++ )
            for (int j = 0; j < M.cols(); j++)
            {
                if (M(i,j) < Min(i,j))
                    M(i,j) = Min(i,j);
                if (M(i,j) > Max(i,j))
                    M(i,j) = Max(i,j);
            }
    }
    return M;
}


bool is_nan( MatrixXd M){
    //cout << isnan(M.sum()) << endl;
    return not((M.array() == M.array())).all();
}

bool is_finite( MatrixXd M){
    return ((M - M).array() == (M - M).array()).all();
}
