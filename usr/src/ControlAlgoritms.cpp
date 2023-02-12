#include "ControlAlgoritms.h"





// Calculate the feedback of the controller, for a given set of eigenvalues P, where P
// contains no eigenvalues of A
// A is an nxn real matrix,
// B is an nxp real matrix
// P is an 1xn complex matrix, all complex eigenvalues must be conjugate pairs.
MatrixXd place(MatrixXd A, MatrixXd B, MatrixXcd P)
{
    MatrixXd Pr = P.real();
    MatrixXd Pi = P.imag();
    MatrixXd F = MatrixXd::Zero(A.rows(),A.cols());
    for ( int i = 0; i < F.cols(); i++ )
    {
        F(i,i) = Pr(0,i);
        if ( i > 0 )
            if ( Pr(0,i-1) == Pr(0,i) and (Pi(0,i-1) == - Pi(0,i)))
            {
                F(i,i-1) = Pi(0,i);
                F(i-1,i) = Pi(0,i-1);
            }
    }

    // Select an arbitrary pxn matrix K_bar such that (F,K_bar) is observable
    MatrixXd K_bar = MatrixXd::Random(B.rows(),B.cols());
    MatrixXd T = MatrixXd::Zero(A.rows(),A.cols());

    while ( not is_invertible(T))
    {
        while ( not is_obsv(F,K_bar))
            K_bar = MatrixXd::Random(B.rows(),B.cols());
        T=lyapunov(A,-F,-B*K_bar);
    }

    return K_bar * pinv(T);
}

/**
    A is an nxn matrix,
    B is an nxm matrix
    Q is an nxn matrix
    R is an mxm matrix
    post:
    Return the controller feedback matrix
*/
MatrixXd dlqr(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R)
{
    MatrixXd K = MatrixXd::Zero(B.cols(),B.rows());
    MatrixXd X = MatrixXd::Identity( A.rows(),A.cols());

    if (is_invertible(A))
    {
        MatrixXd Z_11 = A + B * pinv(R) * B.transpose() * pinv(A).transpose() * Q;
        MatrixXd Z_12 = -B * pinv(R) * B.transpose() * pinv(A).transpose();
        MatrixXd Z_21 = -pinv(A).transpose() * Q;
        MatrixXd Z_22 = pinv(A).transpose();

        MatrixXd Z = MatrixXd::Zero(Z_11.rows()+Z_21.rows(),Z_11.cols() + Z_12.cols());

        Z.block(0,0,Z_11.rows(),Z_11.cols())  = Z_11;
        Z.block(0,Z_11.cols(),Z_12.rows(),Z_12.cols())  = Z_12;
        Z.block(Z_11.rows(),0,Z_21.rows(),Z_21.cols()) = Z_21;
        Z.block(Z_11.rows(),Z_11.cols(),Z_22.rows(),Z_22.cols())  = Z_22;

        MatrixXcd Lambda = eigenvalues( Z );
        MatrixXcd V = eigenvectors(Z);

        list<MatrixXcd> V_l;
        for( int i= 0; i < Lambda.rows(); i++)
        {
            if( abs(Lambda(i,0)) < 1.0 )
            {
                MatrixXcd v = V.block(0,i,V.rows(),1);
                V_l.push_back(v);
            }
        }

        int rows = Z.rows();

        MatrixXcd U = concat_MatrixXcd_list( V_l );

        MatrixXd U1 = MatrixXd::Zero(rows/2,U.cols());
        MatrixXd U2 = MatrixXd::Zero(rows/2,U.cols());


        for ( int i = 0; i < rows/2; i++ )
        {
            for ( int j = 0; j < U.cols(); j++ )
            {
                U1(i,j) = U(i,j).real();
                U2(i,j) = U(i+rows/2,j).real();
            }
        }

        X = U2 * pinv(U1);
    }

    if ( X.array().abs().sum() == NAN or not is_invertible(A))
    {

        //if all fails, iterate
        X = MatrixXd::Identity( A.rows(),A.cols());
        double f0 = 0.0;
        double f1 = X.array().abs().sum();
        int i = 0;
        do
        {
            f0 = f1;
            X = A.transpose()*X*A - (A.transpose()*X*B)*(R+B.transpose()*X*B).inverse()*(B.transpose()*X*A)+Q;
            f1 = X.array().abs().sum();
            i++;
            if (i > 1000)
                break;
        }
        while ( abs(f1-f0) > tol );
    }

    K=(B.transpose()*X*B+R).inverse()*B.transpose()*X*A;

    cout << "Eigenvalues of A-BK = " << endl << eigenvalues(A-B*K) << endl;

    return K;
}

/**
    Pre:
    A is a nxn matrix, state transition matrix
    G is a nxg matrix, process noise matrix
    C is a pxn matrix, measurement matrix
    Q is a gxg matrix, process noise covariance matrix
    R is a pxp matrix, measurement noise covariance matrix

    Post:
    Return the observer feedback matrix
*/
MatrixXd dlqe(MatrixXd A, MatrixXd G, MatrixXd C, MatrixXd Q, MatrixXd R)
{
    MatrixXd L;
    MatrixXd P = MatrixXd::Identity( A.rows(),A.cols());

    // TO BE TESTED MORE PROPERLY, NOT WORKING CORRECTLY
    if (is_invertible(A))
    {
        MatrixXd Z_11 = A.transpose() + C.transpose()*pinv(R)*C*pinv(A)*G*Q*G.transpose();
        MatrixXd Z_12 = -C.transpose() * pinv(R) * C * pinv(A);
        MatrixXd Z_21 = -pinv(A) * G * Q * G.transpose();
        MatrixXd Z_22 = pinv(A);

        MatrixXd Z = MatrixXd::Zero(Z_11.rows()+Z_21.rows(),Z_11.cols() + Z_12.cols());

        Z.block(0,0,Z_11.rows(),Z_11.cols())  = Z_11;
        Z.block(0,Z_11.cols(),Z_12.rows(),Z_12.cols())  = Z_12;
        Z.block(Z_11.rows(),0,Z_21.rows(),Z_21.cols()) = Z_21;
        Z.block(Z_11.rows(),Z_11.cols(),Z_22.rows(),Z_22.cols())  = Z_22;

        MatrixXcd Lambda = eigenvalues( Z );
        MatrixXcd V = eigenvectors(Z);

        list<MatrixXcd> V_l;
        for( int i= 0; i < Lambda.rows(); i++)
        {
            if( abs(Lambda(i,0)) < 1.0 )
            {
                MatrixXcd v = V.block(0,i,V.rows(),1);
                V_l.push_back(v);
            }
        }

        int rows = Z.rows();

        MatrixXcd U = concat_MatrixXcd_list( V_l );

        MatrixXd U1 = MatrixXd::Zero(rows/2,U.cols());
        MatrixXd U2 = MatrixXd::Zero(rows/2,U.cols());


        for ( int i = 0; i < rows/2; i++ )
        {
            for ( int j = 0; j < U.cols(); j++ )
            {
                U1(i,j) = U(i,j).real();
                U2(i,j) = U(i+rows/2,j).real();
            }
        }

        P = U2 * pinv(U1);
    }



    if ( P.array().abs().sum() == NAN or not is_invertible(A))
    {
        //if the above did not converge, try iteration
        P = MatrixXd::Identity( A.rows(),A.cols());

        double f1 = P.array().abs().sum();
        double f0 = 0.0;
        int i = 0;
        do
        {
            f0 = f1;
            P= A * ( P - P * C.transpose()*(C*P*C.transpose()+ R).inverse() * C * P)* A.transpose()+ G*Q*G.transpose();
            f1 = P.array().abs().sum();
            i++;
            if ( i > 1000 )
                break;
        }
        while ( abs(f1-f0) > tol);

    }

    L = P*C.transpose()*(C*P*C.transpose()+R).inverse();

    return L;
}


// X = lyap(A,Q) solves the Lyapunov matrix equation:
//
//      A*X + X*A' + Q = 0
MatrixXd lyapunov( MatrixXd A, MatrixXd Q)
{
    MatrixXcd Xc = sylvester( to_complex(A),to_complex(A.transpose()), to_complex(Q));
    MatrixXd  X = MatrixXd::Zero(Xc.rows(),Xc.cols());
    for (int i = 0; i < Xc.rows(); i++ )
        for (int j = 0; j < Xc.cols(); j++)
            X(i,j) = Xc(i,j).real();
    return X;
}


// X = lyap(A,B,C) solves the Sylvester equation:
//
//     A*X + X*B + C = 0
MatrixXd lyapunov( MatrixXd A, MatrixXd B, MatrixXd C)
{
    MatrixXcd Xc = sylvester( to_complex(A),to_complex(B), to_complex(C));
    MatrixXd  X = MatrixXd::Zero(Xc.rows(),Xc.cols());
    for (int i = 0; i < Xc.rows(); i++ )
        for (int j = 0; j < Xc.cols(); j++)
            X(i,j) = Xc(i,j).real();
    return X;
}


// Solve discrete lyaponov equation
// AXA' - X + B = 0
MatrixXd discrete_lyapunov( MatrixXd A, MatrixXd Q)
{

    MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
    MatrixXd B = pinv(A.transpose()-I )*(A.transpose()+I );
    MatrixXd Pb = lyapunov(B.transpose(),Q);
    return 0.5 * (B-I).transpose()*Pb*(B-I);
}

/**
 *Solve Sylvester matrix equation
 * A*X + X*B + C = 0
 */
MatrixXcd sylvester(MatrixXcd A, MatrixXcd B, MatrixXcd C )
{
    bool solve_backward = false;
    bool solve_forward = false;
    // Get the sizes
    int m = C.rows();
    int n = C.cols();

    // Compute Schur factorizations. TA will be upper triangular. TB will be upper or
    // lower. If TB is upper triangular then we backward solve; if it's lower
    // triangular then do forward solve.
    ComplexSchur<MatrixXcd> schur_of_A(A,true);
    MatrixXcd ZA = schur_of_A.matrixU();
    MatrixXcd TA = schur_of_A.matrixT();
    MatrixXcd ZB;
    MatrixXcd TB;

    if ( A == B.adjoint() )
    {
        ZB = ZA;
        TB = TA.adjoint(); //TB=TA'
        n = A.cols();
        solve_backward = true;
        solve_forward = false;
    }
    else if ( A == B.transpose() )
    {
        ZB = ZA.conjugate();
        TB = TA.transpose();
        solve_backward = false;
        solve_forward = true;
    }
    else
    {
        ComplexSchur<MatrixXcd> schur_of_B(B,true);
        ZB = schur_of_B.matrixU();
        TB = schur_of_B.matrixT();
        solve_backward = false;
        solve_forward = true;
    }

    MatrixXcd F  = ZA.adjoint() * C * ZB;

    // Initialize storage for the transformed solution
    MatrixXcd Y  = MatrixXcd::Zero(m,n);
    MatrixXcd P = TA.diagonal();


    if (not solve_backward and solve_forward )
    {
        for ( int k = 0; k < n; k++ )
        {
            MatrixXcd rhs = F.block(0,k,F.rows(),1) + Y * TB.block(0,k,TB.rows(),1);
            for ( int i = 0; i < TA.rows(); i++)
                TA(i,i) = P(i,0) + TB(k,k);
            JacobiSVD<MatrixXcd> svd(TA, ComputeFullU | ComputeFullV );
            Y.block(0,k,Y.rows(),1) = svd.solve(-rhs);
        }
    }

    if (solve_backward and not solve_forward )
    {
        for ( int k = n; k > 0; k-- )
        {
            MatrixXcd rhs = F.block(0,k-1,F.rows(),1) + Y * TB.block(0,k-1,TB.rows(),1);
            for ( int i = 0; i < TA.rows(); i++)
                TA(i,i) = P(i,0) + TB(k-1,k-1);
            JacobiSVD<MatrixXcd> svd(TA, ComputeFullU | ComputeFullV );
            Y.block(0,k-1,Y.rows(),1) = svd.solve(-rhs);
        }
    }

    // Transform solution back
    MatrixXcd X = ZA * Y * ZB.adjoint();
    return X;
}

MatrixXcd eigenvalues( MatrixXd A )
{
    ComplexEigenSolver<MatrixXd> es(A,false);
    return es.eigenvalues();
}

MatrixXcd eigenvectors( MatrixXd A )
{
    ComplexEigenSolver<MatrixXd> es(A,true);
    return es.eigenvectors();
}

//real matrices always have real characteristic polynomial coefisients
MatrixXd characteristicPolynomial( MatrixXd A )
{
    int n = A.rows();

    MatrixXcd c = MatrixXcd::Zero(1,n+1);
    c(0,0) = 1.0;
    if (A.rows() == A.cols())
    {
        MatrixXcd lambda = eigenvalues(A);
        for (int j=0; j < n ; j++)
        {
            MatrixXcd T = c.block(0,1,1,j+1) - lambda(j,0) * c.block(0,0,1,j+1);
            c.block(0,1,1,j+1) = T;
        }
    }

    MatrixXd Poly = MatrixXd::Zero(c.rows(),c.cols());
    for ( int i=0; i < c.rows(); i++ )
        for ( int j=0; j < c.cols(); j++)
            Poly(i,j)=c(i,j).real();
    return Poly;
}

/**
 * return controlabilty matrix
 */
MatrixXd ctrb( MatrixXd A, MatrixXd B)
{
    list<MatrixXd> Qc;

    int n = A.cols();

    MatrixXd X = B;

    for( int i = 0; i < n ; i++)
    {
        Qc.push_back(X);
        X = A*X;
    }
    return concat_matrix_list(Qc);
}


// See if system is controllable
bool is_ctrb( MatrixXd A, MatrixXd B )
{

    MatrixXd Qc = ctrb(A,B);
    JacobiSVD<MatrixXd> sv(Qc,ComputeThinU | ComputeThinV);
    return sv.rank() == Qc.rows();
}

/**
 * Return observability matrix
 */
MatrixXd obsv( MatrixXd A, MatrixXd C)
{
    return ctrb(A.transpose(),C.transpose()).transpose();
}


bool is_obsv( MatrixXd A, MatrixXd C )
{

    MatrixXd Qo = obsv(A,C);
    JacobiSVD<MatrixXd> sv(Qo, ComputeThinU | ComputeThinV);
    return sv.rank() == Qo.cols();

}

MatrixXd gramian( MatrixXd E, MatrixXd F)
{
    return discrete_lyapunov(E,F*F.transpose());
}


/**
 * Change the sampling rate for invertible Ad, Bc  matrices. Algorithm in 'Computer Controlled systems, Theory and Design' by
 * Karl J. Astrom and Bjorn Wittenmark, 3 ed, p34..p37
 */
bool ResetModelSamplingRate( MatrixXd A, MatrixXd B, time_t old_sampling_rate, MatrixXd& A_new, MatrixXd& B_new, time_t new_sampling_rate)
{
    unsigned n = A.cols() + B.cols();
    MatrixXd M = MatrixXd::Zero(n,n);
    M.block(0,0,A.rows(),A.cols()) = A;
    M.block(0,A.cols(),B.rows(),B.cols()) = B;
    M.block(A.rows(),A.cols(),M.rows()-B.rows(),M.cols()-A.cols()) = MatrixXd::Identity(M.rows()-B.rows(),M.cols()-A.cols());

    if (is_invertible(M))
    {
        // A matrix has a logarithm if and only if it is invertible.
        MatrixXd Ac = M.log() / (double)old_sampling_rate;
        MatrixXd M_= (Ac*(double)new_sampling_rate).exp();

        A_new = M_.block(0,0,A.rows(),A.cols());
        B_new = M_.block(0,A.cols(),B.rows(),B.cols());
        return true;
    }
    else
    {
        //algorithm failed
        return false;
    }
}


//te be deleted
MatrixXd derive_Psi( list<MatrixXd> A, const unsigned order )
{
    list<MatrixXd>::iterator i = A.begin();

    int vect_rows = (*i).rows();
    int vect_cols = (*i).cols();

    int rows = A.size() * vect_rows;
    int cols = A.size() * vect_cols;

    MatrixXd I = MatrixXd::Identity(vect_rows,vect_cols);
    MatrixXd O = MatrixXd::Zero(vect_rows,vect_cols );

    MatrixXd Gamma = MatrixXd::Zero(rows,cols);
    MatrixXd T = concat_matrix_list( A );
    Gamma.block(0,0,T.rows(),T.cols()) = T;
    //unsigned midpoint = (A.size() + 1 )/2;
    unsigned midpoint = order;

    if ( midpoint > 0 )
    {
        for ( unsigned j = 1; j < A.size() ; j++ )
            if (j != midpoint)
                Gamma.block( j*vect_rows,(j-1)*vect_cols, I.rows(),I.cols()) = I;
            else
                Gamma.block( j*vect_rows,(j-1)*vect_cols, O.rows(),O.cols()) = O;
    }
    return Gamma;
}



MatrixXd fir2ss_Psi( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    list<MatrixXd> A_n, B_n;
    list<MatrixXd> A;

    A_n = expand_matrix( A_, order );
    B_n = expand_matrix( B_, order );

    //Get the dimensions of the first entry of the A_n list
    list<MatrixXd>::iterator i = A_n.begin();
    list<MatrixXd>::iterator k = B_n.begin();
    int rows = (*i).rows();
    int b_cols = (*k).cols();

    //generate the list of matrices to be concatenated.
    for( i = A_n.begin(); i != A_n.end(); i++ )
        A.push_back( *i );
    for( i = B_n.begin(); i != B_n.end(); i++ )
    {
        if ( i != B_n.begin())
            A.push_back(*i);
    }

    int Psi_cols = get_sum_of_columns(A);
    MatrixXd Psi = MatrixXd::Zero(Psi_cols,Psi_cols);
    Psi.block(0,0,rows,Psi_cols) = concat_matrix_list( A );

    // return for 1-st order equations.
    if ( Psi_cols == rows )
        return Psi;
    //Prepare the identity matrix for copying over to Psi
    MatrixXd Id = MatrixXd::Identity(Psi_cols-rows,Psi_cols);

    unsigned j;
    int current_row = 0;
    int current_col = 0;

    for ( i = A.begin(),j= 0; i != A.end(); i++,j++)
    {
        if ( j == A_n.size()-1)
        {
            Id.block(current_row,current_col,b_cols,b_cols) = MatrixXd::Zero(b_cols,b_cols);

        }
        current_row += (*i).rows();
        current_col += (*i).cols();
    }

    Psi.block(rows,0,Id.rows(),Id.cols()) = Id;
    return Psi;

}


MatrixXd fir2ss_Gamma( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    list<MatrixXd> A_n, B_n;

    A_n = expand_matrix( A_, order );
    B_n = expand_matrix( B_, order );
    if ( A_n.size() == 1)
        return B_;
    list<MatrixXd>::iterator i = B_n.begin();

    MatrixXd B_0 = *i;

    MatrixXd  I = MatrixXd::Identity(B_0.cols(),B_0.cols());

    int Gamma_rows = get_sum_of_columns(A_n) + get_sum_of_columns(B_n) - B_0.cols();
    MatrixXd Gamma = MatrixXd::Zero(Gamma_rows, B_0.cols());
    //MatrixXd I = MatrixXd::Identity(B_0.rows(),B_0.cols());
    //MatrixXd Gamma = MatrixXd::Zero((A_n.size()+B_n.size()-1)*B_0.rows(), B_0.cols());
    Gamma.block(0,0,B_0.rows(),B_0.cols()) = B_0;
    Gamma.block(order*B_0.rows(),0,I.rows(),I.cols()) = I;
    return Gamma;
}

/**
 * Return only the mininal feedback matrix
 */
MatrixXd fir2ss_C( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    list<MatrixXd> A_n, B_n;
    A_n = expand_matrix( A_, order );
    B_n = expand_matrix( B_, order );
    list<MatrixXd>::iterator i = B_n.begin();

    int C_cols = get_sum_of_columns(A_n) + get_sum_of_columns(B_n) - (*i).cols();
    MatrixXd C = MatrixXd::Identity(A_.rows(),C_cols);
    //MatrixXd C = MatrixXd::Identity(C_cols,C_cols);
    return C;
}




MatrixXd fir2ss_Fixedpoint( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    MatrixXd A = get_sum(expand_matrix(A_, order));
    MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
    return pinv(I-A)*C_;
}

/**
 * return only the pv feedback state matrix for the order of the system
 */
MatrixXd fir2ss_C1(MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    list<MatrixXd> A_n, B_n;
    A_n = expand_matrix( A_, order );
    B_n = expand_matrix( B_, order );
    list<MatrixXd>::iterator i = B_n.begin();

    int C_cols = get_sum_of_columns(A_n) + get_sum_of_columns(B_n) - (*i).cols();
    MatrixXd C = MatrixXd::Identity(A_.rows()*order,C_cols);
    //MatrixXd C = MatrixXd::Identity(C_cols,C_cols);
    return C;
}

MatrixXd fir2ss_Fixedpoint1( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{

    MatrixXd A = get_sum(expand_matrix(A_,order));
    MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
    MatrixXd Fp = pinv(I-A)*C_;
    MatrixXd D = MatrixXd::Zero(Fp.rows()*order,1);
    for( int i = 0; i < order; i++ )
        D.block(i*Fp.rows(),0,Fp.rows(),Fp.cols()) = Fp;
    return D;
}

/**
 * Return the full state feedback C matrix
 */
MatrixXd fir2ss_C2(MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    list<MatrixXd> A_n, B_n;
    A_n = expand_matrix( A_, order );
    B_n = expand_matrix( B_, order );
    list<MatrixXd>::iterator i = B_n.begin();

    int C_cols = get_sum_of_columns(A_n) + get_sum_of_columns(B_n) - (*i).cols();
    MatrixXd C = MatrixXd::Identity(A_.rows(),C_cols);
    //MatrixXd C = MatrixXd::Identity(C_cols,C_cols);
    return C;
}

MatrixXd fir2ss_Fixedpoint2( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    MatrixXd A = fir2ss_Psi( A_, B_, C_, order );
    MatrixXd C = fir2fir_C(A_,B_,C_,order);
    //MatrixXd C = MatrixXd::Zero(A.rows(),1);

    //C.block(0,0,C_.rows(),C_.cols()) = C_;

    MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());

    MatrixXd D = pinv(I - A) * C;
    return D;
}



// Hierdie het ek gebou vir indien 'n ou wil 'n hoe orde-vergelyking na 'n eerste orde
// wil verander, dit nou vir C. Die ander tipeis sal wees fir2ss_Gamma vir B en
// fir2ss_Psi vir A. Ek sal dit nie meer gebruik as ek jy is nie. Die statespace convert
// van FIR af is meer handig.
MatrixXd fir2fir_C( MatrixXd A_, MatrixXd B_, MatrixXd C_, const int order )
{
    list<MatrixXd> A_n, B_n;
    A_n = expand_matrix( A_, order );
    B_n = expand_matrix( B_, order );

    MatrixXd C = MatrixXd::Zero((A_n.size()+B_n.size()-1)*A_.rows(),C_.cols());
    C.block(0,0,C_.rows(),C_.cols()) = C_;
    return C;
}


/**
 * Check if model's matrices are of the correct dimensions for operation
 */
bool is_model_valid( MatrixXd A_,MatrixXd B_, MatrixXd C_, MatrixXd D_)
{
    return A_.rows() == A_.cols() and A_.rows() == B_.rows() and A_.cols() == C_.cols() and C_.rows() == D_.rows();
}


/**
 * estimate V , where Y_{n} = C X_{n} + D + V_{n}
 * Data is in rows, eg 6 variables, multiple columns and 6 rows for each instance of data.
 * X - Observer Estimate, Y - PLant Data
 * Return V_{n}, data in row format
 */
MatrixXd ss_measurement_noise_matrix( MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y )
{
    MatrixXd Y_ = C * X;

    for( int i = 0; i < Y_.rows(); i++ )
    {
        for( int j = 0; j < Y_.cols(); j++)
            Y_(i,j) +=  D(i,0);
    }
    //cout << "Y_=C*X + D" << endl;
    MatrixXd V = Y - Y_;
    //cout << "subtracted Y" << endl;
    return V;
}


/**
 * estimate E( w_n ), where X_{n+1} = A X_{n} + B U_{n} + W_{n}
 * Data is in rows, eg 6 variables, multiple columns and 6 rows for each instance of data.
 * X - Observer Estimate, Y - PLant Data, Matrices A,C,D is from statespace model.
 * Return W_{n}, data in row format
 */
MatrixXd ss_process_noise_matrix( MatrixXd A, MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y)
{
    MatrixXd Y_sub_D = MatrixXd::Zero(Y.rows(),Y.cols());
    /* we may also need to remove the measurement noise before calculating the process noise */
    MatrixXd V = ss_measurement_noise_matrix( C, D, X, Y );

    for( int i = 0; i < Y.rows(); i++ )
    {
        for( int j = 0; j < Y.cols(); j++ )
        {
            Y_sub_D(i,j) = Y(i,j) - D(i,0) - V(i,j);
        }
    }

    /*
        W_n = X_{n+1)} - \hat{X}_{n+1} - A (X_n - \hat{X}_n)
    */
    MatrixXd X_e = pinv(C)*Y_sub_D - X;
    MatrixXd AX_e = A * X_e;

    MatrixXd W = X_e.block(0,1,X_e.rows(),X_e.cols()-1) - AX_e.block(0,0, AX_e.rows(),AX_e.cols()-1);
    //MatrixXd W  = MatrixXd::Zero(X_e.rows(),X_e.cols());
    //W.block(0,0, X_e.rows(),X_e.cols()-1) = X_e.block(0,1,X_e.rows(),X_e.cols()-1) - AX_e.block(0,0, AX_e.rows(),AX_e.cols()-1);
    return W;
}


/**
 *  Calculte R = Cov(V) for to be used in dlqe method.
 */
MatrixXd ss_measurement_noise_cov( MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y )
{
    MatrixXd V = ss_measurement_noise_matrix(C,D,X,Y).transpose();
    MatrixXd Vcov = calc_matrix_covariance(V);
    MatrixXd Vcd = MatrixXd::Identity(Vcov.rows(),Vcov.cols());
    for( int i = 0; i < Vcd.rows(); i++ )
        for ( int j = 0; j < Vcd.cols(); j++)
            if (i==j)
                Vcd(i,j) = Vcov(i,j);
    return Vcd;
}

/**
 *  Calculte Q = Cov(W) for to be used in dlqe method.
 */
MatrixXd ss_process_noise_cov( MatrixXd A, MatrixXd C, MatrixXd D, MatrixXd X, MatrixXd Y)
{
    MatrixXd W = ss_process_noise_matrix(A,C,D,X,Y).transpose();
    return calc_matrix_covariance(W);
}
