#include "identifier.h"


identifier_table::identifier_table( historian_table * h ):historian(h)
{

}


identifier_table::~identifier_table( void )
{

}


bool identifier_table::contains( const string tag )
{
    map< const string, identifier, ltstr >::iterator i = table.find(tag);

    if ( i == table.end())
        return false;
    else
    {
        return true;
    }
}

bool identifier_table::add( const string tag, const int order, time_t sampling_rate)
{
    if ( !contains(tag))
    {
        identifier new_tag( historian , order, sampling_rate );
        return add( tag, new_tag );
    }
    else
        return false;
}

bool identifier_table::add( const string tag, const identifier model )
{
    if (contains(tag))
        return false;
    else
    {
        table.insert(
            map<string,identifier,ltstr>::
            value_type(tag,model ));
        return true;
    }

}

bool identifier_table::remove( const string tag )
{
    if ( contains(tag))
    {
        map<const string,identifier,ltstr>::iterator
        i = table.find(tag);
        table.erase( i );
        return true;
    }
    else
        return false;
}



list<string> identifier_table::get_model_tags(void)
{
    list <string> tags;
    map<const string, identifier,ltstr>::iterator j;

    for( j = table.begin(); j != table.end(); ++j )
        tags.push_back( (*j).first );
    return tags;
}


//you have to make sure that the tag really exist before doing this call.
identifier identifier_table::get_identifier( const string tag )
{
    map< const string, identifier, ltstr >::iterator i = table.find(tag);
    return (*i).second;
}


bool identifier_table::add_cv( const string tag, const string cv )
{
    if ( contains(tag))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).contains_cv( cv ) ? false : ((*j).second).add_cv( cv );
    }
    else
        return false;
}


bool identifier_table::add_cv( const string tag, list<string> cvs )
{

    if( contains( tag ))
    {
        bool ok = true;
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        list<string>::iterator i;
        for( i = cvs.begin(); i != cvs.end(); i++ )
            if ( not ((*j).second).contains_cv( *i))
                ok &= ((*j).second).add_cv( *i );

        return ok;
    }
    else
        return false;
}

bool identifier_table::contains_cv( const string tag, const string cv )
{
    if ( contains(tag))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).contains_cv( cv );
    }
    else
        return false;
}

bool identifier_table::add_mv( const string tag, const string mv )
{
    if ( contains(tag))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).contains_mv( mv ) ? false : ((*j).second).add_mv( mv );
    }
    else
        return false;
}


bool identifier_table::add_mv( const string tag, list<string> mvs )
{
    if( contains( tag ))
    {
        bool ok = true;
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        list<string>::iterator i;
        for( i = mvs.begin(); i != mvs.end(); i++ )
            if ( not ((*j).second).contains_mv( *i))
                ok &= ((*j).second).add_mv( *i );
        return ok;
    }
    else
        return false;
}

bool identifier_table::contains_mv( const string tag, const string mv )
{
    if ( contains(tag))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).contains_mv( mv );
    }
    else
        return false;
}

bool identifier_table::get_system_parameters( const string tag, MatrixXd& A, MatrixXd& B, MatrixXd& C )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
            return ((*j).second).get_system_parameters( A, B, C );
        else
            return false;
    }
    else
        return false;
}


bool identifier_table
::get_unity_steadystate_system( const string tag, MatrixXd& A_, MatrixXd& B_, MatrixXd& C_ )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
            return ((*j).second).get_unity_steadystate_system( A_, B_, C_ );
        else
            return false;
    }
    else
        return false;
}

bool identifier_table
::get_Kp(const string tag, MatrixXd& Kp )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        if( ((*j).second).is_identified() )
        {
            Kp = ((*j).second).get_Kp();
            return true;
        }
        else
            return false;
    }
    else
        return false;
}


bool identifier_table
::get_RGA(const string tag, MatrixXd& RGA)
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
        {
            RGA = ((*j).second).get_RGA();
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

bool identifier_table
::get_RGA_inverse(const string tag, MatrixXd& iRGA )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
        {
            iRGA = ((*j).second).get_RGA_inverse();
            return true;
        }
        else
            return false;
    }
    else
        return false;
}


bool identifier_table
::get_condition_number(const string tag, double& cn )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
        {
            cn = ((*j).second).get_condition_number();
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

bool identifier_table
::get_SV_decomposision( const string tag, MatrixXd& U, MatrixXd& S, MatrixXd& V )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
            return ((*j).second).get_SV_decomposision( U, S, V );
        else
            return false;
    }
    else
        return false;
}


bool identifier_table
::decompose_variables(const string tag)
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);

        if( ((*j).second).is_identified() )
            return ((*j).second).decompose_variables();
        else
            return false;
    }
    else
        return false;
}

bool identifier_table::get_previous_sqrt_error( const string tag, double & err )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        err = ((*j).second).get_previous_sqrt_error();
        return true;
    }
    else
        return false;
}

//die doel is om te bepaal hoe goed die model vergelyk met plant data.
bool identifier_table
::get_model_correlation( const string tag, double &r, const int deadtime )
{
    if( contains( tag ))
    {
        if ( is_identified(tag))
        {
            list<string> cvs;
            getCVs( tag, cvs );

            arx_simulate sim( tag, historian, this );

            MatrixXd simu_data = sim.general_response();


            MatrixXd hist_data;
            historian->return_normalized_history_matrix( cvs, hist_data );

            double sum = 0.0;
            for( unsigned i = 0; i < cvs.size(); i++ )
            {
                VectorXd u = simu_data.block( i, 0, 1, simu_data.cols()).transpose();
                VectorXd v = hist_data.block( i, 0, 1, hist_data.cols()).transpose();
                sum += calc_cross_correlation( u, v, deadtime );
            }
            r = sum / (double)cvs.size();
            return true;
        }

    }
    return false;
}

//Die idee hier is om die model af te trek van die regte date, en dan die variance van die
//resultaat uit te werk. In die geval van 'n multivariate model, aanvaar ek dat die gemiddelde
//variance die antwoord sal wees.
bool identifier_table
::get_noise_variance( const string tag, double &s, const int deadtime )
{
    if( contains( tag ))
    {
        if ( is_identified(tag))
        {
            list<string> cvs;
            getCVs( tag, cvs );

            arx_simulate sim( tag, historian, this );

            MatrixXd simu_data = sim.general_response();

            MatrixXd hist_data;
            historian->return_normalized_history_matrix( cvs, hist_data );

            double sum = 0.0;
            for( unsigned i = 0; i < cvs.size(); i++ )
            {
                VectorXd u = simu_data.block( i, 0, 1, simu_data.cols()).transpose();
                VectorXd v = hist_data.block( i, 0, 1, hist_data.cols()).transpose();
                VectorXd w = v-u;
                sum += calc_variance( w );  //ek hoop maar die variance is dieselfde mbt al die lesings..onwaarksynlik..maar kom ons kyk..
            }
            s = sum / (double)cvs.size();
            return true;
        }

    }
    return false;
}

bool identifier_table::find_model_deadtime( const string tag, int & deadtime )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        deadtime = ((*j).second).find_model_deadtime();
        return true;
    }
    else
        return false;
}

bool identifier_table::identify_model( const string tag, const int deadtime )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        ((*j).second).identify_model(deadtime);
        return true;
    }
    else
        return false;
}



bool identifier_table::return_sample_rate( const string tag, time_t& sample_rate )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        sample_rate = ((*j).second).return_sample_rate();
        return true;
    }
    else
        return false;
}


bool identifier_table::getCVs(const string tag, list <string> & cvs )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        cvs = ((*j).second).getCVs();
        return true;
    }
    else
        return false;
}

bool identifier_table::getMVs(const string tag, list <string> & mvs )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        mvs = ((*j).second).getMVs();
        return true;
    }
    else
        return false;
}



bool identifier_table::get_order( const string tag, int& order )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        order = ((*j).second).get_order();
        return true;
    }
    else
        return false;
}

bool identifier_table::is_identified( const string tag )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).is_identified();
    }
    else
        return false;
}

int identifier_table
::get_CV_count( const string tag )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).get_CV_count();
    }
    else
        return -1;
}

int identifier_table
::get_MV_count( const string tag )
{
    if( contains( tag ))
    {
        map<const string, identifier,ltstr>::iterator j = table.find(tag);
        return ((*j).second).get_MV_count();
    }
    else
        return -1;
}

//------------------------------------------------------------------------------
/* implementation of the identifier block itself
*/

identifier::identifier( historian_table * h, const int o, time_t sr  = 0 ):
    historian(h), order(o), sampling_rate(sr)
{
    sqrt_error = 0.0;
    identified = false;
}

identifier::identifier( const identifier& x)
{
    historian = x.historian;
    CVs = x.CVs;
    MVs = x.MVs;
    order = x.order;
    sampling_rate = x.sampling_rate;
    sqrt_error = x.sqrt_error;
    identified = x.identified;
}

identifier & identifier::operator=( const identifier& x )
{
    if (&x != this )
    {
        historian = x.historian;
        CVs = x.CVs;
        MVs = x.MVs;
        order = x.order;
        sampling_rate = x.sampling_rate;
        sqrt_error = x.sqrt_error;
        identified = x.identified;
    }
    return *this;
}

identifier::~identifier(void)
{

}


bool identifier::add_mv( const string mv )
{
    if( historian->contains( mv ) )
    {
        MVs.push_back( mv );
        return true;
    }
    else
        return false;
}

bool identifier::contains_mv( const string mv )
{
    list<string>::iterator i = find(MVs.begin(), MVs.end(), mv);
    if( i != MVs.end())
        return true;
    else
        return false;
}



bool identifier::add_cv( const string cv )
{
    if( historian->contains( cv ) )
    {
        CVs.push_back( cv );
        return true;
    }
    else
        return false;
}

bool identifier::contains_cv( const string cv )
{
    list<string>::iterator i = find(CVs.begin(), CVs.end(), cv);
    if( i != CVs.end())
        return true;
    else
        return false;
}



double identifier::identify_model( const int deadtime )
{
    identified = false;

    list< VectorXd > CV_Vs, MV_Vs;
    list<string> :: iterator i = CVs.begin();
    unsigned vector_len;
    historian->size( *i, vector_len );

    // dont identify if the historian has too little data
    if( (vector_len - deadtime) > (unsigned) 3*order )
    {



        if ( sampling_rate == 0 )
        {
            for( i = CVs.begin(); i != CVs.end(); i++ )
            {

                VectorXd tmp;
                historian->return_normalized_history_vector( *i, tmp );
                CV_Vs.push_back( tmp );
            }



            for( i = MVs.begin(); i != MVs.end(); i++ )
            {
                VectorXd tmp;
                historian->return_normalized_history_vector( *i, tmp );
                MV_Vs.push_back( tmp );
            }


        }
        else
        {
            for( i = CVs.begin(); i != CVs.end(); i++ )
            {
                VectorXd tmp;
                historian->return_normalized_history_vector( *i, tmp, sampling_rate );
                CV_Vs.push_back( tmp );
            }


            for( i = MVs.begin(); i != MVs.end(); i++ )
            {
                VectorXd tmp;
                historian->return_normalized_history_vector( *i, tmp, sampling_rate );
                MV_Vs.push_back( tmp );
            }
        }

        //all the variable vectors has been added to the CV_Vs,DV_Vs and MV_Vs structures
        //corresponding to the CVs, DVs and MVs lists.

        //generate A_{n+1} vectors
        int Vrows = vector_len - order - deadtime;

        MatrixXd A_NN  = MatrixXd::Zero( Vrows , CVs.size());
        list< VectorXd > :: iterator v;
        int j;
        for( v = CV_Vs.begin(), j = 0; v != CV_Vs.end(); v++, j++ )
        {
            A_NN.block(0, j, Vrows, 1 ) = (*v).segment( order + deadtime , Vrows );
        }

        //generate the A_{nxm} matrix
        MatrixXd A_N = MatrixXd::Ones( Vrows, (CVs.size() + MVs.size() ) * order + 1 );
        for( v = CV_Vs.begin(), j=0; v != CV_Vs.end(); v++, j++ )
        {
            for( int k = 0; k < order; k++ )
            {
                A_N.block( 0, (j*order) + k  , Vrows, 1) = (*v).segment(order - 1 - k + deadtime, Vrows ); //was (k + deadtime, Vrows );
            }
        }

        //insert the MV parts
        for( v = MV_Vs.begin(), j=0; v != MV_Vs.end(); v++, j++ )
        {
            for( int k = 0; k < order; k++ )
            {
                A_N.block( 0, (CVs.size() * order) + ( j * order ) + k, Vrows, 1 ) = (*v).segment(order - 1 - k, Vrows ); //was (k, Vrows );
            }
        }

        //we need to cut off the edges of the control and y-matrices, to ensure
        //that we dont model junk data.
        b = A_N.block(order,0,A_N.rows()- 2*order, A_N.cols()).jacobiSvd(ComputeThinU | ComputeThinV).solve(
                A_NN.block( order,0,A_NN.rows() - 2*order, A_NN.cols()));
        // this was b = A_N.jacobiSvd(ComputeThinU | ComputeThinV).solve(A_NN);


        //Calculate the square root error without the end_data.
        sqrt_error = ((A_NN.block( order,0,A_NN.rows() - 2*order, A_NN.cols())
                       - A_N.block(order,0,A_N.rows()- 2*order, A_N.cols()) * b).transpose()
                      * (A_NN.block( order,0,A_NN.rows() - 2*order, A_NN.cols())
                         - A_N.block(order,0,A_N.rows()- 2*order, A_N.cols()) * b)).sum();
        // this was sqrt_error = ((A_NN - A_N * b).transpose() * (A_NN - A_N * b)).sum();

        identified = true;
        return sqrt(sqrt_error);
    }
    else
    {
        identified = false;
        return -1.0;
    }
}


bool identifier::get_system_parameters( MatrixXd& A, MatrixXd& B, MatrixXd& C )
{
    if( CVs.size() > 0 and MVs.size() > 0 and is_identified())
    {
        MatrixXd bT = b.transpose(); //identify_model(deadtime).transpose();
        //int cols = b.cols();
        int rows = bT.rows();

        A = bT.block( 0, 0, rows, order * CVs.size() );
        B = bT.block( 0, order*CVs.size(), rows, order * MVs.size());
        C = bT.block( 0, order*(CVs.size() + MVs.size()), rows, 1 );

        return true;
    }
    else
        return false;
}


bool identifier
::get_unity_steadystate_system( MatrixXd& A_, MatrixXd& B_, MatrixXd& C_ )
{
    MatrixXd A,B,C;

    if ( get_system_parameters( A, B, C ))
    {
        A_ = get_sum( expand_matrix( A, get_order() ));
        B_ = get_sum( expand_matrix( B, get_order() ));
        C_ = C;
        return true;
    }
    else
        return false;
}

MatrixXd identifier
::get_Kp(void)
{
    MatrixXd A,B,C;
    get_system_parameters( A, B, C );
    list<MatrixXd> A_n = expand_matrix( A, get_order() );
    list<MatrixXd> B_n = expand_matrix( B, get_order() );

    MatrixXd A_sum = get_sum(A_n);
    MatrixXd B_sum = get_sum(B_n);
    MatrixXd I = MatrixXd::Identity(A.rows(),A.rows());

    return ( I - A_sum ).colPivHouseholderQr().solve( B_sum );
}

MatrixXd identifier
::get_RGA(void)
{
    MatrixXd Kp = get_Kp();
    if( Kp.rows() == Kp.cols() )
        return Kp.cwiseProduct( Kp.inverse().transpose() );
    else
    {
        MatrixXd Gp = (Kp.transpose() * Kp).inverse() * Kp.transpose() ;
        return  Kp.cwiseProduct( Gp.transpose() );
    }
}


MatrixXd identifier
::get_RGA_inverse(void)
{
    return get_RGA().inverse();
}

bool identifier
::get_SV_decomposision( MatrixXd& U, MatrixXd& S, MatrixXd& V )
{
    MatrixXd Kp = get_Kp();
    JacobiSVD<MatrixXd> svd(Kp, ComputeThinU | ComputeThinV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();

    return true;
}


bool identifier
::decompose_variables(void)
{
    //Die doel van hierdie funksie is om die PV's en CV's se interaction uit te haal, en om
    //dan nuwe PV_D's en CV_D's terug in die historian te laai.
    MatrixXd A_ss, B_ss, C_ss;

    //Bereken die steady state system
    if( get_unity_steadystate_system( A_ss, B_ss, C_ss ))
    {
        MatrixXd I = MatrixXd::Identity(A_ss.rows(),A_ss.cols());
        MatrixXd Kp = ( I - A_ss ).colPivHouseholderQr().solve( B_ss );

        // Do singular values decomposition
        JacobiSVD<MatrixXd> svd(Kp, ComputeThinU | ComputeThinV);
        MatrixXd U = svd.matrixU();
        MatrixXd D = svd.singularValues();
        MatrixXd V = svd.matrixV();
        MatrixXd CVData = get_CV_data_array().transpose();
        MatrixXd MVData = get_MV_data_array().transpose();

        //subtract the constant from the CVData array
        for( int i = 0; i < CVData.rows(); i++)
            for( int j = 0; j < CVData.cols(); j++ )
                CVData(i,j) -= C_ss(i,0);


        //do the diagonalizing
        MatrixXd CVDataD = U.transpose() * CVData;
        MatrixXd MVDataD = V.transpose() * MVData;


        list<string>::iterator p;
        int i;
        for( i = 0, p = CVs.begin(); i < CVDataD.rows(); i++,p++ )
        {
            //get the information from current cv tag
            string prefix_str("D");
            string new_tagname =  prefix_str.append(*p);

            if (historian->contains( new_tagname ))
                historian->remove( new_tagname );
            historian->add( new_tagname, historian->get_engineering_units(*p), historian->get_description(*p));

            time_t l_time = 0;
            for( int j = 0; j < CVDataD.cols(); j++)
            {
                historian->log(new_tagname,CVDataD(i,j), l_time);
                l_time += sampling_rate;
            }

        }

        //rename MV variable and add to historian
        for( i = 0, p = MVs.begin(); i < MVDataD.rows(); i++,p++ )
        {
            //get the information from current cv tag
            string prefix_str("D");
            string new_tagname =  prefix_str.append(*p);

            if (historian->contains( new_tagname ))
                historian->remove( new_tagname );

            historian->add( new_tagname, historian->get_engineering_units(*p), historian->get_description(*p));

            time_t l_time = 0;
            for( int j = 0; j < MVDataD.cols(); j++)
            {
                historian->log(new_tagname,MVDataD(i,j), l_time);
                l_time += sampling_rate;
            }

        }
        return true;
        //}else{
        //    return false;
        //}
    }
    return false;
}


MatrixXd identifier
::get_CV_data_array(void)
{
    list< VectorXd > CV_Vs;
    list<string> :: iterator i = CVs.begin();
    unsigned vector_len;
    historian->size( *i, vector_len );

    if ( sampling_rate == 0 )
    {
        for( i = CVs.begin(); i != CVs.end(); i++ )
        {
            VectorXd tmp;
            historian->return_normalized_history_vector( *i, tmp );
            CV_Vs.push_back( tmp );
        }
    }
    else
    {
        for( i = CVs.begin(); i != CVs.end(); i++ )
        {
            VectorXd tmp;
            historian->return_normalized_history_vector( *i, tmp, sampling_rate );
            CV_Vs.push_back( tmp );
        }
    }

    //create the data array.
    MatrixXd Data  = MatrixXd::Zero( vector_len , CVs.size());
    list< VectorXd > :: iterator v;
    int j;
    for( v = CV_Vs.begin(), j = 0; v != CV_Vs.end(); v++, j++ )
    {
        Data.block(0, j, vector_len, 1 ) = (*v).segment( 0 , vector_len );
    }

    return Data;
}


MatrixXd identifier
::get_MV_data_array(void)
{
    list< VectorXd > MV_Vs;
    list<string> :: iterator i = MVs.begin();
    unsigned vector_len;
    historian->size( *i, vector_len );

    if ( sampling_rate == 0 )
    {
        for( i = MVs.begin(); i != MVs.end(); i++ )
        {
            VectorXd tmp;
            historian->return_normalized_history_vector( *i, tmp );
            MV_Vs.push_back( tmp );
        }
    }
    else
    {
        for( i = MVs.begin(); i != MVs.end(); i++ )
        {
            VectorXd tmp;
            historian->return_normalized_history_vector( *i, tmp, sampling_rate );
            MV_Vs.push_back( tmp );
        }
    }

    //create the data array.
    MatrixXd Data  = MatrixXd::Zero( vector_len , MVs.size());
    list< VectorXd > :: iterator v;
    int j;
    for( v = MV_Vs.begin(), j = 0; v != MV_Vs.end(); v++, j++ )
    {
        Data.block(0, j, vector_len, 1 ) = (*v).segment( 0 , vector_len );
    }

    return Data;
}


double  identifier
::get_condition_number(void)
{
    MatrixXd Kp = get_Kp();
    JacobiSVD<MatrixXd> svd(Kp, ComputeThinU | ComputeThinV);
    MatrixXd v = svd.singularValues();
    return v.maxCoeff() / v.minCoeff();;
}



double identifier::calc_sqrt_error( const int deadtime )
{
    return identify_model(deadtime);
}



double identifier::get_previous_sqrt_error( void )
{
    if ( is_identified() )
        return sqrt_error;
    else
        return NAN;
}

//this function must be optimized at some time, but currently
//it is fine.
int identifier::find_model_deadtime( void )
{
    double cur_sq_err = DBL_MAX;
    int deadtime = 0;

    while( cur_sq_err > calc_sqrt_error(deadtime) )
    {
        cur_sq_err = calc_sqrt_error(deadtime);
        deadtime++;
    }
    return deadtime - 1;
}


