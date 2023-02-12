#include "history.h"

historian_table::historian_table(const long ml):max_length(ml)
{
}


historian_table::~historian_table()
{

}

historian_table::historian_table( const historian_table& x){
    max_length = x.max_length;
    table = x.table;
}

historian_table& historian_table::operator=(const historian_table& x){
    if (&x != this)
    {
        max_length = x.max_length;
        table = x.table;
    }
    return *this;
}


bool historian_table::add( const string tag_name,
                           const string eu, const string desc )
{
    if (contains(tag_name))
        return false;
    else
    {
        historian_variable new_var( max_length, eu , desc );
        table.insert(
            map<string,historian_variable,ltstr>::
            value_type(tag_name,new_var));
        return true;
    }
}


bool historian_table::remove( const string tag_name )
{
    if ( contains(tag_name))
    {
        map<const string,historian_variable,ltstr>::iterator
        i = table.find(tag_name);
        table.erase( i );
        return true;
    }
    else
        return false;
}

bool historian_table::clear( void )
{
    table.clear();
    return true;
}


bool historian_table::contains( const string tag_name )
{
    map< const string, historian_variable, ltstr >::
    iterator i = table.find(tag_name);
    if ( i == table.end())
        return false;
    else
    {
        return true;
    }
}


bool historian_table
::log( const string tag_name, const double value, const time_t mtime )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        //cout << tag_name << "->" << value << endl;
        ((*j).second).log( value, mtime );
        return true;
    }
    else
        return false;
}

bool historian_table::clear_variable_history( void )
{
    map<const string,historian_variable,ltstr>::iterator i;
    for( i= table.begin(); i != table.end(); ++ i)
        ((*i).second).clear_variable_history();
    return true;
}


bool historian_table::size( const string tag_name, unsigned& len )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        len = ((*j).second).size();
        return true;
    }
    else
        return false;
}


string historian_table::get_engineering_units(const string tag_name )
{
    string EU;
    if ( contains(tag_name))
    {
        map<const string,historian_variable,ltstr>::iterator
        i = table.find(tag_name);
        return ((*i).second).get_engineering_units();
    }
    else
        return EU;
}
string historian_table::get_description( const string tag_name )
{
    string descr;
    if ( contains(tag_name))
    {
        map<const string,historian_variable,ltstr>::iterator
        i = table.find(tag_name);
        return ((*i).second).get_description();
    }
    else
        return descr;
}


bool historian_table::get_snapshot_value(
    const string tag_name,
    const unsigned i,
    double& value )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        value = ((*j).second).get_snapshot_value( i );
        return true;
    }
    else
        return false;

}

//This allows the user to get snapshots at certain times, even the
//logspeed may be different.
bool historian_table::get_snapshot_value(
    const string tag_name,
    const time_t t,
    double& value )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        value = ((*j).second).get_snapshot_value( t );
        return true;
    }
    else
        return false;

}

bool historian_table
::get_normalized_snapshot_value( const string tag_name, const unsigned i, double & val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_normalized_snapshot_value( i );
        return true;
    }
    else
        return false;

}

bool historian_table
::get_normalized_snapshot_value( const string tag_name, const time_t t, double & val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_normalized_snapshot_value( t );
        return true;
    }
    else
        return false;
}



bool historian_table::get_snapshot_time(
    const string tag_name,
    const unsigned i,
    time_t& the_time )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        the_time = ((*j).second).get_snapshot_time( i );
        return true;
    }
    else
        return false;

}



bool historian_table
::get_log_time_interval( const string tag_name, time_t & dT )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator j = table.find(tag_name);
        dT = ((*j).second).get_log_interval();
        return true;
    }
    else
        return false;
}


bool historian_table
::get_start_time( const string tag_name, time_t & T )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator j = table.find(tag_name);
        T = ((*j).second).get_start_time();
        return true;
    }
    else
        return false;

}

bool historian_table
::get_end_time( const string tag_name, time_t & T )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator j = table.find(tag_name);
        T = ((*j).second).get_end_time();
        return true;
    }
    else
        return false;

}


bool historian_table
::return_history_vector( const string tag_name, VectorXd& values )
{
    if ( contains( tag_name ) )
    {

        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        values = ((*j).second).get_raw_history_vector();
        return true;
    }
    else
        return false;
}


bool historian_table
::return_history_vector( const string tag_name, VectorXd& DataVector, const time_t sampling_rate )
{
    if ( contains( tag_name ) )
    {
        time_t dT = 1;
        get_log_time_interval( tag_name, dT );

        if( sampling_rate == dT )
            return return_history_vector( tag_name, DataVector );
        else
        {

            time_t start_time = 0;
            time_t end_time = 1;
            vector<double> V;

            if ( get_start_time( tag_name, start_time ) and get_end_time( tag_name, end_time ))
            {
                //wxString sta_str, sto_str;
                //sta_str << start_time;
                //sto_str << end_time;
                //wxMessageBox( sta_str + wxT(": ") + sto_str );
                for( time_t t = start_time; t < end_time; t += sampling_rate )
                {
                    double value;
                    get_snapshot_value( tag_name, t, value );
                    V.push_back( value );
                }

                VectorXd tmp( V.size() );
                for( unsigned i = 0; i < V.size(); i++ )
                    tmp(i) = V[i];

                DataVector = tmp;
                return true;
            }
            else
                return false;
        }
        return true;
    }
    else
        return false;
}


bool historian_table
::return_normalized_history_vector( const string tag_name, VectorXd& values )
{
    if ( contains( tag_name ) )
    {

        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        values = ((*j).second).get_normalized_history_vector();
        return true;
    }
    else
        return false;
}



bool historian_table
::return_normalized_history_vector( const string tag_name, VectorXd& values, const time_t sampling_rate )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        double URV = ((*j).second).get_UpperRangeValue();
        double LRV = ((*j).second).get_LowerRangeValue();
        double m = 100.0 / (URV - LRV);
        double c = - m * LRV;

        return_history_vector( tag_name, values, sampling_rate );
        // do the scaling
        for( int i = 0; i < values.rows(); i++ )
            for( int j = 0; j < values.cols(); j++ )
                values(i,j) = m * values(i,j) + c;

        return true;
    }
    else
        return false;
}

bool historian_table
::return_normalized_history_matrix( list<string> tagnames, MatrixXd& values )
{
    if (tagnames.size() > 0 )
    {
        list<string>::iterator i = tagnames.begin();

        unsigned cols = 0;
        size( *i, cols );

        if( cols == 0)
            return false;

        unsigned j;
        MatrixXd X = MatrixXd::Zero( tagnames.size(), cols );
        for( i = tagnames.begin(), j=0; i != tagnames.end(); i++, j++ )
        {
            VectorXd current_vect;
            return_normalized_history_vector( *i , current_vect );

            for( unsigned k = 0; k < cols; k++ )
                X(j,k) = current_vect(k);
        }
        values = X;
        return true;
    }
    else
        return false;
}

list<string> historian_table
::return_tagnames(void)
{
    list <string> tagnames;
    map<const string, historian_variable,ltstr>::iterator j;
    for( j = table.begin(); j != table.end(); ++j )
        tagnames.push_back( (*j).first );
    return tagnames;
}


bool historian_table
::get_max_value( const string tag_name, double & val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_max_value();
        return true;
    }
    else
        return false;
}

bool historian_table
::get_min_value( const string tag_name, double & val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_min_value();
        return true;
    }
    else
        return false;

}

bool historian_table::get_last_logged_value( const string tag_name, double & val){
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_last_logged_value();
        return true;
    }
    else
        return false;
}

bool historian_table
::get_min_length( list <string> tag_names, unsigned & len )
{

    //   list<string>::iterator j = input_tags.begin();
    // ek neem aan dat al die data die selfde hoeveelheid kere ge-enter is.
    //historian->size( (*j), len );

    list<string>::iterator j = tag_names.begin();
    len = UINT_MAX;

    for( j = tag_names.begin(); j != tag_names.end(); ++j )
    {
        string tag_name = (*j);

        //soek die kortste lengte van al die tags
        if( contains( tag_name ))
        {
            map<const string,historian_variable,ltstr>::iterator
            k = table.find(tag_name);
            unsigned my_len = ((*k).second).size();

            if ( my_len < len )
                len = my_len;
        }
        else
            len = 0;
    }

    return (len != 0);
}


bool historian_table
::set_LowerRangeValue( const string tag_name, const double val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        //cout << tag_name << "->" << value << endl;
        ((*j).second).set_LowerRangeValue( val  );
        return true;
    }
    else
        return false;
}

bool historian_table
::get_LowerRangeValue( const string tag_name, double & val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_LowerRangeValue();
        return true;
    }
    else
        return false;
}

bool historian_table
::set_UpperRangeValue( const string tag_name, const double val  )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        //cout << tag_name << "->" << value << endl;
        ((*j).second).set_UpperRangeValue( val  );
        return true;
    }
    else
        return false;
}

bool historian_table
::get_UpperRangeValue( const string tag_name, double & val )
{
    if ( contains( tag_name ) )
    {
        map<const string,historian_variable,ltstr>::iterator
        j = table.find(tag_name);
        val = ((*j).second).get_UpperRangeValue();
        return true;
    }
    else
        return false;
}

MatrixXd historian_table
::get_correlation_table( void )
{
    list<string> tag_names = return_tagnames();
    list<string>::iterator i;
    list<string>::iterator j;

    MatrixXd A = MatrixXd::Zero(tag_names.size(), tag_names.size());

    int m, n;
    for( i = tag_names.begin(), m = 0; i!= tag_names.end(); i++, m++ )
        for( j = tag_names.begin(), n = 0; j!= tag_names.end(); j++, n++ )
        {
            VectorXd a;
            VectorXd b;

            return_normalized_history_vector( *i , a );
            return_normalized_history_vector( *j , b );
            A(m,n) = calc_correlation( a, b );

        }
    return A;
}


MatrixXd historian_table
::get_correlation_table( list<string> tag_names )
{
    list<string>::iterator i;
    list<string>::iterator j;

    MatrixXd A = MatrixXd::Zero(tag_names.size(), tag_names.size());

    int m, n;
    for( i = tag_names.begin(), m = 0; i!= tag_names.end(); i++, m++ )
        for( j = tag_names.begin(), n = 0; j!= tag_names.end(); j++, n++ )
        {
            VectorXd a;
            VectorXd b;

            return_normalized_history_vector( *i , a );
            return_normalized_history_vector( *j , b );
            A(m,n) = calc_correlation( a, b );

        }
    return A;
}

MatrixXd historian_table
::calc_covariance_table( void )
{
    list<string> tag_names = return_tagnames();
    list<string>::iterator i;
    list<string>::iterator j;

    MatrixXd A = MatrixXd::Zero(tag_names.size(), tag_names.size());

    int m, n;
    for( i = tag_names.begin(), m = 0; i!= tag_names.end(); i++, m++ )
        for( j = tag_names.begin(), n = 0; j!= tag_names.end(); j++, n++ )
        {
            VectorXd a;
            VectorXd b;

            return_normalized_history_vector( *i , a );
            return_normalized_history_vector( *j , b );
            A(m,n) = calc_covariance( a, b );

        }
    return A;
}

MatrixXd historian_table
::calc_covariance_table( list<string> tag_names )
{
    list<string>::iterator i;
    list<string>::iterator j;

    MatrixXd A = MatrixXd::Zero(tag_names.size(), tag_names.size());

    int m, n;
    for( i = tag_names.begin(), m = 0; i!= tag_names.end(); i++, m++ )
        for( j = tag_names.begin(), n = 0; j!= tag_names.end(); j++, n++ )
        {
            VectorXd a;
            VectorXd b;

            return_normalized_history_vector( *i , a );
            return_normalized_history_vector( *j , b );
            A(m,n) = calc_covariance( a, b );

        }
    return A;
}



double historian_table
::get_mean( const string tag_name )
{
    VectorXd v_;
    return_normalized_history_vector( tag_name , v_ );
    return calc_mean( v_ );
}

double historian_table
::get_variance( const string tag_name )
{
    VectorXd v_;
    return_normalized_history_vector( tag_name , v_ );
    return calc_variance( v_ );
}


//------------------------------------------------------------------------
//  Historian Variable. Stores queue of snapshots of variables
//------------------------------------------------------------------------
historian_variable
::historian_variable( const long ml, const string eu, const string descr  )
    : engineering_units(eu), description(descr),maximum_length(ml)
{
    //optimizing min-max value for efficient scaling
    min_value = DBL_MAX;
    max_value = -DBL_MAX;
    last_logged_value = 0.0;
    LRV = 0.0;
    URV = 100.0;
}


historian_variable::historian_variable( const historian_variable& x)
{
    engineering_units =  x.engineering_units;
    description = x.description;
    maximum_length = x.maximum_length;
    history.clear();
    history = x.history;
    min_value = x.min_value;
    max_value = x.max_value;
    last_logged_value = x.last_logged_value;
    LRV = x.LRV;
    URV = x.URV;
}

historian_variable& historian_variable::operator=(const historian_variable& x)
{
    if (&x != this)
    {
        engineering_units =  x.engineering_units;
        description = x.description;
        maximum_length = x.maximum_length;
        history.clear();
        history = x.history;
        min_value = x.min_value;
        max_value = x.max_value;
        last_logged_value = x.last_logged_value;
        LRV = x.LRV;
        URV = x.URV;
    }
    return *this;
}


historian_variable::~historian_variable()
{
}

void historian_variable::log( const double value, const time_t mtime )
{
    //if ( time != 0 )
    if ( value > max_value ) max_value = value;
    if ( value < min_value ) min_value = value;

    last_logged_value = value;

    timestamped_variable v( value, mtime );
    history.push_back( v );
    while (history.size() > maximum_length )
    {
        history.pop_front();

        //kan later geoptimiseer word, dit is om vinnig scaling te doen indien nodig
        max_value = -DBL_MAX;
        min_value = +DBL_MAX;
        for( unsigned i = 0; i < history.size(); i++ )
        {
            double val = history[i].get_snapshot_value();
            if (  val > max_value )
                max_value = val;
            if (  val < min_value )
                min_value = val;
        }
    }
}

void historian_variable::clear_variable_history(void)
{
    history.clear();
}

double historian_variable::get_snapshot_value( const unsigned i )const
{
    return history[i].get_snapshot_value();
}

//get snapshot at a certain time_t...independant of sample rates.
double historian_variable::get_snapshot_value( const time_t t) const
{
    timestamped_variable tv( 0, t );
    unsigned i = 0;
    while ( (i < history.size()) and (tv > history[i++]) );
    return history[i-1].get_snapshot_value();
}

double historian_variable
::get_normalized_snapshot_value( const unsigned i )const
{
    double m = 100.0 / (URV - LRV);
    double c = - m * LRV;
    return m*get_snapshot_value( i ) + c;

}

double historian_variable
::get_normalized_snapshot_value( const time_t t ) const
{
    double m = 100.0 / (URV - LRV);
    double c = - m * LRV;
    return m * get_snapshot_value( t ) + c;
}


time_t historian_variable::get_log_interval( void )
{
    time_t time_0 = get_snapshot_time( 0 );
    time_t time_1 = get_snapshot_time( 1 );
    return time_1 - time_0;
}


time_t historian_variable::get_start_time( void )
{
    return get_snapshot_time( 0 );
}

time_t historian_variable::get_end_time( void )
{
    return get_snapshot_time( size() - 1 );
}




time_t historian_variable::get_snapshot_time( const unsigned i )const
{
    return history[i].get_snapshot_time();
}



double  historian_variable::operator[](unsigned i)const
{
    return history[i].get_snapshot_value();
}


unsigned historian_variable::size( void )
{
    return history.size();
}


double historian_variable::get_max_value( void )
{
    return max_value;
}

double historian_variable::get_min_value( void )
{
    return min_value;
}

double historian_variable::get_last_logged_value( void){
    return last_logged_value;
}

VectorXd historian_variable::get_raw_history_vector( void )
{
    VectorXd v( size() );

    int i;
    deque< timestamped_variable >::iterator j;

    for( j = history.begin(), i = 0; j != history.end(); ++j, i++ )
        v(i) = (*j).get_snapshot_value();

    return v;
}

VectorXd historian_variable
::get_normalized_history_vector( void )
{

    VectorXd v( size() );
    double m = 100.0 / (URV - LRV);
    double c = - m * LRV;

    int i;
    deque< timestamped_variable >::iterator j;

    for( j = history.begin(), i = 0; j != history.end(); ++j, i++ )
        v(i) = m * (*j).get_snapshot_value() + c;
    return v;
}

//------------------------------------------------------------------------
//  Timestamped Variable. Wrapper class for variables
//------------------------------------------------------------------------
timestamped_variable
::timestamped_variable( const double value, const time_t mtime )
{
    snapshot_value = value;
    //cout << (unsigned) mtime << endl;
    if( mtime < 0 )
        snapshot_time = time( NULL );
    else
        snapshot_time = mtime;
}

timestamped_variable::~timestamped_variable()
{
}

timestamped_variable::timestamped_variable( const timestamped_variable& x)
{
    snapshot_value = x.get_snapshot_value();
    snapshot_time = x.get_snapshot_time();
}

time_t timestamped_variable::get_snapshot_time( void )const
{
    //cout << snapshot_time << endl;
    return snapshot_time;
}


double timestamped_variable::get_snapshot_value( void )const
{
    return snapshot_value;
}


timestamped_variable & timestamped_variable::operator=(
    const timestamped_variable& x)
{
    if (&x != this)
    {
        snapshot_value = x.get_snapshot_value();
        snapshot_time = x.get_snapshot_time();
    }
    return *this;
}




int operator==(const timestamped_variable& x, const timestamped_variable& y)
{
    return (x.snapshot_time == y.snapshot_time );
}

int operator!=(const timestamped_variable& x, const timestamped_variable& y)
{
    return (x.snapshot_time != y.snapshot_time );
}

int operator< (const timestamped_variable& x, const timestamped_variable& y)
{
    return (x.snapshot_time < y.snapshot_time );
}

int operator<=(const timestamped_variable& x, const timestamped_variable& y)
{
    return (x.snapshot_time <= y.snapshot_time );
}

int operator> (const timestamped_variable& x, const timestamped_variable& y)
{
    return (x.snapshot_time > y.snapshot_time );
}

int operator>=(const timestamped_variable& x, const timestamped_variable& y)
{
    return (x.snapshot_time >= y.snapshot_time );
}

ostream& operator<<(ostream& ostr, const timestamped_variable& x)
{
    struct tm * timeinfo = localtime( &x.snapshot_time );
    ostr << asctime( timeinfo )
    << x.get_snapshot_value();
    return ostr;
}
