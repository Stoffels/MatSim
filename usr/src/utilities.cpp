#include "utilities.h"



bool ReadProcFile( const char * FileName, historian_table * history )
{
    string file_name = FileName;

    ifstream fin( FileName, ios::in );
    if( !fin )
    {
        cout << FileName << " could not be opened" << endl;
        return false;
    }
    //declare variables
    list<string> TagNames, E_Units , Descr ;
    list<time_t> LogTimes;
    list<double> LogIntervals, /*LogTimes,*/ LogValues;


    list<string>::iterator name_iterator;
    list<string>::iterator EU_iterator;
    list<string>::iterator desc_iterator;
    list<time_t>::iterator LogTime_iterator;
    list<double>::iterator interval_iterator;
    list<double>::iterator log_iterator;

    //Read tagnames, engineering units and intervals
    TagNames = ReadProcNames( fin );
    if( not (TagNames.size() > 0) )
    {
        //wxMessageBox( wxT("No Tags could be read from ") + wxT(file_name),
        //    wxT("Error"), wxICON_ERROR | wxOK );
        //cerr << "tagnames and engineering units do not correlate" << endl;
        cout << "No Tags could be read from " << file_name << endl;
        history->clear();
        fin.close();
        return false;
    }

    E_Units = ReadEngineeringUnits( fin );
    if( TagNames.size() != E_Units.size())
    {
        //wxMessageBox( wxT("Engineering unit count less of more than number of tags\nError at line number 2 of ")
        //    + wxT(file_name) ,
        //    wxT("Error"), wxICON_ERROR | wxOK );
        //cerr << "tagnames and engineering units do not correlate" << endl;
        cout << "Engineering unit count less of more than number of tags\nError at line number 2 of " << file_name << endl;
        history->clear();
        fin.close();
        return false;
    }
    assert( TagNames.size() == E_Units.size());

    Descr = ReadDescriptions( fin );
    if( TagNames.size() != Descr.size())
    {
        //wxMessageBox( wxT("Description count less of more than number of tags\nError at line number 3 of ")
        //+ wxT(file_name),
        //    wxT("Error"), wxICON_ERROR | wxOK );
        //cerr << "tagnames and descriptions do not correlate" << endl;
        cout << "Description count less of more than number of tags\nError at line number 3 of " << file_name << endl;
        history->clear();
        fin.close();
        return false;
    }
    assert( TagNames.size() == Descr.size());

    LogTimes = ReadStartTimes(fin);
    if( TagNames.size() != LogTimes.size())
    {
        //wxMessageBox( wxT("Start Time count less of more than number of tags\nError at line number 4 of")
        //+ wxT(file_name),
        //    wxT("Error"), wxICON_ERROR | wxOK );
        //cerr << "tagnames and start_times do not correlate" << endl;
        cout << "Start Time count less of more than number of tags\nError at line number 4 of " << file_name << endl;
        history->clear();
        fin.close();
        return false;
    }
    assert( TagNames.size() == LogTimes.size());

    LogIntervals = ReadLogIntervals( fin );
    if( TagNames.size() != LogTimes.size())
    {
        //wxMessageBox( wxT("Intervals count less of more than number of tags\nError at line number 5 of")
        //+ wxT(file_name),
        //    wxT("Error"), wxICON_ERROR | wxOK );
        cout << "Intervals count less of more than number of tags\nError at line number 5 of " << file_name << endl;
        history->clear();
        fin.close();
        //cerr << "tagnames and log intervals do not correlate" << endl;
        return false;
    }
    assert( TagNames.size() == LogIntervals.size());

    //Insert general info into history system;

    for( name_iterator = TagNames.begin(), EU_iterator = E_Units.begin(), desc_iterator = Descr.begin();
            name_iterator != TagNames.end();
            ++name_iterator,++EU_iterator, ++desc_iterator )
        history->add( *name_iterator, *EU_iterator, *desc_iterator );

    int line_number = 5;    //This is only to determine at what line number an error has occured.
    //Log variables into the history structure
    while( !fin.eof() )
    {
        //wxYield();
        list<double>variables = ReadVariables(fin);
        list<double>::iterator var_iterator = variables.begin();
        line_number++;

        if( TagNames.size() != variables.size())
        {
            //wxString line_number_str;
            //line_number_str << line_number;
            //wxMessageBox( wxT("An Error occurred a line number ") + wxT(line_number_str) + wxT(" of\n")
            //    + wxT(file_name),
            //    wxT("Error"), wxICON_ERROR | wxOK );
            //cout << "Error occured at line number " << line_number << endl;
            cout << "An error occured at line number " << line_number << endl;
            fin.close();

            return false;
        }
        assert( TagNames.size() == variables.size() );


        for( name_iterator = TagNames.begin(),
                var_iterator = variables.begin(),
                interval_iterator = LogIntervals.begin(),
                LogTime_iterator = LogTimes.begin();
                name_iterator != TagNames.end();
                ++name_iterator,
                ++var_iterator,
                ++interval_iterator ,
                ++LogTime_iterator )
        {
            //cout << *LogTime_iterator << endl;
            history->log( *name_iterator, *var_iterator ,*LogTime_iterator );
            *LogTime_iterator += (time_t)*interval_iterator;
        }
        //wxYield();
    }
    fin.close();
    return true;
}


void Tokenize(const string& str,
              list<string>& tokens,
              const string& delimiters)
{
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
}


list<string> ReadProcNames( ifstream& fin )
{
    list<string> names;
    string str;
    getline( fin, str);
    Tokenize( str, names, "\t");
    clean_stream(fin);
    return names;
}

list<string> ReadEngineeringUnits( ifstream& fin )
{
    list<string> EngineeringUnits;
    string str;
    getline( fin, str);
    Tokenize( str, EngineeringUnits, "\t");
    clean_stream(fin);
    return EngineeringUnits;
}

list<string> ReadDescriptions( ifstream& fin )
{
    list<string> Descriptions;
    string str;
    getline( fin, str);
    Tokenize( str, Descriptions, "\t");
    clean_stream(fin);
    return Descriptions;
}

list<time_t> ReadStartTimes( ifstream& fin )
{
    list<string> start_times_str;
    list<time_t> start_times;
    string str;
    getline( fin, str);
    Tokenize( str, start_times_str, "\t");
    list<string>::iterator i;
    for( i = start_times_str.begin(); i != start_times_str.end(); i++ )
    {
        time_t t = (time_t)atol( (*i).c_str() );
        start_times.push_back( t );
    }
    clean_stream(fin);

    return start_times;
}

list<double>  ReadLogIntervals( ifstream& fin )
{
    list<string> log_intervals_str;
    list<double> log_intervals;
    string str;
    getline( fin, str);
    Tokenize( str, log_intervals_str, "\t");
    list<string>::iterator i;
    for( i = log_intervals_str.begin(); i != log_intervals_str.end(); i++ )
    {
        double dt = atof( (*i).c_str() );
        log_intervals.push_back( dt );
    }
    clean_stream(fin);

    return log_intervals;
}

list<double> ReadVariables( ifstream& fin )
{
    list<string> variables_str;
    list<double> variables;
    string str;
    getline( fin, str);
    Tokenize( str, variables_str, "\t");
    list<string>::iterator i;
    for( i = variables_str.begin(); i != variables_str.end(); i++ )
    {
        double dt = atof( (*i).c_str() );
        variables.push_back( dt );
    }
    clean_stream(fin);

    return variables;

}

void clean_stream( ifstream& fin )
{
    char ch = fin.peek();
    if (( ch == '\n')||(ch == '\r') || ( ch == ' '))
    {
        fin.get();
        clean_stream(fin);
    }
}

double raise_to_power( double num, int p)
{
    double temp = 1.0;
    for ( int i = 0; i < p; i++ )
        temp = temp * num;
    return temp;
}

bool dump_historian( const string filename )
{
    return true;
}


bool WriteProcFile( const char * FileName, historian_table * history){
    string file_name = FileName;

    ofstream fout( FileName, ofstream::out );

    list<string> TagNames, E_Units , Descr ;
    list<time_t> LogIntervals;
    list<double> LogValues;



    TagNames = history->return_tagnames();
    list<string>::iterator i = TagNames.begin();
    for( i = TagNames.begin();  i != TagNames.end(); i++){
        E_Units.push_back( history->get_engineering_units(*i));
        Descr.push_back( history->get_description(*i));
        time_t interval;
        history->get_log_time_interval(*i,interval);
        LogIntervals.push_back(interval);
    }

    WriteProcnames(fout, TagNames);
    WriteEngineeringUnits(fout,E_Units);
    WriteDescriptions(fout,Descr);
    WriteLogIntervals(fout,LogIntervals);
    MatrixXd values;
    history->return_normalized_history_matrix(TagNames,values);
    WriteVariables(fout,values);
    fout.close();

    return true;
}


bool WriteProcnames( ofstream &fout, list<string> Procnames){
    list<string>::iterator i;
    for( i = Procnames.begin(); i != Procnames.end();i++){
        fout << *i;
        if ( *i != Procnames.back())
            fout << '\t';
    }
    fout << '\n';
    return true;
}

bool WriteEngineeringUnits( ofstream &fout, list<string> eu){
    list<string>::iterator i;
    for( i = eu.begin(); i != eu.end();i++){
        fout << *i;
        if ( *i != eu.back())
            fout << '\t';
    }
    fout << '\n';
    return true;
}

bool WriteDescriptions( ofstream &fout, list<string> descr){
    list<string>::iterator i;
    for( i = descr.begin(); i != descr.end();i++){
        fout << *i;
        if ( *i != descr.back())
            fout << '\t';
    }
    fout << '\n';
    return true;
}

bool WriteLogIntervals( ofstream &fout, list<time_t> logTimes){
    list<time_t>::iterator i;
    for( i = logTimes.begin(); i != logTimes.end();i++){
        fout << *i;
        if ( *i != logTimes.back())
            fout << '\t';
    }
    fout << '\n';
    return true;
}

bool WriteVariables( ofstream &fout, MatrixXd& values ){
    fout << values;
    return true;
}


