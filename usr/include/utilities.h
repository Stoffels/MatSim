//History
// 14-08-2017
// Add procedures to write the historian to a file for later retrieval
//
// 17-05-2006
// Add the following procedure:
// list<time_t>ReadStartTimes( ifstream& fin );
// Read the start time (seconds since 1jan1970) for each tag
//
// 21-09-2004
// J.H.Hoffeldt
// Add the following procedure:
// list<string> ReadDescriptions( ifstream& fin );
// Read tag descriptions from file, this is if i need it

///////////////////////////////////////////////////////////////////////////////
// FORMAT OF FILE
///////////////////////////////////////////////////////////////////////////////
// void ReadProcFile( const char * FileName, historian_table * history )
// pre: Text file has been created
// post: Data from text file has been imported into history_table
// General:
//  Structure
//  <TagName_1>\t<TagName_2>\t...\t<TagName_n>  // Type string
//  <EU_2>\t<EU_2>\t...\t<EU_n> // Type string
//  <start_time_1>\t...\t<start_time_1> //type time_t
//  <Sample_interval_1>\t...\t<Sample_interval_n> // Type int
//  <value_01>\t<value_02>\t...\t<value_0n>    //Type double
//  <value_11>\t<value_12>\t...\t<value_1n>     //Type double
//  ..
//  <value_m1>\t<value_m2>\t...\t<value_mn>     //type double
//  Ek gaan dit modify om *.cfc files te lees eendag dalk..as ek so voel.


#include <iostream>
#include <fstream>
#include <list>
#include <string>
#include <cassert>
#include <time.h>
#include <cmath>
#include "history.h"


using namespace std;


bool ReadProcFile( const char * FileName, historian_table * history );
bool dump_historian( const string filename );
list<string> ReadProcNames( ifstream& fin );
list<string> ReadEngineeringUnits( ifstream& fin );
list<string> ReadDescriptions( ifstream& fin );
list<time_t> ReadStartTimes( ifstream& fin );
list<double> ReadLogIntervals( ifstream& fin );
list<double> ReadVariables( ifstream& fin );

void clean_stream( ifstream& fin );
double raise_to_power( double num, int p);

bool WriteProcFile( const char * FileName, historian_table * history);
bool WriteProcnames( ofstream &fout, list<string> Procnames);
bool WriteEngineeringUnits( ofstream &fout, list<string> EU);
bool WriteDescriptions( ofstream &fout, list<string> descr);
bool WriteLogIntervals( ofstream &fout, list<time_t> logTimes);
bool WriteVariables( ofstream &fout, MatrixXd& values );

