#include "modbusDeviceHistorian.h"

modbusDeviceHistorian::modbusDeviceHistorian()
{
    //ctor
    mb = NULL;
    historian = NULL;
    stop_updating_thread = false;
    historian_tagnames_initialized = false;
    base_modbus_address = 0;
    sampling_rate = 1;
}

modbusDeviceHistorian::~modbusDeviceHistorian()
{
    //dtor
    stop();
}

modbusDeviceHistorian::modbusDeviceHistorian(const modbusDeviceHistorian& other)
{
    //copy ctor
    mb = other.mb;
    historian = other.historian;
    stop_updating_thread = true;
    base_modbus_address = other.base_modbus_address;
    pv_base_modbus_address = other.pv_base_modbus_address;
    mv_base_modbus_address = other.mv_base_modbus_address;
    sampling_rate = other.sampling_rate;
    historian_tagnames_initialized = other.historian_tagnames_initialized;
}

modbusDeviceHistorian& modbusDeviceHistorian::operator=(const modbusDeviceHistorian& rhs)
{
    if (this == &rhs) return *this; // handle self assignment
    //assignment operator
    mb = rhs.mb;
    historian = rhs.historian;
    stop_updating_thread = true;
    base_modbus_address = rhs.base_modbus_address;
    pv_base_modbus_address = rhs.pv_base_modbus_address;
    mv_base_modbus_address = rhs.mv_base_modbus_address;
    sampling_rate = rhs.sampling_rate;
    historian_tagnames_initialized = rhs.historian_tagnames_initialized;

    return *this;
}

bool modbusDeviceHistorian::register_historian( historian_table * h, int sr )
{
    //wxMessageBox(wxString::Format(wxT("%i"), sr),wxT("sampling rate"));
    if ( h != NULL )
    {
        historian = h;
        sampling_rate = sr;
        //wxMessageBox(wxString::Format(wxT("%i"), sampling_rate),wxT("sampling rate"));
        return true;
    }
    else
        return false;
}

bool modbusDeviceHistorian::register_modbus_client( mbGeneralClient* m, int modbus_base_address )
{
    if ( m != NULL )
    {
        mb = m;
        set_base_modbus_address(modbus_base_address);
        return true;
    }
    else
        return false;
}

/**
 * Vir dit word aanvaar dat die dimensies aangedui word in 0..3 -> 40001..40004, van die modbus server. Die eerste vier
 * registers is die process and control variable dimensies respectief.
 */
bool modbusDeviceHistorian::get_system_dimensions( int & pv_number, int & mv_number)
{
    pv_number = 0;
    mv_number = 0;

    if ( mb )
    {
        VectorXd info = mb->read_float_holding_registers(base_modbus_address,2);
        pv_number = info(0);
        mv_number = info(1);
        return true;
    }
    else
        return false;
}



bool modbusDeviceHistorian::initialize_historian_tagnames( void )
{

    int pv_number = 0, mv_number = 0;
    get_system_dimensions( pv_number, mv_number);

    // auto generate all the tagnames
    for ( int i = 0; i < pv_number; i++ )
    {
        //add all the process variables
        string tag_name = "pv_" + static_cast<ostringstream*>( &(ostringstream() << i) )->str();
        historian->add(tag_name,"", "");
    }

    for ( int i = 0; i < mv_number; i++ )
    {
        //add all the manipulated variables to the historian
        string tag_name = "mv_" + static_cast<ostringstream*>( &(ostringstream() << i) )->str();
        historian->add(tag_name,"", "");
    }
    historian_tagnames_initialized = true;

    return true;
}

bool modbusDeviceHistorian::set_base_modbus_address( const int i )
{
    base_modbus_address = i;
    int pv_number=0 , mv_number = 0;
    get_system_dimensions( pv_number, mv_number);

    pv_base_modbus_address = base_modbus_address + 2;
    mv_base_modbus_address = pv_base_modbus_address + pv_number;
    return true;
}



void modbusDeviceHistorian::update_thread(void)
{
    int pv_number = 0, mv_number = 0;
    get_system_dimensions( pv_number, mv_number);

    list<string> tagnames = historian->return_tagnames();
    list<string>::iterator j = tagnames.begin();
    //cout << "tagnames :" << endl;
    //for ( j = tagnames.begin(); j != tagnames.end(); ++j)
    //        cout << '\t' << *j << endl;

    while ( not stop_updating_thread )
    {
        // Read data from the modbus server
        {

            std::lock_guard<std::mutex> lk(mtx);


            VectorXd vars = mb->read_float_holding_registers( pv_base_modbus_address, pv_number + mv_number);
            j = tagnames.begin();

            VectorXd process_vars(pv_number);
            for( int i = 0; i < pv_number; i++ )
                process_vars(i) = vars(i);

            VectorXd control_vars(mv_number);
            for( int i = 0; i < mv_number; i++)
                control_vars(i) = vars(i+pv_number);


            j = tagnames.begin();

            for ( int i = 0; i < mv_number; i++,++j )
                historian->log( *j, control_vars(i));
            for ( int i = 0; i < pv_number; i++,++j)
                historian->log( *j, process_vars(i));

        }
        this_thread::sleep_for(chrono::seconds(sampling_rate));

    }

}

void modbusDeviceHistorian::start( void )
{
    stop_updating_thread = false;

    if (not server_thread.joinable() )
        server_thread = std::thread(&modbusDeviceHistorian::update_thread,this);
}

void modbusDeviceHistorian::stop(void)
{
    stop_updating_thread = true;
    if ( server_thread.joinable())
        server_thread.join();
}

bool modbusDeviceHistorian::is_running(void){
    return server_thread.joinable();
}

historian_table modbusDeviceHistorian::get_current_historian( void )
{
    std::lock_guard<std::mutex> lk(mtx);
    return *historian;
}

/**
*   Die doel van die funksie is om eerder die historian te gebruik om die laaste
*   data te kry, eerder as om die modbus server te veel te gebruik.
*/
MatrixXd modbusDeviceHistorian::get_current_pv_array( void ){
    std::lock_guard<std::mutex> lk(mtx);

    int pv_number = 0, mv_number = 0;
    double value;
    get_system_dimensions( pv_number, mv_number);

    MatrixXd PVs = MatrixXd::Zero(pv_number,1);

    // auto generate all the tagnames
    for ( int i = 0; i < pv_number; i++ )
    {
        //add all the process variables
        string tag_name = "pv_" + static_cast<ostringstream*>( &(ostringstream() << i) )->str();
        historian->get_last_logged_value(tag_name,value);
        PVs(i,0) = value;
    }

    return PVs;
}

MatrixXd modbusDeviceHistorian::get_current_mv_array( void ){
    std::lock_guard<std::mutex> lk(mtx);

    int pv_number = 0, mv_number = 0;
    double value;
    get_system_dimensions( pv_number, mv_number);

    MatrixXd MVs = MatrixXd::Zero(mv_number,1);

    // auto generate all the tagnames
    for ( int i = 0; i < mv_number; i++ )
    {
        //add all the process variables
        string tag_name = "mv_" + static_cast<ostringstream*>( &(ostringstream() << i) )->str();
        historian->get_last_logged_value(tag_name,value);
        MVs(i,0) = value;
    }

    return MVs;
}


