#ifndef MODBUSDEVICEHISTORIAN_H
#define MODBUSDEVICEHISTORIAN_H



#include <iostream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <cstring>
#include <list>
#include <algorithm>
#include <iterator>
#include <thread>
#include <chrono>
#include <mutex>
#include <modbus/modbus.h>
#include <Eigen/Dense>
#include "mbGeneralClient.h"
#include "history.h"
#include <wx/msgdlg.h>

class modbusDeviceHistorian
{
public:
    modbusDeviceHistorian();
    virtual ~modbusDeviceHistorian();
    modbusDeviceHistorian(const modbusDeviceHistorian& other);
    modbusDeviceHistorian& operator=(const modbusDeviceHistorian& other);


    bool register_historian( historian_table * h , int sr);
    bool register_modbus_client( mbGeneralClient * m , int modbus_base_address);

    bool initialize_historian_tagnames( void );
    bool is_historian_tagnames_initialized( void ){return historian_tagnames_initialized;}

    bool set_base_modbus_address( const int i );
    int get_base_modbus_address( void )
    {
        return base_modbus_address;
    }

    int get_sampling_rate( void ){
        return sampling_rate;
    };

    int get_pv_base_modbus_address( void )
    {
        return pv_base_modbus_address;
    };

    int get_mv_base_modbus_address( void )
    {
        return mv_base_modbus_address;
    };

    historian_table get_current_historian( void );

    MatrixXd get_current_pv_array( void );
    MatrixXd get_current_mv_array( void );

    void start( void );
    void stop(void);
    bool is_running( void );

protected:
    mbGeneralClient * mb;
    historian_table * historian;

private:
    bool stop_updating_thread;
    thread server_thread;   // serve the modbus server
    mutex mtx;

    int base_modbus_address;
    int pv_base_modbus_address;
    int mv_base_modbus_address;

    bool get_system_dimensions( int & pv_number, int & cv_number );
    bool historian_tagnames_initialized;

    void update_thread(void);
    int sampling_rate;

};

#endif // MODBUSDEVICEHISTORIAN_H
