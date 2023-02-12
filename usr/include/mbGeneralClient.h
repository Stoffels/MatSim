#ifndef MBGENERALCLIENT_H
#define MBGENERALCLIENT_H

#include <mutex>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <Eigen/Dense>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>
#include <modbus/modbus-tcp.h>

typedef enum {
    BACKEND_TYPE_RTU=0,
    BACKEND_TYPE_TCP
} backend_type;

using namespace std;
using namespace Eigen;



using namespace std;

class mbGeneralClient
{
    public:
        mbGeneralClient();
        virtual ~mbGeneralClient();
        mbGeneralClient(const mbGeneralClient& other);
        mbGeneralClient& operator=(const mbGeneralClient& other);

        //Getter methods
        string get_context( void);
        unsigned get_device_id( void );
        string get_tcp_or_commport_address(void);
        unsigned get_baudrate_or_socketport(void);
        char get_parity(void);
        unsigned get_data_bits(void);
        unsigned get_stop_bits(void);
        modbus_t * get_modbus_reference(void);
        backend_type get_backend_type( void );

        //Setter methods
        void set_context( string c );
        void set_tcp_or_commport_address(string c );
        void set_device_id( unsigned i = 1);
        void set_baudrate_or_socketport(unsigned i = 9600);
        void set_parity(char i = 'E');
        void set_data_bits(unsigned i = 8);
        void set_stop_bits(unsigned i = 1);
        void set_backend_type( backend_type i = BACKEND_TYPE_TCP );

        /* Connect/Disconnect to port and device */
        bool allocate_context( void );
        bool connect_device( void );
        bool disconnect_device( void );
        bool close_service(void);


        /* reading and writing data to/from device */
        VectorXd read_float_input_registers(int addr, int nb );
        VectorXd read_float_holding_registers( int addr, int nb );
        VectorXd read_holding_registers(int addr, int nb );
        bool write_float_holding_registers( int addr, VectorXd data );
        bool write_holding_registers( int addr, VectorXd data );

        /* General information */
        bool is_context_connected( void );
        bool is_device_connected( void);
        string error_message( void );

    protected:

    private:
        std::mutex lock_mtx;
        unsigned device_id;
        string context;
        string tcp_or_commport_address;
        string error_msg = "";
        unsigned baudrate_or_socketport;
        char parity;
        unsigned data_bits;
        unsigned stop_bits;
        modbus_t * ctx;
        backend_type backend;
        bool initialized_connection;
        bool initialized_context;
        uint16_t tab_reg[4095];
};

#endif // MBGENERALCLIENT_H
