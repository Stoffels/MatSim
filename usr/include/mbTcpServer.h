#ifndef MBTCPSERVER_H
#define MBTCPSERVER_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <errno.h>
#include <vector>
#include <Eigen/Dense>


#include <modbus/modbus.h>
#include <sys/socket.h>

//defined size of modbus buffers used


using namespace std;
using namespace Eigen;



class mbTcpServer
{
public:
    mbTcpServer(void);
    mbTcpServer(string tcp_address, int tcp_port = 1502);
    virtual ~mbTcpServer();

    bool Initialize( void );

    // setter and getter procedures
    bool set_buffer_sizes(int,int,int,int );
    bool get_buffer_sizes(int&,int&,int&,int& );
    string get_ip_address( void ){ return address;}
    void set_ip_address( string ip_address ){address = ip_address;}
    int get_port_address( void ){ return port;}
    void set_port_address( int ip_port ){ port = ip_port;}


    bool write_input_registers( int start_address, VectorXd data );
    VectorXd read_input_registers( int start_address, int number );
    bool write_float_input_registers( int start_address, VectorXd data );
    VectorXd read_float_input_registers( int start_address, int number );
    bool write_holding_registers( int start_address, VectorXd data );
    VectorXd read_holding_registers( int start_address, int number);
    bool write_float_holding_registers(int start_address, VectorXd data );
    VectorXd read_float_holding_registers(int start_address, int number );
    bool write_input_bits( int start_address, vector<int> data);
    bool write_holding_bits( int start_address, vector<int> data );
    vector<int> read_holding_bits( int start_address, int number );


    bool is_initialized( void ) { return initialized; }
    bool is_server_ok( void ) {return server_ok;}
    bool is_running( void ){ return server_thread.joinable();}
    void start(void);
    void stop(void);

protected:

private:
    modbus_t *ctx = NULL;
    modbus_mapping_t *mb_mapping = NULL;
    //uint8_t *query;
    int tcp_socket = -1; //listening
    string address;
    int header_length;
    int port;
    bool server_ok = true;  // if connection is healthy, then this flag is true;
    bool initialized = false; //make sure we don't initialize more than one time

    thread server_thread;   // serve the modbus server
    mutex mtx;              // block out critical sections
    bool stop_thread = false;
    void ServerUpdateThread();

    //data buffer stuff
    uint8_t * DISC_REG_BLK;
    uint8_t  * COIL_REG_BLK;
    uint16_t * INPUT_REG_BLK;
    uint16_t *HOLD_REG_BLK;

    int INPUT_REG_NUM = 512;
    int HOLD_REG_NUM  = 512;
    int DISC_REG_NUM  = 128;
    int COIL_REG_NUM  = 128;
    //uint8_t DISC_REG_BLK[DISC_REG_NUM];
    //uint8_t COIL_REG_BLK[COIL_REG_NUM];
    //uint16_t INPUT_REG_BLK[INPUT_REG_NUM];
    //uint16_t HOLD_REG_BLK[HOLD_REG_NUM];

    void init_buffer(void);
};



#endif // MBTCPSERVER_H

