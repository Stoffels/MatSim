#ifndef MODBUSSERVER_H
#define MODBUSSERVER_H

#include <modbus/modbus.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <semaphore.h>

#include <modbus/modbus.h>
#include <iostream>
#include <string>
#include <thread>
#include <stdlib.h>
#include <sys/socket.h>
#include <vector>


using namespace std;




class ModbusServer{
public:
    ModbusServer(const string ip, const int port, const int = 1 );
    virtual ~ModbusServer();

    bool declare_registers( const int bits_num, const int coil_num, const int input_regs_num, const int holding_reg_num );

    bool write_input_registers( vector<uint16_t> v ); /* write input registers , ie 3xxxxx registers */
    vector<uint16_t> read_input_registers( void );
    bool read_holding_registers( vector<uint16_t> &v); /* read holding registers, ie 4xxxxx registers */
    bool read_digital_inputs( vector<uint16_t> &v); /* read digital inputs, 1xxxxx bits*/
    bool write_holding_bits( vector<uint16_t> v ); /*write to coils 0xxxxxx */

protected:
    void InternalThreadEntry();

private:
    std::thread the_thread;
    bool stop_thread = false; // Super simple thread stopping.


    int socket;
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;

    bool updating_registers = false;

    unsigned bit_count, coil_count, input_reg_count, holding_reg_count;

    //hold register and discrete values
    vector<uint16_t> holding_registers; /*holding registers, 4xxxxx*/
    vector<uint16_t> input_registers; /*raw analog inputs, 3xxxxx*/
    vector<uint16_t>   holding_bits; /* coils 0xxxxxx*/
    vector<uint16_t>   input_bits;   /* raw digital inputs 1xxxxxx */
};



#endif // MODBUSSERVER_H
