#ifndef MBTCPCLIENT_H
#define MBTCPCLIENT_H

#include <iostream>
#include <cstring>
#include <modbus/modbus.h>
#include <sys/socket.h>
#include <Eigen/Dense>
#include <mutex>


using namespace std;
using namespace Eigen;



class mbTcpClient
{
    public:
        mbTcpClient(string tcp_address = "127.0.0.1", int tcp_port= 1502);
        virtual ~mbTcpClient();

        VectorXd read_float_input_registers(int addr, int nb );
        VectorXd read_float_holding_registers( int addr, int nb );

        VectorXd read_holding_registers(int addr, int nb );

        bool write_float_holding_registers( int addr, VectorXd data );
        bool write_holding_registers( int addr, VectorXd data );


    protected:

    private:
        std::mutex lock_mtx;
        modbus_t *mb;
        uint16_t tab_reg[4095];
};

#endif // MBTCPCLIENT_H
