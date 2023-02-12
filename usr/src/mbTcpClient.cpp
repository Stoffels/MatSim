#include "mbTcpClient.h"

mbTcpClient::mbTcpClient(string tcp_address, int tcp_port)
{
    mb = modbus_new_tcp(tcp_address.c_str(),tcp_port);
    int rc = modbus_connect(mb);
    if( rc == -1 )
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(mb);
        exit(-1);
    }
    //set the response timeout to 1.0 second
    struct timeval mb_response_timeout;

    mb_response_timeout.tv_sec = 2;
    mb_response_timeout.tv_usec = 500000;
    modbus_set_response_timeout(mb,mb_response_timeout.tv_sec,mb_response_timeout.tv_usec);
    //modbus_set_response_timeout(mb, &mb_response_timeout);
    mb_response_timeout.tv_sec = 1;
    modbus_set_byte_timeout(mb,mb_response_timeout.tv_sec,mb_response_timeout.tv_usec);
    //modbus_set_byte_timeout(mb, &mb_response_timeout);
}


mbTcpClient::~mbTcpClient()
{
    modbus_close(mb);
    modbus_free(mb);
}



VectorXd mbTcpClient::read_float_input_registers(int addr, int nb )
{
    std::lock_guard<std::mutex> lk(lock_mtx);
    VectorXd data(nb);

    int rc = modbus_read_input_registers(mb, addr*2, nb*2, tab_reg);
    if( rc == -1 )
    {
        fprintf(stderr, "Input register read  failed: %s\n", modbus_strerror(errno));
    }
    else
        for( int i = 0; i < nb; i++ )
        {
            uint16_t * pval = new uint16_t[2];
            pval[0] = tab_reg[2*i];
            pval[1] = tab_reg[2*i+1];
            data(i) =  modbus_get_float(pval);
            delete pval;
        }
    return data;
}


VectorXd mbTcpClient::read_holding_registers(int addr, int nb )
{
    std::lock_guard<std::mutex> lk(lock_mtx);
    VectorXd data(nb);
    int rc = modbus_read_registers(mb,addr,nb,tab_reg);

    if( rc == -1 )
    {
        fprintf(stderr, "Holding register read  failed: %s\n", modbus_strerror(errno));
    }
    else
        for( int i = 0; i < nb; i++ )
        {
            data(i) = tab_reg[i];
        }

    return data;
}


VectorXd mbTcpClient::read_float_holding_registers( int addr, int nb )
{
    std::lock_guard<std::mutex> lk(lock_mtx);
    VectorXd data(nb);

    int rc = modbus_read_registers(mb, addr*2, nb*2, tab_reg);
    if( rc == -1 )
    {
        fprintf(stderr, "Holding register read  failed: %s\n", modbus_strerror(errno));
    }
    else
        for( int i = 0; i < nb; i++ )
        {
            uint16_t * pval = new uint16_t[2];
            pval[0] = tab_reg[2*i];
            pval[1] = tab_reg[2*i+1];
            data(i) =  modbus_get_float(pval);
            delete pval;
        }
    return data;
}

bool mbTcpClient::write_float_holding_registers( int addr, VectorXd data )
{
    std::lock_guard<std::mutex> lk(lock_mtx);

    for( unsigned i = 0; i < data.rows(); i++)
    {
        uint16_t * pval = new uint16_t[2];
        modbus_set_float(data(i),pval);
        tab_reg[2*i]=pval[0];
        tab_reg[2*i + 1]=pval[1];
        delete pval;
    }

    int rc = modbus_write_registers(mb, addr*2, data.rows()*2, tab_reg);
    if( rc == -1 ){
        fprintf(stderr, "Float holding register write failed: %s\n", modbus_strerror(errno));
    }
    return rc != -1;
}

bool mbTcpClient::write_holding_registers( int addr, VectorXd data )
{
    std::lock_guard<std::mutex> lk(lock_mtx);
    for( unsigned i = 0; i < data.rows(); i++)
    {
        uint16_t * pval = new uint16_t[2];
        modbus_set_float(data(i),pval);
        tab_reg[i]=data(i);
        delete pval;
    }

    int rc = modbus_write_registers(mb,addr,data.rows(),tab_reg);
    if( rc == -1 ){
        fprintf(stderr, "Holding register write failed: %s\n", modbus_strerror(errno));
    }
    return rc != -1;
}


