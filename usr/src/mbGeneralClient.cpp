#include "mbGeneralClient.h"

mbGeneralClient::mbGeneralClient()
{
    //ctor
    ctx = NULL;
    initialized_context = false;
    initialized_connection = false;
}

mbGeneralClient::~mbGeneralClient()
{
    //dtor
    if( ctx != NULL )
    {
        if ( is_device_connected())
        {
            disconnect_device();
        }

        if ( is_context_connected())
        {
            close_service();
        }
    }
}

mbGeneralClient::mbGeneralClient(const mbGeneralClient& other)
{
    //copy ctor
    context = other.context;
    tcp_or_commport_address = other.tcp_or_commport_address;
    baudrate_or_socketport = other.baudrate_or_socketport;
    parity = other.parity;
    data_bits = other.data_bits;
    stop_bits = other.stop_bits;
    ctx = other.ctx;
    backend = other.backend;
    initialized_connection = other.initialized_connection;
    initialized_context = other.initialized_context;
}

mbGeneralClient& mbGeneralClient::operator=(const mbGeneralClient& rhs)
{
    if (this == &rhs) return *this; // handle self assignment
    //assignment operator
    context = rhs.context;
    tcp_or_commport_address = rhs.tcp_or_commport_address;
    baudrate_or_socketport = rhs.baudrate_or_socketport;
    parity = rhs.parity;
    data_bits = rhs.data_bits;
    stop_bits = rhs.stop_bits;
    ctx = rhs.ctx;
    backend = rhs.backend;
    initialized_connection = rhs.initialized_connection;
    initialized_context = rhs.initialized_context;
    return *this;
}


string mbGeneralClient::get_tcp_or_commport_address(void)
{
    return tcp_or_commport_address;
}

unsigned mbGeneralClient::get_device_id( void )
{
    return device_id;
}

unsigned mbGeneralClient::get_baudrate_or_socketport(void)
{
    return baudrate_or_socketport;
}


char mbGeneralClient::get_parity(void)
{
    return parity;
}


unsigned mbGeneralClient::get_data_bits(void)
{
    return data_bits;
}


unsigned mbGeneralClient::get_stop_bits(void)
{
    return stop_bits;
}


modbus_t * mbGeneralClient::get_modbus_reference(void)
{
    return ctx;
}

backend_type mbGeneralClient::get_backend_type( void )
{
    return backend;
}


//Setter methods
void mbGeneralClient::set_context( string c )
{
    context = c;
}


void mbGeneralClient::set_tcp_or_commport_address(string c )
{
    tcp_or_commport_address = c;
}

void mbGeneralClient::set_device_id( unsigned i)
{
    device_id = i;
}

void mbGeneralClient::set_baudrate_or_socketport(unsigned i)
{
    baudrate_or_socketport = i;
}


void mbGeneralClient::set_parity(char i)
{
    parity = i;
}


void mbGeneralClient::set_data_bits(unsigned i)
{
    data_bits = i;
}


void mbGeneralClient::set_stop_bits(unsigned i)
{
    stop_bits = i;
}



void mbGeneralClient::set_backend_type( backend_type i  )
{
    backend = i;
}

bool mbGeneralClient::allocate_context( void )
{
    if (not is_context_connected())
    {
        switch (backend)
        {
        case BACKEND_TYPE_RTU:
        {
            ctx = modbus_new_rtu(tcp_or_commport_address.c_str(), baudrate_or_socketport, parity, data_bits, stop_bits);
            if (ctx == NULL)
            {
                error_msg = "Unable to allocate libmodbus context";
                initialized_connection = false;
                initialized_context = false;
            }
            else
                initialized_context = true;
        }
        break;

        case BACKEND_TYPE_TCP:
        {
            //wxMessageBox("TCP backend",tcp_or_commport_address.c_str());
            ctx = modbus_new_tcp(tcp_or_commport_address.c_str(), baudrate_or_socketport);
            if (ctx == NULL)
            {
                //fprintf(stderr, "Unable to allocate libmodbus context\n");
                error_msg = "Unable to allocate libmodbus context";
                initialized_connection = false;
                initialized_context = false;
            }
            else
                initialized_context = true;

        }
        break;
        }
    }
    return initialized_context;
}


bool mbGeneralClient::connect_device( void )
{
    if (is_context_connected())
    {
        switch (backend)
        {
        case BACKEND_TYPE_RTU:
        {
            modbus_set_slave(ctx, device_id);

            if ( modbus_connect(ctx) == -1)
            {
                error_msg = modbus_strerror(errno);
                modbus_free(ctx);
                ctx = NULL;
                initialized_connection = false;
                initialized_context = false;

            }
            else
            {
                initialized_connection = true;
            }
        }
        break;

        case BACKEND_TYPE_TCP:
        {

            if (modbus_connect(ctx) == -1)
            {

                error_msg = modbus_strerror(errno);
                modbus_free(ctx);
                ctx = NULL;
                initialized_connection = false;
                initialized_context = false;
            }
            else
                initialized_connection = true;
        }
        break;
        }
    }
    return initialized_connection;
}

bool mbGeneralClient::disconnect_device( void )
{

    if ( is_device_connected())
    {
        std::lock_guard<std::mutex> lk(lock_mtx); //Wait for communications to finish*/
        modbus_close(ctx);
        initialized_connection = false;

        return true;
    }
    return false;
}


bool mbGeneralClient::close_service(void)
{
    if ( is_context_connected())
    {
        disconnect_device();
        initialized_context = false;
        std::lock_guard<std::mutex> lk(lock_mtx); //may uncomment...maybe not*/
        modbus_free(ctx);
        ctx = NULL;
        return true;
    }
    return false;

}




VectorXd mbGeneralClient::read_float_input_registers(int addr, int nb )
{

    VectorXd data(nb);
    if(is_device_connected())
    {
        std::lock_guard<std::mutex> lk(lock_mtx);

        int rc = modbus_read_input_registers(ctx, addr*2, nb*2, tab_reg);
        if( rc == -1 )
        {
            error_msg = modbus_strerror(errno);
            fprintf(stderr, "Input register read  failed: %s\n", error_msg.c_str());

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
    }
    return data;
}


VectorXd mbGeneralClient::read_holding_registers(int addr, int nb )
{

    VectorXd data(nb);
    if(is_device_connected())
    {
        std::lock_guard<std::mutex> lk(lock_mtx);

        int rc = modbus_read_registers(ctx,addr,nb,tab_reg);

        if( rc == -1 )
        {
            error_msg = modbus_strerror(errno);
            fprintf(stderr, "Holding register read  failed: %s\n", error_msg.c_str());
        }
        else
            for( int i = 0; i < nb; i++ )
            {
                data(i) = tab_reg[i];
            }
    }
    return data;
}


VectorXd mbGeneralClient::read_float_holding_registers( int addr, int nb )
{
    VectorXd data(nb);
    if(is_device_connected())
    {
        std::lock_guard<std::mutex> lk(lock_mtx);

        int rc = modbus_read_registers(ctx, addr*2, nb*2, tab_reg);
        if( rc == -1 )
        {
            error_msg = modbus_strerror(errno);
            fprintf(stderr, "Holding register read  failed: %s\n", error_msg.c_str());
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
    }
    return data;
}

bool mbGeneralClient::write_float_holding_registers( int addr, VectorXd data )
{
    if(is_device_connected())
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

        int rc = modbus_write_registers(ctx, addr*2, data.rows()*2, tab_reg);
        if( rc == -1 )
        {
            error_msg = modbus_strerror(errno);
            fprintf(stderr, "Float holding register write failed: %s\n", error_msg.c_str());
        }
        return rc != -1;
    }
    else
        return false;
}

bool mbGeneralClient::write_holding_registers( int addr, VectorXd data )
{
    if(is_device_connected())
    {
        std::lock_guard<std::mutex> lk(lock_mtx);
        for( unsigned i = 0; i < data.rows(); i++)
        {
            uint16_t * pval = new uint16_t[2];
            modbus_set_float(data(i),pval);
            tab_reg[i]=data(i);
        }

        int rc = modbus_write_registers(ctx,addr,data.rows(),tab_reg);
        if( rc == -1 )
        {
            error_msg = modbus_strerror(errno);
            fprintf(stderr, "Holding register write failed: %s\n", error_msg.c_str());
        }
        return rc != -1;
    }
    else
        return false;
}




bool mbGeneralClient::is_context_connected( void )
{
    return initialized_context;
}

bool mbGeneralClient::is_device_connected( void)
{
    return initialized_connection;
}


string mbGeneralClient::get_context( void)
{
    return context;
}


string mbGeneralClient::error_message( void ){
    return error_msg;
}
