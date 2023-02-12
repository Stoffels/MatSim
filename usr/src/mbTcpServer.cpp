#include "mbTcpServer.h"


mbTcpServer::mbTcpServer(void)
{
    address = "127.0.0.1";
    port= 1502;
    initialized = false;
    server_ok = false;
}

mbTcpServer::mbTcpServer(string tcp_address, int tcp_port)
{
    //ctor
    address = tcp_address;
    port = tcp_port;
    initialized = false;
    Initialize();
}

mbTcpServer::~mbTcpServer()
{
    //dtor
    if (! stop_thread)
        stop_thread = true;
    if(server_thread.joinable()) server_thread.join();

    if ( initialized )
    {
        if ( server_ok )
            modbus_mapping_free(mb_mapping);

        delete [] DISC_REG_BLK;
        delete [] COIL_REG_BLK;
        delete [] INPUT_REG_BLK;
        delete [] HOLD_REG_BLK;

        modbus_close(ctx);
        modbus_free(ctx);
    }
    initialized = false;
    //free (query);
}

bool mbTcpServer::Initialize( void )
{
    if ( not initialized )
    {
        ctx = modbus_new_tcp( address.c_str(), port );



        header_length = modbus_get_header_length(ctx);
        //modbus_set_debug(ctx,TRUE);

        //initilize read/write buffers
        DISC_REG_BLK = new uint8_t[DISC_REG_NUM];
        COIL_REG_BLK = new uint8_t[COIL_REG_NUM];
        INPUT_REG_BLK = new uint16_t[INPUT_REG_NUM];
        HOLD_REG_BLK = new uint16_t[HOLD_REG_NUM];

        init_buffer();

        // define modbus mapping sizes
        mb_mapping = modbus_mapping_new(
                         COIL_REG_NUM,
                         DISC_REG_NUM,
                         HOLD_REG_NUM,
                         INPUT_REG_NUM);
        //cout << "mb_mapping = " << mb_mapping << endl;
        server_ok = mb_mapping != NULL;
        initialized = true;
        return server_ok;
    }
    else

        return false;
}

bool mbTcpServer::set_buffer_sizes(int disc_reg_num,int coil_reg_num,int input_reg_num,int hold_reg_num )
{
    if ( not initialized )
    {
        COIL_REG_NUM = coil_reg_num;
        DISC_REG_NUM = disc_reg_num;
        HOLD_REG_NUM = hold_reg_num;
        INPUT_REG_NUM = input_reg_num;
        return true;
    }
    else
        return false;
}

bool mbTcpServer::get_buffer_sizes(int &disc_reg_num,int &coil_reg_num,int &input_reg_num,int &hold_reg_num )
{
    coil_reg_num = COIL_REG_NUM ;
    disc_reg_num = DISC_REG_NUM;
    hold_reg_num = HOLD_REG_NUM;
    input_reg_num = INPUT_REG_NUM;
    return true;
}


void mbTcpServer::init_buffer(void)
{
    for( int i = 0; i < INPUT_REG_NUM; i++ )
        INPUT_REG_BLK[i] = 0;

    for( int i = 0; i < HOLD_REG_NUM; i++ )
        HOLD_REG_BLK[i] = 0;

    for( int i = 0; i < DISC_REG_NUM; i++ )
        DISC_REG_BLK[i] = 0;

    for( int i = 0; i < COIL_REG_NUM; i++ )
        COIL_REG_BLK[i] = 0;

}

void mbTcpServer::start(void)
{

    if (server_ok and not server_thread.joinable() )
    {
        stop_thread = false;
        server_thread = std::thread(&mbTcpServer::ServerUpdateThread,this);
    }
}

void mbTcpServer::stop(void)
{
    stop_thread = true;
    server_thread.join();
}


void mbTcpServer::ServerUpdateThread()
{
    tcp_socket = modbus_tcp_listen(ctx,1);

    modbus_tcp_accept(ctx,&tcp_socket);

    while(!stop_thread and server_ok)
    {

        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        //cout << "ctx=" << ctx << endl;

        int rc = modbus_receive(ctx,query);
        //cout << "rc = " << rc << endl;
        if (rc > 0 )
        {
            //lock for server to get data without messing up the data
            unique_lock<mutex> lck(mtx);

            if(query[header_length] == 0x01)
            {
                //read coils
                int start = MODBUS_GET_INT16_FROM_INT8(query,header_length+1);
                int len = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);
                for ( int i = start; i < start+len; i++ )
                    mb_mapping->tab_bits[i] = COIL_REG_BLK[i];

            }

            if(query[header_length] == 0x02)
            {
                //read discrete inputs
                int start = MODBUS_GET_INT16_FROM_INT8(query,header_length+1);
                int len = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);
                for ( int i = start; i < start+len; i++ )
                    mb_mapping->tab_input_bits[i] = DISC_REG_BLK[i];
            }

            //we have received data
            if(query[header_length] == 0x03)
            {
                //Read holding registers
                int start = MODBUS_GET_INT16_FROM_INT8(query,header_length+1);
                int len = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);
                for( int i = start; i < start+len; i++ )
                    mb_mapping->tab_registers[i] = HOLD_REG_BLK[i];
            }

            if(query[header_length] == 0x04)
            {
                //Read input registers
                int start = MODBUS_GET_INT16_FROM_INT8(query,header_length+1);
                int len = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);
                for( int i = start; i < start+len; i++ )
                    mb_mapping->tab_input_registers[i] = INPUT_REG_BLK[i];
            }

            modbus_reply(ctx,query,rc, mb_mapping);


            if(query[header_length] == 0x05)
            {
                //write single coil
                COIL_REG_BLK[MODBUS_GET_INT16_FROM_INT8(query,header_length+1)] = (query[header_length+3] == 0xFF);
            }

            if(query[header_length] == 0x06)
                //write single register
                HOLD_REG_BLK[MODBUS_GET_INT16_FROM_INT8(query,header_length+1)] = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);

            if(query[header_length] == 0x0F)
            {
                //write multiple coils
                int start = MODBUS_GET_INT16_FROM_INT8(query,header_length+1);
                int len = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);
                for ( int i = start; i < start+len; i++ )
                    COIL_REG_BLK[i] = mb_mapping->tab_bits[i];
            }

            if(query[header_length] == 0x10)
            {
                //write multiple registers
                int start = MODBUS_GET_INT16_FROM_INT8(query,header_length+1);
                int len = MODBUS_GET_INT16_FROM_INT8(query,header_length+3);
                for ( int i = start; i < start+len; i++ )
                {
                    HOLD_REG_BLK[i] = mb_mapping->tab_registers[i];
                }
                //cout << endl;

            }

        }
        else if ( rc == -1 )
        {
            if ( not stop_thread )
            {
                modbus_close(ctx);
                modbus_tcp_accept(ctx,&tcp_socket);
            }
        }

    }
    //start();
    //cout << "exit thread" << endl;
}


bool mbTcpServer::write_input_registers( int start_address, VectorXd data )
{
    unique_lock<mutex> lck(mtx);
    for( unsigned i = 0; i < data.rows(); i++)
        INPUT_REG_BLK[start_address + i] = data(i);
    return true;
}




bool mbTcpServer::write_float_input_registers( int start_address, VectorXd data )
{
    unique_lock<mutex> lck(mtx);
    for( unsigned i = 0; i < data.rows(); i++)
    {

        uint16_t * pval = new uint16_t[2];
        modbus_set_float(data(i),pval);
        INPUT_REG_BLK[2*start_address + 2*i]=pval[0];
        INPUT_REG_BLK[2*start_address + 2*i + 1]=pval[1];
        delete pval;
    }
    return true;
}



bool mbTcpServer::write_input_bits( int start_address, vector<int> data)
{
    unique_lock<mutex> lck(mtx);
    for( unsigned i = 0; i < data.size(); i++)
        DISC_REG_BLK[start_address + i] = data[i];
    return true;
}

bool mbTcpServer::write_holding_registers( int start_address, VectorXd data )
{
    unique_lock<mutex> lck(mtx);
    // todo
    for ( unsigned i = 0; i < data.rows(); i++ )
        HOLD_REG_BLK[start_address + i] = data(i);
    return true;
}




bool mbTcpServer::write_float_holding_registers(int start_address, VectorXd data )
{
    unique_lock<mutex> lck(mtx);
    for( unsigned i = 0; i < data.rows(); i++)
    {

        uint16_t * pval = new uint16_t[2];
        modbus_set_float(data(i),pval);
        HOLD_REG_BLK[2*start_address+2*i]=pval[0];
        HOLD_REG_BLK[2*start_address+2*i + 1]=pval[1];
        //HOLD_REG_BLK[start_address+2*i]=pval[0];
        //HOLD_REG_BLK[start_address+2*i + 1]=pval[1];
        delete pval;

    }
    return true;

}



VectorXd mbTcpServer::read_input_registers( int start_address, int number )
{
    unique_lock<mutex> lck(mtx);
    VectorXd data(number);

    for( int i = 0; i < number; i++ )
    {
        data(i)= INPUT_REG_BLK[start_address + i];
        //data.push_back(val);
    }

    return data;

}



VectorXd mbTcpServer::read_holding_registers( int start_address, int number)
{
    unique_lock<mutex> lck(mtx);
    VectorXd data(number);;
    for( int i = 0; i < number; i++ )
    {
        data(i) = HOLD_REG_BLK[start_address + i];
    }

    return data;
}



VectorXd mbTcpServer::read_float_input_registers( int start_address, int number )
{
    unique_lock<mutex> lck(mtx);
    VectorXd data(number);
    for( int i = 0; i < number; i++ )
    {
        uint16_t * pval = new uint16_t[2];
        pval[0] = INPUT_REG_BLK[2*start_address + 2*i];
        pval[1] = INPUT_REG_BLK[2*start_address + 2*i+1];
        //pval[0] = INPUT_REG_BLK[start_address + 2*i];
        //pval[1] = INPUT_REG_BLK[start_address + 2*i+1];
        data(i) =  modbus_get_float(pval);
        delete pval;
    }
    return data;
}



VectorXd mbTcpServer::read_float_holding_registers(int start_address, int number )
{
    unique_lock<mutex> lck(mtx);
    VectorXd data(number);
    for( int i = 0; i < number; i++ )
    {
        uint16_t * pval = new uint16_t[2];
        pval[0] = HOLD_REG_BLK[2*start_address + 2*i];
        pval[1] = HOLD_REG_BLK[2*start_address + 2*i+1];
        //pval[0] = HOLD_REG_BLK[start_address + 2*i];
        //pval[1] = HOLD_REG_BLK[start_address + 2*i+1];
        data(i) = modbus_get_float(pval);
        delete pval;
    }
    return data;
}



bool mbTcpServer::write_holding_bits( int start_address, vector<int> data )
{
    unique_lock<mutex> lck(mtx);
    for( unsigned i = 0; i < data.size(); i++)
        COIL_REG_BLK[start_address+i] = data[i];
    return true;
}

vector<int> mbTcpServer::read_holding_bits( int start_address, int number )
{
    unique_lock<mutex> lck(mtx);
    vector<int> data;
    for( int i = 0; i < number; i++ )
    {
        int val = COIL_REG_BLK[start_address + i];
        data.push_back(val);
    }

    return data;
}
