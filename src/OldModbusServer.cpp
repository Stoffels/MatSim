#include "ModbusServer.h"

ModbusServer::ModbusServer(const string ip, const int port, const int connections)
{
    //ctor
    //cout << "---create the ctx connection" << endl;
    ctx=modbus_new_tcp(ip.c_str() , port);
    //cout << "---ctx = " << ctx << endl;
    if( ctx > NULL)
    {
        socket = modbus_tcp_listen(ctx, connections);
        //cout << "----socket = " << socket << endl;

        stop_thread = false;
    }
    else
        stop_thread = true;
    //cout << "---ctx done" << endl;
    updating_registers = false;

    mb_mapping = NULL;
    bit_count = 0;
    coil_count = 0;
    input_reg_count = 0;
    holding_reg_count = 0;
}

ModbusServer::~ModbusServer()
{
    //dtor
    if( mb_mapping  != NULL )
        modbus_mapping_free(mb_mapping);

    if( ctx > NULL )
    {

        modbus_close(ctx);
        modbus_free(ctx);
    }

    if ( updating_registers ){
        stop_thread = true;
        the_thread.join();
    }
}



bool ModbusServer::declare_registers( const int bit_num, const int coil_num, const int input_regs_num, const int holding_reg_num )
{
    mb_mapping = modbus_mapping_new(bit_num, coil_num,holding_reg_num,input_regs_num);
    if ( mb_mapping != NULL)
    {
        bit_count = bit_num;
        coil_count = coil_num;
        input_reg_count = input_regs_num;
        holding_reg_count = holding_reg_num;
        return true;
    }
    return false;
}


//pre: registers has to be declared at this point.
void ModbusServer::InternalThreadEntry()
{
    modbus_tcp_accept(ctx,&socket);

    //cout << "modbus_tcp_accept " << endl;

    //while (enable_updating_registers)
    while ( not stop_thread )
    {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc = modbus_receive(ctx,query);
        int header_length = modbus_get_header_length(ctx);
        if( rc >= 0)
        {
            //if (query[header_length]=0x03)

            cout <<  "function = " << (float)query[header_length] << endl;
            modbus_reply(ctx,query,rc,mb_mapping);
            //else
            //
            // Write to input registers registers to the structure from the floating point array
            for ( unsigned i = 0; i < input_registers.size(); i++ )
            {
                uint16_t * pval = new uint16_t[2];
                modbus_set_float( input_registers[i], pval );
                mb_mapping->tab_input_registers[i*2] = pval[0];
                mb_mapping->tab_input_registers[i*2+1] = pval[1];
                delete pval;
            }

            //read holding registers from the structure to the floating point array vector
            for ( int i = 0; i < mb_mapping->nb_registers; i=i+2)
            {
                uint16_t * pval = new uint16_t[2];
                pval[0] = mb_mapping->tab_registers[i];
                pval[1] = mb_mapping->tab_registers[i+1];

                if ( holding_registers.size() < (unsigned)i/2+1)
                    holding_registers.push_back(modbus_get_float(pval));
                else
                    holding_registers[i/2] = modbus_get_float(pval);
            }
        }
        else
        {
            cout << "connection closed" << endl;
            modbus_close(ctx);
            modbus_tcp_accept(ctx,&socket);
            stop_thread = true;

        }

        //update the registers here.
        //holding_registers;
        //input_registers;
        //holding_bits;
        //input_bits;

    }
    //updateing of modbus data
}


bool ModbusServer::write_input_registers( vector<uint16_t> v )
{
    if (v.size() <= input_reg_count / 2 )
    {
        input_registers = v;
        return true;
    }
    return false;

}


vector<uint16_t>  ModbusServer::read_input_registers( void )
{
    return input_registers;
}

bool ModbusServer::read_holding_registers( vector<uint16_t> &v)
{
    v = holding_registers;
    return true;
}

bool ModbusServer::read_digital_inputs( vector<uint16_t> &v)
{
    v = input_bits;
    return true;

    if (v.size() <= bit_count )
    {
        input_bits = v;
        return true;
    }
    return false;
}


/**
 *  Read discretes that can be changed in
 */
bool ModbusServer::write_holding_bits( vector<uint16_t> v )
{
    if ( v.size() <= coil_count )
    {
        holding_bits = v;
        return true;
    }
    return false;
}
