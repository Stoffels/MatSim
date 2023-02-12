#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <vector>

//ip address stuff
#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
//


#include "simulator.h"
#include "mbTcpServer.h"


using namespace std;
using namespace Eigen;


bool run_simulation = false;
thread communications_thread;

string get_tcpip_address( void )
{
    string ext_ip_address;
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            //printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            string ifa_addr_str = ifa->ifa_name;
            if ( ifa_addr_str == "eth0")
                ext_ip_address= addressBuffer;
        } else if (ifa->ifa_addr->sa_family == AF_INET6) { // check it is IP6
            // is a valid IP6 Address
            tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
            char addressBuffer[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
            //printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
        }
    }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return ext_ip_address;
}


string cin_default_str( string default_str)
{
    string str = default_str;
    string input;
    getline( cin, input );
    if ( !input.empty() )
    {
        istringstream stream( input );
        stream >> str;
    }
    return str;
}

int cin_default_int( int default_int )
{
    int number = default_int;
    string input;
    getline( cin, input );
    if ( !input.empty() )
    {
        istringstream stream( input );
        stream >> number;
    }
    return number;
}
//thread = std::thread(&mbTcpServer::ServerUpdateThread,this);

void set_modbus_server( mbTcpServer * mb)
{
    cout << "Modbus Server Parameters" << endl;
    cout << "------------------------" << endl;

    cout << "\tIP address [" << get_tcpip_address() << "] :";
    string ip_address = cin_default_str(get_tcpip_address());

    cout << "\tPort address [1502] :";
    int ip_port = cin_default_int(1502);

    cout << "\tDiscrete buffer sizes [512] :";
    int disc_buffers = cin_default_int(512);

    cout << "\tAnalog buffer sizes [512] :";
    int analog_buffers = cin_default_int(512);

    mb->set_ip_address(ip_address);
    mb->set_port_address(ip_port);
    mb->set_buffer_sizes(disc_buffers,disc_buffers,analog_buffers,analog_buffers);
    mb->Initialize();
    cout << endl;

}


void initialize_ss_simulator( ss_simulator * s);
void update_ss_simulation( ss_simulator * s, mbTcpServer * m);

int main()
{


    mbTcpServer mb;
    set_modbus_server( &mb);

    //fir_simulator sim;
    //set_fir_simulator(&sim);
    ss_simulator sim;
    initialize_ss_simulator(&sim);

    //run the simulation thread
    //communications_thread = std::thread(&update_simulation, &sim, &mb);
    communications_thread = std::thread(&update_ss_simulation, &sim, &mb);

    int i;
    cout << "Enter to stop";
    cin >> i;
    if ( i == 1)
        run_simulation = false;

    cout << "exit" << endl;
    //while( true );

    return 0;
}

void initialize_ss_simulator( ss_simulator * s){
    //fir_simulator sim;
    cout << "Model Simulator Parameters" << endl;
    cout << "--------------------------" << endl;
    cout << "\tSampling Rate <1> : ";
    int sampling_rate = cin_default_int(1);
    cout << "\tUse default model <n> :";
    //s->set_sampling_rate(sampling_rate);

    string ans_str = cin_default_str("n");

    if (ans_str == "y" )
    {

        MatrixXd A(6,6);
        MatrixXd B(6,4);
        MatrixXd C(6,1);
        MatrixXd Y_inf(6,1);



        A <<    0.9052963,-0.107939,-0.042088,0.0929498,-0.133037,0.0417200,
                0.0624270,0.6954625,-0.088910,-0.091471,0.1761103,-0.074434,
                0.0576351,-0.270747,0.9090996,-0.085210,0.1363263,-0.058776,
                -0.020243,0.0265925,0.0395399,0.8941104,-0.215573,0.0004165,
                -0.012983,0.0216866,0.0496963,-0.014398,0.6211680,0.0113073,
                0.0745731,-0.113570,0.0515222,0.0292580,-0.226065,0.8534298;


        B <<    0.0526312,0.1757958,0.0204869,-0.006762,
                -0.002409,0.3294308,-0.045162,0.0194211,
                -0.006656,0.2967259,-0.011896,0.0130403,
                0.0030826,0.0109346,0.2902497,-0.024366,
                -0.000114,-0.015578,0.3655360,-0.023432,
                -0.036839,0.0355231,0.2863805,0.0330702;

        C <<    1.7433744,
                8.1793481,
                8.3547229,
                -1.806900,
                -0.666654,
                6.6816130,
        Y_inf << 20,20,20,20,20,20;

        int order = A.cols() / A.rows();

        MatrixXd A_ = fir2ss_Psi( A,B,C,order );
        MatrixXd B_ = fir2ss_Gamma( A, B, C, order );
        MatrixXd C_ = fir2ss_C(A,B,C,order);
        MatrixXd D_ = fir2ss_Fixedpoint( A,B,C,order);


        s->load_model(A_,B_,C_,D_,sampling_rate);


        s->set_initial_state(Y_inf);
    }
    else
    {
        cout << "\tPV count  <2> : ";
        int pv_count = cin_default_int(4);

        cout << "\tMV count  <2> : ";
        int cv_count = cin_default_int(4);

        cout << "\tOrder      <1> : ";
        int ordr = cin_default_int(1);

        cout << "\tStabilized <y> : ";
        string ans_str = cin_default_str("y");
        if (ans_str == "y"  and (ordr == 1))
            s->generate_stable_random_model( pv_count, cv_count, ordr, sampling_rate);
        else
            s->generate_random_model(pv_count, cv_count,ordr,sampling_rate);

        cout << "\t Enable filtering <n> :";
        ans_str = cin_default_str("n");
        if ( ans_str == "y"){
            double std_dev = 0.0;

            cout << "\tProcess stdev: ";
            cin >> std_dev;
            s->set_process_noise(std_dev);

            cout << "\tMeasurement stdev: ";
            cin >> std_dev;
            s->set_measurement_noise(std_dev);
        }
    }

}

void update_ss_simulation( ss_simulator * s, mbTcpServer * m){
    run_simulation = true;
    s->start();
    m->start();

    //save information for clients to read
    MatrixXd info(2,1);
    info(0,0) = (double)s->get_process_variable_count();
    info(1,0) = (double)s->get_control_variable_count();
    //info(2,0) = (double)s->get_sampling_rate();

    cout << "system dimension [pv,cv] = [" << info.transpose() << "]" << endl;

    m->write_float_holding_registers(0,info);

    while( run_simulation )
    {
        //cout << "start address for the control variable block = " << info.rows()+info(0,0) << endl;
        //get data from the modbus server, load to the simulation
        MatrixXd control_vars = m->read_float_holding_registers( info.rows()+info(0,0), s->get_control_variable_count());
        s->set_input(control_vars);

        //write the process values to the modbus server
        MatrixXd process_vars = s->get_output();
        m->write_float_holding_registers(info.rows(), process_vars);

        cout << "Process variables = [" << s->get_output().transpose() << "]" << endl;

        cout << "Control variables = [" << s->get_input().transpose() << "]" << endl;

        //cout << "all  = " << m->read_float_holding_registers( 0, info.rows() + info.sum()).transpose() << endl;
        cout << endl;



        std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    m->stop();
    s->stop();
}




