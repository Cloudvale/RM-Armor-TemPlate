#include "serialport.h"
#include <string>
#include <iostream>


SerialPort::SerialPort(const string ID, const int BAUD)
{
    serial_id = ID;
    baud = BAUD;
    // if(not initSerialPort()){
    //     exit(0);
    // };
    // system(std::string("echo root@233|sudo chmod 777 /dev/ttyUSB0").c_str());
    // cout << system(std::string("pwd").c_str()) << endl;
     // system(std::string("root@233").c_str());


// #ifdef DEBUG_WITHOUT_COM
//     withoutSerialPort();
// #else
//     initSerialPort();
// #endif //DEBUG_WITHOUT_COM
}