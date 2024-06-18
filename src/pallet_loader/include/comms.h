#ifndef IMUTEST_COMMS_H
#define IMUTEST_COMMS_H
//#include "imutest/comms.h"
//#include <rclcpp/rclcpp.hpp>  
#include <sstream>
#include <cstdlib>
//#include <sensor_msgs/msg/imu.h>
//#include <sensor_msgs/msg/imu.hpp>
//#include <serial/serial.h>
#include <libserial/SerialPort.h>
#include <cstring>
#include<iostream>

using namespace std;

class Comms
{
    public:

    Comms()=default;
        void setup(const string &serial_device,int32_t baud_rate,int32_t timeout_ms);
        void move_pallet(std::string &position);
        bool connected()const{ return serialDriver.IsOpen();}

        //std::string sendMsg(const std::string &msg_to_send,bool print_output=false);
        double accelX=0.0;
        double accelY=0.0;
        double accelZ=0.0;
        double gyroX=0.0;
        double gyroY=0.0;
        double gyroZ=0.0;
        double yaw=0.0;
        double pitch=0.0;
        double roll=0.0;
        string position="";
        private:

        LibSerial::SerialPort serialDriver;
        int timeout_ms=10000;
       

};

#endif
