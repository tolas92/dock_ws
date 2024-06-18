#include "pallet_loader/comms.h"
//#include <rclcpp/rclcpp.hpp>  
#include <sstream>
#include <cstdlib>
//#include <sensor_msgs/msg/imu.h>
//#include <sensor_msgs/msg/imu.hpp>
//#include <serial/serial.h>
#include <libserial/SerialPort.h>
#include <cstring>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std;
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}                                                

void Comms::setup(const std::string &serial_device,int32_t baud_rate,int32_t timeout_ms)
{
    serialDriver.Open(serial_device);
    serialDriver.SetBaudRate(convert_baud_rate(baud_rate));
    //serial::Timeout to=serial::Timeout::simpleTimeout(timeout_ms);
    //serialDriver.setTimeout(to);
    //serialDriver.open();
    if(serialDriver.IsOpen())
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(rclcpp::get_logger("pallet_loader"), "Serial driver is open!");
    }
}


void Comms::move_pallet(std:string &position)
{
    serialDriver.FlushIOBuffers();

    if(&position="up"){
    serialDriver.Write("e\r");
    std::string response = "";
    while (response=""){
    serialDriver.ReadLine(&response, '\n', timeout_ms);   
    }
    }

    /*
    // Parse the YPR values from the response
    size_t delimiterPos = response.find(" ");
    std::string axtoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);
    delimiterPos = response.find(" ");
    std::string gzToken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);
    std::string yawToken = response;

    // Convert the tokens to double values
    accel_x = std::atof(axtoken.c_str());
    gyro_z = std::atof(gzToken.c_str());
    yaw = std::atof(yawToken.c_str());*/

    size_t delimiterPos = response.find(" ");
    std::string axtoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);

    delimiterPos = response.find(" ");
    std::string aytoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);

    delimiterPos = response.find(" ");
    std::string aztoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);

    delimiterPos = response.find(" ");
    std::string gxtoken = response;


    // Convert the tokens to double values
    accel_x = std::atof(axtoken.c_str());
    accel_y = std::atof(aytoken.c_str());
    accel_z = std::atof(aztoken.c_str());
    gyro_x = std::atof(gxtoken.c_str());
    
    }
/*
int main()
{
    Comms arduino_;
    arduino_.setup("/dev/ttyACM0", 115200, 10000);
    double yaw, pitch, roll;

    // Assuming the Arduino is already connected and configured

    // Continuously read and display the YPR values
    while (true)
    {
        arduino_.readYPRValues(yaw, pitch, roll);
        std::cout << "Yaw: " << yaw << ", Pitch: " << pitch << ", Roll: " << roll << std::endl;
        // Add delay if needed
    }

    return 0;
}*/
