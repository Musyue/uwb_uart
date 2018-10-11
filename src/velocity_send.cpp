#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "uwb_uart/velo.h"
#include <sstream>
#include <string>

using std::string;
using std::stringstream;

serial::Serial ser;

void VeloSendCallback(const uwb_uart::velo& v){
    string prefix = "!M ";
//    string velo_1, velo_2;
//    stringstream ss1, ss2;

//    ss1 << v.v1;
//    ss2 << v.v2;
//    ss1 >> velo_1;
//    ss2 >> velo_2;
    
    string final_order = prefix + v.v1 + " " + v.v2 + "\r\n";
    ser.write(final_order);
    std::cout << final_order << std::endl;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "velocity_send");
    ros::NodeHandle nh;

    
/*    ros::Publisher read_pub = nh.advertise<std_msgs::String>("uwb_read", 1000);*/

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Subscriber write_sub = nh.subscribe("get_velo", 1000, VeloSendCallback);
    ros::spin();
/*    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }*/
}

