#include "usb_can.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;
    usb_can controller;
    if(!controller.init(nh))
    {
       return 1;
    }

    if (ros::ok())
    {
       ros::spinOnce();
    }
    
    int count = 100;
    while(count--)
    {
       controller.running();
       ros::spinOnce();
    }

    controller.shutdown();
    controller.close();
}
