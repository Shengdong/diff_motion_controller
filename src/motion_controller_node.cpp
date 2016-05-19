#include "motion_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;
    motion_controller controller;
    if(!controller.init(nh))
    {
       return 1;
    }

    if (ros::ok())
    {
       ros::spinOnce();
    }
    
    int count = 3500;
    while(count--)
    {
       controller.running();
       ros::spinOnce();
    }

    controller.shutdown();
    controller.close();
}
