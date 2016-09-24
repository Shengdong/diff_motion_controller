#include "motion_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;
    motion_controller controller;

    if (ros::ok())
    {
       ros::spinOnce();
    }
    
    while(!controller.init(nh))
    {
       ROS_INFO("Wait for initilisation");
       ros::spinOnce();
       sleep(1);
    }


//    int count = 20000;
    while(nh.ok())
    {
       controller.running();
       ros::spinOnce();
    }

    controller.shutdown();
    controller.close();
}
