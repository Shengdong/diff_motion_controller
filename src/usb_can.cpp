#include "usb_can.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>

usb_can::usb_can()
{
}

void
usb_can::cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cmd_pose = msg;
}

void 
usb_can::absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if(msg->detections.size())
    {   
        abs_pose = boost::make_shared<const apriltags_ros::AprilTagDetection>(msg->detections[0]);
    }
    else
    {
        abs_pose = NULL;
    }
}

bool 
usb_can::init(ros::NodeHandle& nh)
{
    m_cmdSub = nh.subscribe<geometry_msgs::PoseStamped>("cmd_pose", 1, boost::bind(&usb_can::cmdCallback, this, _1));
    m_absposeSub = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("tag_detections", 1, boost::bind(&usb_can::absposeCallback, this, _1));
    m_posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    int send_data_type = 1;
    
    pcan = boost::make_shared<pcan_device>(1,send_data_type);
    pcan->scan_device();
    pcan->open_device();
    pcan->init_device();
    pcan->set_filter();
    pcan->start_device();

    m_state  << 0.0,
                0.0,
                0.0;

    m_cmd   <<  4.0,
                0.0,
                0.0;
/*
    if (cmd_pose)
    {
        ROS_INFO("Command Pose Received!");
        m_cmd << cmd_pose->pose.position.x,
                 cmd_pose->pose.position.y,
                 cmd_pose->pose.position.z;
    }
*/
    m_move = IDLE;
    m_resolution = 0.8;
    dual_motor = boost::make_shared<dualmotor>(0x63F, 0x67F, send_data_type);
    dual_motor->power_on();
    sleep(0.2);
    return true;
}

void
usb_can::running(void)
{
    cycle_start = ros::Time::now();

    if (sqrt((m_state[0]- m_cmd[0])*(m_state[0]-m_cmd[0]) + (m_state[1] -m_cmd[1]) * (m_state[1]-m_cmd[1])) > 0.01)
    {
        m_move = FORWARD;
    }
    else if (fabs(m_cmd[2] - m_state[2])>0.01)
    {
        m_move = TURN;
    }
    else 
    {
        m_move = IDLE;
    }

    switch (m_move)
    {
        case FORWARD:
        {
//            printf("Moving Forward!\n");

            v = dist*p3*10000/3.14;
            if(v>1000)
            {
                v = 1000;
            }
            gain = 0.3265*(p4*alpha + p5*beta);
            gain = gain * 10000/3.14;
            if (fabs(gain) > 100)
            {
                gain = boost::math::sign(gain) * 100;
            }
            dual_motor->set_vel(floor(v)+floor(gain),-floor(v)+floor(gain));
            dual_motor->Read_Callback_Data(); 
            break;
        }

        case TURN:
        {
//            printf("Turning Around!\n");

            gain = 0.3265*(p3*(m_cmd[2] - m_state[2]));
            gain = gain * 10000/3.14;
            if (fabs(gain) > 300)
            {
                gain = boost::math::sign(gain) * 300;
            }
            dual_motor->set_vel(floor(gain), floor(gain));
            dual_motor->Read_Callback_Data(); 
            break;
        }

        case IDLE:
        {   
            dual_motor->set_vel(0,0);
            dual_motor->Read_Callback_Data();
//            printf("Reached Goal!(%f, %f)\n", dist, m_cmd[2]-m_state[2]);
            break;
        }
    }
        
    vel = dual_motor->get_vel();

    cycle_period = (ros::Time::now() - cycle_start).toSec();

    m_state[0] = m_state[0] + (-vel.second+vel.first)/2 * cos(m_state[2]) * 0.018849555 * cycle_period/60;
    m_state[1] = m_state[1] + (-vel.second+vel.first)/2 * sin(m_state[2]) * 0.018849555 * cycle_period/60;        
    m_state[2] = m_state[2] + (vel.first + vel.second)/0.653 * 0.018849555 * cycle_period/60;


    if (abs_pose)
    {
        int id;
        id = abs_pose->id;
        m_state[0] = abs_pose->pose.pose.position.x + m_resolution * (id%10);
        m_state[1] = abs_pose->pose.pose.position.y + m_resolution * floor(id/10.0);
        m_state[2] = atan2(2*(abs_pose->pose.pose.orientation.w * abs_pose->pose.pose.orientation.z + abs_pose->pose.pose.orientation.x * abs_pose->pose.pose.orientation.y), 1-2*(abs_pose->pose.pose.orientation.y * abs_pose->pose.pose.orientation.y + abs_pose->pose.pose.orientation.z*abs_pose->pose.pose.orientation.z));
        ROS_INFO("Absolute pose received!");

        printf("(%f, %f, %f)\n", m_state[0], m_state[1], m_state[2]);
    }
    else
    {
         ROS_INFO("No tag is detected!");
    }

    geometry_msgs::PoseStamped pose_msg;
         
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = m_state[0];
    pose_msg.pose.position.y = m_state[1];
    pose_msg.pose.position.z = 0;

    pose_msg.pose.orientation.w = cos(m_state[2]/2);
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = sin(m_state[2]/2);

    m_posePub.publish(pose_msg);

    beta = -atan2(m_cmd[1]-m_state[1], m_cmd[0]-m_state[0]) + m_cmd[2];
    alpha = -beta - m_state[2];
    dist = sqrt((m_state[0]-m_cmd[0])*(m_state[0]-m_cmd[0]) + (m_state[1] -m_cmd[1]) * (m_state[1]-m_cmd[1]));
}

void 
usb_can::shutdown()
{
    dual_motor->shut_down();
}

void 
usb_can::close(void)
{
    pcan->reset_device();
    pcan->close_device();
}
