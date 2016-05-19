#include "motion_controller.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>

motion_controller::motion_controller()
{
}

bool 
motion_controller::indicator(double theta_x, double theta_y, double threshold)
{
    return (2*pi-fabs(theta_x-theta_y) < threshold)||(fabs(theta_x - theta_y) < threshold);
}

double 
motion_controller::angle(double goal, double start)
{
    if(fabs(goal-start) < pi)
       return goal-start;
    else if ((goal-start)>pi)
       return goal-start - 2*pi;
    else
       return goal-start + 2*pi;
}

Eigen::Matrix3f
motion_controller::qToRotation(Eigen::Quaternion<float> q)
{
    Eigen::Matrix3f temp;

    temp << 1-2*(q.y()*q.y()+q.z()*q.z()), 2*(q.x()*q.y()-q.w()*q.z())  , 2*(q.w()*q.y()+q.x()*q.z())  ,
            2*(q.x()*q.y()+q.w()*q.z())  , 1-2*(q.x()*q.x()+q.z()*q.z()), 2*(q.y()*q.z()-q.w()*q.x())  ,
            2*(q.x()*q.z()-q.w()*q.y())  , 2*(q.y()*q.z()+q.w()*q.x())  , 1-2*(q.x()*q.x()+q.y()*q.y());
    return temp;
}

double
motion_controller::getYawFromMat(Eigen::Matrix3f mat)
{
    return atan2(mat(1,0), mat(0,0));
}

double 
motion_controller::mod2pi(double angle)
{
    if(fabs(angle) < pi)
         return angle;
    else if (angle < -pi)
         return 2*pi+angle;
    else
         return angle-2*pi;
}

void
motion_controller::cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cmd_pose = msg;
}

void 
motion_controller::absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
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
motion_controller::init(ros::NodeHandle& nh)
{
    m_cmdSub = nh.subscribe<geometry_msgs::PoseStamped>("cmd_pose", 1, boost::bind(&motion_controller::cmdCallback, this, _1));
    m_absposeSub = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("tag_detections", 1, boost::bind(&motion_controller::absposeCallback, this, _1));
    m_posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    m_diffPub = nh.advertise<geometry_msgs::Point>("diff", 1);
    m_fbPub = nh.advertise<geometry_msgs::PointStamped>("fb", 1);


    int send_data_type = 0;
    
    pcan = boost::make_shared<pcan_device>(1,send_data_type);
    pcan->scan_device();
    pcan->open_device();
    pcan->init_device();
    pcan->set_filter();
    pcan->start_device();

    sleep(0.2);
    m_state  << 0.0,
                0.8,
                0.0;

    m_cmd   <<  7.2,
                0.8,
                0.0;

    m_move = FORWARD;
    count =0;
    fix_turn = false;
    fix_forward = false;
    m_resolution = 0.8;
    dual_motor = boost::make_shared<dualmotor>(0x63F, 0x67F, send_data_type);
    dual_motor->power_on();
 
    sleep(0.5);
    return true;
}

void
motion_controller::running(void)
{

    if (cmd_pose)
    {
        ROS_INFO("Command Pose Received!");
        m_cmd << cmd_pose->pose.position.x,
                 cmd_pose->pose.position.y,
                 cmd_pose->pose.position.z;
    }

    cycle_start = ros::Time::now();
    switch (m_move)
    {
        case FORWARD:
        {
            v = dist*p3*10000/pi;

            if(fabs(v)>2500)
            {
                v = boost::math::sign(v)*2500;
            }

            gain = 0.3265*(p4*alpha + p5*beta);
            gain = gain * 10000/pi;
            if (fabs(gain) > 500)
            {
                gain = boost::math::sign(gain) * 500;
            }
            dual_motor->set_vel(floor(v)+floor(gain),-floor(v)+floor(gain));
            dual_motor->Read_Callback_Data(); 

            if (sqrt((m_state[0]- m_cmd[0])*(m_state[0]-m_cmd[0]) + (m_state[1] -m_cmd[1]) * (m_state[1]-m_cmd[1])) < 0.02 && indicator(m_state[2],m_cmd[2],0.05))
            {
                m_move = IDLE;
            }

            break;
        }

        case TURN:
        {
            v = cos(alpha) * dist*p3*10000/pi;
            if(fabs(v)>500)
            {
                v = boost::math::sign(v)*500;
            }

            gain = 0.3265*(p_a*angle(m_cmd[2],m_state[2]));
            gain = gain * 10000/pi;
            if (fabs(gain) > 300)
            {
                gain = boost::math::sign(gain) * 300;
            }
            dual_motor->set_vel(floor(v)+floor(gain),-floor(v)+floor(gain));
            dual_motor->Read_Callback_Data(); 

            if (indicator(m_state[2],m_cmd[2],0.01))
            {
                m_move = IDLE;
            }

            break;
        }
  
        case IDLE:
        {   
            printf("Idel!\n");
            fix_turn = false;
            fix_forward = false;

            count++;
            if (count == 1)
            {
                printf("New commanded goal received!\n");
                m_cmd   <<  7.2,
                            0.8,
                            pi;
                dual_motor->set_vel(0,0);
                dual_motor->Read_Callback_Data();
                m_move = TURN;
                sleep(1);
            }

            else if(count == 2)
            {
                printf("New commanded goal received!\n");
                m_cmd   <<  0.0,
                            0.8,
                            pi;
                dual_motor->set_vel(0,0);
                dual_motor->Read_Callback_Data();
                m_move = FORWARD;
                sleep(1);
            }

            else if(count == 3)
            {
                printf("New commanded goal received!\n");
                m_cmd   <<  0.0,
                            0.8,
                            0.0;
                dual_motor->set_vel(0,0);
                dual_motor->Read_Callback_Data();
                m_move = TURN;
                sleep(1);
            }
            else if(count == 4)
            {
                printf("Reached Goal!\n");
                dual_motor->set_vel(0,0);
                dual_motor->Read_Callback_Data();
                m_move = IDLE;
                sleep(1);
            }

            else
            {
                printf("Idle!\n");
                dual_motor->set_vel(0,0);
                dual_motor->Read_Callback_Data();
                m_move = IDLE;
                sleep(0.05);
            }
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

        Eigen::Quaternion<float> quat;
        quat.w() = abs_pose->pose.pose.orientation.w,
        quat.x() = abs_pose->pose.pose.orientation.x,
        quat.y() = abs_pose->pose.pose.orientation.y,
        quat.z() = abs_pose->pose.pose.orientation.z;

        Eigen::Matrix3f Rotation;
        Eigen::Vector3f translation;
        translation << abs_pose->pose.pose.position.x,
                       abs_pose->pose.pose.position.y,
                       abs_pose->pose.pose.position.z;

        Rotation = qToRotation(quat);

        translation = -Rotation.inverse()*translation;

        Eigen::Matrix3f fixTF;
        fixTF << 1, 0,  0,
                 0, -1, 0,
                 0, 0, -1;

        Rotation = Rotation.inverse()*fixTF;

        m_state[0] = translation[0] + m_resolution * (id%10);
        m_state[1] = translation[1] + m_resolution * floor(id/10.0);
        m_state[2] = getYawFromMat(Rotation);
    }
    else
    {
    }
    geometry_msgs::Point diff_msgs;
    diff_msgs.x = vel.first;
    diff_msgs.y = vel.second;
    diff_msgs.z = vel.first+vel.second;
    m_diffPub.publish(diff_msgs);

    geometry_msgs::PoseStamped pose_msg;
         
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = m_state[0];
    pose_msg.pose.position.y = m_state[1]*25;
    pose_msg.pose.position.z = m_state[2]/pi*180;

    pose_msg.pose.orientation.w = cos(m_state[2]/2);
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = sin(m_state[2]/2);

    m_posePub.publish(pose_msg);

    delta_y = m_cmd[1]-m_state[1];
    if(fabs(m_cmd[1]-m_state[1]) > 1.6)
    {
        delta_y = boost::math::sign(m_cmd[1]-m_state[1]) * 1.6;
    }

    delta_x = m_cmd[0]-m_state[0];
    if(fabs(m_cmd[0]-m_state[0]) > 1.6)
    {
        delta_x = boost::math::sign(m_cmd[0]-m_state[0]) * 1.6;
    }


    beta = angle(m_cmd[2], atan2(delta_y, delta_x));

    alpha = -beta - m_state[2]+m_cmd[2];
    alpha = mod2pi(alpha);

    dist = sqrt(delta_x*delta_x + delta_y* delta_y);

    geometry_msgs::PointStamped fb_msgs;
    fb_msgs.header.frame_id = "world";
    fb_msgs.header.stamp = ros::Time::now();

    fb_msgs.point.x = alpha/pi*180;
    fb_msgs.point.y = beta/pi*180;
    fb_msgs.point.z = dist;
    m_fbPub.publish(fb_msgs);

}

void 
motion_controller::shutdown()
{
    dual_motor->shut_down();
}

void 
motion_controller::close(void)
{
    pcan->reset_device();
    pcan->close_device();
}
