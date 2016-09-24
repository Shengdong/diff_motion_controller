#include "motion_controller.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define PI 3.1415926535898


motion_controller::motion_controller()
{
}

bool 
motion_controller::indicator(double theta_x, double theta_y, double threshold)
{
    return (2*PI-fabs(theta_x-theta_y) < threshold)||(fabs(theta_x - theta_y) < threshold);
}

double 
motion_controller::angle(double goal, double start)
{
    if(fabs(goal-start) <= PI)
       return goal-start;
    else if ((goal-start)>PI)
       return goal-start - 2*PI;
    else
       return goal-start + 2*PI;
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
    if(fabs(angle) <= PI)
         return angle;
    else if (angle < -PI)
         return 2*PI+angle;
    else
         return angle-2*PI;
}

void
motion_controller::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(msg->poses.size())
    {   
         m_path = msg;
    }
    else
    {
         m_path = NULL;
    }
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
//  initialise the ros publisher and subscriber
    m_pathSub = nh.subscribe<nav_msgs::Path>("path", 1, boost::bind(&motion_controller::pathCallback, this, _1));
    m_absposeSub = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("tag_detections", 1, boost::bind(&motion_controller::absposeCallback, this, _1));
    m_posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    int send_data_type = 0;

// create usb can interface object and gpio usb interface object    
    pcan = boost::make_shared<pcan_device>(1,send_data_type);
    gpio = boost::make_shared<gpio_io>(1);

// scan device
    pcan->scan_device();

//  configure gpio
    gpio->open_io();
    gpio->set_input(4);
    gpio->set_opendrain(4);
    gpio->set_input(5);
    gpio->set_opendrain(5);
    gpio->set_input(6);
    gpio->set_opendrain(6);

//  configure usb can 
    pcan->open_device();
    pcan->init_device();
    pcan->set_filter();
    pcan->start_device();

    int dev_index;
    dev_index = pcan->device_index();

/*
    usb can and gpio are from the same supplier so, device index should be checked in respective script
    printf("Test Over!\n");
    ros::shutdown();
    exit(1);    
*/
//  TODO: Add localisation initialisation process or manually tune it. If robot is not at the pose, turn on tag detector first
    m_state << 0,
               2.88,
               0;

    sleep(0.2);

//  initialise state and parameter
    m_move = IDLE;
    block_path_receiver = false;
    count =0;
    m_resolution = 0.96;

    lift_motor = boost::make_shared<singlemotor>(0x61F, dev_index,send_data_type);
    gpio->reset_pin(4);
    lift_motor->reset_pos();
    lift_motor->set_vel(1000);
    lift_motor->power_on();
    sleep(1);
/*
    //  if you find the linear module slide is down side the position sensor, move it up first
    lift_motor->set_pos(-143360);
    sleep(3);
*/

//  move up a little bit if position sensor sense the slide
    if (gpio->read_data(4))
    {
       lift_motor->set_pos(-143360);
       sleep(2);
    }

//  move down until the position sensor sense the slide
    lift_motor->set_pos(443360);

    while(!gpio->read_data(4))
    {
        usleep(10000);
    }

//  set the initial position, 28672 is equal to 23 mm
    int pose;
    pose = lift_motor->get_pos()- 28672;
    lift_motor->set_pos(pose);
    sleep(4);
    lift_motor->reset_pos();
    lift_motor->shut_down();
    sleep(1);


//  telefork motor initial process
    telefork_motor = boost::make_shared<singlemotor>(0x65F, dev_index,send_data_type);
    telefork_motor->reset_pos();
    telefork_motor->power_on();
    gpio->reset_pin(5);
    gpio->reset_pin(6);
    telefork_motor->set_vel(500);
    sleep(1);
//  if the telefork is not approximately in the middle
    if (!gpio->read_data(5)||!gpio->read_data(6))
    {
        //  only if one of two gpio pin high, get into this mode, if both pin high deadlock,
        int count_fork =1000;

        int i_sign = gpio->read_data(6) - gpio->read_data(5);

        // TODO:143360 is to be determined
        telefork_motor->set_pos(i_sign*333900);
        // GPIO 6 is left upside down in the back view, right in the back view
        while(!gpio->read_data(5)||!gpio->read_data(6))
        {
            usleep(10000);
            count--;
            if (count_fork<=0)
            {
                printf("Fork can not move to initialise position, maybe position sensor setup gets wrong");
                telefork_motor->shut_down();
                sleep(1);
                gpio->reset_pin(5);
                gpio->reset_pin(6);
                pcan->reset_device();
                pcan->close_device();
                gpio->close();
                return 1;
            }
        }

        int pose;
        pose = telefork_motor->get_pos();
        //the exact number of 28532 is to be determined
        pose = pose + i_sign * 12766;
        telefork_motor->set_pos(pose);
        sleep(2);

        pose = telefork_motor->get_pos();
        telefork_motor->reset_pos();
    }
    // if the telefork is approximately in the middle
    else
    {
        // if fork is proximately in the middle, choose one side as reference and move toward it until another side pin low
        // :143360 is to be determined
        telefork_motor->set_pos(71180);

        while(gpio->read_data(6))
        {
            usleep(10000);            
        }
        int pose;
        pose = telefork_motor->get_pos();
        //the exact number of 28532 is to be determined
        pose = pose - 14266;
        telefork_motor->set_pos(pose);
        sleep(2);

        pose = telefork_motor->get_pos();

        telefork_motor->reset_pos();
    }
    
//  telefork stroke test can be added here  286720 = 1 round = 515.22mm?
/*
    telefork_motor->set_pos(286720);
    sleep(10);
    ros::shutdown();
*/

//  locomotion motor free mode can be active here
/*
    locomotor->free_mode();
    locomotor->power_on();
    sleep(20);
    locomotor->shut_down();
    ros::shutdown();
*/

    locomotor = boost::make_shared<dualmotor>(0x63F, 0x67F, dev_index, send_data_type);
    locomotor->init();
    locomotor->power_on();
 
    vel = locomotor->get_vel();
    sleep(0.5);
    return true;
}

double 
motion_controller::distance2d(Eigen::Vector3d point_end, Eigen::Vector3d point_start)
{
    return sqrt((point_end[0]-point_start[0])*(point_end[0]-point_start[0]) + (point_end[1]-point_start[1])*(point_end[1]-point_start[1]));
}

void
motion_controller::running(void)
{
    cycle_start = ros::Time::now();

//  receive path and when received block path receiver until mission completed
    if (m_path && !block_path_receiver)
    {
        nav_msgs::Path temp_path = *m_path;
        std::vector<geometry_msgs::PoseStamped> temp_path_vector;
        extracted_path.clear();
        for( std::vector<geometry_msgs::PoseStamped>::iterator itr = temp_path.poses.begin() ; itr != temp_path.poses.end(); ++itr)
        {
            temp_path_vector.push_back(*itr);
        }

        if(!temp_path_vector.empty())
        {
             ROS_INFO("Path Received!");
        }
        p_count++;

        while(!temp_path_vector.empty())
        {
            extracted_path.push_back(temp_path_vector.back());
            temp_path_vector.pop_back();
        }
        block_path_receiver = true;
    }

//  switch status according to the command
    switch (m_move)
    {
        // move forward 
        case FORWARD:
        {
            // v is linear speed and gain is angular velocity
            v =  sqrt(dist)*v_max/(1 + gamma*k*k)*10000/PI;

            gain = 0.3265*v*k;
            if (fabs(gain) > 500)
            {
                gain = boost::math::sign(gain) * 500;
            }

            if(fabs(v)>2000)
            {
                v = boost::math::sign(v)*2000;
            }
            // set motor rotation velocity
            locomotor->set_vel(floor(v)+floor(gain),-floor(v)+floor(gain));

            // when the euler distance is less than 100mm, achieved waypoint
            if (distance2d(m_state, m_cmd) < 0.1 && indicator(m_state[2],m_cmd[2],0.1))
            {
                m_move = IDLE;
            }
            break;
        }

        case BACKWARD:
        {
            v =  sqrt(dist) * 0.75/(1+0.2*k_back*k_back) *10000/3.1415926;

            gain = 0.3265*v*k_back;

            if (fabs(gain) > 500)
            {
                gain = boost::math::sign(gain) * 500;
            }

            if(fabs(v)>2000)
            {
                v = boost::math::sign(v)*2000;
            }
            
            locomotor->set_vel(-floor(v)+floor(gain),floor(v)+floor(gain));
            if (distance2d(m_state, m_cmd) < 0.1 && indicator(m_state[2],m_cmd[2],0.1))
            {
                m_move = IDLE;
            }
            break;
        }

        case LIFTFORK:
        {
            // TODO: Lift height should be determined, fork stroke should be determined
            locomotor->set_vel(0, 0);
            sleep(3);
            locomotor->shut_down();
            lift_motor->power_on();
            if ((fork_count%2) == 1)
                lift_motor->set_pos(-306720);
            else
                lift_motor->set_pos(-356720);

            sleep(10);

/*
           //test telefork camera FOV can be set here
           if(abs_pose)
           {
               Eigen::Quaternion<float> quat;
               quat.w() = abs_pose->pose.pose.orientation.w;
               quat.x() = abs_pose->pose.pose.orientation.x;
               quat.y() = abs_pose->pose.pose.orientation.y;
               quat.z() = abs_pose->pose.pose.orientation.z;

               Eigen::Matrix3f Rotation;
               Eigen::Vector3f translation;
               translation << abs_pose->pose.pose.position.x,
                              abs_pose->pose.pose.position.y,
                              abs_pose->pose.pose.position.z;

               Rotation = qToRotation(quat);

               translation = -Rotation.inverse()*translation;

               float x_error;
               float yaw_error;
 
               x_error   = translation(0);                  
               yaw_error = getYawFromMat(Rotation);
               printf("TEST: x_error is %f and yaw_error is %f", x_error, yaw_error);
           }

           sleep(20);
           ros::shutdown();
           exit(1);
*/

            // pose correction code inserted here  first make sure tag is attached vertically, second camera has no pitch angle relative to the vehicle
            
/*
            if ((fork_count%2) == 1)
            {
                int count_detect = 0;
                while(ros::ok())
                {
                    if(abs_pose)
                    {

                        Eigen::Quaternion<float> quat;
                        quat.w() = abs_pose->pose.pose.orientation.w;
                        quat.x() = abs_pose->pose.pose.orientation.x;
                        quat.y() = abs_pose->pose.pose.orientation.y;
                        quat.z() = abs_pose->pose.pose.orientation.z;

                        Eigen::Matrix3f Rotation;

                        Rotation = qToRotation(quat);

                        Eigen::Matrix3f FixTF;
                        FixTF << 1, 0,  0,
                                 0, 0, -1,
                                 0, 1,  0;

                        Rotation = FixTF * Rotation.inverse();

                        float yaw_error;
                    
                        yaw_error = getYawFromMat(Rotation);
                    
                        gain = -3265*(k_angle*yaw_error)/3.1415926;
                        if(fabs(gain)>200)
                        {
                            gain = boost::math::sign(gain) * 200;
                        }
                        locomotor->set_vel(floor(gain), floor(gain));
                        if (fabs(yaw_error*180/3.1415926) < 1)
                        {
                            break;
                        }
                    }
                    else
                    {
                        usleep(10000);
                        count_detect++;
                    }
                    if (count_detect>1000)
                    {
                        ROS_WARN("No Tag detected when stoped");
                        ros::shutdown();
                        exit(1);
                    }
                }
 
                count = 0;
                while(ros::ok())
                {
                    if(abs_pose)
                    {

                        Eigen::Quaternion<float> quat;
                        quat.w() = abs_pose->pose.pose.orientation.w;
                        quat.x() = abs_pose->pose.pose.orientation.x;
                        quat.y() = abs_pose->pose.pose.orientation.y;
                        quat.z() = abs_pose->pose.pose.orientation.z;

                        Eigen::Matrix3f Rotation;
                        Eigen::Vector3f translation;
                        translation << abs_pose->pose.pose.position.x,
                                       abs_pose->pose.pose.position.y,
                                       abs_pose->pose.pose.position.z;

                        Rotation = qToRotation(quat);

                        translation = -Rotation.inverse()*translation;

                        float x_error;
                    
                        x_error = translation[0];
                    
                        v = -x_error*10000/3.1415926;
                        if(fabs(x_error)>200)
                        {
                            v = boost::math::sign(v) * 200;
                        }
                        locomotor->set_vel(floor(v), -floor(v));
                        if (fabs(x_error*180/3.1415926) < 0.008)
                        {
                            break;
                        }
                    }
                    else
                    {
                        usleep(10000);
                        count_detect++;
                    }
                    if (count_detect>1000)
                    {
                        ROS_WARN("No Tag detected when stoped");
                        ros::shutdown();
                        exit(1);
                    }
                }
*/

            telefork_motor->set_pos( boost::math::sign(lift_param)*(-333900) );
            sleep(15);

            if ((fork_count%2) == 1)
                 lift_motor->set_pos(-356720);
            else
                 lift_motor->set_pos(-295720);
            sleep(3);
            telefork_motor->set_pos(0);
            sleep(10);            

            lift_motor->shut_down();
            locomotor->power_on();
            sleep(0.5);
            m_move = IDLE;

            break;
        }

        case TURN:
        {
            v = cos(alpha) * dist* k_dist *10000/PI;
            if(fabs(v)>300)
            {
                v = boost::math::sign(v)*300;
            }

            gain = 3265*(k_angle*angle(m_cmd[2],m_state[2]))/PI;
            if (fabs(gain) > 400)
            {
                gain = boost::math::sign(gain) * 400;
            }
            locomotor->set_vel(floor(v)+floor(gain),-floor(v)+floor(gain));

            if (indicator(m_state[2],m_cmd[2],0.01))
            {
                m_move = IDLE;
            }
            break;
        }
  
        case IDLE:
        {
           locomotor->set_vel(0,0);
           if (extracted_path.size()!=0)
           {
               geometry_msgs::PoseStamped temp_pose = extracted_path.back();
               float yaw_ = 2*atan2(temp_pose.pose.orientation.z,temp_pose.pose.orientation.w);
               m_cmd << temp_pose.pose.position.x * m_resolution,
                        temp_pose.pose.position.y * m_resolution,
                        angle(yaw_,0);

               printf("Next Commanded Pose is (%f, %f, %f)\n", m_cmd[0], m_cmd[1], m_cmd[2]);
               if ( (fabs(m_cmd[0] - m_state[0])>0.5) && (fabs(m_cmd[1] - m_state[1])>0.5) )
               {
                   locomotor->shut_down();
                   exit(1);
               }
               if ( (fabs(m_cmd[0] - m_state[0])>0.5) || (fabs(m_cmd[1] - m_state[1])>0.5) )
               {
                   if (fabs(m_cmd[0] - m_state[0])>0.5)
                   {
                       if (cos(m_state[2]) *  (m_cmd[0] - m_state[0]) > 0)
                           m_move = FORWARD;
                       else
                           m_move = BACKWARD;
                   }
                   else
                   {
                       if (sin(m_state[2]) *  (m_cmd[1] - m_state[1]) > 0)
                           m_move = FORWARD;
                       else
                           m_move = BACKWARD;
                   }
                   if (m_move == FORWARD)
                       printf("Move Forward!\n");
                   else
                       printf("Move Backward!\n");
                }
                else if (fabs(m_cmd[2] - m_state[2])>0.5)
                {
                    m_move = TURN;
                    printf("Turn Around!\n");
                }
                else if (temp_pose.pose.position.z!=0)
                {
                    m_move = LIFTFORK;
                    fork_count++;
                    lift_param = temp_pose.pose.position.z;
                    printf("Lift and fork!\n");
                }
                else
                    m_move = IDLE;
               extracted_path.pop_back();
           }
           else
           {
                if (p_count == 2)
                {
                    lift_motor->power_on();
                    lift_motor->set_pos(0);
                    sleep(12);
                    lift_motor->shut_down();
                    sleep(0.5);
                }

               block_path_receiver = false;
               geometry_msgs::PoseStamped pose_msg;
         
               pose_msg.header.frame_id = "world";
               pose_msg.header.stamp = ros::Time::now();
               pose_msg.pose.position.x = m_state[0];
               pose_msg.pose.position.y = m_state[1];
               if (block_path_receiver)
                  pose_msg.pose.position.z = 1;
               else
                  pose_msg.pose.position.z = 0;

               pose_msg.pose.orientation.w = cos(m_state[2]/2);
               pose_msg.pose.orientation.x = 0;
               pose_msg.pose.orientation.y = 0;
               pose_msg.pose.orientation.z = sin(m_state[2]/2);
               m_posePub.publish(pose_msg);
               printf("IDLE!\n");
               sleep(1);
               m_move = IDLE;
           }

           break;
        }
    }
        
    vel.first = (vel.first + locomotor->get_vel().first)/2;
    vel.second = (vel.second + locomotor->get_vel().second)/2;

    cycle_period = (ros::Time::now() - cycle_start).toSec();

    m_state[0] = m_state[0] + (-vel.second+vel.first)/2 * cos(m_state[2]) * 0.018849555 * cycle_period/60;
    m_state[1] = m_state[1] + (-vel.second+vel.first)/2 * sin(m_state[2]) * 0.018849555 * cycle_period/60;        
    m_state[2] = m_state[2] + (vel.first + vel.second)/0.653 * 0.018849555 * cycle_period/60;


    if (abs_pose)
    {
        int id;
        id = abs_pose->id - 3;

        Eigen::Quaternion<float> quat;
        quat.w() = abs_pose->pose.pose.orientation.w;
        quat.x() = abs_pose->pose.pose.orientation.x;
        quat.y() = abs_pose->pose.pose.orientation.y;
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

    geometry_msgs::PoseStamped pose_msg;
         
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = m_state[0];
    pose_msg.pose.position.y = m_state[1];
    if (block_path_receiver)
        pose_msg.pose.position.z = 1;
    else
        pose_msg.pose.position.z = 0;

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

    alpha = angle(m_state[2], atan2(delta_y, delta_x));

    beta1 = angle(m_cmd[2]+PI, atan2(delta_y, delta_x));

    alpha1 = angle(m_state[2]+3.1415926, atan2(delta_y, delta_x));

    dist = sqrt(delta_x*delta_x + delta_y* delta_y);

    k = -1/dist * (k2*(alpha-atan(-k1*beta)) + sin(alpha)*(1 + k1/(1+(k1*beta) * (k1*beta))));
    k_back = -1/dist * (k2*(alpha1-atan2(-k1*beta1,1)) + sin(alpha1)*(1 + k1/(1+(k1*beta1) * (k1*beta1))));
}

void 
motion_controller::shutdown()
{
    locomotor->shut_down();
    printf("Shut down motor\n");
}

void 
motion_controller::close(void)
{
    gpio->reset_pin(4);
    gpio->close();
    pcan->reset_device();
    pcan->close_device();
}

/*
bool
motion_controller::reachgoal(void)
{
    return reached;
}
*/
