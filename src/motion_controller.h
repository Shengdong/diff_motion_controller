#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "ControlCAN.h"
#include "math.h"
#include "dualmotor.h"
#include "usb_can_device.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>

class motion_controller
{
  public:
    motion_controller(void);
    bool init(ros::NodeHandle& nh);
    void running(void);

    void close(void);
    void shutdown(void);

    enum MOVE
    {
        FORWARD = 0,
        TURN = 1,
        IDLE = 2,
    };


  private:
    void cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);
    ros::Subscriber m_cmdSub;
    ros::Subscriber m_absposeSub;

    apriltags_ros::AprilTagDetectionArray tag_detections;
    apriltags_ros::AprilTagDetection::ConstPtr abs_pose;

    MOVE m_move;
    Eigen::Vector3d m_state;
    Eigen::Vector3d m_cmd;

    boost::shared_ptr<pcan_device> pcan;
    boost::shared_ptr<dualmotor> dual_motor;

    ros::Publisher m_posePub;
    ros::Publisher m_diffPub;
    ros::Publisher m_fbPub;

    geometry_msgs::PoseStamped::ConstPtr m_pose;
    geometry_msgs::PoseStamped::ConstPtr cmd_pose;
    
    bool indicator(double theta_x, double theta_y, double threshold);
    double angle(double goal,double start);
    Eigen::Matrix3f qToRotation(Eigen::Quaternion<float> q);
    double getYawFromMat(Eigen::Matrix3f mat);
    double mod2pi(double angle);

    double gain = 0;
    double dist = 0;
    double alpha = 0; 
    double beta = 0;
    double v;

    double p3 = 0.5;
    double p4 = 1.5;
    double p5 = -0.3;
    double p_a = 0.6;
    double delta_y = 0.0;
    double delta_x = 0.0;

    double pi = 3.141592654;

    std::pair <int,int> vel;
    ros::Time cycle_start;
    double cycle_period;
    double m_resolution;
    int count;
    bool fix_turn;
    bool fix_forward;
};
