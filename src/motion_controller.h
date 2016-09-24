#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "ControlCAN.h"
#include "math.h"
#include "dualmotor.h"
#include "singlemotor.h"
#include "usb_can_device.h"
#include "usb_gpio_device.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>

class motion_controller
{
  public:
    motion_controller(void);
    bool init(ros::NodeHandle& nh);
    void running(void);
    bool reachgoal(void);
    void close(void);
    void shutdown(void);

    enum MOVE
    {
        FORWARD = 0,
        BACKWARD = 1,
        TURN = 2,
        LIFTFORK =3,
        IDLE = 4,
    };


  private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);
    ros::Subscriber m_pathSub;
    ros::Subscriber m_absposeSub;

    apriltags_ros::AprilTagDetectionArray tag_detections;
    apriltags_ros::AprilTagDetection::ConstPtr abs_pose;

    MOVE m_move;
    Eigen::Vector3d m_state;
    Eigen::Vector3d m_cmd;

    boost::shared_ptr<pcan_device> pcan;
    boost::shared_ptr<dualmotor> locomotor;
    boost::shared_ptr<gpio_io> gpio;
    boost::shared_ptr<singlemotor> lift_motor;
    boost::shared_ptr<singlemotor> telefork_motor;

    ros::Publisher m_posePub;
    ros::Publisher m_diffPub;
    ros::Publisher m_fbPub;

    geometry_msgs::PoseStamped::ConstPtr m_pose;
    nav_msgs::Path::ConstPtr m_path;
    
    bool indicator(double theta_x, double theta_y, double threshold);
    double angle(double goal,double start);
    Eigen::Matrix3f qToRotation(Eigen::Quaternion<float> q);
    double getYawFromMat(Eigen::Matrix3f mat);
    double mod2pi(double angle);
    double distance2d(Eigen::Vector3d point_end, Eigen::Vector3d point_start);

    double lift_param = 0.0;
    int fork_count = 0;

    double gain = 0;
    double dist = 0;
    double alpha = 0; 
    double beta = 0;
    double alpha1 = 0;
    double beta1 = 0;
    double v;
    int p_count = 0;

    bool block_path_receiver;
    std::vector<geometry_msgs::PoseStamped> extracted_path;


    double k_dist = 0.6;

    double k_angle = 0.6;
    double delta_y = 0.0;
    double delta_x = 0.0;

    double k1 = 1.0;
    double k2 = 4.0;
    double gamma = 0.2;
    double k = 0.0;
    double k_back = 0.0;
    double v_max = 0.755;

    std::pair <int,int> vel;
    ros::Time cycle_start;
    double cycle_period;
    double m_resolution;
    int count;
    bool reached;
};
