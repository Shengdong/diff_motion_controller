#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <usb_can_node/AbsolutePose.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "ControlCAN.h"
#include "math.h"
#include "dualmotor.h"
#include "device.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>

class usb_can
{
  public:
    usb_can(void);
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
    geometry_msgs::PoseStamped::ConstPtr m_pose;
    geometry_msgs::PoseStamped::ConstPtr cmd_pose;
    
    double gain = 0;
    double dist = 0;
    double alpha = 0; 
    double beta = 0;
    double v;

    double p3 = 0.6;
    double p4 = 1.6;
    double p5 = -0.3;
    std::pair <int,int> vel;
    ros::Time cycle_start;
    double cycle_period;
    double m_resolution;
};
