#ifndef coordinate_transform_HPP
#define coordinate_transform_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ardrone_velocity_ekf/filtervelocity.hpp"
#include <dynamic_reconfigure/server.h>
#include <ardrone_velocity_ekf/dynamic_param_configConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


class CoorTransform
{
public:
    CoorTransform();
    ros::NodeHandle nh_;
    // ROS message callbacks
    void CamCallback(const geometry_msgs::PoseStamped &cam);
    void run();

    ros::Subscriber cam_sub;
    ros::Publisher cam_vel_pub;
    ros::Publisher cam_quad_pub;


    tf::TransformBroadcaster br;
    tf::TransformListener ls;

private:
    geometry_msgs::Twist command;
    nav_msgs::Odometry odo;

    ros::Time curTime;
    ros::Time oldTime;

    Eigen::Vector3d euler;
    Eigen::Vector3d old_euler;

    geometry_msgs::PoseStamped old_quad_pose;
    geometry_msgs::PoseStamped old2_quad_pose;
    geometry_msgs::PoseStamped old3_quad_pose;

    FilterVelocity filterx;
    FilterVelocity filtery;

    ros::Time old, old2;
};


#endif // coordinate_transform
