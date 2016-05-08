#ifndef pid_control_HPP
#define pid_control_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ardrone_velocity/filtervelocity.hpp"
#include <dynamic_reconfigure/server.h>
#include <velocity_control/dynamic_param_configConfig.h>


class PID_Control
{
public:
    PID_Control();
    ros::NodeHandle nh_;

    // ROS message callbacks
    void InputCallback(const geometry_msgs::Twist& cmd_vel_in);
    void OdoCallback(const nav_msgs::Odometry& odo_msg);
    void PingCallback(const std_msgs::StringConstPtr &ping_msg);

    void dynamic_reconfigure_callback(velocity_control::dynamic_param_configConfig &config, uint32_t level);

    // PID Controller
    void pid_control();
    void run();
    void i_term_set(double &i_value, double error, double vel, double old_vel, double wind_up, double dt);
    void set_hover(void);

private:
    ros::Subscriber cmd_sub;
    ros::Subscriber odo_sub;
    ros::Subscriber ping_sub;
    ros::Publisher cmd_pub;
    ros::Publisher cmd_stamped_pub;
    ros::Publisher ref_vel_pub;

    geometry_msgs::Twist command;
    nav_msgs::Odometry odo;

    ros::Time curTime;
    ros::Time oldTime;

    Eigen::Vector2d command_vec;
    Eigen::Vector2d old_command_vec;
    Eigen::Vector2d old_ref;
    Eigen::Vector2d error_xy;
    Eigen::Vector2d old_error_xy;
    Eigen::Vector4d old_er;
    Eigen::Vector2d vel_xy;
    Eigen::Vector2d old_vel_xy;

    Eigen::Vector3d gain_xy;
    Eigen::Vector2d i_term;
    Eigen::Vector2d wind_up;
    Eigen::Vector2d max_output;
    Eigen::Matrix<double,3,2> Control;
    double beta;

    FilterVelocity filterx;
    FilterVelocity filtery;
    FilterVelocity filtervx;
    FilterVelocity filtervy;

    dynamic_reconfigure::Server<velocity_control::dynamic_param_configConfig> m_server;
    double derv_filter, derv_median, derv_smith;

    ros::Duration navPing;
};


#endif // pid_control
