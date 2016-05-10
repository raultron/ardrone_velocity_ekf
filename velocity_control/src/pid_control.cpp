#include "pid_control.h"
#include "math.h"
#include <Eigen/Dense>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "ardrone_velocity/filtervelocity.hpp"

PID_Control::PID_Control()
{
    // Publisher and Subscriber
    ros::NodeHandle params("~");
    std::string s;

    params.param<std::string>("cmd_vel_ref_topic", s, "cmd_vel_ref");
    cmd_sub = nh_.subscribe(s,1, &PID_Control::InputCallback, this);
    params.param<std::string>("odometry_topic", s, "odometry/prediction");
    odo_sub = nh_.subscribe(s,1, &PID_Control::OdoCallback, this, ros::TransportHints().tcpNoDelay());
    ping_sub = nh_.subscribe("ardrone/ping",1, &PID_Control::PingCallback, this);

    params.param<std::string>("cmd_vel_out_topic", s, "/cmd_vel");
    cmd_pub = nh_.advertise<geometry_msgs::Twist>(s, 1);
    params.param<std::string>("cmd_vel_out_topic_stamped", s, "/cmd_vel_stamped");
    cmd_stamped_pub = nh_.advertise<geometry_msgs::TwistStamped>(s, 1);
    ref_vel_pub = nh_.advertise<geometry_msgs::Twist>("ref_vel", 1);

    // Dynamic parameter reconfigure
    dynamic_reconfigure::Server<velocity_control::dynamic_param_configConfig>::CallbackType f;
    f = boost::bind(&PID_Control::dynamic_reconfigure_callback,this, _1, _2);
    m_server.setCallback(f);

    // Default values
    i_term(0) = 0.0;
    i_term(1) = 0.0;

    gain_xy(0) = 0.45;
    gain_xy(1) = 0.15;
    gain_xy(2) = 0.05;

    wind_up(0) = 0.6;
    wind_up(1) = 0.6;

    max_output(0) = 0.5;
    max_output(1) = 0.5;

    beta = 0.9;
    derv_filter = 0;
    derv_median = 0;
    derv_smith  = 1;
}

void PID_Control::PingCallback(const std_msgs::StringConstPtr &ping_msg)
{
    std::string ping_string = ping_msg->data;
    double ping = std::stod (ping_string);
    navPing = ros::Duration(ping*0.001);
}

void PID_Control::InputCallback(const geometry_msgs::Twist& cmd_in)
{
    // Reference velocity
    command = cmd_in;
}

void PID_Control::OdoCallback(const nav_msgs::Odometry& odo_msg)
{
    // Measurement velocity
    odo = odo_msg;
    vel_xy(0) = odo.twist.twist.linear.x;
    vel_xy(1) = odo.twist.twist.linear.y;

    pid_control();
}

bool first_reconfig = true;
void PID_Control::dynamic_reconfigure_callback(velocity_control::dynamic_param_configConfig &config, uint32_t level)
{
    if (first_reconfig)
    {
      first_reconfig = false;
      return;     // Ignore the first call to reconfigure which happens at startup
    }
  gain_xy(0) = config.Kp;
  gain_xy(1) = config.Ki;
  gain_xy(2) = config.Kd;

  wind_up(0) = config.windup;
  wind_up(1) = config.windup;

  beta = config.beta;

  max_output(0) = config.limit_x;
  max_output(1) = config.limit_y;

  derv_filter = config.derv_filter;
  derv_median = config.derv_median;
  derv_smith = config.derv_smith;

  ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", gain_xy(0), gain_xy(1), gain_xy(2));
}

void PID_Control::pid_control()
{
    // Max speed in m/s
    Eigen::Vector2d max;
    max(0) = 0.6;
    max(1) = 0.6;

    command.linear.x = std::min(max(0), std::max(-max(0), command.linear.x));
    command.linear.y = std::min(max(1), std::max(-max(1), command.linear.y));
    command_vec(0) = command.linear.x;
    command_vec(1) = command.linear.y;

    // Error -> P-Term
    error_xy(0) = command.linear.x - vel_xy(0);
    error_xy(1) = command.linear.y - vel_xy(1);

    // Beta is for set point weighting to reduce overshoot
    Control(0,0) = beta*command.linear.x - vel_xy(0); // P-Term x-direction
    Control(0,1)= beta*command.linear.y - vel_xy(1);  // P-Term y-direction

    // Time gap for one spin for derivate and integral part
    curTime = ros::Time::now();
    ros::Duration dt = curTime - oldTime;
    oldTime = curTime;

    // Derivative term (based on error change) -> D-Term
    // Low-pass filtering for smooth derivative part -> adds delay to signal
    Eigen::Vector2d error_derivate;
    Eigen::Vector2d filtered_error;

    filtered_error(0) = filterx.lowpass_filter(error_xy(0));
    filtered_error(1) = filtery.lowpass_filter(error_xy(1));

    error_derivate(0) = (filtered_error(0) - old_error_xy(0))/dt.toSec();
    error_derivate(1) = (filtered_error(1) - old_error_xy(1))/dt.toSec();

    old_error_xy(0) = filtered_error(0);
    old_error_xy(1) = filtered_error(1);



    double median_x;
    double error_nofilter_x  = (error_xy(0) - old_er(0))/dt.toSec();
    filterx.median_filter(error_nofilter_x, median_x);
    old_er(0) = error_xy(0);
    double median_y;
    double error_nofilter_y  = (error_xy(1) - old_er(1))/dt.toSec();
    filtery.median_filter(error_nofilter_y, median_y);
    old_er(1) = error_xy(1);

    double smith_x;
    filterx.smith_filter(2, dt.toSec(), error_xy(0), smith_x);
    double smith_y;
    filtery.smith_filter(2, dt.toSec(), error_xy(1), smith_y);


    // Derivative term (based on veloctiy change) -> D-Term
    // Low-pass filtering for smooth derivative part -> adds delay to signal
    Eigen::Vector2d error_vel_derivate;
    Eigen::Vector2d filtered_vel;
    filtered_vel(0) = filtervx.lowpass_filter(vel_xy(0));
    filtered_vel(1) = filtervy.lowpass_filter(vel_xy(1));

    error_vel_derivate(0) = -(filtered_vel(0) - old_vel_xy(0))/dt.toSec();
    error_vel_derivate(1) = -(filtered_vel(1) - old_vel_xy(1))/dt.toSec();

    old_vel_xy(0) = filtered_vel(0);
    old_vel_xy(1) = filtered_vel(0);

    // Limit Derivative
    max(0) = 1.5;
    max(1) = 1.5;

    error_derivate(0) = derv_filter*error_derivate(0) + derv_median*median_x + derv_smith*smith_x;
    error_derivate(1) = derv_filter*error_derivate(1) + derv_median*median_y + derv_smith*smith_y;


    Control(2,0) = std::min(max(0), std::max(-max(0), error_derivate(0)));  // D-Term x-direction
    Control(2,1) = std::min(max(1), std::max(-max(1), error_derivate(1)));  // D-Term y-direction

    // Intergral part with max values and reset if new reference value is received -> I-Term
    i_term_set(i_term(0), error_xy(0), command_vec(0), old_command_vec(0), wind_up(0), dt.toSec());
    i_term_set(i_term(1), error_xy(1), command_vec(1), old_command_vec(1), wind_up(1), dt.toSec());
    old_command_vec = command_vec;

    Control(1,0) = i_term(0);  // I-Term x-direction
    Control(1,1) = i_term(1);  // I-Term y-direction

    // Set controller to zero around hover state, velocities smaller than 0.01 m/s
    // Internal controller will handle position drift//
    if(command_vec(0) == 0 && command_vec(1) == 0)
    {
        set_hover();
        i_term(0) = 0;
        i_term(1) = 0;
        return;
    }

    if (command_vec(0) == 0)
    {
        i_term(0) = 0;
        Control(1,0)  = 0;
    }
    else if (command_vec(1) == 0)
    {
        i_term(1) = 0;
        Control(1,1) = 0;
    }

    // If no data is available go to hover.
    if (navPing.toSec() >= 0.200)
    {
        set_hover();
        i_term(0) = 0;
        i_term(1) = 0;
        return;
    }

    // Calculate outputs -> tilt angle output_value*12 = ref_tilt_angle
    geometry_msgs::Twist control_output_pid;
    control_output_pid.linear.x = Control(0,0)*gain_xy(0) + Control(1,0)*gain_xy(1) + Control(2,0)*gain_xy(2);
    control_output_pid.linear.y = Control(0,1)*gain_xy(0) + Control(1,1)*gain_xy(1) + Control(2,1)*gain_xy(2);

    // Disable auto-hover on default
    control_output_pid.angular.x = 1;
    control_output_pid.angular.y = 1;

    // Based on u(k)
    control_output_pid.linear.x = std::min(max_output(0), std::max(-max_output(0), control_output_pid.linear.x));
    control_output_pid.linear.y = std::min(max_output(1), std::max(-max_output(1), control_output_pid.linear.y));

    // Debugging information
    Eigen::Vector3d tmp = Control.col(0).cwiseProduct(gain_xy);
    ROS_INFO("d_Time  : %f", dt.toSec());
    ROS_INFO("VelRef: %f , %f", command.linear.x, command.linear.y);
    ROS_INFO("Vel   : %f , %f", odo.twist.twist.linear.x, odo.twist.twist.linear.y);
    ROS_INFO("Error : %f ,  %f", error_xy(0),error_xy(1));
    ROS_INFO("Cmd   : %f , %f", control_output_pid.linear.x,  control_output_pid.linear.y);
    ROS_INFO("Derivatibe Type: FILTER: %f , Median: %f , Smith: %f", derv_filter, derv_median, derv_smith);
    ROS_INFO("Kp | Ki | Kd | MaxOut | WindUp | Beta |   : %f | %f | %f | %f | %f | %f", gain_xy(0), gain_xy(1), gain_xy(2), max_output(0), wind_up(0), beta);
    ROS_INFO("pterm | iterm | dterm   : %f | %f | %f", tmp(0), tmp(1), tmp(2));
    ROS_INFO("------------------------------------------------------");

    // Publish
    cmd_pub.publish(control_output_pid);
    geometry_msgs::TwistStamped stamped_control_output_pid;
    stamped_control_output_pid.twist = control_output_pid;
    stamped_control_output_pid.header.stamp = ros::Time::now();
    cmd_stamped_pub.publish(stamped_control_output_pid);
    ref_vel_pub.publish(command);
}

void PID_Control::set_hover(void)
{
    ROS_INFO("HOVER");

    geometry_msgs::Twist control_output;
    control_output.linear.x = 0;
    control_output.linear.y = 0;
    control_output.linear.z = 0;
    control_output.angular.x = 0;
    control_output.angular.y = 0;
    control_output.angular.z = 0;

    cmd_pub.publish(control_output);
    geometry_msgs::TwistStamped stamped_control_output;
    stamped_control_output.twist = control_output;
    stamped_control_output.header.stamp = ros::Time::now();
    cmd_stamped_pub.publish(stamped_control_output);
    ref_vel_pub.publish(command);
}

void PID_Control::i_term_set(double &i_value, double error, double vel, double old_vel, double wind_up, double dt)
{
    // Reset I-Part to reduce overshoot
    if(error < 0 && i_value > 0)
    {
        i_value = std::max(0.0, i_value + error*dt);
    }

    if(error > 0 && i_value < 0)
    {
        i_value = std::min(0.0, i_value + error*dt);
    }
    else
    {
        i_value = i_value + error*dt;
    }

    // Wind_up to limit I-Part
    if(i_value > wind_up)
    {
        i_value = wind_up;
    }

    if(i_value < - wind_up)
    {
        i_value = - wind_up;
    }

    // Reset I-Part if new reference value is set
//    if( vel != old_vel)
//    {
//        i_value = 0;
//    }
}

void PID_Control::run()
{
    ros::spinOnce();
}



