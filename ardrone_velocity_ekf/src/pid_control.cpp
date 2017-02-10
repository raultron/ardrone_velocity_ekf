#include "ardrone_velocity_ekf/pid_control.h"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ardrone_velocity_ekf/filtervelocity.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

PID_Control::PID_Control()
{
    // Publisher and Subscriber
    ros::NodeHandle params("~");
    std::string s1, s2, s3, s4, s5;

    //Param
    params.param<std::string>("cmd_vel_ref_topic", s1, "cmd_vel_ref");
    params.param<std::string>("odometry_topic", s2, "odometry/prediction");
    params.param<std::string>("cmd_vel_out_topic", s3, "cmd_vel");
    params.param<std::string>("WLAN_PING", s4, "ardrone/ping");
    params.param<std::string>("Ext_quad_pose", s5, "/quad_pose");

    //subscribe
    cmd_sub = nh_.subscribe(s1, 1, &PID_Control::InputCallback, this, ros::TransportHints().tcpNoDelay());
    odo_sub = nh_.subscribe(s2, 1, &PID_Control::OdoCallback, this, ros::TransportHints().tcpNoDelay());
    ping_sub = nh_.subscribe(s4, 1, &PID_Control::PingCallback, this);
    cam_sub = nh_.subscribe(s5, 1, &PID_Control::CamCallback,this, ros::TransportHints().tcpNoDelay());

    //Publish
    cmd_pub = nh_.advertise<geometry_msgs::Twist>(s3, 1);


    // Dynamic parameter reconfigure
    dynamic_reconfigure::Server<ardrone_velocity_ekf::dynamic_param_configConfig>::CallbackType f;
    f = boost::bind(&PID_Control::dynamic_reconfigure_callback, this, _1, _2);
    m_server.setCallback(f);

    // Default values
    i_term(0) = 0.0;
    i_term(1) = 0.0;

    gain_xy(0) = 0.3;
    gain_xy(1) = 0.07;
    gain_xy(2) = 0.01;

    wind_up(0) = 0.6;
    wind_up(1) = 0.6;

    max_output(0) = 0.5;
    max_output(1) = 0.5;

    beta = 0.9;
    gain_yaw = 0.0; //0.4

    derv_filter = 0;
    derv_median = 0;
    derv_smith = 1;
    er = 0;
    vl = 1;
}

void PID_Control::PingCallback(const std_msgs::StringConstPtr &ping_msg)
{
    std::string ping_string = ping_msg->data;
    double ping = std::stod (ping_string);
    navPing = ros::Duration(ping*0.001);
}

void PID_Control::InputCallback(const geometry_msgs::Twist& cmd_in)
{
    // reference velocity
    command = cmd_in;
}

void PID_Control::CamCallback(const geometry_msgs::PoseStamped &cam)
{
    // get pose of drone in world frame
    tf::Quaternion world(cam.pose.orientation.x, cam.pose.orientation.y, cam.pose.orientation.z, cam.pose.orientation.w);
    tf::Matrix3x3 m(world);
    double roll , pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    euler(0) = roll;
    euler(1) = pitch;
    if (yaw < 0)
        yaw = 2*M_PI + yaw;
    euler(2) = yaw;
}



void PID_Control::OdoCallback(const nav_msgs::Odometry& odo_msg)
{
    // measurement velocity
    odo = odo_msg;
    vel_xy(0) = odo.twist.twist.linear.x;
    vel_xy(1) = odo.twist.twist.linear.y;

    pid_control(); // run PID control if new velocity is received
}

bool first_reconfig = true;
void PID_Control::dynamic_reconfigure_callback(ardrone_velocity_ekf::dynamic_param_configConfig &config, uint32_t level)
{
//    if (first_reconfig)
//    {
//      first_reconfig = false;
//      return;     // Ignore the first call to reconfigure which happens at startup
//    }
  gain_xy(0) = config.Kp;
  gain_xy(1) = config.Ki;
  gain_xy(2) = config.Kd;

  wind_up(0) = config.windup;
  wind_up(1) = config.windup;

  beta = config.beta;
  gain_yaw = config.Kp_yaw;

  max_output(0) = config.limit_x;
  max_output(1) = config.limit_y;

  derv_filter = config.derv_filter;
  derv_median = config.derv_median;
  derv_smith = config.derv_smith;

  er = config.derv_error;
  vl = config.derv_vel;

  ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f ", gain_xy(0), gain_xy(1), gain_xy(2));
  ROS_INFO("Pid reconfigure request: Kp_yaw angle: %f ", gain_yaw);

}

void PID_Control::pid_control()
{
    geometry_msgs::Twist control_output_pid;
    // max speed in m/s
    Eigen::Vector2d max;
    max(0) = 0.6;
    max(1) = 0.6;

    command.linear.x = std::min(max(0), std::max(-max(0), command.linear.x));
    command.linear.y = std::min(max(1), std::max(-max(1), command.linear.y));
    command_vec(0) = command.linear.x;
    command_vec(1) = command.linear.y;

    control_output_pid = command;

    // Error -> P-Term
    error_xy(0) = command.linear.x - vel_xy(0);
    error_xy(1) = command.linear.y - vel_xy(1);

    // beta is for set point weighting to reduce overshoot
    Control(0,0) = beta*command.linear.x - vel_xy(0); // P-Term x-direction
    Control(0,1)= beta*command.linear.y - vel_xy(1);  // P-Term y-direction

    // Time gap for one spin for derivate and integral part
    curTime = ros::Time::now();
    ros::Duration dt = curTime - oldTime;
    oldTime = curTime;

    // Derivative term (based on error change) -> D-Term
    // Low-pass filtering for smooth derivative part -> adds delay to signal
    Eigen::Vector2d error_derivative;
    Eigen::Vector2d error_err;

    error_err(0) = (error_xy(0) - old_er(0));
    error_err(1) = (error_xy(1) - old_er(1));
    error_derivative(0) = filterx.filter(error_err(0)/dt.toSec());
    error_derivative(1) = filtery.filter(error_err(1)/dt.toSec());

    old_er(0) = error_xy(0);
    old_er(1) = error_xy(1);

    double smith_x;
    double smith_y;
    filterx.smith_filter(2, dt.toSec(), error_xy(0), smith_x);
    filtery.smith_filter(2, dt.toSec(), error_xy(1), smith_y);

    // Derivative term (based on veloctiy change) -> D-Term
    // Low-pass filtering for smooth derivative part -> adds delay to signal
    Eigen::Vector2d error_vel;

    error_vel(0) = (vel_xy(0) - old_vel_xy(0));
    error_vel(1) = (vel_xy(1) - old_vel_xy(1));

    double smith_vx;
    double smith_vy;
    filtervx.smith_filter(2, dt.toSec(), vel_xy(0), smith_vx);
    filtervy.smith_filter(2, dt.toSec(), vel_xy(1), smith_vy);

    Eigen::Vector2d error_vel_derivative;
    error_vel_derivative(0) = filtervx.filter(error_vel(0)/dt.toSec());
    error_vel_derivative(1) = filtervy.filter(error_vel(1)/dt.toSec());

    old_vel_xy(0) = vel_xy(0);
    old_vel_xy(1) = vel_xy(1);

    // Limit Derivative
    max(0) = 1.5;
    max(1) = 1.5;

    Eigen::Vector2d derivative_error;
    derivative_error(0) = derv_filter*error_derivative(0) + derv_smith*smith_x;
    derivative_error(1) = derv_filter*error_derivative(1) + derv_smith*smith_y;

    Eigen::Vector2d derivative_vel;
    derivative_vel(0) = -derv_filter*error_vel_derivative(0) - derv_smith*smith_vx;
    derivative_vel(1) = -derv_filter*error_vel_derivative(1) - derv_smith*smith_vy;

    Control(2,0) = std::min(max(0), std::max(-max(0), vl*derivative_vel(0)) + er*derivative_error(0));  // D-Term x-direction
    Control(2,1) = std::min(max(1), std::max(-max(1), vl*derivative_vel(1)) + er*derivative_error(1));  // D-Term y-direction

    // Intergral part with max values and reset if new reference value is received -> I-Term
    i_term_set(i_term(0), error_xy(0), command_vec(0), old_command_vec(0), wind_up(0), dt.toSec());
    i_term_set(i_term(1), error_xy(1), command_vec(1), old_command_vec(1), wind_up(1), dt.toSec());
    old_command_vec = command_vec;

    Control(1,0) = i_term(0);  // I-Term x-direction
    Control(1,1) = i_term(1);  // I-Term y-direction


    // Yaw angle control

    // Derivative of yaw -> yaw measurement is suspended drift over time
    double dyaw = 0;
    double delta_yaw = euler(2) - old_euler(2);
    filter_yaw.smith_filter(2, dt.toSec(), delta_yaw, dyaw);
    old_euler = euler;
    dyaw = -dyaw;

    // all values are positiv
    double yaw;
    if (euler(2) < 0)
        yaw = 2*M_PI + euler(2);
    else
        yaw = euler(2);

    double yaw_ref;
    if (command.angular.z < 0)
        yaw_ref = 2*M_PI  + command.angular.z;
    else
        yaw_ref = command.angular.z;

    double yaw_error = yaw - yaw_ref;
    if (yaw_error > M_PI)
        yaw_error = -(2*M_PI - yaw_error);
    else if (yaw_error < -M_PI)
        yaw_error = 2*M_PI + yaw_error;

    if (fabs(yaw_error*180/M_PI) < 5)
        yaw_error = 0;

    // Set controller to zero around hover state, when 0 velocity required.
     if(command.linear.x == 0 && command.linear.y == 0 && command.angular.z == 0)
     {
         set_hover();
         i_term(0) = 0;
         i_term(1) = 0;
         return;
     }

    // If delay is to high then go to hover.
    if (navPing.toSec() >= 0.200)
    {
        set_hover();
        i_term(0) = 0;
        i_term(1) = 0;
        return;
    }




    // Calculate outputs -> tilt angle output_value*12 = ref_tilt_angle

    control_output_pid.linear.x = Control(0,0)*gain_xy(0) + Control(1,0)*gain_xy(1) + Control(2,0)*gain_xy(2);
    control_output_pid.linear.y = Control(0,1)*gain_xy(0) + Control(1,1)*gain_xy(1) + Control(2,1)*gain_xy(2);


    // Based on u(k)
    control_output_pid.linear.x = std::min(max_output(0), std::max(-max_output(0), control_output_pid.linear.x));
    control_output_pid.linear.y = std::min(max_output(1), std::max(-max_output(1), control_output_pid.linear.y));


    // We dont want to hover unless is required.
    control_output_pid.angular.x = 1;
    control_output_pid.angular.y = 1;


//    // Debugging information
//    ros::Time now = ros::Time::now();
//    if ((now - old2).toSec() > 5)
//    {
//        old2 = now;
//        Eigen::Vector3d tmp = Control.col(0).cwiseProduct(gain_xy);
//        ROS_INFO("------------------------------------------------------");
//        ROS_INFO("d_Time  : %f", dt.toSec());
//        ROS_INFO("VelRef: %f , %f", command.linear.x, command.linear.y);
//        ROS_INFO("Vel   : %f , %f", odo.twist.twist.linear.x, odo.twist.twist.linear.y);
//        ROS_INFO("Error : %f ,  %f", error_xy(0),error_xy(1));
//        ROS_INFO("pterm | iterm | dterm   : %f | %f | %f", tmp(0), tmp(1), tmp(2));
//        ROS_INFO("Cmd   : %f , %f", control_output_pid.linear.x,  control_output_pid.linear.y);
//        ROS_INFO("Ref Yaw: %f,  Yaw: %f,  DYaw: %f,  YawRate: %f", command.angular.z*180/M_PI, euler(2)*180/M_PI,yaw_error,yaw_error*gain_yaw);
//    }

    // Publish
    cmd_pub.publish(control_output_pid);
}

void PID_Control::set_hover(void)
{
//    ROS_INFO("HOVER");

    geometry_msgs::Twist control_output;
    control_output.linear.x = 0;
    control_output.linear.y = 0;
    control_output.linear.z = 0;
    control_output.angular.x = 0;
    control_output.angular.y = 0;
    control_output.angular.z = 0;

    cmd_pub.publish(control_output);
}

void PID_Control::i_term_set(double &i_value, double error, double vel, double old_vel, double wind_up, double dt)
{
    // reset I-Part to reduce overshoot
    if(error < 0 && i_value > 0)
    {
        i_value = std::max(0.0, i_value + 5.5*error*dt);
    }

    if(error > 0 && i_value < 0)
    {
        i_value = std::min(0.0, i_value + 5.5*error*dt);
    }
    if(abs(error) < 0.15)
    {
        i_value = i_value + error*dt;
    }

    // wind_up to limit I-Part
    if(i_value > wind_up)
    {
        i_value = wind_up;
    }

    if(i_value < - wind_up)
    {
        i_value = - wind_up;
    }

    // reset I-Part if  reference value changes sign
    if( (vel > 0 && old_vel < 0) || (vel < 0 && old_vel > 0))
    {
        i_value = 0;
    }
}

void PID_Control::run()
{
    ros::spinOnce();
}
