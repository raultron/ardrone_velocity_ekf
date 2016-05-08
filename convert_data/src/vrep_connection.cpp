#include <queue>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <ardrone_autonomy/Navdata.h>
#include "tf/transform_datatypes.h"
#include "vrep_connection.h"

// converts odometry message to navdata message
void convert_to_nav(const nav_msgs::Odometry odo)
{
    tf::Quaternion q(odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ardrone_autonomy::Navdata nav;
    nav.header = odo.header;
    nav.rotX = (-1) * roll * (180/3.14);
    nav.rotY = pitch * (180/3.14);
    nav.rotZ = (-1) * yaw * (180/3.14);
    nav.vx = odo.twist.twist.linear.x * 1000;
    nav.vy = (-1) * odo.twist.twist.linear.y * 1000;
    nav.vz = odo.twist.twist.linear.z * 1000;
    nav_pub.publish(nav);
}

// sends command to v-rep and introduces a time delay
void command_to_vrep(ros::Time curTime)
{
    ros::Duration delay_input = ros::Duration(0.050);

    while(twistqueue.top().time_ < curTime - delay_input && !twistqueue.empty())
    {
       twistqueue.pop();
    }
    if(!twistqueue.empty()) cmd_pub.publish(twistqueue.top().control_);
}

// receives odometry and introduces a time delay
void delay_to_rl(ros::Time curTime)
{
    ros::Duration delay_transfer = ros::Duration(0.05);

    while(odomqueue.top().time_ < curTime - delay_transfer && !odomqueue.empty())
    {
       odomqueue.pop();
    }
    if(!odomqueue.empty()) vrep_pub.publish(odomqueue.top().odom_);
    convert_to_nav(odomqueue.top().odom_);
}


void CmdVelCallback(const geometry_msgs::Twist msg)
{
    Twist input;
    ros::Time curTime = ros::Time::now();

    input.control_.angular = msg.angular;
    input.control_.linear = msg.linear;
    input.time_ = curTime;

    twistqueue.push(input);
    if(!twistqueue.empty()) command_to_vrep(curTime);
}

void VrepCallback(const nav_msgs::Odometry msg)
{
    Odom vrep;
    ros::Time curTime = ros::Time::now();

    vrep.odom_.pose = msg.pose;
    vrep.odom_.twist = msg.twist;
    vrep.odom_.header.stamp = curTime;
    vrep.odom_.header.frame_id = "odom";
    vrep.odom_.child_frame_id = "ardrone_base_link";
    vrep.time_ = curTime;

    odomqueue.push(vrep);
    if(!odomqueue.empty()) delay_to_rl(curTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vrep_connection");
    ros::NodeHandle nh_;

    cmd_sub = nh_.subscribe("cmd_vel", 1, CmdVelCallback);
    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel_delay",1);

    vrep_sub = nh_.subscribe("vrep_odom",1, VrepCallback);
    vrep_pub = nh_.advertise<nav_msgs::Odometry>("vrep/odometry", 1);
    nav_pub = nh_.advertise<ardrone_autonomy::Navdata>("vrep/navdata", 1);

    ros::Rate rate(100.0);
    while(nh_.ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
}

