#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


ros::Subscriber navdata_sub, odomfiltered_sub, cmdvel_sub, odometry_sub;
ros::Publisher odometry_pub, nav_pub, odomfiltered_pub, cmdvel_pub;
geometry_msgs::TwistStamped cmdvel;

// Transform navdata into odometry according to REP-103
// Get a new time stamp
nav_msgs::Odometry CallOdo(const ardrone_autonomy::Navdata &nav)
{
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "ardrone_base_link";
    odom.twist.twist.linear.x  = nav.vx / 1000;
    odom.twist.twist.linear.y  =  (-1)* nav.vy / 1000;
    odom.twist.twist.linear.z  = nav.vz / 1000;
    odom.twist.covariance[0] = 1e-2;
    odom.twist.covariance[7] = 1e-2;
    odom.twist.covariance[14] = 1e-2;

    tf2::Quaternion quat;
    quat.setRPY((-1) * nav.rotX * (3.14/180), nav.rotY * (3.14/180), (-1) * nav.rotZ * (3.14/180));

    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom.pose.pose.position.z = nav.altd;
    odom.pose.covariance[19] = 1e-5;
    odom.pose.covariance[21] = 1e-5;
    odom.pose.covariance[28] = 1e-5;
    odom.pose.covariance[35] = 1e-5;

    return odom;
}

// Callback publishes navdata and odometry message of received ardrone/navdata message
// creates a new time stamp for the message
void NavCallback(const ardrone_autonomy::Navdata &nav)
{
    //publish and run CallOdo
    odometry_pub.publish(CallOdo(nav));

    ardrone_autonomy::Navdata navdata;
    navdata = nav;
    navdata.header.stamp = ros::Time::now();
    nav_pub.publish(navdata);
}

void OdoCallback(const nav_msgs::Odometry &msg)
{
    //publish
    nav_msgs::Odometry odo = msg;
    odo.header.stamp = ros::Time::now();
    odometry_pub.publish(odo);
}

// converts odometry message from robot_localization to navdata message
void RLCallback(const nav_msgs::Odometry rl)
{
    tf::Quaternion q(rl.pose.pose.orientation.x, rl.pose.pose.orientation.y, rl.pose.pose.orientation.z, rl.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ardrone_autonomy::Navdata RL_filter;
    RL_filter.header = rl.header;
    RL_filter.rotX = (-1) * roll * (180/3.14);
    RL_filter.rotY = pitch * (180/3.14);
    RL_filter.rotZ = (-1) * yaw * (180/3.14);
    RL_filter.vx = rl.twist.twist.linear.x * 1000;
    RL_filter.vy = (-1) * rl.twist.twist.linear.y * 1000;
    RL_filter.vz = rl.twist.twist.linear.z * 1000;

    odomfiltered_pub.publish(RL_filter);
}

// creates a new time stamp for velocity command
void CmdVelCallback(const geometry_msgs::Twist msg)
{
    cmdvel.header.stamp = ros::Time::now();
    cmdvel.twist = msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_nav");
    ros::NodeHandle nh_;

   // Subscriber
    odometry_sub = nh_.subscribe("ardrone/odometry", 1, OdoCallback);
    navdata_sub = nh_.subscribe("ardrone/navdata", 1, NavCallback);
    odomfiltered_sub = nh_.subscribe("odometry/prediction", 1, RLCallback);
    cmdvel_sub = nh_.subscribe("cmd_vel", 1, CmdVelCallback);

    navdata_sub = nh_.subscribe("ardrone/navdata", 1, NavCallback);
    odomfiltered_sub = nh_.subscribe("odometry/prediction", 1, RLCallback);
    cmdvel_sub = nh_.subscribe("cmd_vel", 1, CmdVelCallback);

    // Publisher
    odomfiltered_pub = nh_.advertise<ardrone_autonomy::Navdata>("navdata/prediction",1);
    nav_pub = nh_.advertise<nav_msgs::Odometry>("navdata",1);
    odometry_pub = nh_.advertise<nav_msgs::Odometry>("odometry/ardrone",1);
    cmdvel_pub = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1);

    // run at 200 Hz
    // publish cmd_vel command at 100 Hz
    ros::Rate rate(200.0);
    int i = 0;

    while(nh_.ok())
    {
        ros::spinOnce();

        if( i == 2)
        {
            cmdvel_pub.publish(cmdvel);
            i = 0;
        }
        i = i + 1;

        rate.sleep();
    }
}

