#include "math.h"
#include "ardrone_velocity_ekf/coordinate_transform.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ardrone_velocity_ekf/filtervelocity.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

CoorTransform::CoorTransform()
{
        cam_sub = nh_.subscribe("/ar_usb0/pose", 1, &CoorTransform::CamCallback, this, ros::TransportHints().tcpNoDelay());
        cam_quad_pub = nh_.advertise<geometry_msgs::PoseStamped>("quad_pose", 1);
        cam_vel_pub = nh_.advertise<nav_msgs::Odometry>("ardrone/velocity_groundtruth", 1);
}

void CoorTransform::CamCallback(const geometry_msgs::PoseStamped &cam)
{
    // get pose of drone in world frame
    tf::Quaternion world(cam.pose.orientation.x, cam.pose.orientation.y, cam.pose.orientation.z, cam.pose.orientation.w);
    tf::Matrix3x3 m(world);
    double roll_W , pitch_W, yaw_W;
    m.getRPY(roll_W, pitch_W, yaw_W);

    // Do transformation world frame -> marker
    ros::Time now = ros::Time::now();
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(cam.pose.position.x, cam.pose.position.y, cam.pose.position.z));
    transform.setRotation(tf::Quaternion(cam.pose.orientation.x,cam.pose.orientation.y,cam.pose.orientation.z,cam.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(transform, now, "camera0", "marker"));

    // Do transformation marker -> drone
    tf::Transform transform2;
    transform2.setOrigin(tf::Vector3(0.0, 0.0025, -0.0067));
    tf::Quaternion drone;
    drone.setRPY(0, 0, -M_PI/2);
    transform2.setRotation(drone);
    br.sendTransform(tf::StampedTransform(transform2, now, "marker", "ardrone"));

    // Do transformation world frame -> drone_fixed
    tf::StampedTransform transform_ardrone_camera;
    transform_ardrone_camera.setData(transform*transform2);

    tf::StampedTransform stampedTransform_fixed;
    tf::Transform transform_fixed;
    double ro,pi,ya;
    transform_ardrone_camera.getBasis().getRPY(ro, pi, ya);
    transform_fixed.setOrigin(transform_ardrone_camera.getOrigin());
    transform_fixed.setRotation(tf::createQuaternionFromRPY(M_PI,0.0,ya));

    stampedTransform_fixed.child_frame_id_ = "drone_fixed";
    stampedTransform_fixed.frame_id_ = "camera0";
    stampedTransform_fixed.setData(transform_fixed);
    stampedTransform_fixed.stamp_ = ros::Time::now();
    br.sendTransform(stampedTransform_fixed);

    // Obtain marker position in drone_fixed frame
    tf::Pose quad;
    quad.setOrigin(tf::Vector3(0.,0.,0.));
    quad.setRotation(tf::Quaternion(0,0,0,1));
    quad = stampedTransform_fixed*quad;

    tf::Vector3 quadp = quad.getOrigin();
    tf::Quaternion quadr(quad.getRotation());
    tf::Matrix3x3 r(quadr);

    // publish pose in drone_fixed frame
    geometry_msgs::PoseStamped quad_pose;
    quad_pose.header.frame_id = "drone_fixed";
    quad_pose.header.stamp = ros::Time::now();
    quad_pose.pose.position.x = quadp.getX();
    quad_pose.pose.position.y = quadp.getY();
    quad_pose.pose.position.z = 1; //quadp.getZ(); // We do not use height, therefore we set it equal 1.
    quad_pose.pose.orientation.x = quadr.getX();
    quad_pose.pose.orientation.y = quadr.getY();
    quad_pose.pose.orientation.z = quadr.getZ();
    quad_pose.pose.orientation.w = quadr.getW();

    cam_quad_pub.publish(quad_pose);

    // get orientation in euler angles
    double roll, pitch, yaw;
    r.getRPY(roll, pitch, yaw);
    euler(0) = roll;
    euler(1) = pitch;
    if (yaw < 0)
        yaw = 2*M_PI + yaw;
    euler(2) = yaw;

    // Calculate velocity based on camera position estimation
    double delta_x = quad_pose.pose.position.x - old2_quad_pose.pose.position.x;
    double delta_y = quad_pose.pose.position.y - old2_quad_pose.pose.position.y;
    ros::Duration delta_time = quad_pose.header.stamp - old_quad_pose.header.stamp;

    //    double vx = delta_x/delta_time.toSec();
    //    double vy = delta_y/delta_time.toSec();

    old3_quad_pose = old2_quad_pose;
    old2_quad_pose = old_quad_pose;
    old_quad_pose = quad_pose;

    double vx;
    double vy;
    // Low-Pass filter velocity estimate
    filterx.smith_filter(2, delta_time.toSec(), quad_pose.pose.position.x, vx);
    filtery.smith_filter(2, delta_time.toSec(), quad_pose.pose.position.y, vy);

    double vx_drone = vx*cos(euler(2)) + vy*sin(euler(2));
    double vy_drone = vx*sin(euler(2)) - vy*cos(euler(2));

    // publish velocity
    nav_msgs::Odometry drone_velocity;
    drone_velocity.header.stamp = ros::Time::now();
    drone_velocity.header.frame_id = "odom";
    drone_velocity.child_frame_id = "ardrone_base_link";
    drone_velocity.twist.twist.linear.x = vx_drone;
    drone_velocity.twist.twist.linear.y = vy_drone;
    drone_velocity.twist.covariance[0] = 5e-2;
    drone_velocity.twist.covariance[7] = 5e-2;
    cam_vel_pub.publish(drone_velocity);

    // Debugging Information
//    if ((now - old).toSec() > 1)
//    {
//        old = now;
//        ROS_INFO(" ---------------- Fixed Drone ----------------");
//        ROS_INFO(" Pos: x: %f,  y: %f,  z: %f ", quadp.getX(), quadp.getY(), quadp.getZ());
//        ROS_INFO(" rot: roll: %f,  pitch: %f,  yaw: %f ", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
//        ROS_INFO(" ---------------- Marker ----------------");
//        ROS_INFO(" Pos: x: %f,  y: %f,  z: %f ", cam.pose.position.x,  cam.pose.position.y, cam.pose.position.z);
//        ROS_INFO(" rot: roll: %f,  pitch: %f,  yaw: %f ", roll_W*180/M_PI, pitch_W*180/M_PI, yaw_W*180/M_PI);
//        ROS_INFO(" ---------------- Velocity Drone ----------------");
//        ROS_INFO(" velocity_drone: vx: %f,  vy: %f", vx_drone, vy_drone);
//    }
}


void CoorTransform::run()
{
    ros::spinOnce();
}

