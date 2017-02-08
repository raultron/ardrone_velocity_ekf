#include "ardrone_velocity_ekf/pid_control.h"

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"


int main(int argc,char* argv[])
{
    ros::init(argc, argv, "run_control");

    if(!ros::isInitialized())
    {
        ros::Time::init();
    }

    PID_Control control;

    ros::Rate loop_rate(100);
    while(control.nh_.ok())
    {
     control.run();
     loop_rate.sleep();
    }

}
