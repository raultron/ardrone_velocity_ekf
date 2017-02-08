#include "ardrone_velocity_ekf/coordinate_transform.h"
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
    ros::init(argc, argv, "coordinate_transform");

    if(!ros::isInitialized())
    {
        ros::Time::init();
    }

    CoorTransform trans;

    ros::Rate loop_rate(30);
    while(trans.nh_.ok())
    {
     trans.run();
     loop_rate.sleep();
    }

}
