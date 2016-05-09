ArDrone Velocity - TU Darmstadt Parrot Ar.Drone 2.0 Quadcopter

Introduction
This package is designed to use with parrot ardrone 2.0 and delivers a smooth velocity controller which tackles the problem of time delay due to the unstable wifi communication. 

It is based on two parts. First we have implemented a EKF which uses the ardrone/odometry message to predict the velocity of the ardrone future in time. 
Second we use a modified PID control algorithm to control the ardrone. In addition we adapt our solution dynamically depending on the actual time delay of the system.

Running Nodes
/ekf_localization
/run_control
/ping_node

Installation
Open a terminal and go to your catkin workspace for example catkin_ws or you can create a new workspace. 
Then clone the repository in your /src folder of your workspace and build it using catkin_make command. 
For example: 

Terminal: cd ~/catkin_ws/src
	  git clone https://tobias_t_@bitbucket.org/tobias_t_/ardrone_velocity.git
	  cd ..
	  rosdep install --from-paths src -i
	  catkin_make 

Usage
Terminal: cd /yourPath/ardrone_velocity
	  roslaunch launch/ardrone_velocity.launch

This command will start all nodes simultaneously. 


Detailed Information 

Package: robot_localization
	robot_localization -> ekf_localization
	robot_localizazion -> ping_node

For this purpose the EKF implementation is used and adopted to fit needs of this project. The core modifications are:
1. Usage of the dynamical model of the parrot ardrone instead the default use of a 3 dimensional omnidirectional ground robot which is based on a linear identification of the ardrone.
2. Added the ability to use control inputs of the ardrone for the kalman filter prediction state.
3. Added the ability to deal with delayed measurements without running the kalman filter in a delayed state.
4. Usage of two kalman filter states. The first one is used a the prediction and correction state when all measurements are available. 
   The second state is used for long term ahead prediction.
5. Implementation of a h-step ahead predictor based on a kalman filter and the identificated linear model of the ardrone. 
6. Use of the TcpNoDelay feature of ros to reduce the latency if TCP is used.
7. Dynamical measurement of the actual time delay based on a ping measurement. 

The orignal package can be found here: https://github.com/cra-ros-pkg/robot_localization

Package: velocity_control
	velocity_control -> run_control

Different controller: PID (default)
		      MPID
		      PI_PD

This package is using a modified PID control algorithm to control the velocity of the parrot ardrone and is using the predicted state of the robot_localizazion package.
The parameters of the controller can be changed dynamically. These means a change of parameters during the flight is possible. 
The main modifications are:
1. Using of saturation to limit the output
2. Anti-windup to limit the value of the integral part of the pid controller. 
3. Smith filter for the derivative part. 
4. The derivative part can be based on the error or the velocity. In this use case the error based derivative term delivered better results. 
5. Set point weighting is used to reduce the overshoot.
6. A open loop feedforward controller can be additionally used to accelerate the set point reaching based on calculated tilt angle. 

Package: convert_data
	convert_data -> convert_nav
	convert_data -> vrep_connection

This package can be used for two use cases. First the convert_nav node is designed the case of playing rosbag files. It creates new time stamps for the signals and allows the test case with other nodes which are running in the actual ros time. Without this package huge time gaps can occur. This node also converts navdata to odometry messages and vica versa. The second node vrep_connection is build to manage the communication with vrep. It introduces a time delay for both the incoming and outcoming data of vrep to simulate real world communication delays. 







