ArDrone Velocity - TU Darmstadt Parrot Ar.Drone 2.0 Quadcopter

Introduction
This package is designed to use with parrot ardrone 2.0 and delivers a smooth velocity controller which tackles the problem of time delay due to the unstable wifi communication. It is based on two parts. First we have implemented a EKF which uses the ardrone/odometry package to predict the velocity of the ardrone future in time. Second we use a modified PID control algorithm to control the ardrone. In addition we adapt our solution dynamically depending on the actual time delay of the system.

Running Nodes
/ekf_localization
/run_control
/ping_node


