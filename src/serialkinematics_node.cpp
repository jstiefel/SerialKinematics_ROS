//============================================================================
// Name        : serialkinematics_node.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 15.06.2018
// Copyright   : BSD 3-Clause
// Description : Node to control kinematics of robot, initialization of ROS.
//============================================================================

#include <ros/ros.h>
#include "serialkinematics/KinematicsController.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinematics_control");
	ros::NodeHandle nodeHandle("~");
	ros::Rate loop_rate(100); //frequency in Hz

	//create class object "KinematicsController", which coordinates all the subscriber/publisher actions
	serialkinematics::KinematicsController KinematicsController(nodeHandle);

	while (ros::ok()){
		KinematicsController.publisher_loop();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("Node shut down.");

	return 0;
}
