//============================================================================
// Name        : KinematicsController.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 18.06.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the overall kinematics control of the robot.
//============================================================================

#pragma once

#ifndef KINEMATICSCONTROLLER_H_
#define KINEMATICSCONTROLLER_H_

#include <ros/ros.h>
#include "serialkinematics/global_state_info.h"
#include "serialkinematics/joint_pos_service.h"
#include "serialkinematics/ee_pos_service.h"
#include "tdc_controller/tdc_motor_service.h"
#include "custom_communication/TargetAbsPos.h"
#include "epos2_controller/epos_motor_service.h"
#include "serialkinematics/InverseKinematics.hpp"
#include <std_msgs/Float32.h>
#include "tdc_controller/tdc_motor_info.h"
#include "epos2_controller/epos_motor_info.h"

namespace serialkinematics {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class KinematicsController
{
public:
	/*!
	* Constructor.
	* @param nodeHandle the ROS node handle.
	*/
	KinematicsController(ros::NodeHandle& nodeHandle);

	/*!
	* Destructor.
	*/
	virtual ~KinematicsController();

	void publisher_loop();

private:
	/*!
	* Reads and verifies the ROS parameters.
	* @return true if successful.
	*/
	bool readParameters();
	bool jointMoveCallback(serialkinematics::joint_pos_service::Request& request, serialkinematics::joint_pos_service::Response& response);
	bool eeMoveCallback(serialkinematics::ee_pos_service::Request& request, serialkinematics::ee_pos_service::Response& response);
	bool executeMove(float* q);
	void yTopicCallback(const tdc_controller::tdc_motor_info::ConstPtr& msg);
	void xTopicCallback(const tdc_controller::tdc_motor_info::ConstPtr& msg);
	void alphaTopicCallback(const tdc_controller::tdc_motor_info::ConstPtr& msg);
	void betaTopicCallback(const std_msgs::Float32::ConstPtr& msg);
	void dTopicCallback(const epos2_controller::epos_motor_info::ConstPtr& msg);
	float radToDeg(float rad);
	float degToRad(float deg);
	float z_height_;
	float x_offset_;
	float d_offset_;
	float qTargetJoints[5] = {0};
	float qJoints[5] = {0};
	const double pi = 3.14159265358979323846;

	// Create robot object
	InverseKinematics or_robot;

	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;
	//! ROS topic publisher
	ros::Publisher global_state_publisher_;
	//! ROS variable for publishing global state
	serialkinematics::global_state_info global_joint_state_;
	//! ROS service server
	ros::ServiceServer joint_pos_service_;
	ros::ServiceServer ee_pos_service_;
	//! ROS service clients for position:
	ros::ServiceClient thorlabs_y_client_;
	ros::ServiceClient thorlabs_x_client_;
	ros::ServiceClient thorlabs_alpha_client_;
	ros::ServiceClient stepper_beta_client_;
	ros::ServiceClient maxon_d_client_;
	//! ROS service contents:
	tdc_controller::tdc_motor_service y_service_;
	tdc_controller::tdc_motor_service x_service_;
	tdc_controller::tdc_motor_service alpha_service_;
	custom_communication::TargetAbsPos beta_service_;
	epos2_controller::epos_motor_service d_service_;
	custom_communication::TargetAbsPos syringe_position_;
	//! ROS Subscribers to the state topics:
	ros::Subscriber thorlabs_y_subscriber_;
	ros::Subscriber thorlabs_x_subscriber_;
	ros::Subscriber thorlabs_alpha_subscriber_;
	ros::Subscriber stepper_beta_subscriber_;
	ros::Subscriber maxon_d_subscriber_;

}; /* Class */

#endif /* KINEMATICSCONTROLLER_H_ */

} /*namespace */
