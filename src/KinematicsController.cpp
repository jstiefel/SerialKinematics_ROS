//============================================================================
// Name        : KinematicsController.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 18.06.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the overall kinematics control of robot.
//				 (subscribers, parameters, publishers, etc.)
//============================================================================

//units in mm and deg (except for KDL)
#include "serialkinematics/KinematicsController.hpp"

namespace serialkinematics {

KinematicsController::KinematicsController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  //define robot geometry
  or_robot.defineRobot(z_height_, x_offset_, d_offset_);

  //services and topics of this package:
  //ROS publisher containing all state informations
  global_state_publisher_ = nodeHandle_.advertise<serialkinematics::global_state_info>("global_state_topic", 1);
  //ROS service server to control joint and end-effector coordinates
  joint_pos_service_ = nodeHandle_.advertiseService("joint_pos_service", &KinematicsController::jointMoveCallback, this);
  ee_pos_service_ = nodeHandle_.advertiseService("ee_pos_service", &KinematicsController::eeMoveCallback, this);

  //access services and topics from other packages
  //ROS service clients to send joint positions
  thorlabs_y_client_ = nodeHandle_.serviceClient<tdc_controller::tdc_motor_service>("/thorlabs_y/thorlabs_set_pos");
  thorlabs_x_client_ = nodeHandle_.serviceClient<tdc_controller::tdc_motor_service>("/thorlabs_x/thorlabs_set_pos");
  thorlabs_alpha_client_ = nodeHandle_.serviceClient<tdc_controller::tdc_motor_service>("/thorlabs_alpha/thorlabs_set_pos");
  stepper_beta_client_ = nodeHandle_.serviceClient<custom_communication::TargetAbsPos>("/stepper_abs_pos_srv");
  maxon_d_client_ = nodeHandle_.serviceClient<epos2_controller::epos_motor_service>("/maxon/epos_control_service");
  //ROS state subscribers to get joint positions
  thorlabs_y_subscriber_ = nodeHandle_.subscribe("/thorlabs_y/motor_info_topic", 1, &KinematicsController::yTopicCallback, this);
  thorlabs_x_subscriber_ = nodeHandle_.subscribe("/thorlabs_x/motor_info_topic", 1, &KinematicsController::xTopicCallback, this);
  thorlabs_alpha_subscriber_ = nodeHandle_.subscribe("/thorlabs_alpha/motor_info_topic", 1, &KinematicsController::alphaTopicCallback, this);
  stepper_beta_subscriber_ = nodeHandle_.subscribe("/joint_state", 1, &KinematicsController::betaTopicCallback, this);
  maxon_d_subscriber_ = nodeHandle_.subscribe("/maxon/epos_info_topic", 1, &KinematicsController::dTopicCallback, this);


  ROS_INFO("Successfully launched Kinematics Controller node.");
}

KinematicsController::~KinematicsController()
{

}

bool KinematicsController::readParameters()
{
  if (!nodeHandle_.getParam("robot_geometry/z_height", z_height_)) return false;
  if (!nodeHandle_.getParam("robot_geometry/x_offset", x_offset_)) return false;
  if (!nodeHandle_.getParam("robot_geometry/d_offset", d_offset_)) return false;

  return true;
}

bool KinematicsController::jointMoveCallback(serialkinematics::joint_pos_service::Request& request, serialkinematics::joint_pos_service::Response& response)
{
	
	qTargetJoints[0] = request.y_joint; //in mm
	qTargetJoints[1] = request.x_joint;
	qTargetJoints[2] = request.alpha_joint; //in deg
	qTargetJoints[3] = request.beta_joint;
	qTargetJoints[4] = request.d_joint;

	executeMove(qTargetJoints);
	//call executeMove(q_joints)

	response.success = true;
	
	return true;
}

bool KinematicsController::eeMoveCallback(serialkinematics::ee_pos_service::Request& request, serialkinematics::ee_pos_service::Response& response)
{
	//calculate q_joints
	//call executeMove(q_joints)

	// to find a solution, only input possible values
	/*
	 * Attention!: It is not easy to find valid values by hand because of the joint limits!
	 * Error "-3" often means that limits are too tight.
	 *
	 *
	 * Input as mm and deg
	 *
	 */

	float eeCartPos[5] = {0};

	eeCartPos[0] = (request.x_ee)/1000.; //x in mm
	eeCartPos[1] = (request.y_ee)/1000.; //y
	eeCartPos[2] = (request.z_ee)/1000.; //z
	eeCartPos[3] = degToRad(request.phi_ee); //EulerZ in deg
	eeCartPos[4] = degToRad(request.theta_ee); //EulerY

	//qTargetJoints is global available
	float preqTargetJoints[5] = {0}; //don't write to qTargetJoints before knowing if solution is possible
	int status = or_robot.solveIkPos(eeCartPos, preqTargetJoints);

	if(status>=0){
		//convert q to deg and mm as usual:
		qTargetJoints[0] = preqTargetJoints[0]*1000;
		qTargetJoints[1] = preqTargetJoints[1]*1000;
		qTargetJoints[2] = radToDeg(preqTargetJoints[2]);
		qTargetJoints[3] = radToDeg(preqTargetJoints[3]);
		qTargetJoints[4] = preqTargetJoints[4]*1000;

		executeMove(qTargetJoints);

		response.success = true;
	}
	else{
		response.success = false;
		ROS_ERROR("No solution found for inverse kinematics.");
	}
	
	return true;
}

bool KinematicsController::executeMove(float* q)
{
	//call all services to execute move

	//save all q to corresponding service content
	y_service_.request.position_setpoint = q[0];
	x_service_.request.position_setpoint = q[1];
	alpha_service_.request.position_setpoint = q[2];
	beta_service_.request.target_abs_position = q[3];
	d_service_.request.position_setpoint = q[4];

	if (thorlabs_y_client_.call(y_service_)){
		ROS_INFO_STREAM("y service successfully called. Success: " << (int)y_service_.response.success);
	} else {
		ROS_ERROR("Failed to call service thorlabs_y_client");
		return false;
	}

	if (thorlabs_x_client_.call(x_service_)){
		ROS_INFO_STREAM("x service successfully called. Success: " << (int)x_service_.response.success);
	} else {
		ROS_ERROR("Failed to call service thorlabs_x_client");
		return false;
	}

	if (thorlabs_alpha_client_.call(alpha_service_)){
		ROS_INFO_STREAM("alpha service successfully called. Success: " << (int)alpha_service_.response.success);
	} else {
		ROS_ERROR("Failed to call service thorlabs_alpha_client");
		return false;
	}

	if (stepper_beta_client_.call(beta_service_)){
		ROS_INFO_STREAM("beta service successfully called.");
	} else {
		ROS_ERROR("Failed to call service stepper_beta_client");
		return false;
	}

	if (maxon_d_client_.call(d_service_)){
		ROS_INFO_STREAM("d service successfully called. Success: " << (int)d_service_.response.success);
	} else {
		ROS_ERROR("Failed to call service maxon_d_client");
		return false;
	}

	return true;
}

//Save positions to global variable
void KinematicsController::yTopicCallback(const tdc_controller::tdc_motor_info::ConstPtr& msg){
	qJoints[0] = msg->position;
}

void KinematicsController::xTopicCallback(const tdc_controller::tdc_motor_info::ConstPtr& msg){
	qJoints[1] = msg->position;
}

void KinematicsController::alphaTopicCallback(const tdc_controller::tdc_motor_info::ConstPtr& msg){
	qJoints[2] = msg->position;
}

void KinematicsController::betaTopicCallback(const std_msgs::Float32::ConstPtr& msg){
	qJoints[3] = msg->data;
}

void KinematicsController::dTopicCallback(const epos2_controller::epos_motor_info::ConstPtr& msg){
	qJoints[4] = msg->position;
}

void KinematicsController::publisher_loop(){
	global_joint_state_.y_joint = qJoints[0]; //in mm
	global_joint_state_.x_joint = qJoints[1];
	global_joint_state_.alpha_joint = qJoints[2];
	global_joint_state_.beta_joint = qJoints[3];
	global_joint_state_.d_joint = qJoints[4];

	//calculate forward kinematic solution only for position:
	float ee_pos[3] = {0};
	float preq_ee_pos[3] = {0};
	int status = or_robot.solveFkPos(qJoints, preq_ee_pos); //input in mm and deg, output in mm and deg
	if (status>=0){
		ee_pos[0] = preq_ee_pos[0];
		ee_pos[1] = preq_ee_pos[1];
		ee_pos[2] = preq_ee_pos[2];
	}

	global_joint_state_.x_ee = ee_pos[0];
	global_joint_state_.y_ee = ee_pos[1];
	global_joint_state_.z_ee = ee_pos[2];

	global_state_publisher_.publish(global_joint_state_);
}

float KinematicsController::radToDeg(float rad){
	float deg = rad * (180./pi);
	return deg;
}

float KinematicsController::degToRad(float deg){
	float rad = deg * (pi/180.);
	return rad;
}

} /* namespace */
