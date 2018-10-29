//============================================================================
// Name        : InverseKinematics.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 18.06.2018
// Copyright   : BSD 3-Clause
// Description : Class implementing the inverse kinematics of our particular
//				 robot model.
//============================================================================

#pragma once

#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>

namespace serialkinematics {

/*!
 * Class containing the model and inverse kinematics solver. Orocos KDL is used.
 */

class InverseKinematics
{
public:
	/*!
	 * Constructor.
	 */
	InverseKinematics();

	/*!
	 * Destructor.
	 */
	virtual ~InverseKinematics();

	void defineRobot(float z_height, float x_offset, float d_offset);
	int solveFkPos(const float* qJoints, float* eeCartPos);
	int solveIkPos(const float eeCartPos[5], float qJoints[5]);


private:
	//Creation of the kinematic chain:
	KDL::Chain kdlChain;
	const double pi = 3.14159265358979323846;


}; /* Class */

#endif /* INVERSEKINEMATICS_H_ */

} /* namespace */
