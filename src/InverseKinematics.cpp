//============================================================================
// Name        : InverseKinematics.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 18.06.2018
// Copyright   : BSD 3-Clause
// Description : Class implementing the inverse kinematics of our particular
//				 robot model.
//============================================================================

#include "serialkinematics/InverseKinematics.hpp"

namespace serialkinematics {

InverseKinematics::InverseKinematics()
{

}

InverseKinematics::~InverseKinematics()
{

}

void InverseKinematics::defineRobot(float z_height, float x_offset, float d_offset)
{
	/* Defines robot model/ coordinate system transformations according to OR model
	 * - z_height is height of intercept of axes alpha and x_offset direction (should be 200mm)
	 * - d_offset is offset to smallest retraction of arm to needle tip (loaded syringe, magnet turned on)
	 * - x_offset is length of arm x (should be 100mm)
	 * - rotation of x,y,z is same as base frame
	 */


	//Base frame is at intersection of lot of rotational z-axis and x-y-base-plate. Eye coordinates with respect to this frame.

	//base frame/frame0 to frame1 (attached to link1 (basis))
	kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, z_height))));
	//frame1 to frame2 (attached to link2)
	kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
	//frame2 to frame3 (attached to link3)
	kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
	//frame3 to frame4 (attached to link4)
	kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RotZ(1.5708), KDL::Vector(0.0, x_offset, 0.0))));
	//frame4 to frame5 (attached to link5)
	kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, d_offset))));
	//frame5 to frame6 (attached to link6)
	kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
}

int InverseKinematics::solveFkPos(const float* qJoints, float* eeCartPos)
{
	/*
	 * Solve forward kinematics problem for same model, input/ output in mm/deg
	 */

	KDL::JntArray q(kdlChain.getNrOfJoints());
	unsigned int nj = kdlChain.getNrOfJoints();
    // Create the frame that will contain the results
    KDL::Frame cartpos;

	//Creation of the solver:
	KDL::ChainFkSolverPos_recursive fksolver(kdlChain);

	//copy input to q in right units
	q(0) = (double)(qJoints[0])/1000.;
	q(1) = (double)(qJoints[1])/1000.;
	q(2) = (double)(qJoints[2]) * (pi/180.);
	q(3) = (double)(qJoints[3]) * (pi/180.);
	q(4) = (double)(qJoints[4])/1000.;

    // Calculate forward position kinematics
    int status;
    status = fksolver.JntToCart(q,cartpos);
    if(status>=0){
    	KDL::Vector position;
    	position = cartpos.p;
    	KDL::Rotation rotation;
    	rotation = cartpos.M;

    	eeCartPos[0] = (float)1000*(position.data[0]); //x
    	eeCartPos[1] = (float)1000*(position.data[1]); //y
    	eeCartPos[2] = (float)1000*(position.data[2]); //z

    	//rotation gives a 3x3 matrix. Can not be easily computated to EulerZYX angles... multiple solutions possible

	}
    else{
    	ROS_ERROR("No success solving Forward Kinematics.");
    }

    return status;
}

int InverseKinematics::solveIkPos(const float eeCartPos[5], float qJoints[5])
{
	/*
	 * solveIkPos function takes eeCartPos[5] as input, calculates qJoints[5] and return status.
	 * - eeCartPos as cartesian translation and EulerZYX rotation: x, y, z, phi (azimuthal), theta (polar angle)
	 * 		(according to EulerZYX remaining angle can not be set for our configuration to find solutions)
	 * - qJoints as generalized coordinates of joint positions: y, x, alpha, beta, d
	 */


	ROS_INFO_STREAM("Input: " << eeCartPos[0] << " " << eeCartPos[1] << " " << eeCartPos[2] << " " << eeCartPos[3] << " " << eeCartPos[4]);

	//Creation of joint arrays q:
	KDL::JntArray q(kdlChain.getNrOfJoints());
	KDL::JntArray q_init(kdlChain.getNrOfJoints());
	KDL::JntArray q_min(kdlChain.getNrOfJoints());
	KDL::JntArray q_max(kdlChain.getNrOfJoints());
	KDL::SetToZero(q_init); //initialize with zero positions
	//define q_init:
	q_init(0) = 0; //y
	q_init(1) = 0; //x
	q_init(2) = 0; //alpha
	q_init(3) = 0; //beta
	q_init(4) = 0; //d
	unsigned int nj = kdlChain.getNrOfJoints();
	ROS_INFO_STREAM("Number of generalized coordinates: " << nj);

	//define minimum and maximum joint positions, measure this values exactly!
	q_min(0) = 0.0; //y
	q_max(0) = 0.049;
	q_min(1) = 0.0; //x
	q_max(1) = 0.049;
	q_min(2) = -0.57; //alpha
	q_max(2) = 0.78;
	q_min(3) = -0.87; //beta
	q_max(3) = 0.76;
	q_min(4) = -0.045; //d
	q_max(4) = 0;


	//Creation of the solver:
	KDL::ChainFkSolverPos_recursive fksolver(kdlChain); //Forward kinematic position solver needed for IK
	KDL::ChainIkSolverVel_pinv iksolverv(kdlChain); //Inverse velocity solver needed for IK
	//KDL::ChainIkSolverPos_NR iksolver(kdlChain, fksolver, iksolverv, 100, 1e-6); //max 100 iterations, stop at accuracy 1e-6
	KDL::ChainIkSolverPos_NR_JL iksolver(kdlChain, q_min, q_max, fksolver, iksolverv, 100, 1e-6);

	//make calculations:

	//define target frame (rotation&translation)
	//rotation is in Tait-Bryan angles/ZYX Euler
	KDL::Frame F_dest(KDL::Rotation::EulerZYX(eeCartPos[3], eeCartPos[4], 0.0), KDL::Vector(eeCartPos[0], eeCartPos[1], eeCartPos[2]));

	//Solve IK problem for F_dest
	int status = iksolver.CartToJnt(q_init, F_dest, q);

	//copy q to pointer
	if(status>=0){
		for(unsigned int i=0;i<nj;i++){
			qJoints[i] = q(i);
		}
	}

	//ROS_INFO_STREAM("Status is: " << status);

	//Output status:
	switch(status){
	case 1: ROS_ERROR("E_DEGRADED");
			break;
	case 0: //E_NOERROR
			//Print results:
			for(unsigned int i=0;i<nj;i++){
				ROS_INFO_STREAM("Position of joint " << i << " : " << q(i));
			}
			break;
	case -1: ROS_ERROR("E_NO_CONVERGE");
			break;
	case -2: ROS_ERROR("E_UNDEFINED");
			break;
	case -3: ROS_ERROR("E_NOT_UP_TO_DATE: Joint limits reached!");
			break;
	case -4: ROS_ERROR("E_SIZE_MISMATCH");
			break;
	case -5: ROS_ERROR("E_MAX_ITERATIONS_EXCEEDED");
			break;
	case -6: ROS_ERROR("E_OUT_OF_RANGE");
			break;
	case -7: ROS_ERROR("E_NOT_IMPLEMENTED");
			break;
	case -8: ROS_ERROR("E_SVD_FAILED");
			break;
	}

	return status;

}
} /*namespace*/
