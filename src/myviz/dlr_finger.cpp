/*********************************************************************
* Software License Agreement (Type)
* 
*  Copyright (c) [...]
*  All rights reserved.
* 
*********************************************************************/
/**
 * @file dlr_finger.cpp
 *
 * @brief Class handling kinematics and dynamics of a single finger of the DLR HIT II hand
 *
 * @license
 *
 * Copyright (C) [...]
 *
 * @author 	Hamal Marino, <noemailfornow@gmail.com>
 * @date	2013-12-03, updated 2013-12-03
 */

#define JOINTS_NR 4

#include "dlr_finger.h"
#include <iostream>

void dlr_finger::init(KDL::Chain& finger,KDL::JntArray q_min,KDL::JntArray q_max)
{
	this->ikpossolver_ = new KDL::ChainIkSolverPos_NR_JL(finger,q_min,q_max,fkpossolver_,ikvelsolver_);
	
	this->chain_=finger;
	this->q_min_=q_min;
	this->q_max_=q_max;
	
// 	// compute the forward kinematics
// 	int status = this->fkpossolver_.JntToCart(this->current_joints_,this->current_tip_position_);
// 	if (status)
// 	{
// 		std::cout << "Error in fkpossolver: status " << status << std::endl;
// 	}
	
// 	// Other variables setting
// 	this->tip_position_changed_ == false;
// 	KDL::SetToZero(this->current_tip_twist_);
}

dlr_finger::dlr_finger(KDL::Chain& finger,KDL::JntArray q_min,KDL::JntArray q_max) : 
	fkpossolver_(finger), ikvelsolver_(finger), JointToJac_(finger)/*, current_joints_(JOINTS_NR), current_joint_vel_(JOINTS_NR), current_joint_acc_(JOINTS_NR)*/
{
	this->init(finger,q_min,q_max);
}

dlr_finger::dlr_finger(KDL::Chain& finger) : 
	fkpossolver_(finger), ikvelsolver_(finger), JointToJac_(finger)/*, current_joints_(JOINTS_NR), current_joint_vel_(JOINTS_NR), current_joint_acc_(JOINTS_NR)*/
{
	KDL::JntArray q_min(finger.getNrOfJoints()),q_max(finger.getNrOfJoints());
	
	q_min(0) = ABD_MINIMUM;
	q_max(0) = ABD_MAXIMUM;
	q_min(1) = INNER_MINIMUM;
	q_max(1) = INNER_MAXIMUM;
	q_min(2) = OUTER1_MINIMUM;
	q_max(2) = OUTER1_MAXIMUM;
	q_min(3) = OUTER2_MINIMUM;
	q_max(3) = OUTER2_MAXIMUM;
	
	this->init(finger,q_min,q_max);
}

void dlr_finger::print_chain() const
{
	std::cout << "Finger root name: " << this->chain_.getSegment(0).getName() << std::endl;
	std::cout << "Nr. of joints: " << this->chain_.getNrOfJoints() << std::endl;
	std::cout << "Nr. of segments: " << this->chain_.getNrOfSegments() << std::endl;
	
	std::cout << "List of segments:" << std::endl;
	
	for (int i = 0; i < this->chain_.getNrOfJoints(); i++)
	{
		KDL::Segment it = this->chain_.getSegment(i);
		std::cout << it.getName() << std::endl;
	}
	std::cout << std::endl;
}

const KDL::Chain& dlr_finger::get_chain() const
{
	return this->chain_;
}

int dlr_finger::get_jacobian(KDL::Jacobian& J, KDL::JntArray q_in, int root, int tip)
{
	if ((tip >= this->chain_.getNrOfJoints()) || (tip-root+1 != J.columns()) || (q_in.rows() != J.columns()))
	{
		std::cout << "Error in dlr_finger::get_jacobian : Unconsistent dimensions." << std::endl;
		return -1;
	}
	
	// consider the case of a partial jacobian asked
	if (tip-root+1 != this->chain_.getNrOfJoints())
	{
		KDL::Chain partFinger;
		for (int i=root; i<tip+1; i++)
		{
			partFinger.addSegment(this->chain_.getSegment(i));
		}
		KDL::ChainJntToJacSolver JointToJac_part(partFinger);
		
		return JointToJac_part.JntToJac(q_in,J);
	}
	else
	{
		return this->JointToJac_.JntToJac(q_in,J);
	}
}
