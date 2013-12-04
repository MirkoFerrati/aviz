/*********************************************************************
* Software License Agreement (Type)
* 
*  Copyright (c) [...]
*  All rights reserved.
* 
*********************************************************************/
/**
 * @file dlr_finger.h
 *
 * @brief Class handling kinematics and dynamics of a single finger of the DLR HIT II hand
 *
 * @license
 *
 * Copyright (C) [..]
 *
 * @author 	Hamal Marino, <noemailfornow@gmail.com>
 * @date	2013-12-03, updated 2013-12-03
 */


#ifndef DLR_FINGER_H
#define DLR_FINGER_H

#ifndef DEG2RAD
#define DEG2RAD 3.141592654/180
#endif

#define ABD_MINIMUM (-20.0*DEG2RAD)
#define ABD_MAXIMUM (20.0*DEG2RAD)
#define INNER_MINIMUM (0.0*DEG2RAD)
#define INNER_MAXIMUM (75.0*DEG2RAD)
#define OUTER1_MINIMUM (0.0*DEG2RAD)
#define OUTER1_MAXIMUM (75.0*DEG2RAD)
#define OUTER2_MINIMUM (0.0*DEG2RAD)
#define OUTER2_MAXIMUM (75.0*DEG2RAD)

#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <string>


class dlr_finger
{
	
private:
	/**
	 * Chain representing the finger
	 */
	KDL::Chain chain_;
	/**
	 * Joint lower limits
	 */
	KDL::JntArray q_min_;
	/**
	 * Joint upper limits
	 */
	KDL::JntArray q_max_;
    /**
	 * Forward position kinematics solver, needed by the geometric inverse kinematics solver
	 */
    KDL::ChainFkSolverPos_recursive fkpossolver_;
    /**
	 * Inverse velocity kinematics solver, needed by the geometric inverse kinematics solver
	 */
    KDL::ChainIkSolverVel_pinv_givens ikvelsolver_;
    /**
	 * Geometric solver definition (uses joint limits)
	 */
    KDL::ChainIkSolverPos_NR_JL *ikpossolver_;
// 	// Following variables do not seem to be useful: kept as a reminder
// 	/**
// 	 * Current position of the tip frame
// 	 */
// 	KDL::Frame current_tip_position_;
// 	/**
// 	 * Current twist of the tip frame
// 	 */
// 	KDL::Twist current_tip_twist_;
// 	/**
// 	 * Has the configuration of the tip changed from last time it's been asked for?
// 	 */
// 	bool tip_position_changed_;
// 	/**
// 	 * Current joint configuration (q)
// 	 */
// 	KDL::JntArray current_joints_;
// 	/**
// 	 * Current joint velocity (qDot)
// 	 */
// 	KDL::JntArrayVel current_joint_vel_;
// 	/**
// 	 * Current joint acceleration (qDotDot)
// 	 */
// 	KDL::JntArrayAcc current_joint_acc_;
	/**
	 * Solver to compute the jacobian of the finger
	 */
	KDL::ChainJntToJacSolver JointToJac_;
	
	/**
	 * @brief Initialization function, used by all the constructors
	 */
	void init(KDL::Chain& finger,KDL::JntArray q_min,KDL::JntArray q_max);
	
public:
	/**
	 * @brief Class constructor
	 * Takes as input the chain to associate with the finger and minimum/maximum joint values
	 * @param finger The chain to reference the finger
	 * @param q_qmin JntArray of minimum values for the joints
	 * @param q_qmax JntArray of maximum values for the joints
	 */
	dlr_finger(KDL::Chain& finger,KDL::JntArray q_min,KDL::JntArray q_max);
	
	/**
	 * @brief Class constructor
	 * Takes as input the chain to associate with the finger only, and sets the joint limits to the default values
	 */
	dlr_finger(KDL::Chain& finger);

	/**
	 * @brief Class destructor
	 */
	~dlr_finger();
	
	/**
	 * @brief Print the structure of the finger (sorted link names)
	 */
	void print_chain() const;
	
	/**
	 * @brief Outputs the chain representing the finger
	 * @return A reference to the chain representing the finger
	 */
	const KDL::Chain& get_chain() const;
	
	/**
	 * @brief Constructs and returns the finger jacobian from root to tip indexes of joints
	 * @return 0 if everything went correctly, -1 otherwise.
	 * @param J Reference to a KDL::Jacobian which will contain the computed jacobian
	 * @param q_in Current joint values
	 * @param root Joint from which to start building the jacobian (@default 0)
	 * @param tip Joint up to which construct the jacobian (@default 3)
	 */
	int get_jacobian(KDL::Jacobian& J, KDL::JntArray q_in, int root = 0, int tip = 3);
	
	/**
	 * @brief Forward Position Kinematics solver using KDL::ChainFkSolverPos_recursive
	 * @param q_in Joint values to use to compute the forward kinematics
	 * @param p_out KDL::Frame position of the tip as computed by this function (tip as specified by segment_nr)
	 * @param segment_nr Number of segment to be considered as the tip (default:-1 means the whole chain)
	 * @return 0 if everything went correctly, <0 otherwise
	 */
	int fkin_pos(const KDL::JntArray& q_in, KDL::Frame& p_out, int segment_nr = -1)
	{
		return this->fkpossolver_.JntToCart(q_in,p_out,segment_nr);
	}
	
	/**
	 * @brief Forward Velocity Kinematics solver using KDL::ChainIkSolverVel_pinv_givens
	 * @param q_in Joint values to use to compute the forward kinematics
	 * @param v_in KDL::Twist velocity of the tip as computed by this function
	 * @param qDot_out Current derivative of joint values
	 * @return 0 if everything went correctly, <0 otherwise
	 */
	int fkin_vel(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qDot_out)
	{
		return this->ikvelsolver_.CartToJnt(q_in,v_in,qDot_out);
	}
	
	/**
	 * @brief Forward Velocity Kinematics solver using KDL::ChainIkSolverVel_pinv_givens
	 * @warning THIS DOESN'T WORK as CartToJnt is not (yet) implemented for KDL::FrameVel!
	 * @param q_in Joint values to use to compute the forward kinematics
	 * @param v_in KDL::FrameVel velocity of the tip as computed by this function
	 * @param qDot_out Current derivative of joint values
	 * @return always returns -1
	 */
	int fkin_vel(const KDL::JntArray& q_in, const KDL::FrameVel& v_in, KDL::JntArrayVel& qDot_out)
	{
		return this->ikvelsolver_.CartToJnt(q_in,v_in,qDot_out);
	}
	
	/**
	 * @brief Inverse Position Kinematics solver using KDL::ChainIkSolverPos_NR_JL
	 * @param q_init Initial guess for joint values (usually good as (q_max_+q_min_)/2)
	 * @param p_in KDL::Frame position of the tip
	 * @param q_out Computed joint configuration
	 * @return 0 if everything went correctly, <0 otherwise
	 */
	int ikin_pos(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out)
	{
		return this->ikpossolver_->CartToJnt(q_init,p_in,q_out);
	}	
	/**
	 * @brief Inverse Position Kinematics solver using KDL::ChainIkSolverPos_NR_JL
	 * This version without initial guess uses mid-range values q_init = (q_min_+q_max_)/2 as defined in the finger at the beginning
	 * @param p_in KDL::Frame position of the tip
	 * @param q_out Computed joint configuration
	 * @return 0 if everything went correctly, <0 otherwise
	 */
	int ikin_pos(const KDL::Frame& p_in, KDL::JntArray& q_out)
	{
		KDL::JntArray q_init(this->q_min_);
		KDL::Add(this->q_max_,this->q_min_,q_init);
		KDL::Divide(q_init,2,q_init);
		
		return this->ikpossolver_->CartToJnt(q_init,p_in,q_out);
	}
};


#endif
