/*********************************************************************
* Software License Agreement (Type)
* 
*  Copyright (c) [...]
*  All rights reserved.
* 
*********************************************************************/
/**
 * @file dlrffh_kdl.cpp
 *
 * @brief Class handling kinematics and dynamics of the DLR HIT II hand
 *
 * @license
 *
 * Copyright (C) [..]
 *
 * @author 	Hamal Marino, <noemailfornow@gmail.com>
 * @date	2013-11-27, updated 2013-12-03
 */


#ifndef DLRFFH_KDL_H
#define DLRFFH_KDL_H

#include "dlr_finger.h"

#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <string>

class dlrffh_kdl{
  
private:
	/*
	 * Tree representing the hand (used to construct it from a URDF file)
	 */
	KDL::Tree handKDL_;
	
	/**
	 * Objects of class dlr_finger, representing the fingers of the DLR hand
	 */
	std::vector<dlr_finger*> fingers_;

	/**
	 * @brief Return a full link name being the first found inside a tree, specified by its suffix
	 * @param tree Tree to look for the string with a certain suffix
	 * @param suffix Suffix of the string being looked for
	 * @param full_name Reference to a string which will contain the first corresponding full name (if found in the tree)
	 * @return True if the string has been found, false otherwise
	 */
	bool full_link_name(const KDL::Tree& tree, const std::string& suffix, std::string& full_name);

public:
	
	/**
	 * @brief Class constructor
	 * Creates an handler for the kinematics and dynamics of the DLR HIT II hand
	 * @param file String containing the path to a .urdf (Universal Robot Description Format) file to be used for extracting information about the hand
	 */
	dlrffh_kdl(const std::string& file);
	
	/**
	 * @brief Print out the structure of the hand
	 * @RETURN 0 if everything went correctly, -1 otherwise
	 */
	int print_tree();
	
	/**
	 * @brief Read the number of joints in the tree.
	 * @RETURN the number of joints in the tree.
	 */
	double getNumberOfJoints();
	
	/**
	 * @brief Read the number of links in the tree.
	 * @RETURN the number of links in the tree.
	 */
	double getNumberOfLinks();
	
	/**
	 * @brief Return a link given its name
	 * @PARAM linkName string containing the name of the link to look for.
	 * @PARAM link a pointer to a KDL::Segment to store the link with name @linkname
	 * @RETURN Returns true if the link with name @linkName exists, false otherwise.
	 */
	bool getLink(std::string linkName, KDL::Segment* link);
	
	/**
	 * @brief Return a chain representing a finger given its number from 0 to 4.
	 * @PARAM fingerNr an integer in the range 0..4 containing the number of the finger we want to be returned
	 * 	0: thumb
	 * 	1: index
	 * 	2: middle
	 * 	3: ring
	 * 	4: pinky
	 * @PARAM finger a pointer to a KDL::Chain to store the finger number @fingerNr .
	 * @RETURN Returns true if everything went correctly, false otherwise.
	 */
	bool getFingerChain(int fingerNr, KDL::Chain& finger);
	
	/**
	 * @brief Return a reference to a dlr_finger object
	 * @param i Index of the finger we want to be returned
	 * 	0: thumb
	 * 	1: index
	 * 	2: middle
	 * 	3: ring
	 * 	4: pinky
	 */
	dlr_finger& getFinger(int i);
	
	
	// TO BE ADDED IN ORIGINAL ONE
	KDL::Tree& getTree()
	{
		return this->handKDL_;
	}

/* at now this part doesn't seem useful at all
	/**
	 * @brief Read all joint angles from the tree.
	 * @PARAM jointValues A pointer to an array of joint values to be read
	 * /
	void getJointValues(double* jointValues);
	
	/**
	 * @brief Read all joint angles from a specific finger.
	 * @PARAM fingerNr an integer in the range 0..4 containing the number of the finger we want to be returned
	 * 	0: thumb
	 * 	1: index
	 * 	2: middle
	 * 	3: ring
	 * 	4: pinky
	 * @PARAM jointValues A pointer to an array of joint values to be read
	 * @RETURN Returns true if everything went correctly, false otherwise.
	 * /
	bool getFingerJointValues(int fingerNr, double* jointValues);
	
	void getAllLinksPosition(double* linkValues);
*/  
};

#endif
