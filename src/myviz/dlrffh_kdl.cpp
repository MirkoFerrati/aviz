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
 * Copyright (C) [...]
 *
 * @author 	Hamal Marino, <noemailfornow@gmail.com>
 * @date	2013-11-27, updated 2013-12-03
 */

#include "dlrffh_kdl.h"
#include "dlr_finger.h"
#include <kdl_format_io/urdf_import.hpp>
#include <iostream>

dlrffh_kdl::dlrffh_kdl(const std::string& file)
{
	bool res;
	res = kdl_format_io::treeFromUrdfFile(file,this->handKDL_);

	if (!res)
	{
		printf("Not able to read the URDF file - aborting.");
		exit(1);
	}
	
	for (int i=0; i<5; i++)
	{
		KDL::Chain finger;
		res = (this->getFingerChain(i,finger) && res);
		if (!res)
		{
			std::cout << "Not able to generate finger " << i << " - aborting." << std::endl;
			exit(1);
		}
		else
		{
			this->fingers_.push_back(new dlr_finger(finger));
		}
	}
}

int dlrffh_kdl::print_tree()
{
	std::cout << "Hand root name: " << this->handKDL_.getRootSegment()->first << std::endl;
	std::cout << "Nr. of joints: " << this->handKDL_.getNrOfJoints() << std::endl;
	std::cout << "Nr. of segments: " << this->handKDL_.getNrOfSegments() << std::endl;
	
	std::cout << "List of segments:" << std::endl;
	
	for (KDL::SegmentMap::const_iterator it = this->handKDL_.getSegments().begin(); it != this->handKDL_.getSegments().end(); ++it)
	{
		std::cout << it->first << std::endl;
	}
	std::cout << std::endl;
}

double dlrffh_kdl::getNumberOfJoints()
{
  return this->handKDL_.getNrOfJoints();
}

double dlrffh_kdl::getNumberOfLinks()
{
  return this->handKDL_.getNrOfSegments();
}

bool dlrffh_kdl::getLink(std::string linkName, KDL::Segment* link)
{
  KDL::SegmentMap::const_iterator it = this->handKDL_.getSegment(linkName);
  if (it == this->handKDL_.getSegments().end())
  {
    return false;
  }
  
  *link = it->second.segment;
  return true;
}

bool dlrffh_kdl::full_link_name(const KDL::Tree& tree, const std::string& suffix, std::string& full_name)
{
	for (KDL::SegmentMap::const_iterator it = tree.getSegments().begin(); it != tree.getSegments().end(); ++it)
	{
		if ((it->first.length() >= suffix.length()) && !(std::strcmp(it->first.substr(it->first.length()-suffix.length()).c_str(),suffix.c_str())))
		{
			full_name = it->first;
			// std::cout << "full_link_name() : " << full_name << std::endl;
			return true;
		}
	}
	
	return false;
}

bool dlrffh_kdl::getFingerChain(int fingerNr, KDL::Chain& finger)
{
	std::string root = "_palm_link";
	std::string tip;

	if (!full_link_name(this->handKDL_,root,root))
	{
		std::cout << "Error in dlrffh_kdl::getFingerChain : " << root << " link NOT found" << std::endl;
		return false;
	}

	switch (fingerNr)
	{
		case 0:
			tip = "_thumb_distal_link";
			break;
		case 1:
			tip = "_index_distal_link";
			break;
		case 2:
			tip = "_middle_distal_link";
			break;
		case 3:
			tip = "_ring_distal_link";
			break;
		case 4:
			tip = "_pinky_distal_link";
			break;
		default:
			return false;
	}
	
	if (!full_link_name(this->handKDL_,tip,tip))
	{
		std::cout << "Error in dlrffh_kdl::getFingerChain : " << tip << " link NOT found" << std::endl;
		return false;
	}
	
	this->handKDL_.getChain(root,tip,finger);
	return true;
}

dlr_finger& dlrffh_kdl::getFinger(int i)
{
	if (i<5)
		return *(this->fingers_[i]);
	
	std::cerr << "dlrffh_kdl::getFinger : Asked for finger " << i << " - out of range [0..4]" << std::endl;
	exit(1);
}















/* at now this part doesn't seem useful at all
void dlrffh_kdl::getJointValues(double* jointValues)
{
  int i;
  i = 0;
  
  for (KDL::SegmentMap::const_iterator it = this->handKDL_.getSegments().begin(); it != this->handKDL_.getSegments().end(); ++it)
  {
    double tmp;
    
    if (it->second.segment.getJoint().getType() == KDL::Joint::None)
    {
      continue;
    }
    it->second.segment.getJoint().pose(tmp);
    jointValues[i++] = tmp;
  }
}

bool dlrffh_kdl::getFingerJointValues(int fingerNr, double* jointValues)
{
  int i;
  i = 0;
  
  KDL::Chain finger;  
  this->getFingerChain(fingerNr,finger);
  
  for (std::vector<KDL::Segment>::const_iterator it = finger.segments.begin(); it != finger.segments.end(); ++it)
  {
    double tmp;
    it->getJoint().pose(tmp);
    jointValues[i++] = tmp;
  }
}

void dlrffh_kdl::getAllLinksPosition(double* linkValues)
{
  
}
*/
