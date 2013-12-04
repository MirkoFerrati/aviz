#include "KDLTreeViz.h"

// #include <kdl/tree.hpp>
// #include <kdl/segment.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/joint.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/jntarrayvel.hpp>
// #include <kdl/jntarrayacc.hpp>
// #include <kdl/frames.hpp>
// #include <kdl/jacobian.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolvervel_pinv_givens.hpp>
// #include <string>

KDLTreeViz::KDLTreeViz(const KDL::Tree& tree, const boost::shared_ptr< tf::Transformer > tf) : 
 tf_(tf), tree_(tree)
{
}


void KDLTreeViz::setPose(std::map< std::string, double > q_in)
{
	std::string parent,child;
	KDL::Frame frame;
	tf::StampedTransform t;
	
	std::map<std::string,KDL::TreeElement>::const_iterator tree_elem_it;
	
	for (std::map<std::string,double>::const_iterator it = q_in.begin(); it != q_in.end(); ++it)
	{
		std::cout << "Looking for... " << it->first << std::endl;
		
		KDL::Segment elem,childElem;
		double x,y,z,w;
		tree_elem_it = this->tree_.getSegment(it->first);
		elem = tree_elem_it->second.segment;

		parent = elem.getName();

		// set values common to all transformations
		t.stamp_ = ros::Time(1);
		t.frame_id_ = parent;
		
		// for each child
		for (int i=0; i<tree_elem_it->second.children.size(); i++)
		{
			child = tree_elem_it->second.children[i]->second.segment.getName();
			
			// test: print out the name - is it going right?
			std::cout << "Child: " << child << std::endl;
			
			// NOT elem but child element
// 			childElem = elem.children[i]->second;
			childElem = tree_elem_it->second.children[i]->second.segment;
			frame = childElem.pose(it->second);
			frame.M.GetQuaternion(x,y,z,w);
			
			// set values in t
			t.child_frame_id_ = child;
			t.setOrigin(tf::Vector3(frame.p.x(),frame.p.y(),frame.p.z()));
			t.setRotation(tf::Quaternion(x,y,z,w));
			
			this->tf_->setTransform(t, "default_authority");
		}
		
	}
	std::cout << std::endl;
}

