#include "KDLTreeViz.h"


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
// 		std::cout << "Looking for... " << it->first << std::endl;
		
		KDL::Segment elem,childElem;
		double x,y,z,w;
		tree_elem_it = this->tree_.getSegment(it->first);
		elem = tree_elem_it->second.segment;

		child = elem.getName();
		parent = tree_elem_it->second.parent->second.segment.getName();
// 		std::cout << "Parent: " << parent << std::endl;
		
		frame = elem.pose(it->second);
		frame.M.GetQuaternion(x,y,z,w);
		
		// set values in t
		t.stamp_ = ros::Time(1);
		t.frame_id_ = parent;
		t.child_frame_id_ = child;
		t.setOrigin(tf::Vector3(frame.p.x(),frame.p.y(),frame.p.z()));
		t.setRotation(tf::Quaternion(x,y,z,w));
		
		this->tf_->setTransform(t, "default_authority");
// 		std::cout << "Transformation no problem!" << std::endl;
	}
// 	std::cout << std::endl;
}

void KDLTreeViz::setInitialVisualization()
{
	std::map< std::string, double > q_in;
	std::cout << std::endl;
	
	for (KDL::SegmentMap::const_iterator it = this->tree_.getSegments().begin(); it != this->tree_.getSegments().end(); ++it)
	{
		if (it->second.segment.getName() != this->tree_.getRootSegment()->second.segment.getName())
		{
// 			std::cout << it->second.segment.getName() << std::endl;
			q_in.insert(std::pair<std::string,double>(it->second.segment.getName(),0));
		}
	}
	
	tf::StampedTransform t;
	
	t.stamp_ = ros::Time(1);
	t.frame_id_ = "map";
	t.setIdentity();
	t.setOrigin(tf::Vector3(0,0,0));
	t.child_frame_id_ = this->tree_.getRootSegment()->second.segment.getName();
	std::cout << "RootSegmentName: " << t.child_frame_id_ << std::endl;
	this->tf_->setTransform(t,"default_authority");
	
	this->setPose(q_in);
}


