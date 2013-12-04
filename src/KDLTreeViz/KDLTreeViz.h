#include <kdl/tree.hpp>
// #include <tf/transform_datatypes.h>
#include <boost/shared_ptr.hpp>
// #include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class KDLTreeViz
{
private:
	/**
	 * The tree representing the robot kinematics
	 */
	const KDL::Tree& tree_;
	
	/**
	 * A vector of pointers to
	 */
	const boost::shared_ptr<tf::Transformer> tf_;
	
public:
	
	/**
	 * @brief Class constructor
	 * @param tree Tree to be visualized
	 * @param tf Pointer to a Transformer object which sets the appropriate transformations when setPose is called
	 */
	KDLTreeViz(const KDL::Tree& tree, const boost::shared_ptr<tf::Transformer> tf);
	
	/**
	 * @brief Given a list of joint angles (mapped by names), set the robot visualized pose
	 * @param q_in map of joint angles (the key is the segment name)
	 */
	void setPose(std::map<std::string,double> q_in);
	
};
