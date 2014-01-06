/**
 * @file JointPositionCtrl.h
 * @brief Class to create a sort of "joint state publisher", a grid of sliders which are created based on a KDL::Tree.
 * Class to create a sort of "joint state publisher", a grid of sliders which are created based on a KDL::Tree.
 * Each slider has an associated name which is the corresponding name of a segment in the tree, and (if provided)
 * bounds of the slider are those of the joint (extra KDL::JntArray lb and ub should be passed as an argument).
 */

#ifndef JOINTPOSITIONCTRL_H
#define JOINTPOSITIONCTRL_H

#include <QWidget>
#include <QSignalMapper>
// #include "tools/KDLTreeViz/KDLTreeViz.h"
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <string>

class JointPositionCtrl : public QWidget
{
Q_OBJECT
public:
	
	/**
	 * @brief Class constructor
	 * @param tree KDL::Tree from which to read the configuration (segment names)
	 */
	JointPositionCtrl(const KDL::Tree& tree, QWidget* parent = 0);
	
	/**
	 * @brief Class constructor
	 * @param tree KDL::Tree from which to read the configuration (segment names)
	 * @param lb KDL::JntArray where lower bounds are stored (used to create the sliders) [radians]
	 * @param ub KDL::JntArray where upper bounds are stored (used to create the sliders) [radians]
	 */
	JointPositionCtrl(const KDL::Tree& tree, const KDL::JntArray& lb, const KDL::JntArray& ub, QWidget* parent = 0);
	
	/**
	 * @brief Class destructor
	 */
	virtual ~JointPositionCtrl() {}
	
	/**
	 * @brief Function to read current values of the sliders
	 * @return Map of string/value pairs for the sliders
	 */
	const std::map<std::string,double>& getJointValues();

private Q_SLOTS:
	
	/**
	 * @brief Function to call when a slider changes value
	 */
	void slider_valueChanged(int slider);
	
private:
	
	/**
	 * Map where to store pairs name-value for each joint slider
	 * The string is the joint (slider) name, and the double is it's values.
	 * Note: values are stored in radians.
	 */
	std::map<std::string,double> slider_values_;
	// 	std::map<std::string,std::pair<bool,double> > slider_values_;
	
	/**
	 * Vector containing only the names of the sliders, to get the correct value when moving a slider.
	 */
	std::vector<std::string> slider_names_;
	
	/**
	 * A signal mapper to map from a vector of sliders to a position in slider_values_
	 */
	QSignalMapper *signalMapper;
	
// 	/**
// 	 * Vector where to store changed status for each slider (avoiding to pass values which have not been changed)
// 	 */
// 	std::vector<bool> slider_changed_;
	
	/**
	 * @brief Common initialization function
	 * @param tree KDL::Tree from which to read the configuration (segment names)
	 */
	void init_JointPositionCtrl(const KDL::Tree& tree, QWidget* parent);
};

#endif // JOINTPOSITIONCTRL_H
