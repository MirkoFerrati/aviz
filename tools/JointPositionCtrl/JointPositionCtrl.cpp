#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <iostream>

#ifndef DEG2RAD
#define DEG2RAD (3.1415926535/180)
#endif

#include "JointPositionCtrl.h"

JointPositionCtrl::JointPositionCtrl(const KDL::Tree& tree, QWidget* parent)
: QWidget( parent )
{
	this->init_JointPositionCtrl(tree,parent);
}

JointPositionCtrl::JointPositionCtrl(const KDL::Tree& tree, const KDL::JntArray& lb, const KDL::JntArray& ub, QWidget* parent)
: QWidget( parent )
{
	this->init_JointPositionCtrl(tree,parent);
	
	// set appropriate limits to the sliders
	for (int slider=0; slider<tree.getNrOfSegments(); slider++)
	{
		QSlider *qSlider = qobject_cast<QSlider*>(signalMapper->mapping(slider));
		qSlider->setMinimum(lb(slider)/DEG2RAD);
		qSlider->setMaximum(ub(slider)/DEG2RAD);
	}
}

const std::map<std::string,double>& JointPositionCtrl::getJointValues()
{
	return this->slider_values_;
}

void JointPositionCtrl::init_JointPositionCtrl(const KDL::Tree& tree, QWidget* parent)
{

// 	QWidget *mainWidget  = new QWidget();
	QGridLayout *controlsLayout = new QGridLayout(parent);
	int i=0;
	
	// map signals from position sliders (whole mapping)
	this->signalMapper = new QSignalMapper(this);
	connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(slider_valueChanged(int)));
	
	for (KDL::SegmentMap::const_iterator it = tree.getSegments().begin(); it != tree.getSegments().end(); ++it)
	{
		if (it->second.segment.getName() != tree.getRootSegment()->second.segment.getName() && it->second.segment.getJoint().getType() != KDL::Joint::None)
		{
			// set name-value pairs for each slider
			this->slider_values_.insert(std::pair<std::string,double>(it->second.segment.getName(),0));
			this->slider_names_.push_back(it->second.segment.getName());
			
			// create a new label-slider pair and put them into the grid
			QSlider *slider = new QSlider(Qt::Horizontal);
			QLabel *label = new QLabel(it->second.segment.getName().c_str());
			controlsLayout->addWidget( label, i, 0 );
			controlsLayout->addWidget( slider, i, 1 );
			slider->setMinimum( -180 );
			slider->setMaximum( 180 );
			slider->setValue( 0 );
			
			// map signals from position sliders (single slider)
			connect(slider,SIGNAL(valueChanged(int)),signalMapper,SLOT(map()));
			signalMapper->setMapping(slider,i++);
		}
	}
	
	setLayout(controlsLayout);
}

void JointPositionCtrl::slider_valueChanged(int slider)
{
	QSlider *qSlider = qobject_cast<QSlider*>(signalMapper->mapping(slider));
	
	if (!qSlider)
	{
		return;
	}
	
	float sliderValue = ((float)qSlider->value())*DEG2RAD;
	this->slider_values_.at(this->slider_names_[slider]) = sliderValue;
	
// 	std::cout << slider_names_[slider] << ": " << this->slider_values_.at(this->slider_names_[slider]) << std::endl;
}