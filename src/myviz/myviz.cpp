/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/grid_display.h"
#include <rviz/default_plugin/robot_model_display.h>
#include <rviz/frame_manager.h>
#include <rviz/yaml_config_reader.h>
#include "myviz.h"
#include <tf/transform_datatypes.h>
#include <../package.h>

#include "../KDLTreeViz/KDLTreeViz.h"
#include "dlrffh_kdl.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz(std::string config_file , QWidget* parent)
  : QWidget( parent )
{
	int i = 0;
  
    QString urdf_file, path_file;
    rviz::YamlConfigReader reader;
    rviz::Config cfg;
    reader.readFile( cfg, QString(config_file.c_str()) );
    if( !reader.error() )
    {
      int height, width;
      if( cfg.mapGetInt( "Height", &height ) &&
          cfg.mapGetInt( "Width", &width ))
      {
        resize( width, height );
      }
       if (!cfg.mapGetString("Urdf",&urdf_file))
       {
	 std::cout<<"error in finding urdf file inside configuration, please add Urdf: yourfile.urdf"<<std::endl;
       }
       if (cfg.mapGetString("PackagePath",&path_file))
       {
	 ros::package::setPath(path_file.toStdString());
       }
	 else
       {
	 std::cout<<"error in finding path file inside configuration, please add PackagePath: path_to_rviz"<<std::endl;
       }
    }
    else
    {
      printf( "%s", qPrintable( reader.errorMessage() ));
    }
  
  // Construct and lay out labels and slider controls.
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();
  // Create a Grid display.
  rviz::Display* temp=new rviz::GridDisplay();
  grid_ = manager_->createDisplay( temp, "adjustable grid", true );
  ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( Qt::yellow );

  // Initialize the slider values.
  thickness_slider->setValue( 25 );
  cell_size_slider->setValue( 10 );
  
  
  rviz::Display* robot=new rviz::RobotModelDisplay();
  
  
	dlrffh_kdl* right;
	KDL::Chain thumb; //,middle,ring,pinky;
	right = new dlrffh_kdl(urdf_file.toStdString());
	
	std::map< std::string, double > q_in;
	KDLTreeViz treeViz(right->getTree(),manager_->getFrameManager()->getTFClientPtr());

	treeViz.setInitialVisualization();

// 	Collection of strings usable for right DLR HIT2 hand
	q_in.insert(std::pair<std::string,double>("right_index_abd_link",10*DEG2RAD));
	q_in.insert(std::pair<std::string,double>("right_index_distal_link",0));
	q_in.insert(std::pair<std::string,double>("right_index_medial_link",0));
	q_in.insert(std::pair<std::string,double>("right_index_proximal_link",0));
	q_in.insert(std::pair<std::string,double>("right_middle_abd_link",10*DEG2RAD));
	q_in.insert(std::pair<std::string,double>("right_middle_distal_link",0));
	q_in.insert(std::pair<std::string,double>("right_middle_medial_link",0));
	q_in.insert(std::pair<std::string,double>("right_middle_proximal_link",0));
	q_in.insert(std::pair<std::string,double>("right_palm_link",0));
	q_in.insert(std::pair<std::string,double>("right_pinky_abd_link",10*DEG2RAD));
	q_in.insert(std::pair<std::string,double>("right_pinky_distal_link",0));
	q_in.insert(std::pair<std::string,double>("right_pinky_medial_link",0));
	q_in.insert(std::pair<std::string,double>("right_pinky_proximal_link",0));
	q_in.insert(std::pair<std::string,double>("right_ring_abd_link",10*DEG2RAD));
	q_in.insert(std::pair<std::string,double>("right_ring_distal_link",0));
	q_in.insert(std::pair<std::string,double>("right_ring_medial_link",0));
	q_in.insert(std::pair<std::string,double>("right_ring_proximal_link",0));
	q_in.insert(std::pair<std::string,double>("right_thumb_abd_link",10*DEG2RAD));
	q_in.insert(std::pair<std::string,double>("right_thumb_distal_link",0));
	q_in.insert(std::pair<std::string,double>("right_thumb_medial_link",0));
	q_in.insert(std::pair<std::string,double>("right_thumb_proximal_link",0));

	treeViz.setPose(q_in);
	
	robot_ = manager_->createDisplay(robot,"urdf robot",true);
	ROS_ASSERT( robot_ != NULL );
	
	//robot_->subProp("Robot Description")->setValue("/opt/ros/hydro/share/urdf_tutorial/05-visual.urdf");
	//robot_->subProp("Robot Description")->setValue("/home/mirko/projects/walkman/rivz/rviz/src/myviz/pi_robot.urdf");
	robot_->subProp("Robot Description")->setValue(urdf_file);

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}

