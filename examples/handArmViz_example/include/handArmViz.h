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
#ifndef HANDARMVIZ_H
#define HANDARMVIZ_H

#include <QWidget>
#include "../tools/KDLTreeViz/KDLTreeViz.h"
#include "../tools/JointPositionCtrl/JointPositionCtrl.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "handArmViz" implements the top level widget for this example.
class handArmViz: public QWidget
{
Q_OBJECT
public:
	handArmViz(std::string config_path, QWidget* parent = 0 );
	virtual ~handArmViz();

private Q_SLOTS:
	void setThickness( int thickness_percent );
	void setCellSize( int cell_size_percent );
	void drawTree();

private:
	rviz::VisualizationManager* manager_;
	rviz::RenderPanel* render_panel_;
	rviz::Display* grid_;
	rviz::Display* robot_;
	KDLTreeViz *treeViz;
	JointPositionCtrl *jointPosCtrl;
};
// END_TUTORIAL
#endif // HANDARMVIZ_H
