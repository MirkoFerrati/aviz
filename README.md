aviz
====

Agnostic visualization library based on Rviz


You will need lot of things in order to compile rviz without the ROS stack. 
At the moment, we still include geometry messages from ros and 
probably a couple of headers that are still not tracked or included directly.
The requirement is to have all ros headers in your include path. 
No need to run any ros script or to have ros-master running.

You will need a ROS-clean version of orocos-kdl, which can be found 
here https://github.com/traversaro/orocos_kinematics_dynamics.

By installing that version of orocos_kinematics_dynamics 
you also get console_bridge, urdfdom and urdfdom_headers, which are required.

If you have ROS installed, all these requirements are already in your computer,
so you may use them for now, but you will nedd to change the CMakeLists.txt
to tell cmake where to find them (opt/ros/hydro is not in the default path for cmake)

The other requirements are included inside the source code, inside the folder ros. 
This project is still in prealpha, meaning that it still has lot of errors and does not work as expected. 
Still, you can compile a simple example inside the folder myviz.


what is working
---------

pi_robot.urdf is a simple urdf file that should work with myviz example.
Copy config.yaml inside the folder from where you are running myviz. 
Inside config.yaml edit the paths so that PackagePath points to a folder "/home/youruser/whatever/foldername" like this:
```
foldername
- ogre_media
- icons
- ...
```
Copy pi_robot.urdf inside the folder from where you are running myviz executable (or edit config.yaml path).

Run.

If you take a look at myviz.cpp, you will see how to send frame transformations directly to the visualizer 
without using ROS nodes and so on.


