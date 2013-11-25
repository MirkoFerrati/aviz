aviz
====

Agnostic visualization library based on Rviz


You will need lot of things in order to compile rviz without the ROS stack. 
At the moment, we still include geometry messages from ros and probably a couple of headers that are still not tracked or included directly.
The requirement is to have all ros headers in your include path. No need to run any ros script or to have ros-master running.


You will need a ROS-clean version of orocos-kdl, which can be found here https://github.com/traversaro/orocos_kinematics_dynamics
The other requirements are included inside the source code, inside the folder ros. 
This project is still in prealpha, meaning that it still has lot of errors and does not work as expected. Still, you can compile a simple example inside the folder myviz.
