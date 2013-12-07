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


installing the library
---------------------
Once all requirements are fullfilled (i.e., all ROS include not provided directly with the package are in your include path), you can use the following command to install the library [LINUX]:

mkdir build
cd build
cmake .. [*NOTE1]
make
[sudo] make install

Sudo rights are needed only if you decide to install this library in a place where you don't have writing rights.
This will install in the system some static libraries and [maybe too many] headers for you to include to write code with the library itself.
*NOTE1:
To change where the library gets installed, do not use [DESTDIR=${mydir}] in the "make install" command, as this is not yet supported; instead,
set the INSTALL_BASE_DIR variable in the cmake configuration to ${mydir} (without last "/", which is added by default).

a full working example
----------------------
Once the library is installed, you can build the example "handArmViz" from the example folder [LINUX]:

cd ../examples
mkdir build
cd build
[ccmake ..  -- this is only needed if you set INSTALL_BASE_DIR to install the library in a folder which is not SYSTEM:
        you will need to set Aviz_DIR to ${mydir}/lib/CMake/aviz in order to find the AvizConfig.cmake file]
cmake ..
make

cp ../config.yaml ./config.yaml
[edit the file config.yaml just copied inside build such that path to unipi_robot.urdf, rviz folder (needed for the icons), 
and robot_description directories are corresponding to global path in your computer]
[*NOTE2]
./handArmViz


*NOTE2:
if you are using a virtual machine to run this, you will need to run the following command before running the executable
"export LIBGL_ALWAYS_SOFTWARE=1"
This is to obtain 3D graphics acceleration from OpenGL not using hardware (the program - or even the Virtual Machine - would crash otherwise).
