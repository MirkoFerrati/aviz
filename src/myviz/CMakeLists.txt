

cmake_minimum_required(VERSION 2.8.3)
project(librviz_tutorial)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

# ## path to be able to find local inclusions made in the source files
# include_directories("../../../robotics-manipulation/DLR_FFH_code/DLRFFH_kdl/include/dlrffh_kdl")
# ## add custom path to look for libraries
# set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "../../../robotics-manipulation/DLR_FFH_code/DLRFFH_kdl/build")
# find_library(dlrffh_kdl_LIB dlrffh_kdl)
# # find_package(dlrffh_kdl REQUIRED)
# # message("Include dirs: " ${dlrffh_kdl_INCLUDE_DIRS})
find_package(orocos_kdl REQUIRED)
find_package(kdl_format_io REQUIRED)
#support also for the old version of kdl cmake package
if(NOT orocos_kdl_FOUND)
   find_package(Orocos_KDL)
   if(NOT Orocos_KDL_FOUND)
      message(WARNING "KDL not found: neither orocos_kdl or Orocos_KDL cmake packages are available")
   else(NOT Orocos_KDL_FOUND)
      set(orocos_kdl_INCLUDE_DIRS ${Orocos_KDL_INCLUDE_DIRS})
      set(orocos_kdl_LIBRARY_DIRS ${Orocos_KDL_LIBRARY_DIRS})
      set(orocos_kdl_LIBRARIES ${Orocos_KDL_LIBRARIES})
      set(orocos_kdl_FOUND true)
   endif(NOT Orocos_KDL_FOUND)
endif(NOT orocos_kdl_FOUND)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
#include_directories("../rviz/")
include_directories("/opt/ros/hydro/include")

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  myviz.h
  ../rviz/default_plugin/grid_display.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  myviz.cpp
  main.cpp
  ../rviz/default_plugin/grid_display.cpp
  ${MOC_FILES}
  dlr_finger.cpp
  dlrffh_kdl.cpp
  ../KDLTreeViz/KDLTreeViz.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
add_executable(myviz ${SOURCE_FILES})


## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(myviz ${QT_LIBRARIES} default_plugin  tinyxml ${urdfdom_LIBRARIES} urdf resource_retriever ${kdl_format_io_LIBRARIES} ${orocos_kdl_LIBRARIES})
## END_TUTORIAL
