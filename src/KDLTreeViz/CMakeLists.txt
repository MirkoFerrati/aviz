cmake_minimum_required(VERSION 2.8.3)
project(KDLTreeViz)

## This plugin includes Qt widgets, so we must include Qt like so:
# find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
# include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)
include_directories("/opt/ros/hydro/include")

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
# qt4_wrap_cpp(MOC_FILES
#   myviz.h
#   ../rviz/default_plugin/grid_display.h
# )

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  KDLTreeViz.cpp
)
#   main.cpp
#   ../rviz/default_plugin/grid_display.cpp
#   ${MOC_FILES}
# )

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
# add_executable(myviz ${SOURCE_FILES})

find_package(orocos_kdl REQUIRED)

add_library(KDLTreeViz ${SOURCE_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(KDLTreeViz tinyxml ${urdfdom_LIBRARIES} urdf ${orocos_kdl_LIBRARIES})
# target_link_libraries(myviz ${QT_LIBRARIES} default_plugin  tinyxml ${urdfdom_LIBRARIES} urdf resource_retriever)
## END_TUTORIAL

