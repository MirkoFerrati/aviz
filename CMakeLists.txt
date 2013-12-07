cmake_minimum_required(VERSION 2.8.3)
project(aviz)

# Only for versioning
set(AVIZ_MAJOR_VERSION 0)
set(AVIZ_MINOR_VERSION 1)
set(AVIZ_PATCH_VERSION 0)
set(AVIZ_VERSION
  ${AVIZ_MAJOR_VERSION}.${AVIZ_MINOR_VERSION}.${AVIZ_PATCH_VERSION})

find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  signals
  system
  thread
)

## if ROS is installed in the current system, it is possible to use the following 4 lines instead of having all the packages inside aviz
# set(urdfdom_DIR /opt/ros/hydro/share/urdfdom/cmake/)
# set(urdfdom_headers_DIR /opt/ros/hydro/share/urdfdom_headers/cmake/)
# set(orocos_kdl_DIR /opt/ros/hydro/share/orocos_kdl/)
# set(console_bridge_DIR /opt/ros/hydro/share/console_bridge/cmake/)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(Eigen REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE OGRE)
## Find OGRE Plugin path (not necessarily platform-independent, I guess)
execute_process(COMMAND
pkg-config --variable=plugindir OGRE
OUTPUT_VARIABLE OGRE_PLUGIN_PATH
OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})

if(APPLE)
  FIND_LIBRARY(Cocoa_LIBRARIES Cocoa)
  set(aviz_ADDITIONAL_LIBRARIES ${Cocoa_LIBRARIES})
endif()

find_package(OpenGL REQUIRED)

find_package(PythonLibs REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)

find_package(urdfdom REQUIRED)

# find_package(catkin REQUIRED
#   COMPONENTS
#   angles
#   geometry_msgs
#   image_geometry
#   image_transport
#   interactive_markers
#   laser_geometry
#   map_msgs
#   message_filters
#   nav_msgs
#   pluginlib
#   python_qt_binding
#   resource_retriever
#   rosbag
#   rosconsole
#   roscpp
#   roslib
#   rospy
#   sensor_msgs
#   std_msgs
#   std_srvs
#   tf
#   urdf
#   visualization_msgs
# )
# 
# catkin_python_setup()
# 
# catkin_package(
#   INCLUDE_DIRS
#     src
#     ${EIGEN_INCLUDE_DIRS}
#     ${OGRE_INCLUDE_DIRS}
#     ${OPENGL_INCLUDE_DIR}
#     ${urdfdom_INCLUDE_DIRS}
#   LIBRARIES
#     rviz
#     default_plugin
#     ${OGRE_LIBRARIES}
#     ${rviz_ADDITIONAL_LIBRARIES}
#     ${OPENGL_LIBRARIES}
#     ${urdfdom_LIBRARIES}
#   CATKIN_DEPENDS
#     geometry_msgs
#     image_geometry
#     image_transport
#     interactive_markers
#     laser_geometry
#     map_msgs
#     message_filters
#     nav_msgs
#     pluginlib
#     resource_retriever
#     roscpp
#     roslib
#     sensor_msgs
#     std_msgs
#     tf
#     urdf
#     visualization_msgs
# )

include_directories(SYSTEM
  ${EIGEN_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${PYTHON_INCLUDE_PATH}
  ${urdfdom_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})
message (STATUS "eigen include dir: " ${EIGEN3_INCLUDE_DIR})

include_directories(src ${catkin_INCLUDE_DIRS})
# include_directories("/opt/ros/hydro/include")
include_directories(src/ros/geometry)
link_directories(${OGRE_LIBRARY_DIRS})
include_directories(other_ros_include)

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

#### If gtk ends up being the best way to get the correct window
#### position under X11, this is how to compile it in.
#
# find_package(GTK2)
# include_directories(${GTK2_INCLUDE_DIRS})
# include_directories(/usr/include/gdk-pixbuf-2.0)

add_subdirectory(src)
add_subdirectory(tools)
# add_subdirectory(examples)

# install(DIRECTORY ogre_media
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY icons
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY images
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(FILES default.rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(FILES plugin_description.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

###################################################################
## Add possibility to install the library (set INSTALL_BASE_DIR) ##
###################################################################
# set(INSTALL_BASE_DIR ${PROJECT_BINARY_DIR})
set(INSTALL_BASE_DIR "" CACHE PATH "Base installation directory") #default install in root

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR ${INSTALL_BASE_DIR}/lib/aviz CACHE INTERNAL "Installation directory for libraries")
set(INSTALL_BIN_DIR ${INSTALL_BASE_DIR}/bin/aviz CACHE INTERNAL "Installation directory for executables")
set(INSTALL_INCLUDE_DIR ${INSTALL_BASE_DIR}/include/aviz CACHE INTERNAL
  "Installation directory for header files")
# Where do I install cmake packages?
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/aviz)
endif()
set(INSTALL_CMAKE_DIR ${INSTALL_BASE_DIR}/${DEF_INSTALL_CMAKE_DIR} CACHE INTERNAL
  "Installation directory for CMake files")

# # Make relative paths absolute (needed later on)
# foreach(p LIB BIN INCLUDE CMAKE)
#   set(var INSTALL_${p}_DIR)
#   if(NOT IS_ABSOLUTE "${${var}}")
#     set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
#   endif()
# endforeach()

# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}"   # to find foo/foo.h
  "${PROJECT_BINARY_DIR}")  # to find foo/config.h

# # Add sub-directories
# add_subdirectory(foo)
# add_subdirectory(bar)

# The interesting stuff goes here
# ===============================

## No longer export the example - build it apart
# # Add all targets to the build-tree export set
# export(TARGETS handArmViz
#   FILE "${PROJECT_BINARY_DIR}/AvizTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE aviz)

# if wanting to use relative path use the variable REL_INCLUDE_DIR,
# otherwise, use the variable INSTALL_INCLUDE_DIR
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")

# Create the AvizConfig.cmake and AvizConfigVersion files
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(AvizConfig.cmake.in
  "${PROJECT_BINARY_DIR}/AvizConfig.cmake" @ONLY)
# ... for the install tree
## -- relative path --
set(CONF_INCLUDE_DIRS "\${AVIZ_CMAKE_DIR}/${REL_INCLUDE_DIR}")
## -- absolute path --
# set(CONF_INCLUDE_DIRS "${INSTALL_INCLUDE_DIR}")
configure_file(AvizConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/AvizConfig.cmake" @ONLY)
# ... for both
configure_file(AvizConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/AvizConfigVersion.cmake" @ONLY)

# Install the FooBarConfig.cmake and FooBarConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/AvizConfig.cmake"
  "${PROJECT_BINARY_DIR}/AvizConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT AvizTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# This should be done inside each specific folder CMakeLists but, as the project is quite big and confused by now,
# is done here just in order to have it working; improvements will be well-accepted
# At now,  more than needed is installed
## install include files in the appropriate directory
install(DIRECTORY src tools DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT include
	FILES_MATCHING  REGEX "((src/ros/|src/rviz|tools/).*[.](h|hpp)$)"
)
# ((src/ros/|src/rviz|tools/).*([.](h|hpp)|_display[.]c(c|pp))$)
# this would also install *_display.cpp files; anyway, these are not needed once the library is compiled

install(DIRECTORY other_ros_include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT include
	FILES_MATCHING  REGEX "([.](h|hpp)$)"
) 

# install(DIRECTORY dirs... DESTINATION <dir>
#           [FILE_PERMISSIONS permissions...]
#           [DIRECTORY_PERMISSIONS permissions...]
#           [USE_SOURCE_PERMISSIONS]
#           [CONFIGURATIONS [Debug|Release|...]]
#           [COMPONENT <component>] [FILES_MATCHING]
#           [[PATTERN <pattern> | REGEX <regex>]
#            [EXCLUDE] [PERMISSIONS permissions...]] [...])
