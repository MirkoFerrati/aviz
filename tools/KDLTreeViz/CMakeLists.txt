cmake_minimum_required(VERSION 2.8.3)
project(KDLTreeViz)

include_directories("/opt/ros/hydro/include")

## Here we specify the list of source files
set(SOURCE_FILES
  KDLTreeViz.cpp
)

find_package(orocos_kdl REQUIRED)

add_library(KDLTreeViz ${SOURCE_FILES})

## Link the library to what it needs
target_link_libraries(KDLTreeViz tinyxml ${urdfdom_LIBRARIES} urdf ${orocos_kdl_LIBRARIES})

# IMPORTANT: Add the KDLTreeViz library to the "export-set"
install(TARGETS KDLTreeViz
  EXPORT AvizTargets
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib)
