cmake_minimum_required(VERSION 2.8.3)
project(libaviz_tutorial)

## this looks in current folder for a file named find-aviz.cmake
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

# give it a first try: AvizConfig.cmake is located in the source directory
# and contains global path to the library?
set(Aviz_DIR ${PROJECT_SOURCE_DIR} CACHE PATH "Directory where to look for AvizConfig.cmake file")
find_package(Aviz)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

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

include_directories("${AVIZ_INCLUDE_DIRS}")
include_directories("${AVIZ_INCLUDE_DIRS}/src")
# include_directories("/opt/ros/hydro/include")
include_directories("include/")

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/handArmViz.h
#  ${AVIZ_INCLUDE_DIRS}/src/rviz/default_plugin/grid_display.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/handArmViz.cpp
  src/main.cpp
#  ${AVIZ_INCLUDE_DIRS}/src/rviz/default_plugin/grid_display.cpp
  ${MOC_FILES}
#   ../../tools/KDLTreeViz/KDLTreeViz.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
add_executable(handArmViz_prova_cmake ${SOURCE_FILES})


## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(handArmViz_prova_cmake ${QT_LIBRARIES} tinyxml ${urdfdom_LIBRARIES} urdf resource_retriever ${kdl_format_io_LIBRARIES} ${orocos_kdl_LIBRARIES} default_plugin KDLTreeViz)
## END_TUTORIAL

# # IMPORTANT: Add the handArmViz executable to the "export-set"
# install(TARGETS handArmViz
#   EXPORT AvizTargets
#   RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

  # ${CMAKE_CURRENT_BINARY_DIR}/file3.h
#   EXPORT AvizTargets
# INSTALL(FILES  ../../src/rviz/default_plugin/grid_display.h 
#   RUNTIME DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT include)
# INSTALL(FILES  ../../src/rviz/default_plugin/grid_display.h 
#   DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT include)
