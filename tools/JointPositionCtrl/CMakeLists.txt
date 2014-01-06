cmake_minimum_required(VERSION 2.8.3)
project(JointPositionCtrl)

find_package(orocos_kdl REQUIRED)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_FILES
	JointPositionCtrl.h
)

## Here we specify the list of source files
set(SOURCE_FILES
	JointPositionCtrl.cpp
	${MOC_FILES}
)

add_library(JointPositionCtrl ${SOURCE_FILES})

## Link the library to what it needs
target_link_libraries(JointPositionCtrl ${orocos_kdl_LIBRARIES} ${QT_LIBRARIES})
# target_link_libraries(JointPositionCtrl tinyxml ${urdfdom_LIBRARIES} urdf ${orocos_kdl_LIBRARIES} ${QT_LIBRARIES})

# IMPORTANT: Add the KDLTreeViz library to the "export-set"
install(TARGETS JointPositionCtrl
	EXPORT AvizTargets
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib)
