cmake_minimum_required(VERSION 2.8.3)
project(urdf)

find_package(Boost REQUIRED thread)
find_package(urdfdom REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#  urdfdom_headers urdf_parser_plugin pluginlib rosconsole_bridge roscpp cmake_modules)

#find_package(TinyXML REQUIRED)

# catkin_package(
#   LIBRARIES ${PROJECT_NAME}
#   INCLUDE_DIRS include ${TinyXML_INCLUDE_DIRS}
#   CATKIN_DEPENDS rosconsole_bridge roscpp
#   DEPENDS urdfdom_headers urdfdom Boost
# )

include_directories(SYSTEM ${Boost_INCLUDE_DIR} )
link_directories(${Boost_LIBRARY_DIRS})

include_directories(
  include
#  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )
#link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES}
#  assimp
#  yaml-cpp
  tinyxml)

# install(TARGETS ${PROJECT_NAME} 
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# IMPORTANT: Add the urdf library to the "export-set"
install(TARGETS urdf
  EXPORT AvizTargets
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib)
