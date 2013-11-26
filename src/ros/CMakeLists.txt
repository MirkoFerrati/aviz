include_directories(geometry_experimental/tf2/include)

add_subdirectory(robot_model)
include_directories(robot_model)
add_subdirectory(geometry)
include_directories(geometry)
add_subdirectory(geometry_experimental)
include_directories(geometry_experimental)

add_library(package package.cpp)
add_library(time time.cpp)