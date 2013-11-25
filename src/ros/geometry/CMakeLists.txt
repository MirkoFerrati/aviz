include_directories(../geometry_experimental/tf2/include)
include_directories(../geometry_experimental/tf2_ros/include)
include_directories("/opt/ros/hydro/include")

add_subdirectory(eigen_conversions)
include_directories(eigen_conversions)


add_subdirectory(kdl_conversions)
include_directories(kdl_conversions)

add_subdirectory(tf)
include_directories(tf)

add_subdirectory(tf_conversions)
include_directories(tf_conversions)
