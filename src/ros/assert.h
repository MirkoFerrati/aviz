
#include </opt/ros/hydro/include/ros/assert.h>

#include <assert.h>

#undef ROS_ASSERT
#define ROS_ASSERT(...) assert(__VA_ARGS__)
