//#ifndef ROS_H_MIRKO
//#define ROS_H_MIRKO


#define ROS_PACKAGE_NAME "/home/mirko/projects/walkman/rivz/rviz/"

//#include </opt/ros/hydro/include/ros/ros.h>

#include "ros/time.h"
#include "ros/rate.h"
//#define ROSCONSOLE_ROSCONSOLE_H
#include "ros/console.h"
//#define ROSCONSOLE_ROSASSERT_H
#include "ros/assert.h"

#include "ros/common.h"
#include "ros/types.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/single_subscriber_publisher.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "ros/service.h"
#include "ros/init.h"
#include "ros/master.h"
#include "ros/this_node.h"
#include "ros/param.h"
#include "ros/topic.h"
// #include "ros/names.h"



#undef ROS_INFO
#define ROS_INFO(...) printf(__VA_ARGS__)

#undef ROS_ERROR
#define ROS_ERROR(...) printf(__VA_ARGS__)


#undef ROS_WARN
#define ROS_WARN(...) printf(__VA_ARGS__)


#undef ROS_ERROR_STREAM
#define ROS_ERROR_STREAM(...) std::cout<<__VA_ARGS__<<std::endl

#undef ROS_DEBUG_STREAM
#define ROS_DEBUG_STREAM(...) std::cout<<__VA_ARGS__<<std::endl

#undef ROS_DEBUG
#define ROS_DEBUG(...) printf(__VA_ARGS__)


#undef ROS_DEBUG_NAMED
#define ROS_DEBUG_NAMED(x,...) printf(__VA_ARGS__)


#undef ROS_LOG
#define ROS_LOG(x,y,...) printf(__VA_ARGS__)


#undef ROS_INFO_STREAM
#define ROS_INFO_STREAM(...) std::cout<<__VA_ARGS__<<std::endl

#include "ros/time.h"//Fake time
//#endif //ROS_H_MIRKO
