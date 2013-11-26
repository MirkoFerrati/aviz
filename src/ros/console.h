 #include "/opt/ros/hydro/include/ros/console.h"


#undef ROS_INFO
#define ROS_INFO(...) printf(__VA_ARGS__); printf("\n")

#undef ROS_ERROR
#define ROS_ERROR(...) printf(__VA_ARGS__); printf("\n")


#undef ROS_WARN
#define ROS_WARN(...) printf(__VA_ARGS__); printf("\n")

#undef ROS_WARN_STREAM
#define ROS_WARN_STREAM(...) std::cout<<__VA_ARGS__<<std::endl

#undef ROS_ERROR_STREAM
#define ROS_ERROR_STREAM(...) std::cout<<__VA_ARGS__<<std::endl

#undef ROS_DEBUG_STREAM
#define ROS_DEBUG_STREAM(...) std::cout<<__VA_ARGS__<<std::endl

#undef ROS_DEBUG
#define ROS_DEBUG(...) printf(__VA_ARGS__); printf("\n")


#undef ROS_DEBUG_NAMED
#define ROS_DEBUG_NAMED(x,...) printf(__VA_ARGS__)


#undef ROS_LOG
#define ROS_LOG(x,y,...) printf(__VA_ARGS__); printf("\n")


#undef ROS_INFO_STREAM
#define ROS_INFO_STREAM(...) std::cout<<__VA_ARGS__<<std::endl
