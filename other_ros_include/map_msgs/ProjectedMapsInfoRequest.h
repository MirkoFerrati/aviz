/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /tmp/buildd/ros-hydro-map-msgs-0.0.2-0raring-20131010-0223/srv/ProjectedMapsInfo.srv
 *
 */


#ifndef MAP_MSGS_MESSAGE_PROJECTEDMAPSINFOREQUEST_H
#define MAP_MSGS_MESSAGE_PROJECTEDMAPSINFOREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <map_msgs/ProjectedMapInfo.h>

namespace map_msgs
{
template <class ContainerAllocator>
struct ProjectedMapsInfoRequest_
{
  typedef ProjectedMapsInfoRequest_<ContainerAllocator> Type;

  ProjectedMapsInfoRequest_()
    : projected_maps_info()  {
    }
  ProjectedMapsInfoRequest_(const ContainerAllocator& _alloc)
    : projected_maps_info(_alloc)  {
    }



   typedef std::vector< ::map_msgs::ProjectedMapInfo_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::map_msgs::ProjectedMapInfo_<ContainerAllocator> >::other >  _projected_maps_info_type;
  _projected_maps_info_type projected_maps_info;




  typedef boost::shared_ptr< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct ProjectedMapsInfoRequest_

typedef ::map_msgs::ProjectedMapsInfoRequest_<std::allocator<void> > ProjectedMapsInfoRequest;

typedef boost::shared_ptr< ::map_msgs::ProjectedMapsInfoRequest > ProjectedMapsInfoRequestPtr;
typedef boost::shared_ptr< ::map_msgs::ProjectedMapsInfoRequest const> ProjectedMapsInfoRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace map_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/hydro/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/hydro/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/hydro/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg'], 'map_msgs': ['/tmp/buildd/ros-hydro-map-msgs-0.0.2-0raring-20131010-0223/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7980a33202421c8cd74565e57a4d229";
  }

  static const char* value(const ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7980a33202421c8ULL;
  static const uint64_t static_value2 = 0xcd74565e57a4d229ULL;
};

template<class ContainerAllocator>
struct DataType< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "map_msgs/ProjectedMapsInfoRequest";
  }

  static const char* value(const ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "map_msgs/ProjectedMapInfo[] projected_maps_info\n\
\n\
\n\
================================================================================\n\
MSG: map_msgs/ProjectedMapInfo\n\
string frame_id\n\
float64 x\n\
float64 y\n\
float64 width\n\
float64 height\n\
float64 min_z\n\
float64 max_z\n\
";
  }

  static const char* value(const ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.projected_maps_info);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ProjectedMapsInfoRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::map_msgs::ProjectedMapsInfoRequest_<ContainerAllocator>& v)
  {
    s << indent << "projected_maps_info[]" << std::endl;
    for (size_t i = 0; i < v.projected_maps_info.size(); ++i)
    {
      s << indent << "  projected_maps_info[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::map_msgs::ProjectedMapInfo_<ContainerAllocator> >::stream(s, indent + "    ", v.projected_maps_info[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAP_MSGS_MESSAGE_PROJECTEDMAPSINFOREQUEST_H
