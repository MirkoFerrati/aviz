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
 * Auto-generated by genmsg_cpp from file /tmp/buildd/ros-hydro-visualization-msgs-1.10.2-0raring-20131010-0206/msg/InteractiveMarkerFeedback.msg
 *
 */


#ifndef VISUALIZATION_MSGS_MESSAGE_INTERACTIVEMARKERFEEDBACK_H
#define VISUALIZATION_MSGS_MESSAGE_INTERACTIVEMARKERFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

namespace visualization_msgs
{
template <class ContainerAllocator>
struct InteractiveMarkerFeedback_
{
  typedef InteractiveMarkerFeedback_<ContainerAllocator> Type;

  InteractiveMarkerFeedback_()
    : header()
    , client_id()
    , marker_name()
    , control_name()
    , event_type(0)
    , pose()
    , menu_entry_id(0)
    , mouse_point()
    , mouse_point_valid(false)  {
    }
  InteractiveMarkerFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , client_id(_alloc)
    , marker_name(_alloc)
    , control_name(_alloc)
    , event_type(0)
    , pose(_alloc)
    , menu_entry_id(0)
    , mouse_point(_alloc)
    , mouse_point_valid(false)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _client_id_type;
  _client_id_type client_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _marker_name_type;
  _marker_name_type marker_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _control_name_type;
  _control_name_type control_name;

   typedef uint8_t _event_type_type;
  _event_type_type event_type;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef uint32_t _menu_entry_id_type;
  _menu_entry_id_type menu_entry_id;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _mouse_point_type;
  _mouse_point_type mouse_point;

   typedef uint8_t _mouse_point_valid_type;
  _mouse_point_valid_type mouse_point_valid;


    enum { KEEP_ALIVE = 0 };
     enum { POSE_UPDATE = 1 };
     enum { MENU_SELECT = 2 };
     enum { BUTTON_CLICK = 3 };
     enum { MOUSE_DOWN = 4 };
     enum { MOUSE_UP = 5 };
 

  typedef boost::shared_ptr< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct InteractiveMarkerFeedback_

typedef ::visualization_msgs::InteractiveMarkerFeedback_<std::allocator<void> > InteractiveMarkerFeedback;

typedef boost::shared_ptr< ::visualization_msgs::InteractiveMarkerFeedback > InteractiveMarkerFeedbackPtr;
typedef boost::shared_ptr< ::visualization_msgs::InteractiveMarkerFeedback const> InteractiveMarkerFeedbackConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace visualization_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/tmp/buildd/ros-hydro-visualization-msgs-1.10.2-0raring-20131010-0206/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab0f1eee058667e28c19ff3ffc3f4b78";
  }

  static const char* value(const ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab0f1eee058667e2ULL;
  static const uint64_t static_value2 = 0x8c19ff3ffc3f4b78ULL;
};

template<class ContainerAllocator>
struct DataType< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visualization_msgs/InteractiveMarkerFeedback";
  }

  static const char* value(const ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Time/frame info.\n\
Header header\n\
\n\
# Identifying string. Must be unique in the topic namespace.\n\
string client_id\n\
\n\
# Feedback message sent back from the GUI, e.g.\n\
# when the status of an interactive marker was modified by the user.\n\
\n\
# Specifies which interactive marker and control this message refers to\n\
string marker_name\n\
string control_name\n\
\n\
# Type of the event\n\
# KEEP_ALIVE: sent while dragging to keep up control of the marker\n\
# MENU_SELECT: a menu entry has been selected\n\
# BUTTON_CLICK: a button control has been clicked\n\
# POSE_UPDATE: the pose has been changed using one of the controls\n\
uint8 KEEP_ALIVE = 0\n\
uint8 POSE_UPDATE = 1\n\
uint8 MENU_SELECT = 2\n\
uint8 BUTTON_CLICK = 3\n\
\n\
uint8 MOUSE_DOWN = 4\n\
uint8 MOUSE_UP = 5\n\
\n\
uint8 event_type\n\
\n\
# Current pose of the marker\n\
# Note: Has to be valid for all feedback types.\n\
geometry_msgs/Pose pose\n\
\n\
# Contains the ID of the selected menu entry\n\
# Only valid for MENU_SELECT events.\n\
uint32 menu_entry_id\n\
\n\
# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point\n\
# may contain the 3 dimensional position of the event on the\n\
# control.  If it does, mouse_point_valid will be true.  mouse_point\n\
# will be relative to the frame listed in the header.\n\
geometry_msgs/Point mouse_point\n\
bool mouse_point_valid\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.client_id);
      stream.next(m.marker_name);
      stream.next(m.control_name);
      stream.next(m.event_type);
      stream.next(m.pose);
      stream.next(m.menu_entry_id);
      stream.next(m.mouse_point);
      stream.next(m.mouse_point_valid);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct InteractiveMarkerFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visualization_msgs::InteractiveMarkerFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "client_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.client_id);
    s << indent << "marker_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.marker_name);
    s << indent << "control_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.control_name);
    s << indent << "event_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.event_type);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "menu_entry_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.menu_entry_id);
    s << indent << "mouse_point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.mouse_point);
    s << indent << "mouse_point_valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mouse_point_valid);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISUALIZATION_MSGS_MESSAGE_INTERACTIVEMARKERFEEDBACK_H
