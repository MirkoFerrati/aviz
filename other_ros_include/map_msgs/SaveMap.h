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
 * Auto-generated by gensrv_cpp from file /tmp/buildd/ros-hydro-map-msgs-0.0.2-0raring-20131010-0223/srv/SaveMap.srv
 *
 */


#ifndef MAP_MSGS_MESSAGE_SAVEMAP_H
#define MAP_MSGS_MESSAGE_SAVEMAP_H

#include <ros/service_traits.h>


#include <map_msgs/SaveMapRequest.h>
#include <map_msgs/SaveMapResponse.h>


namespace map_msgs
{

struct SaveMap
{

typedef SaveMapRequest Request;
typedef SaveMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SaveMap
} // namespace map_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::map_msgs::SaveMap > {
  static const char* value()
  {
    return "716e25f9d9dc76ceba197f93cbf05dc7";
  }

  static const char* value(const ::map_msgs::SaveMap&) { return value(); }
};

template<>
struct DataType< ::map_msgs::SaveMap > {
  static const char* value()
  {
    return "map_msgs/SaveMap";
  }

  static const char* value(const ::map_msgs::SaveMap&) { return value(); }
};


// service_traits::MD5Sum< ::map_msgs::SaveMapRequest> should match 
// service_traits::MD5Sum< ::map_msgs::SaveMap > 
template<>
struct MD5Sum< ::map_msgs::SaveMapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::map_msgs::SaveMap >::value();
  }
  static const char* value(const ::map_msgs::SaveMapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::map_msgs::SaveMapRequest> should match 
// service_traits::DataType< ::map_msgs::SaveMap > 
template<>
struct DataType< ::map_msgs::SaveMapRequest>
{
  static const char* value()
  {
    return DataType< ::map_msgs::SaveMap >::value();
  }
  static const char* value(const ::map_msgs::SaveMapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::map_msgs::SaveMapResponse> should match 
// service_traits::MD5Sum< ::map_msgs::SaveMap > 
template<>
struct MD5Sum< ::map_msgs::SaveMapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::map_msgs::SaveMap >::value();
  }
  static const char* value(const ::map_msgs::SaveMapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::map_msgs::SaveMapResponse> should match 
// service_traits::DataType< ::map_msgs::SaveMap > 
template<>
struct DataType< ::map_msgs::SaveMapResponse>
{
  static const char* value()
  {
    return DataType< ::map_msgs::SaveMap >::value();
  }
  static const char* value(const ::map_msgs::SaveMapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAP_MSGS_MESSAGE_SAVEMAP_H
