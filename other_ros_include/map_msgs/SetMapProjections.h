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
 * Auto-generated by gensrv_cpp from file /tmp/buildd/ros-hydro-map-msgs-0.0.2-0raring-20131010-0223/srv/SetMapProjections.srv
 *
 */


#ifndef MAP_MSGS_MESSAGE_SETMAPPROJECTIONS_H
#define MAP_MSGS_MESSAGE_SETMAPPROJECTIONS_H

#include <ros/service_traits.h>


#include <map_msgs/SetMapProjectionsRequest.h>
#include <map_msgs/SetMapProjectionsResponse.h>


namespace map_msgs
{

struct SetMapProjections
{

typedef SetMapProjectionsRequest Request;
typedef SetMapProjectionsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetMapProjections
} // namespace map_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::map_msgs::SetMapProjections > {
  static const char* value()
  {
    return "d7980a33202421c8cd74565e57a4d229";
  }

  static const char* value(const ::map_msgs::SetMapProjections&) { return value(); }
};

template<>
struct DataType< ::map_msgs::SetMapProjections > {
  static const char* value()
  {
    return "map_msgs/SetMapProjections";
  }

  static const char* value(const ::map_msgs::SetMapProjections&) { return value(); }
};


// service_traits::MD5Sum< ::map_msgs::SetMapProjectionsRequest> should match 
// service_traits::MD5Sum< ::map_msgs::SetMapProjections > 
template<>
struct MD5Sum< ::map_msgs::SetMapProjectionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::map_msgs::SetMapProjections >::value();
  }
  static const char* value(const ::map_msgs::SetMapProjectionsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::map_msgs::SetMapProjectionsRequest> should match 
// service_traits::DataType< ::map_msgs::SetMapProjections > 
template<>
struct DataType< ::map_msgs::SetMapProjectionsRequest>
{
  static const char* value()
  {
    return DataType< ::map_msgs::SetMapProjections >::value();
  }
  static const char* value(const ::map_msgs::SetMapProjectionsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::map_msgs::SetMapProjectionsResponse> should match 
// service_traits::MD5Sum< ::map_msgs::SetMapProjections > 
template<>
struct MD5Sum< ::map_msgs::SetMapProjectionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::map_msgs::SetMapProjections >::value();
  }
  static const char* value(const ::map_msgs::SetMapProjectionsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::map_msgs::SetMapProjectionsResponse> should match 
// service_traits::DataType< ::map_msgs::SetMapProjections > 
template<>
struct DataType< ::map_msgs::SetMapProjectionsResponse>
{
  static const char* value()
  {
    return DataType< ::map_msgs::SetMapProjections >::value();
  }
  static const char* value(const ::map_msgs::SetMapProjectionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAP_MSGS_MESSAGE_SETMAPPROJECTIONS_H