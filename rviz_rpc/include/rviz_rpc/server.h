/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_RPC_SERVER_H
#define RVIZ_RPC_SERVER_H

#include "exceptions.h"
#include "rviz_rpc/Request.h"

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/function.hpp>

#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/is_const.hpp>

#include <ros/serialization.h>
#include <ros/message_event.h>

namespace ros
{
class NodeHandle;
class Publisher;
}

namespace rviz_rpc
{

typedef boost::shared_ptr<void> VoidPtr;
typedef boost::shared_ptr<void const> VoidConstPtr;

template<typename T>
ros::SerializedMessage serialize(const VoidConstPtr& message)
{
  namespace ser = ros::serialization;
  boost::shared_ptr<T const> m = boost::static_pointer_cast<T const>(message);
  return ser::serializeMessage(*m);
}

struct SerializableMessage
{
  VoidConstPtr message;
  boost::function<ros::SerializedMessage(const VoidConstPtr& message)> serialize;
};

class CallbackHelper
{
public:
  virtual ~CallbackHelper() {}
  virtual SerializableMessage call(const ros::MessageEvent<Request const>& incoming_event) = 0;
  virtual const std::type_info& getRequestTypeInfo() = 0;
  virtual const std::type_info& getResponseTypeInfo() = 0;
};
typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;

template<typename Req, typename Res>
class CallbackHelperT : public CallbackHelper
{
public:
  typedef boost::shared_ptr<Req> ReqPtr;
  typedef boost::shared_ptr<Req const> ReqConstPtr;
  typedef ros::MessageEvent<Req const> ReqEvent;
  typedef boost::shared_ptr<Res> ResPtr;
  typedef boost::shared_ptr<Res const> ResConstPtr;
  typedef boost::function<ResConstPtr(const ReqEvent&)> Callback;

  CallbackHelperT(const Callback& cb)
  : cb_(cb)
  {}

  virtual SerializableMessage call(const ros::MessageEvent<Request const>& incoming_event)
  {
    namespace ser = ros::serialization;

    ReqPtr req = boost::make_shared<Req>();

    if (!req)
    {
      throw CallException("Allocate failed");
    }

    ROS_ASSERT(!incoming_event.getMessage()->data.empty());
    ser::IStream stream((uint8_t*)&incoming_event.getMessage()->data.front(), incoming_event.getMessage()->data.size());
    ser::deserialize(stream, *req);

    ros::MessageEvent<Req> evt(req, incoming_event.getConnectionHeaderPtr(), ros::Time::now());

    ResConstPtr res = cb_(evt);
    SerializableMessage sm;
    sm.message = res;
    sm.serialize = serialize<Res>;
    return sm;
  }

  virtual const std::type_info& getRequestTypeInfo() { return typeid(Req); }
  virtual const std::type_info& getResponseTypeInfo() { return typeid(Res); }

private:
  Callback cb_;
};

class Server
{
public:
  Server(const std::string& name, const ros::NodeHandle& nh);

  template<typename Req, typename Res>
  void addMethod(const std::string& name, const boost::function<boost::shared_ptr<Res const>(const ros::MessageEvent<Req const>&)>& callback)
  {
    CallbackHelperPtr helper(new CallbackHelperT<Req, Res>(callback));
    addMethod(name, helper);
  }

  void addMethod(const std::string& name, const CallbackHelperPtr& helper);

  void ready();

private:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  ImplPtr impl_;
};

} // namespace rviz_rpc

#endif // RVIZ_RPC_SERVER_H
