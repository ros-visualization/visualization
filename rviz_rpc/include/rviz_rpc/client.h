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

#ifndef RVIZ_RPC_CLIENT_H
#define RVIZ_RPC_CLIENT_H


#include "exceptions.h"
#include <rviz_rpc/Request.h>
#include <rviz_rpc/Response.h>
#include <rviz_uuid/uuid.h>

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ros/serialization.h>

namespace ros
{
class NodeHandle;
}

namespace rviz_rpc
{

template<typename Req, typename Res>
class Method
{
public:
  typedef boost::shared_ptr<Req> ReqPtr;
  typedef boost::shared_ptr<Req const> ReqConstPtr;
  typedef boost::shared_ptr<Res> ResPtr;
  typedef boost::shared_ptr<Res const> ResConstPtr;

  typedef boost::function<ResponseConstPtr(const RequestPtr&)> CallFn;
  typedef boost::function<void(const RequestPtr&)> AsyncCallFn;

  Method() {}

  Method(const std::string& name, const CallFn& c, const AsyncCallFn& ac)
  : impl_(new Impl)
  {
    impl_->name = name;
    impl_->call_fn = c;
    impl_->async_call_fn = ac;
  }

  ResConstPtr call(const ReqConstPtr& req)
  {
    ROS_ASSERT(impl_);

    namespace ser = ros::serialization;

    RequestPtr request(new Request);
    request->method = impl_->name;

    ros::SerializedMessage sm = ser::serializeMessage(*req);
    request->data.insert(request->data.end(), sm.message_start, sm.message_start + sm.num_bytes);

    ResponseConstPtr response = impl_->call_fn(request);

    ResPtr res = boost::make_shared<Res>();
    ROS_ASSERT(!response->data.empty());
    ser::IStream stream((uint8_t*)&response->data.front(), response->data.size());
    ser::deserialize(stream, *res);

    return res;
  }

  void callAsync(const ReqConstPtr& req)
  {
    ROS_ASSERT(impl_);

    namespace ser = ros::serialization;

    RequestPtr request(new Request);
    request->method = impl_->name;

    ros::SerializedMessage sm = ser::serializeMessage(*req);
    request->data.insert(request->data.end(), sm.buf.get(), sm.buf.get() + sm.num_bytes);

    impl_->async_call_fn(request);
  }

private:
  struct Impl
  {
    CallFn call_fn;
    AsyncCallFn async_call_fn;
    std::string name;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;
  ImplPtr impl_;
};

class Client
{
public:
  Client(const std::string& name, const ros::NodeHandle& nh);
  void connect();

  template<typename Req, typename Res>
  Method<Req, Res> addMethod(const std::string& name)
  {
    Method<Req, Res> method(name, boost::bind(&Client::call, this, _1), boost::bind(&Client::callAsync, this, _1));
    addMethod(name);
    return method;
  }

private:
  void addMethod(const std::string& name);
  ResponseConstPtr call(const RequestPtr& req);
  void callAsync(const RequestPtr& req);

  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  ImplPtr impl_;
};

} // namespace rviz_rpc

#endif // RVIZ_RPC_CLIENT_H
