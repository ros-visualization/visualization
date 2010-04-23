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
#include "request_wrapper.h"
#include "response_wrapper.h"
#include <rviz_uuid/uuid.h>

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

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

  typedef boost::function<ResponseWrapperConstPtr(const RequestWrapperPtr&)> CallFn;
  typedef boost::function<void(const RequestWrapperPtr&)> AsyncCallFn;

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

    RequestWrapperPtr request(boost::make_shared<RequestWrapper>());
    request->method = impl_->name;
    request->message.message = req;
    request->message.serialize = serialize<Req>;
    request->message.serialized_length = serializedLength<Req>;
    request->message.ti = &typeid(Req);

    ResponseWrapperConstPtr response = impl_->call_fn(request);
    return response->instantiate<Res>();
  }

  void callAsync(const ReqConstPtr& req)
  {
    ROS_ASSERT(impl_);

    RequestWrapperPtr request(boost::make_shared<RequestWrapper>());
    request->method = impl_->name;
    request->message.message = req;
    request->message.serialize = serialize<Req>;
    request->message.serialized_length = serializedLength<Req>;
    request->message.ti = &typeid(Req);

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
private:
  struct MethodInfo
  {
    std::string name;
    std::string request_md5sum;
    std::string request_datatype;
    std::string request_definition;
    std::string response_md5sum;
    std::string response_datatype;
    std::string response_definition;
  };

public:
  Client(const std::string& name, const ros::NodeHandle& nh);
  void connect();
  void connectAsync();
  void waitForConnection();
  bool isConnected();

  template<typename Req, typename Res>
  Method<Req, Res> addMethod(const std::string& name)
  {
    Method<Req, Res> method(name, boost::bind(&Client::call, this, _1), boost::bind(&Client::callAsync, this, _1));
    MethodInfo info;
    info.name = name;
    info.request_md5sum = ros::message_traits::md5sum<Req>();
    info.request_datatype = ros::message_traits::datatype<Req>();
    info.request_definition = ros::message_traits::definition<Req>();
    info.response_md5sum = ros::message_traits::md5sum<Res>();
    info.response_datatype = ros::message_traits::datatype<Res>();
    info.response_definition = ros::message_traits::definition<Res>();
    addMethod(info);
    return method;
  }

private:
  void addMethod(const MethodInfo& name);
  ResponseWrapperConstPtr call(const RequestWrapperPtr& req);
  void callAsync(const RequestWrapperPtr& req);

  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  ImplPtr impl_;
};

} // namespace rviz_rpc

#endif // RVIZ_RPC_CLIENT_H
