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

#include <rviz_rpc/server.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rviz_rpc/ProtocolResponseHeader.h>

namespace rviz_rpc
{

struct Server::Impl
{
  Impl(const std::string& name, const ros::NodeHandle& nh);
  void ready();
  void addMethod(const std::string& name, const CallbackHelperPtr& helper);

  void callback(const ros::MessageEvent<RequestWrapper const>& evt);

  void subscriberConnected(const ros::SingleSubscriberPublisher& pub);

  template<typename T>
  void fillResponse(const ResponseWrapperPtr& res, const boost::shared_ptr<T>& msg)
  {
    res->message.message = msg;
    res->message.serialize = serialize<T>;
    res->message.serialized_length = serializedLength<T>;
    res->message.ti = &typeid(T);
  }

  ros::CallbackQueue cbqueue_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::string name_;

  typedef std::map<std::string, CallbackHelperPtr> M_Method;
  M_Method methods_;
};

Server::Impl::Impl(const std::string& name, const ros::NodeHandle& nh)
: nh_(nh, name)
, name_(name)
{
}

void Server::Impl::subscriberConnected(const ros::SingleSubscriberPublisher& pub)
{
  ResponseWrapperPtr res(boost::make_shared<ResponseWrapper>());
  res->request_id = rviz_uuid::UUID::Generate();
  res->protocol = Response::PROTOCOL_HEADER;

  ProtocolResponseHeaderPtr header(boost::make_shared<ProtocolResponseHeader>());

  M_Method::const_iterator it = methods_.begin();
  M_Method::const_iterator end = methods_.end();
  for (; it != end; ++it)
  {
    const std::string& name = it->first;
    const CallbackHelperPtr& helper = it->second;
    MethodSpec spec;
    spec.name = name;
    spec.request_md5sum = helper->getRequestMD5Sum();
    spec.request_datatype = helper->getRequestDataType();
    spec.request_definition = helper->getRequestDefinition();
    spec.response_md5sum = helper->getResponseMD5Sum();
    spec.response_datatype = helper->getResponseDataType();
    spec.response_definition = helper->getResponseDefinition();
    header->methods.push_back(spec);
  }

  fillResponse(res, header);

  pub.publish(res);
}

void Server::Impl::ready()
{
  ROS_ASSERT(!pub_);
  ROS_ASSERT(!sub_);
  pub_ = nh_.advertise<ResponseWrapper>("response", 0, boost::bind(&Impl::subscriberConnected, this, _1));
  sub_ = nh_.subscribe("request", 0, &Impl::callback, this);
}

void Server::Impl::addMethod(const std::string& name, const CallbackHelperPtr& helper)
{
  ROS_ASSERT(!pub_);
  ROS_ASSERT(!sub_);

  ROS_ASSERT(methods_.count(name) == 0);
  methods_[name] = helper;
}

void Server::Impl::callback(const ros::MessageEvent<RequestWrapper const>& evt)
{
  RequestWrapperConstPtr req = evt.getMessage();
  ResponseWrapperPtr res(new ResponseWrapper);
  res->request_id = req->request_id;
  M_Method::iterator it = methods_.find(req->method);
  if (it == methods_.end())
  {
    res->error_code = Response::UNKNOWN_METHOD;
    res->error_string = req->method;
    pub_.publish(res);
    return;
  }

  const CallbackHelperPtr& helper = it->second;

  try
  {
    res->message = helper->call(evt);
  }
  catch (std::exception& e)
  {
    res->error_code = Response::EXCEPTION;
    res->error_string = e.what();
  }

  pub_.publish(res);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface
//////////////////////////////////////////////////////////////////////////////////////////////////////

Server::Server(const std::string& name, const ros::NodeHandle& nh)
: impl_(new Impl(name, nh))
{

}

void Server::ready()
{
  impl_->ready();
}

void Server::addMethod(const std::string& name, const CallbackHelperPtr& helper)
{
  impl_->addMethod(name, helper);
}

} // namespace rviz_rpc
