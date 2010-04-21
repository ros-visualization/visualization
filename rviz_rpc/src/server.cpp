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

#include <rviz_rpc/Request.h>
#include <rviz_rpc/Response.h>

namespace rviz_rpc
{

struct Server::Impl
{
  Impl(const std::string& name, const ros::NodeHandle& nh);
  void ready();
  void addMethod(const std::string& name, const CallbackHelperPtr& helper);

  void callback(const ros::MessageEvent<rviz_rpc::Request const>& evt);

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

void Server::Impl::ready()
{
  ROS_ASSERT(!pub_);
  ROS_ASSERT(!sub_);
  pub_ = nh_.advertise<rviz_rpc::Response>("response", 0);
  sub_ = nh_.subscribe("request", 0, &Impl::callback, this);
}

void Server::Impl::addMethod(const std::string& name, const CallbackHelperPtr& helper)
{
  ROS_ASSERT(!pub_);
  ROS_ASSERT(!sub_);

  ROS_ASSERT(methods_.count(name) == 0);
  methods_[name] = helper;
}

void Server::Impl::callback(const ros::MessageEvent<rviz_rpc::Request const>& evt)
{
  RequestConstPtr req = evt.getMessage();
  ResponsePtr res(new Response);
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
    SerializableMessage m = helper->call(evt);
    ros::SerializedMessage sm = m.serialize(m.message);

    res->data.insert(res->data.end(), sm.message_start, sm.message_start + sm.num_bytes);
    pub_.publish(res);
  }
  catch (std::exception& e)
  {
    res->error_code = Response::EXCEPTION;
    res->error_string = e.what();
    pub_.publish(res);
    return;
  }
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
