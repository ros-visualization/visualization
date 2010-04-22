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

#include <rviz_rpc/client.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rviz_rpc/Request.h>
#include <rviz_rpc/Response.h>

namespace rviz_rpc
{

struct Client::Impl
{
  Impl(const std::string& name, const ros::NodeHandle& nh);

  void connect();
  ResponseWrapperConstPtr call(const RequestWrapperPtr& req);
  void callAsync(const RequestWrapperPtr& req);
  void cb(const ResponseWrapperConstPtr& res);
  void addMethod(const std::string& name);

  ros::CallbackQueue cbqueue_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::NodeHandle nh_;

  struct RequestInfo
  {
    RequestInfo()
    : received(false)
    {}

    rviz_uuid::UUID id;
    ResponseWrapperConstPtr res;
    volatile bool received;
  };
  typedef boost::shared_ptr<RequestInfo> RequestInfoPtr;

  typedef std::map<rviz_uuid::UUID, RequestInfoPtr> M_RequestInfo;
  M_RequestInfo requests_;
  boost::mutex requests_mutex_;
};

Client::Impl::Impl(const std::string& name, const ros::NodeHandle& nh)
: nh_(nh, name)
{
}

void Client::Impl::connect()
{
  pub_ = nh_.advertise<RequestWrapper>("request", 0);

  ros::SubscribeOptions sub_ops;
  sub_ops.init<ResponseWrapper>("response", 0, boost::bind(&Impl::cb, this, _1));
  sub_ops.callback_queue = &cbqueue_;
  sub_ = nh_.subscribe(sub_ops);

  while (pub_.getNumSubscribers() == 0 || sub_.getNumPublishers() == 0)
  {
    ros::WallDuration(0.01).sleep();
  }
}

void Client::Impl::cb(const ResponseWrapperConstPtr& res)
{
  rviz_uuid::UUID id = res->request_id;

  {
    boost::mutex::scoped_lock lock(requests_mutex_);
    M_RequestInfo::iterator it = requests_.find(id);
    if (it != requests_.end())
    {
      it->second->res = res;
      it->second->received = true;
    }
  }
}

void Client::Impl::addMethod(const std::string& name)
{

}

ResponseWrapperConstPtr Client::Impl::call(const RequestWrapperPtr& req)
{
  req->request_id = rviz_uuid::UUID::Generate();

  RequestInfoPtr info(boost::make_shared<RequestInfo>());
  info->id = req->request_id;

  {
    boost::mutex::scoped_lock lock(requests_mutex_);
    requests_[info->id] = info;
  }

  pub_.publish(req);

  while (!info->received)
  {
    cbqueue_.callOne(ros::WallDuration(0.1));
  }

  ROS_ASSERT(info->res);

  uint8_t error_code = info->res->error_code;
  if (error_code == Response::SUCCESS)
  {
    return info->res;
  }
  else if (error_code == Response::EXCEPTION)
  {
    throw CallException(info->res->error_string);
  }
  else if (error_code == Response::UNKNOWN_METHOD)
  {
    std::stringstream ss;
    ss << "Server does not have method [" << info->res->error_string << "]";
    throw CallException(ss.str());
  }
  else
  {
    std::stringstream ss;
    ss << "Unknown error code [" << error_code << "] while calling [" << pub_.getTopic() << "]";
    throw CallException(ss.str());
  }
}

void Client::Impl::callAsync(const RequestWrapperPtr& req)
{
  req->request_id = rviz_uuid::UUID::Generate();
  pub_.publish(req);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

Client::Client(const std::string& name, const ros::NodeHandle& nh)
: impl_(new Impl(name, nh))
{

}

void Client::connect()
{
  impl_->connect();
}

ResponseWrapperConstPtr Client::call(const RequestWrapperPtr& req)
{
  return impl_->call(req);
}

void Client::callAsync(const RequestWrapperPtr& req)
{
  impl_->callAsync(req);
}

void Client::addMethod(const std::string& name)
{
  impl_->addMethod(name);
}

} // namespace rviz_rpc
