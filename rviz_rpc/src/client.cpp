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
#include <rviz_rpc/ProtocolResponseHeader.h>

namespace rviz_rpc
{

struct Client::Impl
{
  Impl(const std::string& name, const ros::NodeHandle& nh);

  void connect();
  void connectAsync();
  bool isConnected() { return connected_; }
  ResponseWrapperConstPtr call(const RequestWrapperPtr& req);
  void callAsync(const RequestWrapperPtr& req);
  void cb(const ros::MessageEvent<ResponseWrapper const>& res);
  void addMethod(const MethodInfo& method);
  void handleHeader(const ros::MessageEvent<ResponseWrapper const>& evt);

  void subAndPub();
  void waitForConnection();

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

  typedef std::map<std::string, MethodInfo> M_MethodInfo;
  M_MethodInfo methods_;

  typedef std::set<std::string> S_string;
  S_string broken_methods_;

  bool header_received_;
  bool connected_;

  void onQueueTimer(const ros::WallTimerEvent&);
  ros::WallTimer queue_timer_;
};

Client::Impl::Impl(const std::string& name, const ros::NodeHandle& nh)
: nh_(nh, name)
, header_received_(false)
, connected_(false)
{
}

void Client::Impl::subAndPub()
{
  ROS_ASSERT(!pub_);
  ROS_ASSERT(!sub_);
  ROS_ASSERT(!queue_timer_);

  pub_ = nh_.advertise<RequestWrapper>("request", 0);

  ros::SubscribeOptions sub_ops;
  sub_ops.initByFullCallbackType<const ros::MessageEvent<ResponseWrapper>&>("response", 0, boost::bind(&Impl::cb, this, _1));
  sub_ops.callback_queue = &cbqueue_;
  sub_ = nh_.subscribe(sub_ops);

  queue_timer_ = nh_.createWallTimer(ros::WallDuration(0.05), &Impl::onQueueTimer, this);
}

void Client::Impl::connect()
{
  subAndPub();
  waitForConnection();
}

void Client::Impl::waitForConnection()
{
  while (nh_.ok() && !connected_)
  {
    cbqueue_.callAvailable(ros::WallDuration(0.1));
  }
}

void Client::Impl::onQueueTimer(const ros::WallTimerEvent&)
{
  cbqueue_.callAvailable();
}

void Client::Impl::connectAsync()
{
  subAndPub();
}

void Client::Impl::handleHeader(const ros::MessageEvent<ResponseWrapper const>& evt)
{
  ResponseWrapperConstPtr res = evt.getMessage();

  try
  {
    ProtocolResponseHeaderConstPtr header = res->instantiate<ProtocolResponseHeader>();
    std::set<std::string> server_methods;
    for (size_t i = 0; i < header->methods.size(); ++i)
    {
      const MethodSpec& spec = header->methods[i];
      server_methods.insert(spec.name);

      M_MethodInfo::iterator it = methods_.find(spec.name);
      if (it != methods_.end())
      {
        const MethodInfo& info = it->second;
        if (info.request_md5sum != spec.request_md5sum)
        {
          ROS_ERROR("RPC Server wants method [%s:%s] to have request of type [%s/%s] but I have [%s/%s]", nh_.getNamespace().c_str(), spec.name.c_str(),
              spec.request_datatype.c_str(), spec.request_md5sum.c_str(), info.request_datatype.c_str(), info.request_md5sum.c_str());

          broken_methods_.insert(info.name);
        }

        if (info.response_md5sum != spec.response_md5sum)
        {
          ROS_ERROR("RPC Server wants method [%s:%s] to have response of type [%s/%s] but I have [%s/%s]", nh_.getNamespace().c_str(), spec.name.c_str(),
              spec.response_datatype.c_str(), spec.response_md5sum.c_str(), info.response_datatype.c_str(), info.response_md5sum.c_str());

          broken_methods_.insert(info.name);
        }
      }
      else
      {
        ROS_WARN("RPC Server for [%s] at [%s] has extra method [%s]", nh_.getNamespace().c_str(), evt.getPublisherName().c_str(), spec.name.c_str());
      }
    }

    M_MethodInfo::iterator it = methods_.begin();
    M_MethodInfo::iterator end = methods_.end();
    for (; it != end; ++it)
    {
      const std::string& name = it->first;
      if (server_methods.find(name) == server_methods.end())
      {
        ROS_ERROR("RPC Server for [%s] at [%s] does not have client method [%s]", nh_.getNamespace().c_str(), evt.getPublisherName().c_str(), name.c_str());
        broken_methods_.insert(name);
      }
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Exception thrown while processing header from rpc server for [%s] at [%s]", nh_.getNamespace().c_str(), evt.getPublisherName().c_str());
  }

  header_received_ = true;
  connected_ = true;
}

void Client::Impl::cb(const ros::MessageEvent<ResponseWrapper const>& evt)
{
  ResponseWrapperConstPtr res = evt.getMessage();
  if (res->protocol != 0)
  {
    switch (res->protocol)
    {
    case Response::PROTOCOL_HEADER:
      handleHeader(evt);
      break;
    default:
      ROS_ERROR("Unknown protocol message type [%d]", res->protocol);
    }
  }
  else
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
}

void Client::Impl::addMethod(const MethodInfo& method)
{
  ROS_ASSERT(methods_.count(method.name) == 0);

  methods_[method.name] = method;
}

ResponseWrapperConstPtr Client::Impl::call(const RequestWrapperPtr& req)
{
  if (!connected_)
  {
    throw CallException("Client to [" + nh_.getNamespace() + "] is not connected");
  }

  if (broken_methods_.find(req->method) != broken_methods_.end())
  {
    throw CallException("Unconnected method [" + req->method + "], see previously generated errors for more details");
  }

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

void Client::connectAsync()
{
  impl_->connectAsync();
}

bool Client::isConnected()
{
  return impl_->isConnected();
}

void Client::waitForConnection()
{
  impl_->waitForConnection();
}

ResponseWrapperConstPtr Client::call(const RequestWrapperPtr& req)
{
  return impl_->call(req);
}

void Client::callAsync(const RequestWrapperPtr& req)
{
  impl_->callAsync(req);
}

void Client::addMethod(const MethodInfo& method)
{
  impl_->addMethod(method);
}

} // namespace rviz_rpc
