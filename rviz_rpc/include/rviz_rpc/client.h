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


#include "traits.h"
#include "exceptions.h"
#include <rviz_uuid/uuid.h>

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

namespace ros
{
class NodeHandle;
}

namespace rviz_rpc
{

template<typename Req, typename Res>
class Client
{
public:
  typedef boost::shared_ptr<Req> ReqPtr;
  typedef boost::shared_ptr<Req const> ReqConstPtr;
  typedef boost::shared_ptr<Res> ResPtr;
  typedef boost::shared_ptr<Res const> ResConstPtr;

  Client(const std::string& name, const ros::NodeHandle& nh)
  : nh_(nh, name)
  {
    pub_ = nh_.advertise<Req>("request", 0);

    ros::SubscribeOptions sub_ops;
    sub_ops.init<Res>("response", 0, boost::bind(&Client::cb, this, _1));
    sub_ops.callback_queue = &cbqueue_;
    sub_ = nh_.subscribe(sub_ops);
  }

  void waitForServer()
  {
    while (sub_.getNumPublishers() == 0 && pub_.getNumSubscribers() == 0)
    {
      ros::WallDuration(0.01).sleep();
    }
  }

  ResConstPtr call(const ReqPtr& req)
  {
    RequestInfoPtr info(new RequestInfo);
    info->id = rviz_uuid::UUID::Generate();
    traits::RequestID<Req>::reference(*req) = info->id;

    {
      boost::mutex::scoped_lock lock(requests_mutex_);
      requests_.insert(std::make_pair(info->id, info));
    }

    pub_.publish(req);

    while (!info->received)
    {
      cbqueue_.callOne(ros::WallDuration(0.1));
    }

    ROS_ASSERT(info->res);

    uint8_t error_code = traits::ErrorCode<Res>::value(*info->res);
    if (error_code == error_codes::Success)
    {
      return info->res;
    }
    else if (error_code == error_codes::Exception)
    {
      throw CallException(traits::ErrorString<Res>::constReference(*info->res));
    }
    else
    {
      std::stringstream ss;
      ss << "Unknown error code [" << error_code << "] while calling [" << pub_.getTopic() << "]";
      throw CallException(ss.str().c_str());
    }
  }

  void callAsync(const ReqPtr& req)
  {
    traits::RequestID<Req>::reference(*req) = rviz_uuid::UUID::Generate();
    pub_.publish(req);
  }

private:
  void cb(const ResConstPtr& res)
  {
    rviz_uuid::UUID id = traits::RequestID<Res>::value(*res);

    {
      boost::mutex::scoped_lock lock(requests_mutex_);
      typename M_RequestInfo::iterator it = requests_.find(id);
      if (it != requests_.end())
      {
        it->second->res = res;
        it->second->received = true;
      }
    }
  }

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
    ResConstPtr res;
    volatile bool received;
  };
  typedef boost::shared_ptr<RequestInfo> RequestInfoPtr;

  typedef std::map<rviz_uuid::UUID, RequestInfoPtr> M_RequestInfo;
  M_RequestInfo requests_;
  boost::mutex requests_mutex_;
};

} // namespace rviz_rpc

#endif // RVIZ_RPC_CLIENT_H
