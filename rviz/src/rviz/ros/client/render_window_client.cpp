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

#include "render_window_client.h"
#include "init.h"

#include <ros/ros.h>

#include <rviz_msgs/RenderWindowCommand.h>
#include <rviz_msgs/CreateRenderWindow.h>
#include <rviz_msgs/DestroyRenderWindow.h>

namespace rviz
{
namespace ros_client
{

RenderWindowClient::RenderWindowClient(const std::string& name, const std::string& parent_window, uint32_t width, uint32_t height)
: name_(name)
, destroyed_(false)
{
  ros::ServiceClient client = getNodeHandle().serviceClient<rviz_msgs::CreateRenderWindowRequest, rviz_msgs::CreateRenderWindowResponse>("renderer/render_window/create");
  rviz_msgs::CreateRenderWindow srv;
  srv.request.name = name;
  srv.request.parent_window = parent_window;
  srv.request.width = width;
  srv.request.height = height;
  if (!client || !client.call(srv))
  {
    throw std::runtime_error("Could not call service [" + client.getService() + "]");
  }

  if (!srv.response.success)
  {
    throw std::runtime_error("Failed to create render window [" + name + "]: " + srv.response.error_msg);
  }

  pub_.reset(new ros::Publisher(getNodeHandle().advertise<rviz_msgs::RenderWindowCommand>("renderer/render_window/command", 1)));
}

RenderWindowClient::~RenderWindowClient()
{
  destroy();
}

void RenderWindowClient::destroy()
{
  if (!destroyed_)
  {
    destroyed_ = true;

    ros::ServiceClient client = getNodeHandle().serviceClient<rviz_msgs::DestroyRenderWindowRequest, rviz_msgs::DestroyRenderWindowResponse>("renderer/render_window/destroy");
    rviz_msgs::DestroyRenderWindow srv;
    srv.request.name = name_;
    if (!client || !client.call(srv))
    {
      throw std::runtime_error("Could not call service [" + client.getService() + "]");
    }

    if (!srv.response.success)
    {
      throw std::runtime_error("Failed to destroy render window [" + name_ + "]: " + srv.response.error_msg);
    }
  }
}

void RenderWindowClient::resized(uint32_t width, uint32_t height)
{
  rviz_msgs::RenderWindowCommandPtr cmd(new rviz_msgs::RenderWindowCommand);
  cmd->type = rviz_msgs::RenderWindowCommand::RESIZED;
  cmd->name = name_;

  cmd->resized.width = width;
  cmd->resized.height = height;
  pub_->publish(cmd);
}

} // namespace ros
} // namespace rviz
