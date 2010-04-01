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

#include "init.h"
#include "scene.h"
#include <rviz/uuid.h>

#include <ros/ros.h>

#include <rviz_msgs/CreateScene.h>
#include <rviz_msgs/DestroyScene.h>

namespace rviz
{
namespace ros_client
{

static NodeHandlePtr g_node_handle;

void initClient(const std::string& server_namespace)
{
  g_node_handle.reset(new ros::NodeHandle(server_namespace));
}

ros::NodeHandle& getNodeHandle()
{
  return *g_node_handle;
}

Scene createScene()
{
  ros::ServiceClient client = getNodeHandle().serviceClient<rviz_msgs::CreateSceneRequest, rviz_msgs::CreateSceneResponse>("renderer/scene/create");

  rviz_msgs::CreateScene srv;

  UUID id = UUID::Generate();
  srv.request.id = id;

  if (!client || !client.call(srv))
  {
    throw std::runtime_error("Could not call service [" + client.getService() + "]");
  }

  if (!srv.response.success)
  {
    throw std::runtime_error("Failed to create camera [" + id.toString() + "]: " + srv.response.error_msg);
  }

  return Scene(id);
}

} // namespace ros
} // namespace rviz

