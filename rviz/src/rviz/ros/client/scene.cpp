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

#include "scene.h"
#include "camera.h"
#include "init.h"

#include <ros/ros.h>
#include <rviz_msgs/CreateCamera.h>

namespace rviz
{
namespace ros_client
{

Scene::Scene()
{
}

Scene::Scene(const UUID& id)
: Object(id)
{
}

Camera Scene::createCamera()
{
  ros::ServiceClient client = getNodeHandle().serviceClient<rviz_msgs::CreateCameraRequest, rviz_msgs::CreateCameraResponse>("renderer/camera/create");

  rviz_msgs::CreateCamera srv;

  UUID id = UUID::Generate();
  srv.request.scene_id = getID();
  srv.request.camera_id = id;

  if (!client || !client.call(srv))
  {
    throw std::runtime_error("Could not call service [" + client.getService() + "]");
  }

  if (!srv.response.success)
  {
    throw std::runtime_error("Failed to create camera [" + id.toString() + "]: " + srv.response.error_msg);
  }

  return Camera(id, getID());
}

} // namespace ros_client
} // namespace rviz
