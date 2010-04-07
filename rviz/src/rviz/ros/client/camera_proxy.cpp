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

#include "camera_proxy.h"
#include "init.h"

#include <rviz/math/vector3.h>
#include <rviz/math/quaternion.h>
#include <rviz/uuid.h>

#include <rviz_msgs/CameraCommand.h>

#include <ros/ros.h>

namespace rviz
{
namespace ros_client
{

CameraProxy::CameraProxy()
{
  pub_.reset(new ros::Publisher(getNodeHandle().advertise<rviz_msgs::CameraCommand>("renderer/camera/command", 0)));

  waitForPub(pub_);
}

#define CAMERA_COMMAND(var, flag, val) \
  rviz_msgs::CameraCommandPtr cmd(new rviz_msgs::CameraCommand); \
  cmd->id = id; \
  cmd->flags = rviz_msgs::CameraCommand::flag; \
  cmd->var = val; \
  pub_->publish(cmd);

void CameraProxy::setPosition(const UUID& id, const Vector3& pos)
{
  CAMERA_COMMAND(position, POSITION, pos);
}

void CameraProxy::setOrientation(const UUID& id, const Quaternion& orient)
{
  CAMERA_COMMAND(orientation, ORIENTATION, orient);
}

void CameraProxy::lookAt(const UUID& id, const Vector3& point)
{
  CAMERA_COMMAND(look_at, LOOK_AT, point);
}

void CameraProxy::move(const UUID& id, const Vector3& vec)
{
  rviz_msgs::CameraCommandPtr cmd(new rviz_msgs::CameraCommand);
  cmd->id = id;
  cmd->flags = rviz_msgs::CameraCommand::MOVE;
  cmd->move.relative = false;
  cmd->move.vector = vec;
  pub_->publish(cmd);
}

void CameraProxy::moveRelative(const UUID& id, const Vector3& vec)
{
  rviz_msgs::CameraCommandPtr cmd(new rviz_msgs::CameraCommand);
  cmd->id = id;
  cmd->flags = rviz_msgs::CameraCommand::MOVE;
  cmd->move.relative = true;
  cmd->move.vector = vec;
  pub_->publish(cmd);
}

void CameraProxy::rotate(const UUID& id, const Quaternion& q)
{
  CAMERA_COMMAND(rotate, ROTATE, q);
}

void CameraProxy::setFOVY(const UUID& id, float fovy)
{
  CAMERA_COMMAND(fovy, FOVY, fovy);
}

void CameraProxy::setAspectRatio(const UUID& id, float aspect)
{
  CAMERA_COMMAND(aspect_ratio, ASPECT_RATIO, aspect);
}

void CameraProxy::setAutoAspectRatio(const UUID& id, bool autoaspect)
{
  CAMERA_COMMAND(auto_aspect_ratio, AUTO_ASPECT_RATIO, autoaspect);
}

void CameraProxy::setNearClipDistance(const UUID& id, float near_clip)
{
  CAMERA_COMMAND(near_clip, NEAR_CLIP, near_clip);
}

void CameraProxy::setFarClipDistance(const UUID& id, float far_clip)
{
  CAMERA_COMMAND(far_clip, FAR_CLIP, far_clip);
}

void CameraProxy::setTransform(const UUID& id, const Vector3& pos, const Quaternion& orient)
{
  rviz_msgs::CameraCommandPtr cmd(new rviz_msgs::CameraCommand);
  cmd->id = id;
  cmd->flags = rviz_msgs::CameraCommand::POSITION|rviz_msgs::CameraCommand::ORIENTATION;
  cmd->position = pos;
  cmd->orientation = orient;
  pub_->publish(cmd);
}

} // namespace ros_client
} // namespace rviz


