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

#ifndef RVIZ_ROS_CLIENT_CAMERA_H
#define RVIZ_ROS_CLIENT_CAMERA_H

#include "object.h"

namespace rviz
{
class Vector3;
class Quaternion;

namespace render_client_proxy_interface
{
class ICameraProxy;
}

namespace ros_client
{

class Camera : public Object
{
public:
  Camera();
  Camera(const UUID& id, const UUID& scene_id);

  const UUID& getSceneID() { return scene_id_; }

  virtual void setPosition(const Vector3& pos);
  virtual void setOrientation(const Quaternion& orient);
  virtual void lookAt(const Vector3& point);
  virtual void move(const Vector3& v);
  virtual void moveRelative(const Vector3& v);
  virtual void rotate(const Quaternion& q);
  virtual void setFOVY(float fovy);
  virtual void setAspectRatio(float aspect);
  virtual void setAutoAspectRatio(bool autoaspect);
  virtual void setNearClipDistance(float near_clip);
  virtual void setFarClipDistance(float far_clip);

  virtual void setTransform(const Vector3& pos, const Quaternion& orient);

private:
  UUID scene_id_;
  render_client_proxy_interface::ICameraProxy* proxy_;
};

} // namespace ros_client
} // namespace rviz

#endif // RVIZ_ROS_CLIENT_CAMERA_H
