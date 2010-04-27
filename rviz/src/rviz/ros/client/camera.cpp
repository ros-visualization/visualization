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

#include "camera.h"
#include <rviz/math/vector3.h>
#include <rviz/math/quaternion.h>

#include <rviz_interfaces/Camera.h>

#include "init.h"

using namespace rviz_uuid;

namespace rviz
{
namespace ros_client
{

Camera::Camera()
{
}

Camera::Camera(const UUID& id, const UUID& scene_id)
: Object(id)
, scene_id_(scene_id)
{
  proxy_ = getProxyInterface<rviz_interfaces::CameraProxy>("camera");
}

void Camera::setPosition(const Vector3& pos)
{
  proxy_->setPosition(getID(), pos);
}

void Camera::setOrientation(const Quaternion& orient)
{
  proxy_->setOrientation(getID(), orient);
}

void Camera::lookAt(const Vector3& point)
{
  proxy_->lookAt(getID(), point);
}

void Camera::move(const Vector3& v)
{
  proxy_->move(getID(), v);
}

void Camera::moveRelative(const Vector3& v)
{
  proxy_->moveRelative(getID(), v);
}

void Camera::rotate(const Quaternion& q)
{
  proxy_->rotate(getID(), q);
}

void Camera::setFOVY(float fovy)
{
  proxy_->setFOVY(getID(), fovy);
}

void Camera::setAspectRatio(float aspect)
{
  proxy_->setAspectRatio(getID(), aspect);
}

void Camera::setAutoAspectRatio(bool autoaspect)
{
  proxy_->setAutoAspectRatio(getID(), autoaspect);
}

void Camera::setNearClipDistance(float near_clip)
{
  proxy_->setNearClipDistance(getID(), near_clip);
}

void Camera::setFarClipDistance(float far_clip)
{
  proxy_->setFarClipDistance(getID(), far_clip);
}

void Camera::setTransform(const Vector3& pos, const Quaternion& orient)
{
  proxy_->setPosition(getID(), pos);
  proxy_->setOrientation(getID(), orient);
}


} // namespace ros_client
} // namespace rviz
