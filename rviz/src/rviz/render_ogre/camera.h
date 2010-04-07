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

#ifndef RVIZ_RENDER_OGRE_CAMERA_H
#define RVIZ_RENDER_OGRE_CAMERA_H

#include <rviz/render_interface/icamera.h>

namespace Ogre
{
class Camera;
}

namespace rviz
{
namespace render
{
namespace ogre
{

class Camera : public ICamera
{
public:
  Camera(Ogre::Camera* cam);

  Ogre::Camera* getOgreCamera() { return cam_; }

  virtual void setPosition(const Vector3&);
  virtual void setOrientation(const Quaternion&);
  virtual void lookAt(const Vector3&);
  virtual void move(const Vector3&);
  virtual void moveRelative(const Vector3&) ;
  virtual void rotate(const Quaternion&);
  virtual void setFOVY(float fovy);
  virtual void setAspectRatio(float aspect);
  virtual void setAutoAspectRatio(bool autoratio);
  virtual void setNearClipDistance(float dist);
  virtual void setFarClipDistance(float dist);
  virtual void setTransform(const Vector3& pos, const Quaternion& orient);

private:
  Ogre::Camera* cam_;
};

} // namespace ogre
} // namespace render
} // namespace rviz

#endif // RVIZ_RENDER_ICAMERA_H

