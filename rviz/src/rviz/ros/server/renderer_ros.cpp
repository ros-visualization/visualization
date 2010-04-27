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

#include "renderer_ros.h"
#include <rviz/render_interface/irenderer.h>
#include <rviz/render_interface/irender_window.h>
#include <rviz/render_interface/iscene.h>
#include <rviz/render_interface/icamera.h>
#include <rviz/render_interface/irender_loop_listener.h>
#include <rviz_uuid/uuid.h>
#include <rviz/math/vector3.h>
#include <rviz/math/quaternion.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rviz_interfaces/Camera.h>
#include <rviz_interfaces/RenderWindow.h>
#include <rviz_interfaces/Scene.h>

using namespace rviz_uuid;

namespace rviz
{

class RendererROS::RenderLoopListener : public render::IRenderLoopListener
{
public:
  RenderLoopListener(ros::CallbackQueue* queue)
  : queue_(queue)
  {

  }

  virtual void postRender(render::IRenderer* renderer)
  {
    queue_->callAvailable();
  }

private:
  ros::CallbackQueue* queue_;
};

class CameraServer : public rviz_interfaces::CameraServer
{
public:
  CameraServer(render::IRenderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::CameraServer(name, nh)
  , renderer_(rend)
  {
  }

  render::ICamera* lookupCamera(const rviz_uuid::UUID& id)
  {
    render::ICamera* cam = renderer_->getCamera(id);
    if (!cam)
    {
      std::stringstream ss;
      ss << "Invalid camera [" << id << "]";
      throw std::runtime_error(ss.str());
    }

    return cam;
  }

  virtual void setPosition(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& pos)
  {
    lookupCamera(id)->setPosition(pos);
  }

  virtual void setOrientation(const rviz_msgs::UUID& id, const geometry_msgs::Quaternion& orient)
  {
    lookupCamera(id)->setOrientation(orient);
  }

  virtual void lookAt(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& pos)
  {
    lookupCamera(id)->lookAt(pos);
  }

  virtual void move(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& vec)
  {
    lookupCamera(id)->move(vec);
  }

  virtual void moveRelative(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& vec)
  {
    lookupCamera(id)->moveRelative(vec);
  }

  virtual void rotate(const rviz_msgs::UUID& id, const geometry_msgs::Quaternion& quat)
  {
    lookupCamera(id)->rotate(quat);
  }

  virtual void setFOVY(const rviz_msgs::UUID& id, float fovy)
  {
    lookupCamera(id)->setFOVY(fovy);
  }

  virtual void setAspectRatio(const rviz_msgs::UUID& id, float aspect)
  {
    lookupCamera(id)->setAspectRatio(aspect);
  }

  virtual void setAutoAspectRatio(const rviz_msgs::UUID& id, uint8_t autoaspect)
  {
    lookupCamera(id)->setAutoAspectRatio(autoaspect);
  }

  virtual void setNearClipDistance(const rviz_msgs::UUID& id, float dist)
  {
    lookupCamera(id)->setNearClipDistance(dist);
  }

  virtual void setFarClipDistance(const rviz_msgs::UUID& id, float dist)
  {
    lookupCamera(id)->setFarClipDistance(dist);
  }

private:
  render::IRenderer* renderer_;
};

class RenderWindowServer : public rviz_interfaces::RenderWindowServer
{
public:
  RenderWindowServer(render::IRenderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::RenderWindowServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void resized(const rviz_msgs::UUID& id, uint32_t width, uint32_t height)
  {
    render::IRenderWindow* wnd = renderer_->getRenderWindow(id);
    wnd->resized(width, height);
  }

  virtual void attachCamera(const rviz_msgs::UUID& id, const rviz_msgs::UUID& camera_id)
  {
    render::IRenderWindow* wnd = renderer_->getRenderWindow(id);
    wnd->attachCamera(camera_id);
  }

  virtual void create(const rviz_msgs::UUID& id, const std::string& parent_window, uint32_t width, uint32_t height)
  {
    renderer_->createRenderWindow(id, parent_window, width, height);
  }

  virtual void destroy(const rviz_msgs::UUID& id)
  {
    renderer_->destroyRenderWindow(id);
  }

private:
  render::IRenderer* renderer_;
};

class SceneServer : public rviz_interfaces::SceneServer
{
public:
  SceneServer(render::IRenderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::SceneServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void create(const rviz_msgs::UUID& id)
  {
    render::IScene* scene = renderer_->createScene(id);
    if (!scene)
    {
      throw std::runtime_error("Could not create scene [" + UUID(id).toString() + "]");
    }
  }

  virtual void destroy(const rviz_msgs::UUID& id)
  {
    renderer_->destroyScene(id);
  }

  virtual void createCamera(const rviz_msgs::UUID& id, const rviz_msgs::UUID& camera_id)
  {
    render::IScene* scene = renderer_->getScene(id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(id).toString() + "] does not exist");
    }

    scene->createCamera(camera_id);
  }

  virtual void destroyCamera(const rviz_msgs::UUID& id, const rviz_msgs::UUID& camera_id)
  {
    render::IScene* scene = renderer_->getScene(id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(id).toString() + "] does not exist");
    }

    scene->destroyCamera(camera_id);
  }

private:
  render::IRenderer* renderer_;
};

RendererROS::RendererROS(render::IRenderer* renderer, const ros::NodeHandle& nh)
: renderer_(renderer)
, callback_queue_(new ros::CallbackQueue)
, nh_(new ros::NodeHandle(nh, "renderer"))
, render_loop_listener_(new RenderLoopListener(callback_queue_.get()))
{
  nh_->setCallbackQueue(callback_queue_.get());
  camera_server_.reset(new CameraServer(renderer_, "camera", *nh_));
  render_window_server_.reset(new RenderWindowServer(renderer_, "render_window", *nh_));
  scene_server_.reset(new SceneServer(renderer_, "scene", *nh_));

  renderer_->addRenderLoopListener(render_loop_listener_.get());
}

RendererROS::~RendererROS()
{
  renderer_->removeRenderLoopListener(render_loop_listener_.get());
}


} // namespace rviz
