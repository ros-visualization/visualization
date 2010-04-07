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
#include <rviz/uuid.h>
#include <rviz/math/vector3.h>
#include <rviz/math/quaternion.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rviz_msgs/CreateRenderWindow.h>
#include <rviz_msgs/DestroyRenderWindow.h>
#include <rviz_msgs/RenderWindowCommand.h>

#include <rviz_msgs/CreateCamera.h>
#include <rviz_msgs/DestroyCamera.h>
#include <rviz_msgs/CameraCommand.h>

#include <rviz_msgs/CreateScene.h>
#include <rviz_msgs/DestroyScene.h>

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

RendererROS::RendererROS(render::IRenderer* renderer, const ros::NodeHandle& nh)
: renderer_(renderer)
, callback_queue_(new ros::CallbackQueue)
, nh_(new ros::NodeHandle(nh, "renderer"))
, render_loop_listener_(new RenderLoopListener(callback_queue_.get()))
{
  nh_->setCallbackQueue(callback_queue_.get());
  srvs_.push_back(ServiceServerPtr(new ros::ServiceServer(nh_->advertiseService("render_window/create", &RendererROS::onCreateRenderWindow, this))));
  srvs_.push_back(ServiceServerPtr(new ros::ServiceServer(nh_->advertiseService("render_window/destroy", &RendererROS::onDestroyRenderWindow, this))));

  srvs_.push_back(ServiceServerPtr(new ros::ServiceServer(nh_->advertiseService("camera/create", &RendererROS::onCreateCamera, this))));
  srvs_.push_back(ServiceServerPtr(new ros::ServiceServer(nh_->advertiseService("camera/destroy", &RendererROS::onDestroyCamera, this))));
  srvs_.push_back(ServiceServerPtr(new ros::ServiceServer(nh_->advertiseService("scene/create", &RendererROS::onCreateScene, this))));
  srvs_.push_back(ServiceServerPtr(new ros::ServiceServer(nh_->advertiseService("scene/destroy", &RendererROS::onDestroyScene, this))));

  subs_.push_back(SubscriberPtr(new ros::Subscriber(nh_->subscribe("render_window/command", 0, &RendererROS::onRenderWindowCommand, this))));
  subs_.push_back(SubscriberPtr(new ros::Subscriber(nh_->subscribe("camera/command", 0, &RendererROS::onCameraCommand, this))));

  renderer_->addRenderLoopListener(render_loop_listener_.get());
}

RendererROS::~RendererROS()
{
  renderer_->removeRenderLoopListener(render_loop_listener_.get());
}

bool RendererROS::onCreateRenderWindow(rviz_msgs::CreateRenderWindowRequest& req, rviz_msgs::CreateRenderWindowResponse& res)
{
  try
  {
    renderer_->createRenderWindow(req.name, req.parent_window, req.width, req.height);
    res.success = true;
  }
  catch (std::exception& e)
  {
    res.success = false;
    res.error_msg = e.what();
  }

  return true;
}

bool RendererROS::onDestroyRenderWindow(rviz_msgs::DestroyRenderWindowRequest& req, rviz_msgs::DestroyRenderWindowResponse& res)
{
  try
  {
    renderer_->destroyRenderWindow(req.name);
    res.success = true;
  }
  catch (std::exception& e)
  {
    res.success = false;
    res.error_msg = e.what();
  }

  return true;
}

void RendererROS::onRenderWindowCommand(const rviz_msgs::RenderWindowCommandConstPtr& msg)
{
  try
  {
    render::IRenderWindow* wnd = renderer_->getRenderWindow(msg->name);

    switch (msg->type)
    {
    case rviz_msgs::RenderWindowCommand::RESIZED:
      wnd->resized(msg->resized.width, msg->resized.height);
      break;
    case rviz_msgs::RenderWindowCommand::ATTACH_CAMERA:
      wnd->attachCamera(msg->attach_camera.id);
      break;
    }
  }
  catch (std::exception& e)
  {

  }
}

bool RendererROS::onCreateCamera(rviz_msgs::CreateCameraRequest& req, rviz_msgs::CreateCameraResponse& res)
{
  try
  {
    render::IScene* scene = renderer_->getScene(req.scene_id);
    if (!scene)
    {
      throw std::runtime_error("Could not find scene [" + UUID(req.scene_id).toString() + "]");
    }

    scene->createCamera(req.camera_id);
    res.success = true;
  }
  catch (std::exception& e)
  {
    res.success = false;
    res.error_msg = e.what();
  }

  return true;
}

bool RendererROS::onDestroyCamera(rviz_msgs::DestroyCameraRequest& req, rviz_msgs::DestroyCameraResponse& res)
{
  try
  {
    render::IScene* scene = renderer_->getScene(req.scene_id);
    if (!scene)
    {
      throw std::runtime_error("Could not find scene [" + UUID(req.scene_id).toString() + "]");
    }

    scene->destroyCamera(req.camera_id);

    res.success = true;
  }
  catch (std::exception& e)
  {
    res.success = false;
    res.error_msg = e.what();
  }

  return true;
}

void RendererROS::onCameraCommand(const rviz_msgs::CameraCommandConstPtr& cmd)
{
  try
  {
    render::ICamera* cam = renderer_->getCamera(cmd->id);
    if (!cam)
    {
      return;
    }

    uint16_t flags = cmd->flags;
    if (flags & rviz_msgs::CameraCommand::POSITION)
    {
      cam->setPosition(cmd->position);
    }

    if (flags & rviz_msgs::CameraCommand::ORIENTATION)
    {
      cam->setOrientation(cmd->orientation);
    }

    if (flags & rviz_msgs::CameraCommand::LOOK_AT)
    {
      cam->lookAt(cmd->look_at);
    }

    if (flags & rviz_msgs::CameraCommand::FOVY)
    {
      cam->setFOVY(cmd->fovy);
    }

    if (flags & rviz_msgs::CameraCommand::ASPECT_RATIO)
    {
      cam->setAspectRatio(cmd->aspect_ratio);
    }

    if (flags & rviz_msgs::CameraCommand::AUTO_ASPECT_RATIO)
    {
      cam->setAutoAspectRatio(cmd->auto_aspect_ratio);
    }

    if (flags & rviz_msgs::CameraCommand::NEAR_CLIP)
    {
      cam->setNearClipDistance(cmd->near_clip);
    }

    if (flags & rviz_msgs::CameraCommand::FAR_CLIP)
    {
      cam->setFarClipDistance(cmd->far_clip);
    }

    if (flags & rviz_msgs::CameraCommand::MOVE)
    {
      if (cmd->move.relative)
      {
        cam->moveRelative(cmd->move.vector);
      }
      else
      {
        cam->move(cmd->move.vector);
      }
    }

    if (flags & rviz_msgs::CameraCommand::ROTATE)
    {
      cam->rotate(cmd->rotate);
    }
  }
  catch (std::exception&)
  {
  }
}

bool RendererROS::onCreateScene(rviz_msgs::CreateSceneRequest& req, rviz_msgs::CreateSceneResponse& res)
{
  try
  {
    render::IScene* scene = renderer_->createScene(req.id);
    if (!scene)
    {
      throw std::runtime_error("Could not create scene [" + UUID(req.id).toString() + "]");
    }

    res.success = true;
  }
  catch (std::exception& e)
  {
    res.success = false;
    res.error_msg = e.what();
  }

  return true;
}

bool RendererROS::onDestroyScene(rviz_msgs::DestroySceneRequest& req, rviz_msgs::DestroySceneResponse& res)
{
  try
  {
    renderer_->destroyScene(req.id);
    res.success = true;
  }
  catch (std::exception& e)
  {
    res.success = false;
    res.error_msg = e.what();
  }

  return true;
}


} // namespace rviz
