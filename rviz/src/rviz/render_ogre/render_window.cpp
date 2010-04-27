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

#include "render_window.h"
#include "camera.h"
#include "renderer.h"

#include <OGRE/OgreRenderWindow.h>

#include <ros/assert.h>

using namespace rviz_uuid;

namespace rviz
{
namespace render
{
namespace ogre
{

RenderWindow::RenderWindow(const rviz_uuid::UUID& id, Ogre::RenderWindow* wnd, Renderer* rend)
: id_(id)
, render_window_(wnd)
, renderer_(rend)
, cam_(0)
{
}

const rviz_uuid::UUID& RenderWindow::getID()
{
  return id_;
}

void RenderWindow::resized(uint32_t width, uint32_t height)
{
  // Resize tries to actually resize the window on OSX, which can cause unfortunate results
#if !defined(__APPLE__)
  render_window_->resize(width, height);
#endif

  render_window_->windowMovedOrResized();
}

void RenderWindow::attachCamera(const UUID& id)
{
  if (cam_)
  {
    render_window_->removeAllViewports();
  }

  Camera* cam = static_cast<Camera*>(renderer_->getCamera(id));
  ROS_ASSERT(cam);
  render_window_->addViewport(cam->getOgreCamera());
}

} // namespace ogre
} // namespace render
} // namespace rviz
