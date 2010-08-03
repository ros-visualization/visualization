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

#include "rviz_renderer_ogre/render_window.h"
#include "rviz_renderer_ogre/camera.h"
#include "rviz_renderer_ogre/renderer.h"

#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderTarget.h>
#include <OGRE/OgreRenderTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/assert.h>

using namespace rviz_uuid;

namespace rviz_renderer_ogre
{

RenderWindow::RenderWindow(const rviz_uuid::UUID& id, Ogre::RenderWindow* wnd, Renderer* rend)
: id_(id)
, render_window_(wnd)
, renderer_(rend)
, cam_(0)
, width_(1)
, height_(1)
, screen_rect_(new Ogre::Rectangle2D(true))
{
  screen_rect_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  screen_rect_->setCorners(0, 0, 1, 1, false);
  createResources();

  wnd->addListener(this);
}

RenderWindow::~RenderWindow()
{
  destroyResources();
}

void RenderWindow::destroyMRT(MRT& mrt)
{
  uint32_t count = mrt.textures.size() - 1;

  Ogre::Root::getSingleton().getRenderSystem()->destroyRenderTarget(mrt.mrt->getName());

  V_OgreTexture::reverse_iterator it = mrt.textures.rbegin();
  V_OgreTexture::reverse_iterator end = mrt.textures.rend();
  for (; it != end; ++it, --count)
  {
    Ogre::TexturePtr& tex = *it;
    Ogre::TextureManager::getSingleton().remove(tex->getName());
  }

  mrt.textures.clear();
  mrt.rtts.clear();
}

void RenderWindow::destroyResources()
{
  destroyMRT(gbuffer_target_);
}

void RenderWindow::createResources()
{
  std::stringstream ss;
  ss << getID() << "_gbuffer_target";
  gbuffer_target_.mrt = Ogre::Root::getSingleton().getRenderSystem()->createMultiRenderTarget(ss.str());
  gbuffer_target_.mrt->setAutoUpdated(false);
  gbuffer_target_.mrt->setActive(true);
  setupRT(gbuffer_target_.mrt);

  if (gbuffer_target_.mrt->getNumViewports())
  {
    gbuffer_target_.mrt->getViewport(0)->setMaterialScheme("GBuffer");
  }

  for (uint32_t i = 0; i < 2; ++i)
  {
    std::stringstream ss2;
    ss2 << ss.str() << "rt" << i;
    gbuffer_target_.textures.push_back(Ogre::TextureManager::getSingleton().createManual(ss2.str(), ROS_PACKAGE_NAME, Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_FLOAT16_RGBA, Ogre::TU_RENDERTARGET));
    gbuffer_target_.rtts.push_back(gbuffer_target_.textures[i]->getBuffer()->getRenderTarget());
    gbuffer_target_.mrt->bindSurface(i, gbuffer_target_.rtts[i]);
  }
}

void RenderWindow::setupRT(Ogre::RenderTarget* rt)
{
  rt->removeAllViewports();

  if (cam_)
  {
    Ogre::Viewport* vp = rt->addViewport(cam_->getOgreCamera());
    vp->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0, 0.0));
    vp->setClearEveryFrame(false);
    vp->setOverlaysEnabled(false);
  }
}

const rviz_uuid::UUID& RenderWindow::getID()
{
  return id_;
}

void RenderWindow::resized(uint32_t width, uint32_t height)
{
  destroyResources();

  width_ = width;
  height_ = height;

  createResources();

  // Resize tries to actually resize the window on OSX, which can cause unfortunate results
#if !defined(__APPLE__)
  render_window_->resize(width, height);
#endif

  render_window_->windowMovedOrResized();
}

void RenderWindow::attachCamera(const UUID& id)
{
  Camera* cam = renderer_->getCamera(id);
  ROS_ASSERT(cam);
  cam_ = cam;

  setupRT(gbuffer_target_.mrt);
  setupRT(render_window_);
}

void RenderWindow::beginRender()
{
  render_window_->update(false);
  //ROS_INFO("%f", render_window_->getLastFPS());
}

void RenderWindow::finishRender()
{
  render_window_->swapBuffers(false);
}

void RenderWindow::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{

}

void RenderWindow::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{

}

} // namespace rviz_renderer_ogre
