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

#ifndef RVIZ_OGRE_RENDER_WINDOW_H
#define RVIZ_OGRE_RENDER_WINDOW_H

#include <rviz_uuid/uuid.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreRenderTargetListener.h>

namespace Ogre
{
class RenderWindow;
class RenderTexture;
class RenderTarget;
class MultiRenderTarget;
class Rectangle2D;
class SceneNode;
}

namespace rviz_renderer_ogre
{

class Camera;
class Renderer;

class RenderWindow : public Ogre::RenderTargetListener
{
public:
  RenderWindow(const rviz_uuid::UUID& id, Ogre::RenderWindow* wnd, Renderer* rend);
  ~RenderWindow();

   const rviz_uuid::UUID& getID();
   void resized(uint32_t width, uint32_t height);
   void attachCamera(const rviz_uuid::UUID& id);

  Ogre::RenderWindow* getOgreRenderWindow() { return render_window_; }

  void beginRender();
  void finishRender();

private:
  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
  virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

  typedef std::vector<Ogre::RenderTexture*> V_OgreRenderTexture;
  typedef std::vector<Ogre::TexturePtr> V_OgreTexture;
  struct MRT
  {
    MRT()
    : mrt(0)
    //, rect(0)
    {}

    Ogre::MultiRenderTarget* mrt;
    V_OgreRenderTexture rtts;
    V_OgreTexture textures;
  };

  struct RTT
  {
    RTT()
    : rt(0)
    //, rect(0)
    {}

    Ogre::RenderTexture* rt;
    Ogre::TexturePtr texture;
  };

  void destroyResources();
  void createResources();

  void destroyMRT(MRT& mrt);
  void destroyRTT(RTT& rtt);

  void setupRT(Ogre::RenderTarget* rt);

  rviz_uuid::UUID id_;
  Ogre::RenderWindow* render_window_;
  Renderer* renderer_;
  Camera* cam_;
  uint32_t width_;
  uint32_t height_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::SceneNode* rect_node_;

  MRT gbuffer_target_;
};

} // namespace rviz_renderer_ogre

#endif // RVIZ_OGRE_RENDER_WINDOW_H
