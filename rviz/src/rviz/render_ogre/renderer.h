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

#ifndef RVIZ_OGRE_RENDERER_H
#define RVIZ_OGRE_RENDERER_H

#include <string>
#include <vector>
#include <map>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rviz/render_interface/irenderer.h>
#include <rviz_uuid/uuid.h>

namespace rviz
{
namespace render
{
class IRenderLoopListener;

namespace ogre
{
class RenderWindow;
class Scene;
class Camera;

class Renderer : public IRenderer
{
public:
  Renderer(const std::string& root_path, bool enable_ogre_log);
  ~Renderer();

  void start();
  void stop();

  virtual IRenderWindow* createRenderWindow(const std::string& name, const std::string& parent_window, uint32_t width, uint32_t height);
  virtual void destroyRenderWindow(const std::string& name);

  virtual void addRenderLoopListener(IRenderLoopListener* listener);
  virtual void removeRenderLoopListener(IRenderLoopListener* listener);

  virtual IScene* createScene(const rviz_uuid::UUID& id);
  virtual void destroyScene(const rviz_uuid::UUID& id);
  virtual IScene* getScene(const rviz_uuid::UUID& id);

  virtual IRenderWindow* getRenderWindow(const std::string& name);

  virtual ICamera* getCamera(const rviz_uuid::UUID& id);

private:
  void init();
  void renderThread();

  void oneTimeInit();

  boost::thread render_thread_;
  bool running_;
  bool first_window_created_;
  std::string root_path_;
  bool enable_ogre_log_;

  typedef std::vector<IRenderLoopListener*> V_RenderLoopListener;
  V_RenderLoopListener render_loop_listeners_;

  typedef boost::shared_ptr<RenderWindow> RenderWindowPtr;
  typedef std::map<std::string, RenderWindowPtr> M_RenderWindow;
  M_RenderWindow render_windows_;

  typedef boost::shared_ptr<Scene> ScenePtr;
  typedef std::map<rviz_uuid::UUID, ScenePtr> M_Scene;
  M_Scene scenes_;
};

} // namespace ogre
} // namespace render
} // namespace rviz

#endif // RVIZ_OGRE_RENDERER_H
