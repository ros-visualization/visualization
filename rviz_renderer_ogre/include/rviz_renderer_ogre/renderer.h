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

#include <rviz_uuid/uuid.h>

namespace ros
{
class CallbackQueue;
}

namespace rviz_renderer_ogre
{

class RenderWindow;
class Scene;
class Camera;
class Material;
typedef boost::shared_ptr<Material> MaterialPtr;
class Mesh;
typedef boost::shared_ptr<Mesh> MeshPtr;
class DisableRenderingSchemeListener;
typedef boost::shared_ptr<DisableRenderingSchemeListener> DisableRenderingSchemeListenerPtr;

class Renderer;
Renderer* getRenderer();

class Renderer
{
public:
  Renderer(bool enable_ogre_log);
  ~Renderer();

  void start();
  void stop();

   RenderWindow* createRenderWindow(const rviz_uuid::UUID& id, const std::string& parent_window, uint32_t width, uint32_t height);
   void destroyRenderWindow(const rviz_uuid::UUID& id);

   Scene* createScene(const rviz_uuid::UUID& id);
   void destroyScene(const rviz_uuid::UUID& id);
   Scene* getScene(const rviz_uuid::UUID& id);

   RenderWindow* getRenderWindow(const rviz_uuid::UUID& id);

   Camera* getCamera(const rviz_uuid::UUID& id);

   ros::CallbackQueue* getServerThreadCallbackQueue() { return callback_queue_.get(); }

   void addMaterial(const rviz_uuid::UUID& id, const MaterialPtr& mat);
   void removeMaterial(const rviz_uuid::UUID& id);
   MaterialPtr getMaterial(const rviz_uuid::UUID& id);

   void addMesh(const std::string& resource_name, const MeshPtr& mesh);
   void removeMesh(const std::string& resource_name);
   MeshPtr getMesh(const std::string& resource_name);
   bool meshExists(const std::string& resource_name);

private:
  void init();
  void renderThread();

  void oneTimeInit();

  boost::thread render_thread_;
  bool running_;
  bool first_window_created_;
  bool enable_ogre_log_;

  typedef boost::shared_ptr<RenderWindow> RenderWindowPtr;
  typedef std::map<rviz_uuid::UUID, RenderWindowPtr> M_RenderWindow;
  M_RenderWindow render_windows_;

  typedef boost::shared_ptr<Scene> ScenePtr;
  typedef std::map<rviz_uuid::UUID, ScenePtr> M_Scene;
  M_Scene scenes_;

  typedef boost::shared_ptr<ros::CallbackQueue> CallbackQueuePtr;
  CallbackQueuePtr callback_queue_;

  typedef std::map<rviz_uuid::UUID, MaterialPtr> M_Material;
  M_Material materials_;

  typedef std::map<std::string, MeshPtr> M_Mesh;
  M_Mesh meshes_;

  DisableRenderingSchemeListenerPtr scheme_listener_;
};

} // namespace rviz_renderer_ogre

#endif // RVIZ_OGRE_RENDERER_H
