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

#ifndef RVIZ_RENDER_OGRE_SCENE_H
#define RVIZ_RENDER_OGRE_SCENE_H

#include <rviz_uuid/uuid.h>

#include <map>

#include <boost/shared_ptr.hpp>

#include "simple_shape.h"

namespace Ogre
{
class SceneManager;
}

namespace rviz_renderer_ogre
{

class Camera;
typedef boost::shared_ptr<Camera> CameraPtr;

class SimpleShape;
typedef boost::shared_ptr<SimpleShape> SimpleShapePtr;

class TransformNode;
typedef boost::shared_ptr<TransformNode> TransformNodePtr;

class Scene
{
public:
  Scene(const rviz_uuid::UUID& id, Ogre::SceneManager* scene_manager);
  ~Scene();

  Camera* createCamera(const rviz_uuid::UUID& id);
  void destroyCamera(const rviz_uuid::UUID& id);
  Camera* getCamera(const rviz_uuid::UUID& id);
  SimpleShape* createSimpleShape(const rviz_uuid::UUID& id, SimpleShape::Type type, const rviz_uuid::UUID& node_id);
  SimpleShape* getSimpleShape(const rviz_uuid::UUID& id);
  void destroySimpleShape(const rviz_uuid::UUID& id);

  TransformNode* createTransformNode(const rviz_uuid::UUID& id, const rviz_uuid::UUID& parent);
  void destroyTransformNode(const rviz_uuid::UUID& id);
  TransformNode* getTransformNode(const rviz_uuid::UUID& id);

  Ogre::SceneManager* getSceneManager() { return scene_manager_; }
  const rviz_uuid::UUID& getID() { return id_; }

private:
  rviz_uuid::UUID id_;
  Ogre::SceneManager* scene_manager_;

  typedef std::map<rviz_uuid::UUID, CameraPtr> M_Camera;
  M_Camera cameras_;

  typedef std::map<rviz_uuid::UUID, SimpleShapePtr> M_SimpleShape;
  M_SimpleShape simple_shapes_;

  typedef std::map<rviz_uuid::UUID, TransformNodePtr> M_TransformNode;
  M_TransformNode transform_nodes_;;
};

} // namespace rviz_renderer_ogre

#endif // RVIZ_RENDER_OGRE_SCENE_H
