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

#include "rviz_renderer_ogre/scene.h"
#include "rviz_renderer_ogre/camera.h"
#include "rviz_renderer_ogre/simple_shape.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreEntity.h>

#include <ros/assert.h>

using namespace rviz_uuid;
using namespace rviz_renderer_interface;

namespace rviz_renderer_ogre
{

Scene::Scene(const UUID& id, Ogre::SceneManager* scene_manager)
: id_(id)
, scene_manager_(scene_manager)
{
}

Scene::~Scene()
{
  cameras_.clear();
  simple_shapes_.clear();
  Ogre::Root::getSingleton().destroySceneManager(scene_manager_);
}

ICamera* Scene::createCamera(const UUID& id)
{
  Ogre::Camera* ogre_cam = scene_manager_->createCamera(id.toString());
  ogre_cam->setPosition(0, 10, 10);
  ogre_cam->lookAt(0, 0, 0);
  ogre_cam->setNearClipDistance(0.01);

  CameraPtr cam(new Camera(ogre_cam));
  cameras_[id] = cam;

  return cam.get();
}

void Scene::destroyCamera(const UUID& id)
{
  M_Camera::iterator it = cameras_.find(id);
  ROS_ASSERT(it != cameras_.end());

  const CameraPtr& cam = it->second;
  scene_manager_->destroyCamera(cam->getOgreCamera());

  cameras_.erase(it);
}

ICamera* Scene::getCamera(const UUID& id)
{
  M_Camera::iterator it = cameras_.find(id);
  if (it == cameras_.end())
  {
    return 0;
  }

  return it->second.get();
}

ISimpleShape* Scene::createSimpleShape(const rviz_uuid::UUID& id, ISimpleShape::Type type)
{
  SimpleShapePtr shape(new SimpleShape(type, scene_manager_));
  simple_shapes_[id] = shape;
  return shape.get();
}

void Scene::destroySimpleShape(const rviz_uuid::UUID& id)
{
  simple_shapes_.erase(id);
}

ISimpleShape* Scene::getSimpleShape(const rviz_uuid::UUID& id)
{
  M_SimpleShape::iterator it = simple_shapes_.find(id);
  ROS_ASSERT(it != simple_shapes_.end());

  return it->second.get();
}

} // namespace rviz_renderer_ogre
