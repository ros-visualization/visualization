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

#include <rviz_renderer_ogre/mesh_instance.h>
#include <rviz_renderer_ogre/transform_node.h>
#include <rviz_renderer_ogre/material.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/types.h>
#include <ros/console.h>
#include <ros/assert.h>

namespace rviz_renderer_ogre
{

MeshInstance::MeshInstance(Ogre::SceneManager* scene_manager, TransformNode* node, const std::string& mesh_resource)
: scene_manager_(scene_manager)
{
  std::stringstream ss;
  static size_t count = 0;
  ss << "MeshInstance" << count++;

  entity_ = scene_manager_->createEntity(ss.str(), mesh_resource);
  node->getOgreSceneNode()->attachObject(entity_);

  uint32_t count4 = count;
  entity_->getSubEntity(0)->setCustomParameter(0, Ogre::Vector4(((count4 >> 24) & 0xff) / 255.0, ((count4 >> 16) & 0xff) / 255.0, ((count4 >> 8) & 0xff) / 255.0, (count4 & 0xff) / 255.0));
}

MeshInstance::~MeshInstance()
{
  M_Material::iterator it = mat_to_subs_.begin();
  M_Material::iterator end = mat_to_subs_.end();
  for (; it != end; ++it)
  {
    it->first->detachRenderable(this);
  }

  scene_manager_->destroyEntity(entity_);
}

void MeshInstance::setMaterial(Material* mat)
{
  M_Material::iterator it = mat_to_subs_.begin();
  M_Material::iterator end = mat_to_subs_.end();
  for (; it != end; ++it)
  {
    it->first->detachRenderable(this);
  }

  mat_to_subs_.clear();

  if (mat)
  {
    for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i)
    {
      Ogre::SubEntity* sub = entity_->getSubEntity(i);
      mat_to_subs_[mat].insert(sub);
      sub_to_mat_[sub] = mat;

      mat->attachRenderable(this, sub);
    }
  }
}

void MeshInstance::setMaterial(uint32_t submesh_index, Material* mat)
{
  ROS_ASSERT(submesh_index < entity_->getNumSubEntities());
  Ogre::SubEntity* sub = entity_->getSubEntity(submesh_index);
  M_SubEntityToMaterial::iterator subent_it = sub_to_mat_.find(sub);
  if (subent_it != sub_to_mat_.end())
  {
    Material* old_mat = subent_it->second;
    mat_to_subs_[old_mat].erase(sub);
  }

  sub_to_mat_[sub] = mat;
  mat_to_subs_[mat].insert(sub);
  mat->attachRenderable(this, sub);
}

void MeshInstance::onOgreMaterialChanged(Material* mat)
{
  M_Material::iterator it = mat_to_subs_.find(mat);
  if (it != mat_to_subs_.end())
  {
    Material* mat = it->first;
    S_OgreSubEntity::iterator sub_it = it->second.begin();
    S_OgreSubEntity::iterator sub_end = it->second.end();
    for (; sub_it != sub_end; ++sub_it)
    {
      Ogre::SubEntity* sub = *sub_it;
      sub->setMaterial(mat->getOgreMaterial());
    }
  }
}

} // namespace rviz_renderer_ogre
