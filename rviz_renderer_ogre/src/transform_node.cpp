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

#include <rviz_renderer_ogre/transform_node.h>
#include <rviz_renderer_ogre/convert.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneNode.h>

namespace rviz_renderer_ogre
{

TransformNode::TransformNode(Ogre::SceneManager* scene_manager, TransformNode* parent)
: scene_manager_(scene_manager)
{
  if (!parent)
  {
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }
  else
  {
    scene_node_ = parent->getOgreSceneNode()->createChildSceneNode();
  }

  scene_node_->setOrientation(fromRobot(Ogre::Quaternion::IDENTITY));
}

TransformNode::~TransformNode()
{
  scene_manager_->destroySceneNode(scene_node_);
}

void TransformNode::setPosition(const rviz_math::Vector3& pos)
{
  scene_node_->setPosition(fromRobot(pos));
}

void TransformNode::setOrientation(const rviz_math::Quaternion& orient)
{
  scene_node_->setOrientation(fromRobot(orient));
}

void TransformNode::setScale(const rviz_math::Vector3& scale)
{
  scene_node_->setScale(fromRobot(scale));
}

} // namespace rviz_renderer_ogre

