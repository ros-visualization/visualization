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

#ifndef RVIZ_RENDERER_OGRE_POINTS_MANAGER_H
#define RVIZ_RENDERER_OGRE_POINTS_MANAGER_H

#include "points_renderer_desc.h"

#include <ros/message_forward.h>

#include <rviz_uuid/uuid.h>

#include <OGRE/OgreVector3.h>

namespace rviz_msgs
{
ROS_DECLARE_MESSAGE(Points);
}

namespace Ogre
{
class SceneManager;
}

namespace rviz_renderer_ogre
{

class PointsRenderer;

class PointsManager
{
public:
  PointsManager(Ogre::SceneManager* scene_manager);
  ~PointsManager();

  void addPoints(const rviz_uuid::UUID& id, const rviz_msgs::Points& points);
  void removePoints(const rviz_uuid::UUID& id);

private:
  PointsRenderer* getOrCreateRendererForPoints(const rviz_msgs::Points& points);
  PointsRendererDesc descFromPoints(const rviz_msgs::Points& points);

  struct PointsInfo
  {
    rviz_uuid::UUID id;
    uint32_t internal_id;
    PointsRenderer* renderer;
  };
  typedef std::map<rviz_uuid::UUID, PointsInfo> M_PointsInfo;
  M_PointsInfo points_to_renderer_;

  typedef std::map<PointsRendererDesc, PointsRenderer*> M_DescToRenderer;
  M_DescToRenderer renderers_;

  Ogre::SceneManager* scene_manager_;
};

} // namespace rviz_renderer_ogre

#endif // RVIZ_RENDERER_OGRE_POINTS_MANAGER_H
