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

#include <rviz_renderer_ogre/points_manager.h>
#include <rviz_renderer_ogre/points_renderer.h>

#include <rviz_msgs/Points.h>

namespace rviz_renderer_ogre
{

PointsManager::PointsManager(Ogre::SceneManager* scene_manager)
: scene_manager_(scene_manager)
{

}

PointsManager::~PointsManager()
{
  M_DescToRenderer::iterator it = renderers_.begin();
  M_DescToRenderer::iterator end = renderers_.end();
  for (; it != end; ++it)
  {
    delete it->second;
  }
  renderers_.clear();
}

void PointsManager::addPoints(const rviz_uuid::UUID& id, const rviz_msgs::Points& points)
{
  PointsRenderer* renderer = getOrCreateRendererForPoints(points);
  uint32_t internal_id = renderer->add(points);
  PointsInfo info;
  info.id = id;
  info.internal_id = internal_id;
  info.renderer = renderer;
  points_to_renderer_[id] = info;
}

void PointsManager::removePoints(const rviz_uuid::UUID& id)
{
  M_PointsInfo::iterator it = points_to_renderer_.find(id);

  const PointsInfo& info = it->second;
  info.renderer->remove(info.internal_id);

  points_to_renderer_.erase(it);
}

PointsRenderer* PointsManager::getOrCreateRendererForPoints(const rviz_msgs::Points& points)
{
  PointsRendererDesc desc = descFromPoints(points);
  M_DescToRenderer::iterator it = renderers_.find(desc);
  if (it != renderers_.end())
  {
    return it->second;
  }

  PointsRenderer* rend = new PointsRenderer(scene_manager_, desc);

  renderers_.insert(std::make_pair(desc, rend));
  return rend;
}

PointsRendererDesc PointsManager::descFromPoints(const rviz_msgs::Points& points)
{
  PointsRendererDesc desc;
  desc.type = points.type;
  desc.scale.x = points.scale.x;
  desc.scale.y = points.scale.y;
  desc.scale.z = points.scale.z;
  desc.has_orientation = !points.orientations.empty() && points.type != rviz_msgs::Points::TYPE_POINTS;
  return desc;
}


} // namespace rviz_renderer_ogre
