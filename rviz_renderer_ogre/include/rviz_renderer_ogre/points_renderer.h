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

#ifndef RVIZ_RENDERER_OGRE_POINTS_RENDERER_H
#define RVIZ_RENDERER_OGRE_POINTS_RENDERER_H

#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSimpleRenderable.h>
#include <OGRE/OgreMaterial.h>

#include "points_renderer_desc.h"

#include <ros/message_forward.h>

#include <boost/unordered_map.hpp>
#include <boost/random/linear_congruential.hpp>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace rviz_msgs
{
ROS_DECLARE_MESSAGE(Points);
}

namespace rviz_renderer_ogre
{

class PointsRenderer;
class PointsRenderable : public Ogre::SimpleRenderable
{
public:
  PointsRenderable(PointsRenderer* parent, const PointsRendererDesc& desc);
  ~PointsRenderable();

  void add(const rviz_msgs::Points& points, uint32_t start, uint32_t& out_start, uint32_t& out_count);
  void remove(uint32_t start, uint32_t count);
  bool isEmpty();
  bool isFull();

  virtual Ogre::Real getBoundingRadius(void) const;
  virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const;
  virtual void _notifyCurrentCamera(Ogre::Camera* camera);
  virtual void getWorldTransforms(Ogre::Matrix4* xform) const;
  virtual const Ogre::LightList& getLights() const;

private:
  uint32_t getVerticesPerPoint();
  float* getVertices();
  float* getNormals();
  uint32_t getPointStride();

  PointsRenderer* parent_;
  PointsRendererDesc desc_;
  bool supports_geometry_programs_;

  uint32_t point_count_;

  bool needs_offsets_;
  bool needs_normals_;
};
typedef boost::shared_ptr<PointsRenderable> PointsRenderablePtr;
typedef std::vector<PointsRenderablePtr> V_PointsRenderable;

class PointsRenderer : public Ogre::MovableObject
{
public:
  PointsRenderer(Ogre::SceneManager* scene_manager, const PointsRendererDesc& desc);
  ~PointsRenderer();

  uint32_t add(const rviz_msgs::Points& points);
  void remove(uint32_t id);
  void clear();

  virtual const Ogre::String& getMovableType() const { return sm_type; }
  virtual const Ogre::AxisAlignedBox& getBoundingBox() const;
  virtual float getBoundingRadius() const;
  virtual void getWorldTransforms( Ogre::Matrix4* xform ) const;
  virtual void _updateRenderQueue( Ogre::RenderQueue* queue );
  virtual void _notifyCurrentCamera( Ogre::Camera* camera );
  virtual void _notifyAttached(Ogre::Node *parent, bool isTagPoint=false);
  virtual void visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables);

private:
  PointsRenderablePtr getOrCreateRenderable();
  void shrinkRenderables();
  void recalculateBounds();

  struct PointsInfo
  {
    struct RenderableInfo
    {
      PointsRenderablePtr rend;
      uint32_t start;
      uint32_t count;
    };

    std::vector<RenderableInfo> rends;
  };

  typedef boost::unordered_map<uint32_t, PointsInfo> M_PointsInfo;
  M_PointsInfo points_;
  boost::rand48 id_gen_;

  PointsRendererDesc desc_;
  Ogre::SceneManager* scene_manager_;

  Ogre::AxisAlignedBox bounding_box_;
  float bounding_radius_;

  V_PointsRenderable renderables_;

  Ogre::MaterialPtr opaque_material_;
  Ogre::MaterialPtr alpha_material_;
  Ogre::SceneNode* scene_node_;

  static const Ogre::String sm_type;
};

} // namespace rviz_renderer_ogre

#endif // RVIZ_RENDERER_OGRE_POINTS_RENDERER_H
