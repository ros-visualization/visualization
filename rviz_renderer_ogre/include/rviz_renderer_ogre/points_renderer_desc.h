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

#ifndef RVIZ_RENDERER_OGRE_POINTS_RENDERER_DESC_H
#define RVIZ_RENDERER_OGRE_POINTS_RENDERER_DESC_H

#include <OGRE/OgreVector3.h>
#include <ros/types.h>

namespace rviz_renderer_ogre
{

struct PointsRendererDesc
{
  enum CustomParam
  {
    CustomParam_Size,
  };

  uint8_t type;
  Ogre::Vector3 scale;
  bool has_normals;
  bool has_orientation;

  bool operator<(const PointsRendererDesc& rhs) const
  {
    if (type != rhs.type)
    {
      return type < rhs.type;
    }

    if (!scale.positionEquals(rhs.scale, 0.005))
    {
      return scale < rhs.scale;
    }

    if (has_normals != rhs.has_normals)
    {
      return has_normals < rhs.has_normals;
    }

    return has_orientation < rhs.has_orientation;
  }
};

} // namespace rviz_renderer_ogre

#endif // RVIZ_RENDERER_OGRE_POINTS_RENDERER_DESC_H
