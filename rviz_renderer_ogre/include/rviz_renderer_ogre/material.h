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

#ifndef RVIZ_RENDERER_OGRE_MATERIAL_H
#define RVIZ_RENDERER_OGRE_MATERIAL_H

#include <rviz_uuid/uuid.h>
#include <OGRE/OgreMaterial.h>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace Ogre
{
class Renderable;
}

namespace rviz_renderer_ogre
{

class Renderable;

class Material : public boost::enable_shared_from_this<Material>
{
public:
  Material(const rviz_uuid::UUID& id)
  : id_(id)
  {}

  const rviz_uuid::UUID& getID() { return id_; }

  virtual const Ogre::MaterialPtr& getOgreMaterial() { return material_; }
  virtual void attachRenderable(Renderable* rend, Ogre::Renderable* ogre_rend);
  virtual void detachRenderable(Renderable* rend, Ogre::Renderable* ogre_rend);
  virtual void detachRenderable(Renderable* rend);

protected:
  virtual void onRenderableAttached(Renderable* rend, Ogre::Renderable* ogre_rend) = 0;
  virtual void onRenderableDetached(Renderable* rend, Ogre::Renderable* ogre_rend) = 0;

  rviz_uuid::UUID id_;

  Ogre::MaterialPtr material_;

  typedef std::vector<Ogre::Renderable*> V_OgreRenderable;
  typedef std::map<Renderable*, V_OgreRenderable> M_Renderable;
  M_Renderable rends_;
};
typedef boost::shared_ptr<Material> MaterialPtr;

}

#endif // RVIZ_RENDERER_OGRE_MATERIAL_H
