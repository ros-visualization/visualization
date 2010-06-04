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

#include <rviz_renderer_ogre/simple_color_material.h>
#include <rviz_renderer_ogre/renderable.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreVector4.h>
#include <OGRE/OgreRenderable.h>

#define CUSTOM_PARAMETER_OBJECT_ID 0
#define CUSTOM_PARAMETER_COLOR 1

namespace rviz_renderer_ogre
{

static const char* g_simple_color_material_name = "rviz/SimpleColor";
static const char* g_simple_color_alpha_material_name = "rviz/SimpleColorWithAlpha";

SimpleColorMaterial::SimpleColorMaterial()
: transparent_(false)
{
  getMaterial(false);
}

void SimpleColorMaterial::getMaterial(bool transparent)
{
  if (transparent)
  {
    material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_alpha_material_name);
  }
  else
  {
    material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_material_name);
  }
}

void SimpleColorMaterial::setColor(const Ogre::ColourValue& color)
{
  color_ = color;

  bool transparent = color.a < 0.99;
  bool trans_changed = false;
  if (transparent != transparent_)
  {
    transparent_ = transparent;
    getMaterial(transparent);
    trans_changed = true;
  }

  M_Renderable::iterator it = rends_.begin();
  M_Renderable::iterator end = rends_.end();
  for (; it != end; ++it)
  {
    Renderable* rend = it->first;
    if (trans_changed)
    {
      rend->onOgreMaterialChanged(this);
    }

    V_OgreRenderable::iterator og_it = it->second.begin();
    V_OgreRenderable::iterator og_end = it->second.end();
    for (; og_it != og_end; ++og_it)
    {
      Ogre::Renderable* ogre_rend = *og_it;
      ogre_rend->setCustomParameter(CUSTOM_PARAMETER_COLOR, Ogre::Vector4(color.r, color.g, color.b, color.a));
    }
  }
}

void SimpleColorMaterial::onRenderableAttached(Renderable* rend, Ogre::Renderable* ogre_rend)
{
  rend->onOgreMaterialChanged(this);
  ogre_rend->setCustomParameter(CUSTOM_PARAMETER_COLOR, Ogre::Vector4(color_.r, color_.g, color_.b, color_.a));
}

void SimpleColorMaterial::onRenderableDetached(Renderable* rend, Ogre::Renderable* ogre_rend)
{
}

} // namespace rviz_renderer_ogre
