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

#include <rviz_renderer_ogre/material.h>
#include <rviz_renderer_ogre/renderable.h>

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRenderable.h>

namespace rviz_renderer_ogre
{

static const char* g_simple_color_material_name = "rviz/SimpleColor";
static const char* g_simple_color_alpha_material_name = "rviz/SimpleColorWithAlpha";

void Material::setMaterial(const rviz_msgs::Material& mat)
{
  input_material_ = mat;
  createMaterialFromInput();
  materialUpdated();
}

void Material::createMaterialFromInput()
{
  Ogre::MaterialPtr old_mat = material_;

  // TODO this is terrible.  Need a material/shader manager that generates materials/shaders on the fly
  // based on input materials
  bool transparent = input_material_.opacity < 0.99;
  if (transparent)
  {
    if (input_material_.has_color && input_material_.has_texture)
    {
      material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_alpha_material_name);
    }
    else if (input_material_.has_color)
    {
      material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_alpha_material_name);
    }
    else if (input_material_.has_texture)
    {
      material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_alpha_material_name);
    }
    else
    {
      ROS_BREAK();
    }
  }
  else
  {
    if (input_material_.has_color && input_material_.has_texture)
    {
      material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_material_name);
    }
    else if (input_material_.has_color)
    {
      material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_material_name);
    }
    else if (input_material_.has_texture)
    {
      material_ = Ogre::MaterialManager::getSingleton().getByName(g_simple_color_material_name);
    }
    else
    {
      ROS_BREAK();
    }
  }

  if (old_mat != material_)
  {
    ogreMaterialChanged();
  }
}

void Material::ogreMaterialChanged()
{
  M_Renderable::iterator it = rends_.begin();
  M_Renderable::iterator end = rends_.end();
  for (; it != end; ++it)
  {
    Renderable* rend = it->first;
    rend->onOgreMaterialChanged(shared_from_this());
  }
}

void Material::materialUpdated()
{
  M_Renderable::iterator it = rends_.begin();
  M_Renderable::iterator end = rends_.end();
  for (; it != end; ++it)
  {
    V_OgreRenderable::iterator og_it = it->second.begin();
    V_OgreRenderable::iterator og_end = it->second.end();
    for (; og_it != og_end; ++og_it)
    {
      Ogre::Renderable* ogre_rend = *og_it;
      ogre_rend->setCustomParameter(CustomParam_Color, Ogre::Vector4(input_material_.color.r, input_material_.color.g, input_material_.color.b, input_material_.opacity));
    }
  }
}

void Material::attachRenderable(Renderable* rend, Ogre::Renderable* ogre_rend)
{
  rends_[rend].push_back(ogre_rend);

  onRenderableAttached(rend, ogre_rend);
}

void Material::detachRenderable(Renderable* rend, Ogre::Renderable* ogre_rend)
{
  M_Renderable::iterator it = rends_.find(rend);
  if (it != rends_.end())
  {
    V_OgreRenderable& ogre_rends = it->second;
    std::remove(ogre_rends.begin(), ogre_rends.end(), ogre_rend);
    ogre_rends.pop_back();

    if (ogre_rends.empty())
    {
      rends_.erase(it);
    }

    onRenderableDetached(rend, ogre_rend);
  }
}

void Material::detachRenderable(Renderable* rend)
{
  M_Renderable::iterator it = rends_.find(rend);
  if (it != rends_.end())
  {
    rends_.erase(it);
    onRenderableDetached(rend, 0);
  }
}

void Material::onRenderableAttached(Renderable* rend, Ogre::Renderable* ogre_rend)
{
  rend->onOgreMaterialChanged(shared_from_this());
  ogre_rend->setCustomParameter(CustomParam_Color, Ogre::Vector4(input_material_.color.r, input_material_.color.g, input_material_.color.b, input_material_.opacity));
}

void Material::onRenderableDetached(Renderable* rend, Ogre::Renderable* ogre_rend)
{
}

} // namespace rviz_renderer_ogre


