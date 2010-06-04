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

namespace rviz_renderer_ogre
{

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

} // namespace rviz_renderer_ogre


