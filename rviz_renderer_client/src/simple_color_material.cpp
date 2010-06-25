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

#include <rviz_renderer_client/simple_color_material.h>
#include <rviz_renderer_client/init.h>
#include <rviz_renderer_client/color.h>

#include <rviz_math/vector3.h>
#include <rviz_math/quaternion.h>

#include <rviz_interfaces/Material.h>

using namespace rviz_math;
using namespace rviz_interfaces;
using namespace rviz_uuid;

namespace rviz_renderer_client
{

SimpleColorMaterial::SimpleColorMaterial()
: proxy_(0)
{}

SimpleColorMaterial::SimpleColorMaterial(const rviz_uuid::UUID& id)
: Material(id)
{
  proxy_ = getProxyInterface<MaterialProxy>("material");
}

void SimpleColorMaterial::setColor(const Color& c)
{
  rviz_msgs::Material mat;
  mat.id = getID();
  mat.has_color = true;
  mat.opacity = c.a;
  mat.color.r = c.r;
  mat.color.g = c.g;
  mat.color.b = c.b;
  proxy_->setMaterial(getID(), mat);
}

void SimpleColorMaterial::setColor(float r, float g, float b, float a)
{
  setColor(Color(r, g, b, a));
}

SimpleColorMaterial createSimpleColorMaterial()
{
  rviz_interfaces::MaterialProxy* proxy = getProxyInterface<rviz_interfaces::MaterialProxy>("material");
  UUID id = UUID::Generate();
  proxy->create(id);
  return SimpleColorMaterial(id);
}

void destroySimpleColorMaterial(const SimpleColorMaterial& mat)
{
  rviz_interfaces::MaterialProxy* proxy = getProxyInterface<rviz_interfaces::MaterialProxy>("material");
  proxy->destroy(mat.getID());
}

}

