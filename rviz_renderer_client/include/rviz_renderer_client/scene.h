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

#ifndef RVIZ_ROS_CLIENT_SCENE_H
#define RVIZ_ROS_CLIENT_SCENE_H

#include "object.h"

namespace rviz_interfaces
{
class SceneProxy;
}

namespace rviz_renderer_client
{

class RenderWindow;
class Camera;
class SimpleShape;
class TransformNode;

class Scene : public Object
{
public:
  Scene();
  Scene(const rviz_uuid::UUID& id);

  Camera createCamera();
  void destroyCamera(const Camera& cam);

  SimpleShape createSimpleShape(const std::string& type, const TransformNode& node);
  void destroySimpleShape(const SimpleShape& shape);

  TransformNode createTransformNode();
  TransformNode createTransformNode(const TransformNode& parent);
  TransformNode createTransformNode(const rviz_uuid::UUID& parent);
  void destroyTransformNode(const TransformNode& node);

private:
  rviz_interfaces::SceneProxy* proxy_;
};

Scene createScene();
void destroyScene(const Scene& scene);

} // namespace rviz_renderer_client

#endif // RVIZ_ROS_CLIENT_SCENE_H

