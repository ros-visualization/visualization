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

#include "rviz_renderer_client/init.h"
#include "rviz_renderer_client/scene.h"
#include <rviz_uuid/uuid.h>

#include <rviz_interfaces/Camera.h>
#include <rviz_interfaces/RenderWindow.h>
#include <rviz_interfaces/Scene.h>
#include <rviz_interfaces/SimpleShape.h>
#include <rviz_interfaces/TransformNode.h>
#include <rviz_interfaces/SimpleColorMaterial.h>
#include <rviz_interfaces/MeshInstance.h>

#include <ros/ros.h>

using namespace rviz_uuid;
using namespace rviz_renderer_client;

static ros::NodeHandlePtr g_node_handle;

void rviz_renderer_client::init(const std::string& server_namespace)
{
  g_node_handle.reset(new ros::NodeHandle(server_namespace));

  rviz_renderer_client::addProxyInterface("camera", rviz_interface_gen::InterfacePtr(new rviz_interfaces::CameraProxy("camera", *g_node_handle)));
  rviz_renderer_client::addProxyInterface("render_window", rviz_interface_gen::InterfacePtr(new rviz_interfaces::RenderWindowProxy("render_window", *g_node_handle)));
  rviz_renderer_client::addProxyInterface("scene", rviz_interface_gen::InterfacePtr(new rviz_interfaces::SceneProxy("scene", *g_node_handle)));
  rviz_renderer_client::addProxyInterface("simple_shape", rviz_interface_gen::InterfacePtr(new rviz_interfaces::SimpleShapeProxy("simple_shape", *g_node_handle)));
  rviz_renderer_client::addProxyInterface("transform_node", rviz_interface_gen::InterfacePtr(new rviz_interfaces::TransformNodeProxy("transform_node", *g_node_handle)));
  rviz_renderer_client::addProxyInterface("simple_color_material", rviz_interface_gen::InterfacePtr(new rviz_interfaces::SimpleColorMaterialProxy("simple_color_material", *g_node_handle)));
  rviz_renderer_client::addProxyInterface("mesh_instance", rviz_interface_gen::InterfacePtr(new rviz_interfaces::MeshInstanceProxy("mesh_instance", *g_node_handle)));
}

ros::NodeHandle& rviz_renderer_client::getNodeHandle()
{
  return *g_node_handle;
}

typedef std::map<std::string, rviz_interface_gen::InterfacePtr> M_Proxy;
M_Proxy g_proxies;

void rviz_renderer_client::addProxyInterface(const std::string& name, const rviz_interface_gen::InterfacePtr& proxy)
{
  if (!g_proxies.insert(std::make_pair(name, proxy)).second)
  {
    throw std::runtime_error("Proxy interface [" + name + "] already exists!");
  }
}

void rviz_renderer_client::removeProxyInterface(const std::string& name)
{
  g_proxies.erase(name);
}

rviz_interface_gen::Interface* rviz_renderer_client::getProxyInterface(const std::string& name)
{
  M_Proxy::iterator it = g_proxies.find(name);
  if (it == g_proxies.end())
  {
    throw std::runtime_error("Proxy interface [" + name + "] does not exist!");
  }

  return it->second.get();
}

void rviz_renderer_client::shutdown()
{
  g_proxies.clear();
  g_node_handle.reset();
}


