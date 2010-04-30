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

#include <rviz_renderer_ogre/server.h>
#include <rviz_renderer_ogre/renderer.h>
#include <rviz_renderer_ogre/render_window.h>
#include <rviz_renderer_ogre/scene.h>
#include <rviz_renderer_ogre/transform_node.h>
#include <rviz_renderer_ogre/camera.h>
#include <rviz_renderer_ogre/simple_color_material.h>
#include <rviz_uuid/uuid.h>
#include <rviz_math/vector3.h>
#include <rviz_math/quaternion.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rviz_interfaces/Camera.h>
#include <rviz_interfaces/RenderWindow.h>
#include <rviz_interfaces/Scene.h>
#include <rviz_interfaces/SimpleShape.h>
#include <rviz_interfaces/TransformNode.h>
#include <rviz_interfaces/SimpleColorMaterial.h>

using namespace rviz_uuid;

namespace rviz_renderer_ogre
{

class CameraServer : public rviz_interfaces::CameraServer
{
public:
  CameraServer(Renderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::CameraServer(name, nh)
  , renderer_(rend)
  {
  }

  Camera* lookupCamera(const rviz_uuid::UUID& id)
  {
    Camera* cam = renderer_->getCamera(id);
    if (!cam)
    {
      std::stringstream ss;
      ss << "Invalid camera [" << id << "]";
      throw std::runtime_error(ss.str());
    }

    return cam;
  }

  virtual void setPosition(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& pos)
  {
    lookupCamera(id)->setPosition(pos);
  }

  virtual void setOrientation(const rviz_msgs::UUID& id, const geometry_msgs::Quaternion& orient)
  {
    lookupCamera(id)->setOrientation(orient);
  }

  virtual void lookAt(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& pos)
  {
    lookupCamera(id)->lookAt(pos);
  }

  virtual void move(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& vec)
  {
    lookupCamera(id)->move(vec);
  }

  virtual void moveRelative(const rviz_msgs::UUID& id, const geometry_msgs::Vector3& vec)
  {
    lookupCamera(id)->moveRelative(vec);
  }

  virtual void rotate(const rviz_msgs::UUID& id, const geometry_msgs::Quaternion& quat)
  {
    lookupCamera(id)->rotate(quat);
  }

  virtual void setFOVY(const rviz_msgs::UUID& id, float fovy)
  {
    lookupCamera(id)->setFOVY(fovy);
  }

  virtual void setAspectRatio(const rviz_msgs::UUID& id, float aspect)
  {
    lookupCamera(id)->setAspectRatio(aspect);
  }

  virtual void setAutoAspectRatio(const rviz_msgs::UUID& id, uint8_t autoaspect)
  {
    lookupCamera(id)->setAutoAspectRatio(autoaspect);
  }

  virtual void setNearClipDistance(const rviz_msgs::UUID& id, float dist)
  {
    lookupCamera(id)->setNearClipDistance(dist);
  }

  virtual void setFarClipDistance(const rviz_msgs::UUID& id, float dist)
  {
    lookupCamera(id)->setFarClipDistance(dist);
  }

private:
  Renderer* renderer_;
};

class RenderWindowServer : public rviz_interfaces::RenderWindowServer
{
public:
  RenderWindowServer(Renderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::RenderWindowServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void resized(const rviz_msgs::UUID& id, uint32_t width, uint32_t height)
  {
    RenderWindow* wnd = renderer_->getRenderWindow(id);
    wnd->resized(width, height);
  }

  virtual void attachCamera(const rviz_msgs::UUID& id, const rviz_msgs::UUID& camera_id)
  {
    RenderWindow* wnd = renderer_->getRenderWindow(id);
    wnd->attachCamera(camera_id);
  }

  virtual void create(const rviz_msgs::UUID& id, const std::string& parent_window, uint32_t width, uint32_t height)
  {
    renderer_->createRenderWindow(id, parent_window, width, height);
  }

  virtual void destroy(const rviz_msgs::UUID& id)
  {
    renderer_->destroyRenderWindow(id);
  }

private:
  Renderer* renderer_;
};

class SceneServer : public rviz_interfaces::SceneServer
{
public:
  SceneServer(Renderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::SceneServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void create(const rviz_msgs::UUID& id)
  {
    Scene* scene = renderer_->createScene(id);
    if (!scene)
    {
      throw std::runtime_error("Could not create scene [" + UUID(id).toString() + "]");
    }
  }

  virtual void destroy(const rviz_msgs::UUID& id)
  {
    renderer_->destroyScene(id);
  }

  virtual void createCamera(const rviz_msgs::UUID& id, const rviz_msgs::UUID& camera_id)
  {
    Scene* scene = renderer_->getScene(id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(id).toString() + "] does not exist");
    }

    scene->createCamera(camera_id);
  }

  virtual void destroyCamera(const rviz_msgs::UUID& id, const rviz_msgs::UUID& camera_id)
  {
    Scene* scene = renderer_->getScene(id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(id).toString() + "] does not exist");
    }

    scene->destroyCamera(camera_id);
  }

  virtual void createSimpleShape(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& shape_id, const std::string& type, const rviz_msgs::UUID& node_id)
  {
    SimpleShape::Type t;
    if (type == "cone")
    {
      t = SimpleShape::Cone;
    }
    else if (type == "cube")
    {
      t = SimpleShape::Cube;
    }
    else if (type == "cylinder")
    {
      t = SimpleShape::Cylinder;
    }
    else if (type == "sphere")
    {
      t = SimpleShape::Sphere;
    }
    else
    {
      throw std::runtime_error("Unrecognized simple shape [" + type + "]");
    }

    Scene* scene = renderer_->getScene(scene_id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(scene_id).toString() + "] does not exist");
    }

    scene->createSimpleShape(shape_id, t, node_id);
  }

  virtual void destroySimpleShape(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& shape_id)
  {
    Scene* scene = renderer_->getScene(scene_id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(scene_id).toString() + "] does not exist");
    }

    SimpleShape* shape = scene->getSimpleShape(shape_id);
    Material* mat = shape->getMaterial();
    if (mat)
    {
      mat->detachRenderable(shape);
    }

    scene->destroySimpleShape(shape_id);
  }

  virtual void createTransformNode(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& node_id, const rviz_msgs::UUID& parent_id)
  {
    Scene* scene = renderer_->getScene(scene_id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(scene_id).toString() + "] does not exist");
    }

    scene->createTransformNode(node_id, parent_id);
  }

  virtual void destroyTransformNode(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& node_id)
  {
    Scene* scene = renderer_->getScene(scene_id);
    if (!scene)
    {
      throw std::runtime_error("Scene [" + UUID(scene_id).toString() + "] does not exist");
    }

    scene->destroyTransformNode(node_id);
  }

private:
  Renderer* renderer_;
};

class SimpleShapeServer : public rviz_interfaces::SimpleShapeServer
{
public:
  SimpleShapeServer(Renderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::SimpleShapeServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void setType(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& shape_id, const std::string& type)
  {

  }

  virtual void setMaterial(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& shape_id, const rviz_msgs::UUID& material_id)
  {
    Scene* scene = renderer_->getScene(scene_id);
    SimpleShape* shape = scene->getSimpleShape(shape_id);
    Material* mat = renderer_->getMaterial(material_id);
    shape->setMaterial(mat);
    mat->attachRenderable(shape);
  }

private:
  Renderer* renderer_;
};

class TransformNodeServer : public rviz_interfaces::TransformNodeServer
{
public:
  TransformNodeServer(Renderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::TransformNodeServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void setPosition(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& id, const geometry_msgs::Vector3& pos)
  {
    Scene* scene = renderer_->getScene(scene_id);
    TransformNode* node = scene->getTransformNode(id);
    node->setPosition(pos);
  }

  virtual void setOrientation(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& id, const geometry_msgs::Quaternion& orient)
  {
    Scene* scene = renderer_->getScene(scene_id);
    TransformNode* node = scene->getTransformNode(id);
    node->setOrientation(orient);
  }

  virtual void setScale(const rviz_msgs::UUID& scene_id, const rviz_msgs::UUID& id, const geometry_msgs::Vector3& scale)
  {
    Scene* scene = renderer_->getScene(scene_id);
    TransformNode* node = scene->getTransformNode(id);
    node->setScale(scale);
  }

private:
  Renderer* renderer_;
};

class SimpleColorMaterialServer : public rviz_interfaces::SimpleColorMaterialServer
{
public:
  SimpleColorMaterialServer(Renderer* rend, const std::string& name, const ros::NodeHandle& nh)
  : rviz_interfaces::SimpleColorMaterialServer(name, nh)
  , renderer_(rend)
  {
  }

  virtual void create(const rviz_msgs::UUID& id)
  {
    SimpleColorMaterial* mat = new SimpleColorMaterial();
    renderer_->addMaterial(id, MaterialPtr(mat));
  }

  virtual void destroy(const rviz_msgs::UUID& id)
  {
    renderer_->removeMaterial(id);
  }

  virtual void setColor(const rviz_msgs::UUID& id, const std_msgs::ColorRGBA& col)
  {
    SimpleColorMaterial* mat = dynamic_cast<SimpleColorMaterial*>(renderer_->getMaterial(id));
    if (!mat)
    {
      std::stringstream ss;
      ss << "SimpleColorMaterial [" << id << "] does not exist!";
      throw std::runtime_error(ss.str());
    }

    mat->setColor(Ogre::ColourValue(col.r, col.g, col.b, col.a));
  }

private:
  Renderer* renderer_;
};


Server::Server(Renderer* renderer, const ros::NodeHandle& nh)
: renderer_(renderer)
, nh_(new ros::NodeHandle(nh))
{
  nh_->setCallbackQueue(renderer_->getServerThreadCallbackQueue());
  camera_server_.reset(new CameraServer(renderer_, "camera", *nh_));
  render_window_server_.reset(new RenderWindowServer(renderer_, "render_window", *nh_));
  scene_server_.reset(new SceneServer(renderer_, "scene", *nh_));
  simple_shape_server_.reset(new SimpleShapeServer(renderer_, "simple_shape", *nh_));
  transform_node_server_.reset(new TransformNodeServer(renderer_, "transform_node", *nh_));
  simple_color_material_server_.reset(new SimpleColorMaterialServer(renderer_, "simple_color_material", *nh_));
}

Server::~Server()
{
}


} // namespace rviz_renderer_ogre
