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

#include "ogre_renderer.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{
namespace render
{

OgreRenderer::OgreRenderer(const std::string& root_path, bool enable_ogre_log)
: running_(true)
, first_window_created_(false)
, root_path_(root_path)
{
  render_thread_ = boost::thread(&OgreRenderer::renderThread, this, enable_ogre_log);
}

OgreRenderer::~OgreRenderer()
{
  running_ = false;
  render_thread_.join();
}

void OgreRenderer::init(bool enable_ogre_log)
{
  Ogre::LogManager* log_manager = new Ogre::LogManager();
  log_manager->createLog( "Ogre.log", false, false, !enable_ogre_log );

  std::string plugin_cfg = "/etc/OGRE/plugins.cfg";
  bool has_plugin_cfg = false;
#ifdef HAS_INSTALLED_OGRE
  has_plugin_cfg = true;
#else
  plugin_cfg = "";
#endif

  Ogre::Root* root = new Ogre::Root( plugin_cfg );
  if ( !has_plugin_cfg )
  {
    root->loadPlugin( "RenderSystem_GL" );
    root->loadPlugin( "Plugin_OctreeSceneManager" );
    root->loadPlugin( "Plugin_ParticleFX" );
    root->loadPlugin( "Plugin_CgProgramManager" );
  }

  // Taken from gazebo
  Ogre::RenderSystem* render_system = NULL;
#if OGRE_VERSION_MAJOR >=1 && OGRE_VERSION_MINOR >= 7
  const Ogre::RenderSystemList& rsList = root->getAvailableRenderers();
  Ogre::RenderSystemList::const_iterator renderIt = rsList.begin();
  Ogre::RenderSystemList::const_iterator renderEnd = rsList.end();
#else
  Ogre::RenderSystemList* rsList = root->getAvailableRenderers();
  Ogre::RenderSystemList::iterator renderIt = rsList->begin();
  Ogre::RenderSystemList::iterator renderEnd = rsList->end();
#endif
  for ( ; renderIt != renderEnd; ++renderIt )
  {
    render_system = *renderIt;

    if ( render_system->getName() == "OpenGL Rendering Subsystem" )
    {
      break;
    }
  }

  if ( render_system == NULL )
  {
    throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
  }

  render_system->setConfigOption("FSAA","2");
  render_system->setConfigOption("RTT Preferred Mode", "FBO");

  root->setRenderSystem( render_system );

  root->initialise( false );
}

void OgreRenderer::oneTimeInit()
{
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( root_path_ + "/ogre_media/textures", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( root_path_ + "/ogre_media/fonts", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( root_path_ + "/ogre_media/models", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( root_path_ + "/ogre_media/materials/programs", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( root_path_ + "/ogre_media/materials/scripts", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

void OgreRenderer::createRenderWindow(const std::string& name, const std::string& parent_window)
{
  Ogre::Root* root = Ogre::Root::getSingletonPtr();
  Ogre::NameValuePairList params;
  params["parentWindowHandle"] = parent_window;
  Ogre::RenderWindow* win2 = root->createRenderWindow(name, 1024, 768, false, &params);

  if (!first_window_created_)
  {
    oneTimeInit();
    first_window_created_ = true;
  }

  Ogre::SceneManager* man = root->createSceneManager(Ogre::ST_GENERIC, "primary");
  Ogre::Camera* cam = man->createCamera("cam1");
  Ogre::Entity* ent = man->createEntity("sphere", "sphere.mesh");
  ent->setMaterialName("BaseWhiteNoLighting");
  Ogre::SceneNode* node = man->getRootSceneNode()->createChildSceneNode();
  node->attachObject(ent);
  node->setVisible(true);
  node->setPosition(0, 0, 0);

  win2->addViewport(cam);
  win2->setActive(true);
  win2->setVisible(true);
  win2->setAutoUpdated(true);

  cam->setPosition(0, 15, 15);
  cam->lookAt(0, 0, 0);
  cam->setNearClipDistance(0.01);
}

void OgreRenderer::renderThread(bool enable_ogre_log)
{
  init(enable_ogre_log);

  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(test_mutex);
      if (!test.empty())
      {
        createRenderWindow(test, test);
        test.clear();
      }
    }

    Ogre::Root::getSingleton().renderOneFrame();
  }

  delete Ogre::Root::getSingletonPtr();
}

} // namespace render
} // namespace rviz
