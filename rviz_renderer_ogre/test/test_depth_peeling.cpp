#include <OGRE/Ogre.h>

#include <ros/package.h>

int main(int argc, char** argv)
{
  Ogre::LogManager* log_manager = new Ogre::LogManager();
  log_manager->createLog( "Ogre.log", false, false, false );

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
  render_system->setConfigOption("Full Screen", "False");
  render_system->setConfigOption("Video Mode", "800 x 600");

  root->setRenderSystem( render_system );

  //if (root->restoreConfig() || root->showConfigDialog())
  {
    Ogre::RenderWindow* wnd = root->initialise(true);
    wnd->setActive(true);
    wnd->setAutoUpdated(true);
    wnd->setVisible(true);

    Ogre::ResourceGroupManager::getSingleton().createResourceGroup(ROS_PACKAGE_NAME);

    std::string pkg_path = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( pkg_path + "/media/textures", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( pkg_path + "/media/fonts", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( pkg_path + "/media/models", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( pkg_path + "/media/materials/programs", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( pkg_path + "/media/materials/scripts", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( pkg_path + "/media/compositors", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    Ogre::LogManager::getSingleton().getDefaultLog()->setDebugOutputEnabled(true);
    Ogre::LogManager::getSingleton().getDefaultLog()->setLogDetail(Ogre::LL_BOREME);

    Ogre::SceneManager* sm = root->createSceneManager(Ogre::ST_GENERIC);
    Ogre::Camera* cam = sm->createCamera("Camera");
    Ogre::Viewport* vp = wnd->addViewport(cam);

    cam->setPosition(0, 0, 2);
    cam->lookAt(0, 0, 0);
    cam->setNearClipDistance(0.01);

    Ogre::SceneNode* sn = sm->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* ent = sm->createEntity("sphere.mesh");//Ogre::SceneManager::PT_SPHERE);
    ent->setMaterialName("BaseWhite");
    sn->attachObject(ent);
    sn->setVisible(true);
    sn->setPosition(0, 0, 0);

    root->startRendering();
  }

  return 0;
}
