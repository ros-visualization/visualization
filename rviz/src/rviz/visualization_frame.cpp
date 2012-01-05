/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <QSplashScreen>
#include <QDockWidget>
#include <QDir>
#include <QCloseEvent>
#include <QToolBar>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QFileDialog>
#include <QDesktopServices>
#include <QUrl>

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <ros/package.h>
#include <ros/console.h>

#include <ogre_tools/initialization.h>
#include <ogre_tools/render_system.h>

#include "visualization_frame.h"
#include "render_panel.h"
#include "displays_panel.h"
#include "views_panel.h"
#include "time_panel.h"
#include "selection_panel.h"
#include "tool_properties_panel.h"
#include "visualization_manager.h"
#include "tools/tool.h"
#include "loading_dialog.h"
#include "config.h"
#include "panel_dock_widget.h"

//// If need to use gtk to get window position under X11.
// #include <gdk/gdk.h>
// #include <gdk/gdkx.h>

namespace fs = boost::filesystem;

#define CONFIG_WINDOW_X "/Window/X"
#define CONFIG_WINDOW_Y "/Window/Y"
#define CONFIG_WINDOW_WIDTH "/Window/Width"
#define CONFIG_WINDOW_HEIGHT "/Window/Height"
// I am not trying to preserve peoples' window layouts from wx to Qt,
// just saving the Qt layout in a new config tag.
#define CONFIG_QMAINWINDOW "/QMainWindow"
#define CONFIG_AUIMANAGER_PERSPECTIVE "/AuiManagerPerspective"
#define CONFIG_AUIMANAGER_PERSPECTIVE_VERSION "/AuiManagerPerspectiveVersion"
#define CONFIG_RECENT_CONFIGS "/RecentConfigs"
#define CONFIG_LAST_DIR "/LastConfigDir"

#define CONFIG_EXTENSION "vcg"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION
#define PERSPECTIVE_VERSION 2

#define RECENT_CONFIG_COUNT 10

namespace rviz
{

VisualizationFrame::VisualizationFrame( QWidget* parent )
  : QMainWindow( parent )
  , render_panel_(NULL)
  , displays_panel_(NULL)
  , views_panel_(NULL)
  , time_panel_(NULL)
  , selection_panel_(NULL)
  , tool_properties_panel_(NULL)
  , file_menu_(NULL)
  , recent_configs_menu_(NULL)
  , toolbar_(NULL)
  , manager_(NULL)
  , position_correction_( 0, 0 )
  , num_move_events_( 0 )
  , toolbar_actions_( NULL )
{
  setWindowTitle( "RViz" );
}

VisualizationFrame::~VisualizationFrame()
{
  if( manager_ )
  {
    manager_->removeAllDisplays();
  }

  delete render_panel_;
  delete manager_;
}

void VisualizationFrame::closeEvent( QCloseEvent* event )
{
  if( general_config_ )
  {
    saveConfigs();
  }
  event->accept();
}

void VisualizationFrame::onSplashLoadStatus( const std::string& status )
{
  splash_->showMessage( QString::fromStdString( status ));
}

void VisualizationFrame::initialize(const std::string& display_config_file,
                                    const std::string& fixed_frame,
                                    const std::string& target_frame,
                                    const std::string& splash_path,
                                    bool verbose )
{
  initConfigs();

  int new_x, new_y, new_width, new_height;
  general_config_->get( CONFIG_WINDOW_X, &new_x, x() );
  general_config_->get( CONFIG_WINDOW_Y, &new_y, y() );
  general_config_->get( CONFIG_WINDOW_WIDTH, &new_width, width() );
  general_config_->get( CONFIG_WINDOW_HEIGHT, &new_height, height() );

  {
    std::string recent;
    if( general_config_->get( CONFIG_RECENT_CONFIGS, &recent ))
    {
      boost::trim( recent );
      boost::split( recent_configs_, recent, boost::is_any_of (":"), boost::token_compress_on );
    }

    general_config_->get( CONFIG_LAST_DIR, &last_config_dir_ );
  }

  move( new_x, new_y );
  resize( new_width, new_height );

  package_path_ = ros::package::getPath("rviz_qt");

  std::string final_splash_path = splash_path;

  if ( splash_path.empty() )
  {
#if BOOST_FILESYSTEM_VERSION == 3
    final_splash_path = (fs::path(package_path_) / "images/splash.png").string();
#else
    final_splash_path = (fs::path(package_path_) / "images/splash.png").file_string();
#endif
  }
  QPixmap splash_image( QString::fromStdString( final_splash_path ));
  splash_ = new QSplashScreen( splash_image );
  splash_->show();
  splash_->showMessage( "Initializing" );

  if( !ros::isInitialized() )
  {
    int argc = 0;
    ros::init( argc, 0, "rviz", ros::init_options::AnonymousName );
  }

  render_panel_ = new RenderPanel( ogre_tools::RenderSystem::get(), 0, this );
  displays_panel_ = new DisplaysPanel( this );
  views_panel_ = new ViewsPanel( this );
  time_panel_ = new TimePanel( this );
  selection_panel_ = new SelectionPanel( this );
  tool_properties_panel_ = new ToolPropertiesPanel( this );

  splash_->showMessage( "Initializing OGRE resources" );
  ogre_tools::V_string paths;
  paths.push_back( package_path_ + "/ogre_media/textures" );
  ogre_tools::initializeResources( paths );

  initMenus();
  toolbar_ = addToolBar( "Tools" );
  toolbar_->setObjectName( "Tools" );
  toolbar_actions_ = new QActionGroup( this );
  connect( toolbar_actions_, SIGNAL( triggered( QAction* )), this, SLOT( onToolbarActionTriggered( QAction* )));
  view_menu_->addAction( toolbar_->toggleViewAction() );

  setCentralWidget( render_panel_ );

  addPane( "Displays", displays_panel_, Qt::LeftDockWidgetArea, false );
  addPane( "Tool Properties", tool_properties_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Views", views_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Selection", selection_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Time", time_panel_, Qt::BottomDockWidgetArea, false );

  manager_ = new VisualizationManager( render_panel_, this );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  displays_panel_->initialize( manager_ );
  views_panel_->initialize( manager_ );
  time_panel_->initialize(manager_);
  selection_panel_->initialize( manager_ );
  tool_properties_panel_->initialize( manager_ );

  connect( manager_, SIGNAL( toolAdded( Tool* )), this, SLOT( addTool( Tool* )));
  connect( manager_, SIGNAL( toolChanged( Tool* )), this, SLOT( indicateToolIsCurrent( Tool* )));

  manager_->initialize( StatusCallback(), verbose );
  manager_->loadGeneralConfig(general_config_, boost::bind( &VisualizationFrame::onSplashLoadStatus, this, _1 ));

  bool display_config_valid = !display_config_file.empty();
  if( display_config_valid && !fs::exists( display_config_file ))
  {
    ROS_ERROR("File [%s] does not exist", display_config_file.c_str());
    display_config_valid = false;
  }

  if( !display_config_valid )
  {
    manager_->loadDisplayConfig( display_config_, boost::bind( &VisualizationFrame::onSplashLoadStatus, this, _1 ));
  }
  else
  {
    boost::shared_ptr<Config> config( new Config );
    config->readFromFile( display_config_file ); 
    manager_->loadDisplayConfig( config, boost::bind( &VisualizationFrame::onSplashLoadStatus, this, _1 ));
  }

  if( !fixed_frame.empty() )
  {
    manager_->setFixedFrame( fixed_frame );
  }

  if( !target_frame.empty() )
  {
    manager_->setTargetFrame( target_frame );
  }

  splash_->showMessage( "Loading perspective" );

  std::string main_window_config;
  if( general_config_->get( CONFIG_QMAINWINDOW, &main_window_config ))
  {
    restoreState( QByteArray::fromHex( main_window_config.c_str() ));
  }

  updateRecentConfigMenu();
  if( display_config_valid )
  {
    markRecentConfig( display_config_file );
  }

  delete splash_;
  splash_ = 0;

  manager_->startUpdate();
}

void VisualizationFrame::initConfigs()
{
  config_dir_ = QDir::toNativeSeparators( QDir::homePath() ).toStdString();
#if BOOST_FILESYSTEM_VERSION == 3
  std::string old_dir = (fs::path(config_dir_) / ".standalone_visualizer").string();
  config_dir_ = (fs::path(config_dir_) / ".rviz_qt").string();
  general_config_file_ = (fs::path(config_dir_) / "config").string();
  display_config_file_ = (fs::path(config_dir_) / "display_config").string();
#else
  std::string old_dir = (fs::path(config_dir_) / ".standalone_visualizer").file_string();
  config_dir_ = (fs::path(config_dir_) / ".rviz_qt").file_string();
  general_config_file_ = (fs::path(config_dir_) / "config").file_string();
  display_config_file_ = (fs::path(config_dir_) / "display_config").file_string();
#endif

  if( fs::exists( old_dir ) && !fs::exists( config_dir_ ))
  {
    ROS_INFO("Migrating old config directory to new location ([%s] to [%s])", old_dir.c_str(), config_dir_.c_str());
    fs::rename( old_dir, config_dir_ );
  }

  if( fs::is_regular_file( config_dir_ ))
  {
    ROS_INFO("Migrating old config file to new location ([%s] to [%s])", config_dir_.c_str(), general_config_file_.c_str());
    std::string backup_file = config_dir_ + "bak";

    fs::rename(config_dir_, backup_file);
    fs::create_directory(config_dir_);
    fs::rename(backup_file, general_config_file_);
  }
  else if (!fs::exists(config_dir_))
  {
    fs::create_directory(config_dir_);
  }

  if (fs::exists(general_config_file_) && !fs::exists(display_config_file_))
  {
    ROS_INFO("Creating display config from general config");
    fs::copy_file(general_config_file_, display_config_file_);
  }

  ROS_INFO("Loading general config from [%s]", general_config_file_.c_str());
  general_config_.reset( new Config );
  general_config_->readFromFile( general_config_file_ );

  ROS_INFO("Loading display config from [%s]", display_config_file_.c_str());
  display_config_.reset( new Config );
  display_config_->readFromFile( display_config_file_ );
}

void VisualizationFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu( "&File" );
  file_menu_->addAction( "&Open Config", this, SLOT( onOpen() ), QKeySequence( "Ctrl+O" ));
  file_menu_->addAction( "&Save Config", this, SLOT( onSave() ), QKeySequence( "Ctrl+S" ));
  recent_configs_menu_ = file_menu_->addMenu( "&Recent Configs" );
  file_menu_->addSeparator();
  file_menu_->addAction( "&Quit", this, SLOT( close() ), QKeySequence( "Ctrl+Q" ));

  view_menu_ = menuBar()->addMenu( "&View" );

/////  plugins_menu_ = new wxMenu("");
/////  item = plugins_menu_->Append(wxID_ANY, "&Manage...");
/////  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onManagePlugins), NULL, this);
/////  menubar_->Append(plugins_menu_, "&Plugins");
/////

  QMenu* help_menu = menuBar()->addMenu( "&Help" );
  help_menu->addAction( "Wiki", this, SLOT( onHelpWiki() ));
}

void VisualizationFrame::updateRecentConfigMenu()
{
  recent_configs_menu_->clear();

  D_string::iterator it = recent_configs_.begin();
  D_string::iterator end = recent_configs_.end();
  for (; it != end; ++it)
  {
    if( *it != "" )
    {
      recent_configs_menu_->addAction( QString::fromStdString( *it ), this, SLOT( onRecentConfigSelected() ));
    }
  }
}

void VisualizationFrame::markRecentConfig( const std::string& path )
{
  D_string::iterator it = std::find( recent_configs_.begin(), recent_configs_.end(), path );
  if( it != recent_configs_.end() )
  {
    recent_configs_.erase( it );
  }

  recent_configs_.push_front( path );

  if( recent_configs_.size() > RECENT_CONFIG_COUNT )
  {
    recent_configs_.pop_back();
  }

  updateRecentConfigMenu();
}

void VisualizationFrame::loadDisplayConfig( const std::string& path )
{
  if( !fs::exists( path ))
  {
    QString message = QString::fromStdString( path  ) + " does not exist!";
    QMessageBox::critical( this, "Config file does not exist", message );
    return;
  }

  manager_->removeAllDisplays();

  LoadingDialog dialog( this );
  dialog.show();

  boost::shared_ptr<Config> config( new Config );
  config->readFromFile( path );
  manager_->loadDisplayConfig( config, boost::bind( &LoadingDialog::setState, &dialog, _1 ));

  markRecentConfig(path);
}


void VisualizationFrame::moveEvent( QMoveEvent* event )
{
//  GdkRectangle rect;
//  GdkWindow* gdk_window = gdk_window_foreign_new( winId() ); 
//  gdk_window_get_frame_extents( gdk_window, &rect );
//  printf( "gdk x=%d, y=%d\n", rect.x, rect.y );
// the above works!  should I just use gdk??

  // HACK to work around a bug in Qt-for-X11.  The first time we get a
  // moveEvent, the position is that of the top-left corner of the
  // window frame.  The second time we get one, the position is the
  // top-left corner *inside* the frame.  There is no significant time
  // lag between the two calls, certainly no user events, so I just
  // remember the first position and diff it with the second position
  // and remember the diff as a corrective offset for future geometry
  // requests.
  //
  // This seems like it would be brittle to OS, code changes, etc, so
  // sometime I should get something better going here.  Maybe call
  // out to gdk (as above), which seems to work right.
  switch( num_move_events_ )
  {
  case 0:
    first_position_ = pos();
    num_move_events_++;
    break;
  case 1:
    position_correction_ = first_position_ - pos();
    num_move_events_++;
    break;
  }
}

QRect VisualizationFrame::hackedFrameGeometry()
{
  QRect geom = frameGeometry();
  geom.moveTopLeft( pos() + position_correction_ );
  return geom;
}

void VisualizationFrame::saveConfigs()
{
  ROS_INFO("Saving general config to [%s]", general_config_file_.c_str());
  general_config_->clear();
  QRect geom = hackedFrameGeometry();
  general_config_->set( CONFIG_WINDOW_X, geom.x() );
  general_config_->set( CONFIG_WINDOW_Y, geom.y() );
  general_config_->set( CONFIG_WINDOW_WIDTH, geom.width() );
  general_config_->set( CONFIG_WINDOW_HEIGHT, geom.height() );

  QByteArray window_state = saveState().toHex();
  general_config_->set( CONFIG_QMAINWINDOW, std::string( window_state.constData() ));

  {
    std::stringstream ss;
    D_string::iterator it = recent_configs_.begin();
    D_string::iterator end = recent_configs_.end();
    for (; it != end; ++it)
    {
      if (it != recent_configs_.begin())
      {
        ss << ":";
      }
      ss << *it;
    }

    general_config_->set( CONFIG_RECENT_CONFIGS, ss.str() );
  }

  general_config_->set( CONFIG_LAST_DIR, last_config_dir_ );

  manager_->saveGeneralConfig( general_config_ );
  general_config_->writeToFile( general_config_file_ );

  ROS_INFO( "Saving display config to [%s]", display_config_file_.c_str() );
  display_config_->clear();
  manager_->saveDisplayConfig( display_config_ );
  display_config_->writeToFile( display_config_file_ );
}

void VisualizationFrame::onOpen()
{
  QString filename = QFileDialog::getOpenFileName( this, "Choose a file to open",
                                                   QString::fromStdString( last_config_dir_ ),
                                                   "RViz config files (" CONFIG_EXTENSION_WILDCARD ")" );

  if( !filename.isEmpty() )
  {
    std::string filename_string = filename.toStdString();
    loadDisplayConfig( filename_string );
    last_config_dir_ = fs::path( filename_string ).parent_path().string();
  }
}

void VisualizationFrame::onSave()
{
  QString q_filename = QFileDialog::getSaveFileName( this, "Choose a file to save to",
                                                     QString::fromStdString( last_config_dir_ ),
                                                     "RViz config files (" CONFIG_EXTENSION_WILDCARD ")" );

  if( !q_filename.isEmpty() )
  {
    std::string filename = q_filename.toStdString();
    fs::path path( filename );
    if( path.extension() != "."CONFIG_EXTENSION )
    {
      filename += "."CONFIG_EXTENSION;
    }

    boost::shared_ptr<Config> config( new Config() );
    manager_->saveDisplayConfig( config );
    config->writeToFile( filename );

    markRecentConfig( filename );

    last_config_dir_ = fs::path( filename ).parent_path().string();
  }
}

void VisualizationFrame::onRecentConfigSelected()
{
  QAction* action = dynamic_cast<QAction*>( sender() );
  if( action )
  {
    std::string path = action->text().toStdString();
    if( !path.empty() )
    {
      loadDisplayConfig( path );
    }
  }
}

void VisualizationFrame::addTool( Tool* tool )
{
  QAction* action = new QAction( QString::fromStdString( tool->getName() ), toolbar_actions_ );
  action->setCheckable( true );
  action->setShortcut( QKeySequence( QString( tool->getShortcutKey() )));
  toolbar_->addAction( action );
  action_to_tool_map_[ action ] = tool;
  tool_to_action_map_[ tool ] = action;
}

void VisualizationFrame::onToolbarActionTriggered( QAction* action )
{
  Tool* tool = action_to_tool_map_[ action ];
  if( tool )
  {
    manager_->setCurrentTool( tool );
  }
}

void VisualizationFrame::indicateToolIsCurrent( Tool* tool )
{
  QAction* action = tool_to_action_map_[ tool ];
  if( action )
  {
    action->setChecked( true );
  }
}

/////void VisualizationFrame::onManagePlugins(wxCommandEvent& event)
/////{
/////  PluginManagerDialog dialog(this, manager_->getPluginManager());
/////  dialog.ShowModal();
/////}
/////
void VisualizationFrame::onHelpWiki()
{
  QDesktopServices::openUrl( QUrl( "http://www.ros.org/wiki/rviz" ));
}

QWidget* VisualizationFrame::getParentWindow()
{
  return this;
}

PanelDockWidget* VisualizationFrame::addPane( const std::string& name, QWidget* panel, Qt::DockWidgetArea area, bool floating )
{
  QString q_name = QString::fromStdString( name );
  PanelDockWidget *dock;
  dock = new PanelDockWidget( q_name, this );
  dock->setWidget( panel );
  dock->setFloating( floating );
  dock->setObjectName( q_name );
  addDockWidget( area, dock );
  view_menu_->addAction( dock->toggleViewAction() );
  return dock;
}

}
