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

#include "render_ogre/renderer.h"
#include "ros/server/renderer_ros.h"
#include "ros/client/init.h"
#include "ros/client/render_window.h"
#include "ros/client/scene.h"
#include "ros/client/camera.h"
#include "math/vector3.h"
#include "math/quaternion.h"

#include <rviz_interfaces/Camera.h>
#include <rviz_interfaces/RenderWindow.h>
#include <rviz_interfaces/Scene.h>

#include <ros/package.h>
#include <ros/time.h>

#include <wx/wx.h>

#include <ros/time.h>
#include <ros/ros.h>

#ifdef __WXGTK__
#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <gdk/gdkx.h>
#include <wx/gtk/win_gtk.h>
#include <GL/glx.h>
#endif

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace rviz;

std::string getOgreHandle(wxWindow* wx_wind)
{
  std::string handle;

#ifdef __WXMSW__
  // Handle for Windows systems
  handle = Ogre::StringConverter::toString((size_t)((HWND)wx_wind->GetHandle()));
#elif defined(__WXGTK__)
  // Handle for GTK-based systems
  std::stringstream str;
  GtkWidget *widget = wx_wind->m_wxwindow;
  gtk_widget_set_double_buffered (widget, FALSE);
  gtk_widget_realize( widget );

  // Grab the window object
  GdkWindow *gdkWin = GTK_PIZZA (widget)->bin_window;
  Window wid = GDK_WINDOW_XWINDOW(gdkWin);

  XSync(GDK_WINDOW_XDISPLAY(widget->window), False);

  str << wid;// << ':';
  handle = str.str();
#elif defined(__WXMAC__)
  handle = Ogre::StringConverter::toString((size_t)(wx_wind->GetHandle()));
#else
  // Any other unsupported system
  #error("Not supported on this platform.")
#endif

  return handle;
}

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent)
  : wxFrame(parent, -1, _("rviz"), wxDefaultPosition, wxSize(800,600), wxDEFAULT_FRAME_STYLE)
  , private_nh_("~")
  , renderer_(ros::package::getPath(ROS_PACKAGE_NAME), true)
  , renderer_ros_(&renderer_, private_nh_)
  , timer_(this)
  {
    renderer_.start();

    ros::NodeHandle renderer_nh(private_nh_, "renderer");
    rviz::ros_client::addProxyInterface("camera", rviz_interface_gen::InterfacePtr(new rviz_interfaces::CameraProxy("camera", renderer_nh)));
    rviz::ros_client::addProxyInterface("render_window", rviz_interface_gen::InterfacePtr(new rviz_interfaces::RenderWindowProxy("render_window", renderer_nh)));
    rviz::ros_client::addProxyInterface("scene", rviz_interface_gen::InterfacePtr(new rviz_interfaces::SceneProxy("scene", renderer_nh)));

    render_window_ = ros_client::createRenderWindow(getOgreHandle(this), 800, 600);

    ros_client::Scene s = ros_client::createScene();
    ros_client::Camera c = s.createCamera();
    render_window_.attachCamera(c);

    c.setAutoAspectRatio(true);
    c.setPosition(Vector3(10, 0, 10));
    c.lookAt(Vector3(0, 0, 0));
    camera_ = c;

    Connect(wxEVT_SIZE, wxSizeEventHandler(MyFrame::onSize));
    Connect(wxEVT_TIMER, wxTimerEventHandler(MyFrame::onTimer));

    timer_.Start(16);
  }

  ~MyFrame()
  {
    ros_client::destroyRenderWindow(render_window_);

    renderer_.stop();

    rviz::ros_client::shutdown();
  }

  void onSize(wxSizeEvent& evt)
  {
    render_window_.resized(evt.GetSize().GetWidth(), evt.GetSize().GetHeight());
  }

  void onTimer(wxTimerEvent& evt)
  {
    if (last_update_.isZero())
    {
      last_update_ = ros::WallTime::now();
      return;
    }

    ros::WallTime cur = ros::WallTime::now();
    ros::WallDuration diff = cur - last_update_;
    double dt = diff.toSec();

    const float factor = 0.1;
    if (wxGetKeyState(WXK_UP))
    {
      camera_.moveRelative(Vector3(dt * factor, 0.0, 0.0));
    }

    if (wxGetKeyState(WXK_DOWN))
    {
      camera_.moveRelative(Vector3(-dt * factor, 0.0, 0.0));
    }

    if (wxGetKeyState(WXK_LEFT))
    {
      camera_.moveRelative(Vector3(0.0, dt * factor, 0.0));
    }

    if (wxGetKeyState(WXK_RIGHT))
    {
      camera_.moveRelative(Vector3(0.0, -dt * factor, 0.0));
    }
}

private:
  ros::NodeHandle private_nh_;
  render::ogre::Renderer renderer_;
  RendererROS renderer_ros_;
  ros_client::Camera camera_;
  ros_client::RenderWindow render_window_;

  wxTimer timer_;

  ros::WallTime last_update_;
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp
{
public:
  char** local_argv_;

  bool OnInit()
  {
#ifdef __WXMAC__
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
    SetFrontProcess(&PSN);
#endif

    // create our own copy of argv, with regular char*s.
    local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv_, "rviz", ros::init_options::NoSigintHandler);
    ros_client::init(ros::names::resolve("~"));

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);

