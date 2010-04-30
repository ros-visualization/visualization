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

#include <rviz_renderer_ogre/init.h>
#include "rviz_renderer_client/init.h"
#include "rviz_renderer_client/render_window.h"
#include "rviz_renderer_client/scene.h"
#include "rviz_renderer_client/camera.h"
#include "rviz_renderer_client/simple_shape.h"
#include "rviz_renderer_client/transform_node.h"
#include "rviz_renderer_client/simple_color_material.h"
#include "rviz_math/vector3.h"
#include "rviz_math/quaternion.h"

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

using namespace rviz_math;

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
  , timer_(this)
  {
    render_window_ = rviz_renderer_client::createRenderWindow(getOgreHandle(this), 800, 600);

    rviz_renderer_client::Scene s = rviz_renderer_client::createScene();
    rviz_renderer_client::Camera c = s.createCamera();
    render_window_.attachCamera(c);

    c.setAutoAspectRatio(true);
    c.setPosition(Vector3(10, 0, 10));
    c.lookAt(Vector3(0, 0, 0));
    camera_ = c;

    rviz_renderer_client::SimpleShape shape = s.createSimpleShape("sphere", s.createTransformNode());
    rviz_renderer_client::SimpleColorMaterial mat = rviz_renderer_client::createSimpleColorMaterial();
    mat.setColor(0.0, 1.0, 1.0, 0.5);
    shape.setMaterial(mat);

    rviz_renderer_client::TransformNode n = s.createTransformNode();
    n.setPosition(2, 0, 0);
    shape = s.createSimpleShape("cube", n);
    mat = rviz_renderer_client::createSimpleColorMaterial();
    mat.setColor(0.0, 1.0, 0.0, 1.0);
    shape.setMaterial(mat);

    n = s.createTransformNode();
    n.setPosition(-2, 0, 0);
    shape = s.createSimpleShape("cylinder", n);
    mat = rviz_renderer_client::createSimpleColorMaterial();
    mat.setColor(0.0, 0.0, 1.0, 1.0);
    shape.setMaterial(mat);

    n = s.createTransformNode();
    n.setPosition(-4, 0, 0);
    shape = s.createSimpleShape("cone", n);
    mat = rviz_renderer_client::createSimpleColorMaterial();
    mat.setColor(1.0, 0.0, 0.0, 1.0);
    shape.setMaterial(mat);

    Connect(wxEVT_SIZE, wxSizeEventHandler(MyFrame::onSize));
    Connect(wxEVT_TIMER, wxTimerEventHandler(MyFrame::onTimer));

    timer_.Start(16);
  }

  ~MyFrame()
  {
    rviz_renderer_client::destroyRenderWindow(render_window_);

    // TODO: Once rviz_renderer_ogre creates its own initial rendering window, move this to OnExit
    rviz_renderer_ogre::shutdown();
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

    const float factor = 0.01;
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
  rviz_renderer_client::Camera camera_;
  rviz_renderer_client::RenderWindow render_window_;

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

    //wxHandleFatalExceptions();

    // create our own copy of argv, with regular char*s.
    local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv_, "rviz", ros::init_options::NoSigintHandler);
    rviz_renderer_ogre::init(true, "~/renderer");
    rviz_renderer_client::init("~/renderer");

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }

  int OnExit()
  {
    rviz_renderer_client::shutdown();
    return 0;
  }

  int OnRun()
  {
    try
    {
      wxApp::OnRun();
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Unhandled exception: %s", e.what());
      return 1;
    }

    return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);

