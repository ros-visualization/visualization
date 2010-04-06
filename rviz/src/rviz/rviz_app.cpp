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

#include <ros/package.h>
#include <ros/time.h>

#include <wx/wx.h>

#include <ros/time.h>
#include <ros/ros.h>
#include <rviz_msgs/CreateRenderWindow.h>

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
  {
    renderer_.start();

    window_client_.reset(new ros_client::RenderWindow("primary", getOgreHandle(this), 800, 600));

    ros_client::Scene s = ros_client::createScene();
    ros_client::Camera c = s.createCamera();
    window_client_->attachCamera(c);

    Connect(wxEVT_SIZE, wxSizeEventHandler(MyFrame::onSize));
  }

  ~MyFrame()
  {
    window_client_->destroy();

    renderer_.stop();
  }

  void onSize(wxSizeEvent& evt)
  {
    window_client_->resized(evt.GetSize().GetWidth(), evt.GetSize().GetHeight());
  }


private:
  ros::NodeHandle private_nh_;
  render::ogre::Renderer renderer_;
  RendererROS renderer_ros_;
  boost::shared_ptr<ros_client::RenderWindow> window_client_;
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
    ros_client::initClient(ros::names::resolve("~"));

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);

