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
#include "rviz_renderer_client/mesh_instance.h"
#include "rviz_renderer_client/simple_color_material.h"
#include "rviz_renderer_client/points.h"
#include "rviz_math/vector3.h"
#include "rviz_math/quaternion.h"

#include <rviz_msgs/Points.h>

// TODO: remove use of these
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

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
  : wxFrame(parent, -1, _("rviz"), wxDefaultPosition, wxSize(1024, 768), wxDEFAULT_FRAME_STYLE)
  , private_nh_("~")
  , left_mouse_down_( false )
  , middle_mouse_down_( false )
  , right_mouse_down_( false )
  , mouse_x_( 0 )
  , mouse_y_( 0 )
  , timer_(this)
  {
    render_window_ = rviz_renderer_client::createRenderWindow(getOgreHandle(this), 1024, 768);

    rviz_renderer_client::Scene s = rviz_renderer_client::createScene();
    rviz_renderer_client::Camera c = s.createCamera();
    render_window_.attachCamera(c);

    c.setAutoAspectRatio(true);
    c.setPosition(Vector3(0, 0, 10));
    c.lookAt(Vector3(0, 0, 0));
    camera_ = c;

    rviz_renderer_client::TransformNode n;

#if 01
    rviz_renderer_client::SimpleShape shape = s.createSimpleShape("sphere", s.createTransformNode());
    shape.setColor(1.0, 1.0, 1.0, 1.0);

    n = s.createTransformNode();
    n.setPosition(2, 0, 0);
    shape = s.createSimpleShape("cube", n);
    shape.setColor(0.0, 1.0, 0.0, 1.0);

    n = s.createTransformNode();
    n.setPosition(-2, 0, 0);
    shape = s.createSimpleShape("cylinder", n);
    shape.setColor(0.0, 0.0, 1.0, 1.0);

    n = s.createTransformNode();
    n.setPosition(-4, 0, 0);
    shape = s.createSimpleShape("cone", n);
    shape.setColor(1.0, 0.0, 0.0, 1.0);

#if 01
    for (uint32_t x = 0; x < 10; ++x)
    {
      for (uint32_t y = 0; y < 10; ++y)
      {
        for (uint32_t z = 0; z < 10; ++z)
        {
          n = s.createTransformNode();
          n.setPosition(x + 10, y, z);
          shape = s.createSimpleShape("sphere", n);
          shape.setColor(x * 0.1, y * 0.1, z * 0.1, y * 0.1);
        }
      }
    }
#endif
#endif

    n = s.createTransformNode();
    n.setPosition(0, 0, 2);
    rviz_renderer_client::MeshInstance inst = s.createMeshInstance("package://pr2_description/meshes/base_v0/base.dae", n);

    n = s.createTransformNode();
    n.setPosition(2, 0, 2);
    inst = s.createMeshInstance("package://pr2_description/meshes/base_v0/wheel.dae", n);

#if 0
    {
      rviz_msgs::Points points;
      points.type = rviz_msgs::Points::TYPE_BILLBOARDS;
      points.scale.x = 0.5;
      points.scale.y = 0.5;
      points.scale.z = 0.5;

      for (uint32_t x = 0; x < 100; ++x)
      {
        for (uint32_t y = 0; y < 100; ++y)
        {
          for (uint32_t z = 0; z < 100; ++z)
          {
            rviz_msgs::Vector3 pos;
            pos.x = -10.0 - x;
            pos.y = y;
            pos.z = z;
            points.positions.push_back(pos);

            std_msgs::ColorRGBA col;
            col.r = x * 0.1;
            col.g = y * 0.1;
            col.b = z * 0.1;
            col.a = 1.0;//(z % 2 == 0) ? 1.0 : 0.5;
            points.colors.push_back(col);
          }
        }
      }

      rviz_renderer_client::Points p = s.createPoints(points);
    }
#endif

    {
      rviz_msgs::Points points;
      points.type = rviz_msgs::Points::TYPE_BILLBOARD_SPHERES;
      points.scale.x = 0.05;
      points.scale.y = 0.1;
      points.scale.z = 0.1;

      for (float th = 0.0; th <= M_PI * 2.0; th += 0.1)
      {
        for (float phi = 0.0; phi <= M_PI; phi += 0.1)
        {
          float x = cos(th) * sin(phi);
          float y = sin(th) * sin(phi);
          float z = cos(phi);

          rviz_msgs::Vector3 pos;
          pos.x = 5*x;
          pos.y = 5*y;
          pos.z = 5*z;
          points.positions.push_back(pos);

          std_msgs::ColorRGBA col;
          col.r = x * 0.1;
          col.g = y * 0.1;
          col.b = z * 0.1;
          col.a = 1.0;//(z % 2 == 0) ? 1.0 : 0.5;
          points.colors.push_back(col);

#if 01
          rviz_msgs::Vector3 norm;
          Ogre::Vector3 n(x, y, z);
          n.normalise();
          norm.x = n.x;
          norm.y = n.y;
          norm.z = n.z;
          points.normals.push_back(norm);
#endif
        }
      }

      rviz_renderer_client::Points p = s.createPoints(points);
    }


    Connect(wxEVT_SIZE, wxSizeEventHandler(MyFrame::onSize));
    Connect(wxEVT_TIMER, wxTimerEventHandler(MyFrame::onTimer));

    Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_MOTION, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_LEFT_UP, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );
    Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MyFrame::onMouseEvents ), NULL, this );

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
    last_update_ = cur;
    double dt = diff.toSec();

    const float factor = 1;
    if (wxGetKeyState(WXK_UP))
    {
      camera_.moveRelative(Vector3(0.0, 0.0, -dt * factor));
    }

    if (wxGetKeyState(WXK_DOWN))
    {
      camera_.moveRelative(Vector3(0.0, 0.0, dt * factor));
    }

    if (wxGetKeyState(WXK_LEFT))
    {
      camera_.moveRelative(Vector3(-dt * factor, 0.0, 0.0));
    }

    if (wxGetKeyState(WXK_RIGHT))
    {
      camera_.moveRelative(Vector3(dt * factor, 0.0, 0.0));
    }
  }

  void onMouseEvents( wxMouseEvent& event )
  {
   int lastX = mouse_x_;
   int lastY = mouse_y_;

   mouse_x_ = event.GetX();
   mouse_y_ = event.GetY();

   if ( event.LeftDown() )
   {
     left_mouse_down_ = true;
     middle_mouse_down_ = false;
     right_mouse_down_ = false;
   }
   else if ( event.MiddleDown() )
   {
     left_mouse_down_ = false;
     middle_mouse_down_ = true;
     right_mouse_down_ = false;
   }
   else if ( event.RightDown() )
   {
     left_mouse_down_ = false;
     middle_mouse_down_ = false;
     right_mouse_down_ = true;
   }
   else if ( event.LeftUp() )
   {
     left_mouse_down_ = false;
   }
   else if ( event.MiddleUp() )
   {
     middle_mouse_down_ = false;
   }
   else if ( event.RightUp() )
   {
     right_mouse_down_ = false;
   }
   else if ( event.Dragging() )
   {
     int32_t diff_x = mouse_x_ - lastX;
     int32_t diff_y = mouse_y_ - lastY;

     bool handled = false;
     if ( left_mouse_down_ )
     {
       Ogre::Quaternion quat, quat2, combined;
       quat.FromAngleAxis(Ogre::Radian(-diff_y * 0.002), Ogre::Vector3::UNIT_X);
       quat2.FromAngleAxis(Ogre::Radian(-diff_x * 0.002), Ogre::Vector3::UNIT_Y);
       combined = quat * quat2;
       camera_.rotateRelative(rviz_math::Quaternion(combined.x, combined.y, combined.z, combined.w));

       handled = true;
     }
     else if ( middle_mouse_down_ )
     {
       camera_.moveRelative(rviz_math::Vector3(diff_x * 0.01, -diff_y * 0.01, 0.0));

       handled = true;
     }
     else if ( right_mouse_down_ )
     {
       camera_.moveRelative(rviz_math::Vector3(0.0, 0.0, diff_y * 0.01));

       handled = true;
     }
   }

   if ( event.GetWheelRotation() != 0 )
   {
     //camera_->scrollWheel( event.GetWheelRotation(), event.CmdDown(), event.AltDown(), event.ShiftDown() );
   }
  }

private:
  ros::NodeHandle private_nh_;
  rviz_renderer_client::Camera camera_;
  rviz_renderer_client::RenderWindow render_window_;

  // Mouse handling
  bool left_mouse_down_;
  bool middle_mouse_down_;
  bool right_mouse_down_;
  int mouse_x_;
  int mouse_y_;

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

