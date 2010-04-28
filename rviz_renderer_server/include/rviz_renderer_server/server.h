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

#ifndef RVIZ_RENDERER_SERVER_SERVER_H
#define RVIZ_RENDERER_SERVER_SERVER_H

#include <vector>

#include <ros/types.h>

#include <boost/shared_ptr.hpp>

namespace ros
{
class NodeHandle;
class CallbackQueue;
}

namespace rviz_renderer_interface
{
class IRenderer;
}

namespace rviz_renderer_server
{

class CameraServer;
class RenderWindowServer;
class SceneServer;

typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
typedef boost::shared_ptr<ros::CallbackQueue> CallbackQueuePtr;

class Server
{
public:
  Server(rviz_renderer_interface::IRenderer* renderer, const ros::NodeHandle& nh);
  ~Server();

private:

  rviz_renderer_interface::IRenderer* renderer_;

  CallbackQueuePtr callback_queue_;

  boost::shared_ptr<CameraServer> camera_server_;
  boost::shared_ptr<RenderWindowServer> render_window_server_;
  boost::shared_ptr<SceneServer> scene_server_;

  NodeHandlePtr nh_;

  class RenderLoopListener;
  typedef boost::shared_ptr<RenderLoopListener> RenderLoopListenerPtr;
  RenderLoopListenerPtr render_loop_listener_;
};

} // namespace rviz_renderer_server


#endif // RVIZ_RENDERER_SERVER_SERVER_H
