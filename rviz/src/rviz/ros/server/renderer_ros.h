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

#include "../forwards.h"

#include <vector>

#include <ros/types.h>
#include <ros/message_traits.h>

namespace ros
{
class ServiceServer;
class Subscriber;
class NodeHandle;
class CallbackQueue;
}

namespace rviz_msgs
{
ROS_DECLARE_MESSAGE(CreateRenderWindowRequest);
ROS_DECLARE_MESSAGE(CreateRenderWindowResponse);
ROS_DECLARE_MESSAGE(DestroyRenderWindowRequest);
ROS_DECLARE_MESSAGE(DestroyRenderWindowResponse);
ROS_DECLARE_MESSAGE(RenderWindowCommand);
ROS_DECLARE_MESSAGE(CreateCameraRequest);
ROS_DECLARE_MESSAGE(CreateCameraResponse);
ROS_DECLARE_MESSAGE(DestroyCameraRequest);
ROS_DECLARE_MESSAGE(DestroyCameraResponse);
ROS_DECLARE_MESSAGE(CreateSceneRequest);
ROS_DECLARE_MESSAGE(CreateSceneResponse);
ROS_DECLARE_MESSAGE(DestroySceneRequest);
ROS_DECLARE_MESSAGE(DestroySceneResponse);
}

namespace rviz
{

namespace render
{
class IRenderer;
} // namespace render

class RendererROS
{
public:
  RendererROS(render::IRenderer* renderer, const ros::NodeHandle& nh);
  ~RendererROS();

private:
  bool onCreateRenderWindow(rviz_msgs::CreateRenderWindowRequest& req, rviz_msgs::CreateRenderWindowResponse& res);
  bool onDestroyRenderWindow(rviz_msgs::DestroyRenderWindowRequest& req, rviz_msgs::DestroyRenderWindowResponse& res);
  void onRenderWindowCommand(const rviz_msgs::RenderWindowCommandConstPtr& msg);

  bool onCreateCamera(rviz_msgs::CreateCameraRequest& req, rviz_msgs::CreateCameraResponse& res);
  bool onDestroyCamera(rviz_msgs::DestroyCameraRequest& req, rviz_msgs::DestroyCameraResponse& res);

  bool onCreateScene(rviz_msgs::CreateSceneRequest& req, rviz_msgs::CreateSceneResponse& res);
  bool onDestroyScene(rviz_msgs::DestroySceneRequest& req, rviz_msgs::DestroySceneResponse& res);

  render::IRenderer* renderer_;

  CallbackQueuePtr callback_queue_;
  NodeHandlePtr nh_;

  typedef std::vector<ServiceServerPtr> V_ServiceServerPtr;
  V_ServiceServerPtr srvs_;

  typedef std::vector<SubscriberPtr> V_SubscriberPtr;
  V_SubscriberPtr subs_;

  class RenderLoopListener;
  typedef boost::shared_ptr<RenderLoopListener> RenderLoopListenerPtr;
  RenderLoopListenerPtr render_loop_listener_;
};

}
