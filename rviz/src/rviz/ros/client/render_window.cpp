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

#include "render_window.h"
#include "init.h"
#include "camera.h"

#include <rviz_interfaces/RenderWindow.h>

using namespace rviz_uuid;

namespace rviz
{
namespace ros_client
{

RenderWindow createRenderWindow(const std::string& parent_window, uint32_t width, uint32_t height)
{
  rviz_interfaces::RenderWindowProxy* proxy = getProxyInterface<rviz_interfaces::RenderWindowProxy>("render_window");
  UUID id = UUID::Generate();
  proxy->create(id, parent_window, width, height);
  return RenderWindow(id);
}

void destroyRenderWindow(const RenderWindow& wnd)
{
  rviz_interfaces::RenderWindowProxy* proxy = getProxyInterface<rviz_interfaces::RenderWindowProxy>("render_window");
  proxy->destroy(wnd.getID());
}

RenderWindow::RenderWindow()
{
}

RenderWindow::RenderWindow(const rviz_uuid::UUID& id)
: Object(id)
{
  proxy_ = getProxyInterface<rviz_interfaces::RenderWindowProxy>("render_window");
}

RenderWindow::~RenderWindow()
{
}

void RenderWindow::resized(uint32_t width, uint32_t height)
{
  proxy_->resized(getID(), width, height);
}

void RenderWindow::attachCamera(const Camera& cam)
{
  proxy_->attachCamera(getID(), cam.getID());
}

} // namespace ros
} // namespace rviz
