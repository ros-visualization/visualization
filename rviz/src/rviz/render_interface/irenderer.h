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

#ifndef RVIZ_IRENDERER_H
#define RVIZ_IRENDERER_H

namespace rviz
{
class UUID;

namespace render
{

class IRenderLoopListener;
class IRenderWindow;
class IScene;

class IRenderer
{
public:
  virtual IRenderWindow* createRenderWindow(const std::string& name, const std::string& parent_window, uint32_t width, uint32_t height) = 0;
  virtual void destroyRenderWindow(const std::string& name) = 0;
  virtual IRenderWindow* getRenderWindow(const std::string& name) = 0;

  virtual IScene* createScene(const UUID& id) = 0;
  virtual void destroyScene(const UUID& id) = 0;
  virtual IScene* getScene(const UUID& id) = 0;

  virtual void addRenderLoopListener(IRenderLoopListener* listener) = 0;
  virtual void removeRenderLoopListener(IRenderLoopListener* listener) = 0;
};

} // namespace render
} // namespace rviz

#endif // RVIZ_IRENDERER_H
