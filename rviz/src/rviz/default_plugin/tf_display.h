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

#ifndef RVIZ_TF_DISPLAY_H
#define RVIZ_TF_DISPLAY_H

#include "rviz/display.h"
#include "rviz/properties/forwards.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <map>
#include <set>

namespace rviz
{
class Arrow;
class Axes;
class MovableText;
}

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class BoolProperty;
class Vector3Property;
class QuaternionProperty;
class FloatProperty;
class CategoryProperty;
class StringProperty;

struct FrameInfo;
typedef std::set<FrameInfo*> S_FrameInfo;

/**
 * \class TFDisplay
 * \brief Displays a visual representation of the TF hierarchy
 */
class TFDisplay : public Display
{
public:
  TFDisplay();
  virtual ~TFDisplay();

  virtual void onInitialize();

  bool getShowNames() { return show_names_; }
  void setShowNames( bool show );

  bool getShowAxes() { return show_axes_; }
  void setShowAxes( bool show );

  bool getShowArrows() { return show_arrows_; }
  void setShowArrows( bool show );

  float getUpdateRate() { return update_rate_; }
  void setUpdateRate( float rate );

  bool getAllEnabled() { return all_enabled_; }
  void setAllEnabled(bool enabled);

  float getFrameTimeout() { return frame_timeout_; }
  void setFrameTimeout(float timeout);

  float getScale() { return scale_; } 
  void setScale(float scale); 

  void setFrameEnabled(FrameInfo* frame, bool enabled);

  // Overrides from Display
  virtual void update(float wall_dt, float ros_dt);
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void reset();

protected:
  void updateFrames();
  FrameInfo* createFrame(const std::string& frame);
  void updateFrame(FrameInfo* frame);
  void deleteFrame(FrameInfo* frame, bool delete_properties);

  FrameInfo* getFrameInfo(const std::string& frame);

  void clear();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  Ogre::SceneNode* root_node_;
  Ogre::SceneNode* names_node_;
  Ogre::SceneNode* arrows_node_;
  Ogre::SceneNode* axes_node_;

  typedef std::map<std::string, FrameInfo*> M_FrameInfo;
  M_FrameInfo frames_;

  float update_timer_;
  float update_rate_;

  bool show_names_;
  bool show_arrows_;
  bool show_axes_;
  float frame_timeout_;
  bool all_enabled_;

  float scale_;

  Property* show_names_property_;
  Property* show_arrows_property_;
  Property* show_axes_property_;
  FloatProperty* update_rate_property_;
  FloatProperty* frame_timeout_property_;
  Property* all_enabled_property_;

  FloatProperty* scale_property_;

  Property* frames_category_;
  Property* tree_category_;
};

} // namespace rviz

 #endif

