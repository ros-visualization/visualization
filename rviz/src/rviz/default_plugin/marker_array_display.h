/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef RVIZ_MARKER_ARRAY_DISPLAY_H
#define RVIZ_MARKER_ARRAY_DISPLAY_H

#include "marker_display.h"

namespace rviz
{

/**
 * Display for an array of markers.  The MarkerDisplay class handles
 * MarkerArray messages.  This is just a wrapper to let MarkerArray
 * topics get selected in the topic browser.
 */
class MarkerArrayDisplay : public MarkerDisplay
{
public:
  MarkerArrayDisplay();
  virtual ~MarkerArrayDisplay();

  void setTopic(const std::string& topic);
  const std::string& getTopic() { return topic_; }

  virtual void createProperties();

protected:
  /**
   * \brief Subscribes to the marker array topic
   */
  virtual void subscribe();
  /**
   * \brief Unsubscribes from the marker array topic
   */
  virtual void unsubscribe();

  void handleMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& array);

  std::string topic_;
  ROSTopicStringPropertyWPtr topic_property_;
};

} // end namespace rviz

#endif // RVIZ_MARKER_ARRAY_DISPLAY_H
