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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "rviz/default_plugin/interactive_marker_display.h"


namespace rviz
{

//////////////
bool validateFloats(const visualization_msgs::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.pose);
  valid = valid && validateFloats(msg.scale);
  for ( unsigned c=0; c<msg.controls.size(); c++)
  {
    valid = valid && validateFloats( msg.controls[c].orientation );
    for ( unsigned m=0; m<msg.controls[c].markers.size(); m++ )
    {
      valid = valid && validateFloats(msg.controls[c].markers[m].pose);
      valid = valid && validateFloats(msg.controls[c].markers[m].scale);
      valid = valid && validateFloats(msg.controls[c].markers[m].color);
      valid = valid && validateFloats(msg.controls[c].markers[m].points);
    }
  }
  return valid;
}
/////////////



InteractiveMarkerDisplay::InteractiveMarkerDisplay()
  : Display()
{
  marker_update_topic_property_ = new RosTopicProperty( "Update Topic", "",
                                                        ros::message_traits::datatype<visualization_msgs::InteractiveMarkerUpdate>(),
                                                        "visualization_msgs::InteractiveMarkerUpdate topic to subscribe to.",
                                                        this, SLOT( updateTopic() ));

  show_descriptions_property_ = new BoolProperty( "Show Descriptions", true,
                                                  "Whether or not to show the descriptions of each Interactive Marker.",
                                                  this, SLOT( updateShowDescriptions() ));

  show_axes_property_ = new BoolProperty( "Show Axes", false,
                                          "Whether or not to show the axes of each Interactive Marker.",
                                          this, SLOT( updateShowAxes() ));
}

void InteractiveMarkerDisplay::onInitialize()
{
  tf::Transformer* tf = context_->getFrameManager()->getTFClient();
  im_client_.reset( new interactive_markers::InteractiveMarkerClient( *tf, fixed_frame_.toStdString() ) );

  im_client_->setInitCb( boost::bind( &InteractiveMarkerDisplay::initCb, this, _1 ) );
  im_client_->setUpdateCb( boost::bind( &InteractiveMarkerDisplay::updateCb, this, _1 ) );
  im_client_->setResetCb( boost::bind( &InteractiveMarkerDisplay::resetCb, this, _1 ) );
  im_client_->setStatusCb( boost::bind( &InteractiveMarkerDisplay::statusCb, this, _1, _2, _3 ) );

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  client_id_ = ros::this_node::getName() + "/" + getNameStd();

  onEnable();
}

InteractiveMarkerDisplay::~InteractiveMarkerDisplay()
{
  scene_manager_->destroySceneNode( scene_node_ );
}

void InteractiveMarkerDisplay::onEnable()
{
  subscribe();
  scene_node_->setVisible( true );
}

void InteractiveMarkerDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible( false );
}

void InteractiveMarkerDisplay::updateTopic()
{
  unsubscribe();

  std::string update_topic = marker_update_topic_property_->getTopicStd();

  size_t idx = update_topic.find( "/update" );
  if ( idx != std::string::npos )
  {
    topic_ns_ = update_topic.substr( 0, idx );
    subscribe();
  }
  else
  {
    setStatusStd( StatusProperty::Error, "Topic", "Invalid topic name: " + update_topic );
  }

}

void InteractiveMarkerDisplay::subscribe()
{
  if ( isEnabled() )
  {
    im_client_->subscribe(topic_ns_);
  }
}

void InteractiveMarkerDisplay::unsubscribe()
{
  im_client_->shutdown();
  Display::reset();
}

void InteractiveMarkerDisplay::update(float wall_dt, float ros_dt)
{
  im_client_->update();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->update( wall_dt );
    }
  }
}

InteractiveMarkerDisplay::M_StringToIMPtr& InteractiveMarkerDisplay::getImMap( std::string server_id )
{
  M_StringToStringToIMPtr::iterator im_map_it = interactive_markers_.find( server_id );

  if ( im_map_it == interactive_markers_.end() )
  {
    im_map_it = interactive_markers_.insert( std::make_pair( server_id, M_StringToIMPtr() ) ).first;
  }

  return im_map_it->second;
}

void InteractiveMarkerDisplay::updateMarkers(
    const std::string& server_id,
    const std::vector<visualization_msgs::InteractiveMarker>& markers
    )
{
  M_StringToIMPtr& im_map = getImMap( server_id );

  for ( size_t i=0; i<markers.size(); i++ )
  {
    const visualization_msgs::InteractiveMarker& marker = markers[i];

    if ( !validateFloats( marker ) )
    {
      setStatusStd( StatusProperty::Error, marker.name, "Marker contains invalid floats!" );
      //setStatusStd( StatusProperty::Error, "General", "Marker " + marker.name + " contains invalid floats!" );
      continue;
    }
    ROS_DEBUG("Processing interactive marker '%s'. %d", marker.name.c_str(), (int)marker.controls.size() );

    std::map< std::string, IMPtr >::iterator int_marker_entry = im_map.find( marker.name );

    if ( int_marker_entry == im_map.end() )
    {
      int_marker_entry = im_map.insert( std::make_pair(marker.name, IMPtr ( new InteractiveMarker(this, context_, topic_ns_, client_id_) ) ) ).first;
    }

    if ( int_marker_entry->second->processMessage( marker ) )
    {
      int_marker_entry->second->setShowAxes( show_axes_property_->getBool() );
      int_marker_entry->second->setShowDescription( show_descriptions_property_->getBool() );
    }
    else
    {
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::eraseMarkers(
    const std::string& server_id,
    const std::vector<std::string>& erases )
{
  M_StringToIMPtr& im_map = getImMap( server_id );

  for ( size_t i=0; i<erases.size(); i++ )
  {
    im_map.erase( erases[i] );
  }
}

void InteractiveMarkerDisplay::updatePoses(
    const std::string& server_id,
    const std::vector<visualization_msgs::InteractiveMarkerPose>& marker_poses )
{
  M_StringToIMPtr& im_map = getImMap( server_id );

  for ( size_t i=0; i<marker_poses.size(); i++ )
  {
    const visualization_msgs::InteractiveMarkerPose& marker_pose = marker_poses[i];

    if ( !validateFloats( marker_pose.pose ) )
    {
      setStatusStd( StatusProperty::Error, marker_pose.name, "Pose message contains invalid floats!" );
      return;
    }

    std::map< std::string, IMPtr >::iterator int_marker_entry = im_map.find( marker_pose.name );

    if ( int_marker_entry != im_map.end() )
    {
      int_marker_entry->second->processMessage( marker_pose );
    }
    else
    {
      setStatusStd( StatusProperty::Error, marker_pose.name, "Pose received for non-existing marker '" + marker_pose.name );
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::initCb( visualization_msgs::InteractiveMarkerInitConstPtr msg )
{
  resetCb( msg->server_id );
  updateMarkers( msg->server_id, msg->markers );
}

void InteractiveMarkerDisplay::updateCb( visualization_msgs::InteractiveMarkerUpdateConstPtr msg )
{
  updateMarkers( msg->server_id, msg->markers );
  updatePoses( msg->server_id, msg->poses );
  eraseMarkers( msg->server_id, msg->erases );
}

void InteractiveMarkerDisplay::resetCb( std::string server_id )
{
  interactive_markers_.erase( server_id );
  deleteStatusStd(server_id);
}

void InteractiveMarkerDisplay::statusCb(
    interactive_markers::InteractiveMarkerClient::StatusT status,
    const std::string& server_id,
    const std::string& msg )
{
  setStatusStd( static_cast<StatusProperty::Level>(status), server_id, msg );
}

void InteractiveMarkerDisplay::fixedFrameChanged()
{
  im_client_->setTargetFrame( fixed_frame_.toStdString() );
  reset();
}

void InteractiveMarkerDisplay::reset()
{
  Display::reset();
  unsubscribe();
  subscribe();
}

void InteractiveMarkerDisplay::updateShowDescriptions()
{
  bool show = show_descriptions_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->setShowDescription( show );
    }
  }
}

void InteractiveMarkerDisplay::updateShowAxes()
{
  bool show = show_axes_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->setShowAxes( show );
    }
  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz, InteractiveMarkers, rviz::InteractiveMarkerDisplay, rviz::Display )
