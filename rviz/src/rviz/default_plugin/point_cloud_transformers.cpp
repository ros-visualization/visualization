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

#include "point_cloud_transformers.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/validate_floats.h"
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix4.h>

namespace rviz
{

static void getRainbowColor(float value, Ogre::ColourValue& color)
{
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if ( !(i&1) ) f = 1 - f; // if i is even
  float n = 1 - f;

  if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
}

uint8_t IntensityPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  updateChannels(cloud);
  return Support_Color;
}

uint8_t IntensityPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return 255;
}

bool IntensityPCTransformer::transform( const sensor_msgs::PointCloud2ConstPtr& cloud,
                                        uint32_t mask,
                                        const Ogre::Matrix4& transform,
                                        V_PointCloudPoint& points_out )
{
  if( !( mask & Support_Color ))
  {
    return false;
  }

  int32_t index = findChannelIndex( cloud, channel_name_property_->getStdString() );

  if( index == -1 )
  {
    if( channel_name_property_->getStdString() == "intensity" )
    {
      index = findChannelIndex( cloud, "intensities" );
      if( index == -1 )
      {
	return false;
      }
    }
    else
    {
      return false;
    }
  }

  const uint32_t offset = cloud->fields[index].offset;
  const uint8_t type = cloud->fields[index].datatype;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;

  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;
  if( auto_compute_intensity_bounds_property_->getBool() )
  {
    for( uint32_t i = 0; i < num_points; ++i )
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      min_intensity = std::min(val, min_intensity);
      max_intensity = std::max(val, max_intensity);
    }

    min_intensity = std::max(-999999.0f, min_intensity);
    max_intensity = std::min(999999.0f, max_intensity);
    min_intensity_property_->setFloat( min_intensity );
    max_intensity_property_->setFloat( max_intensity );
  }
  else
  {
    min_intensity = min_intensity_property_->getFloat();
    max_intensity = max_intensity_property_->getFloat();
  }
  float diff_intensity = max_intensity - min_intensity;
  if( diff_intensity == 0 )
  {
    // If min and max are equal, set the diff to something huge so
    // when we divide by it, we effectively get zero.  That way the
    // point cloud coloring will be predictably uniform when min and
    // max are equal.
    diff_intensity = 1e20;
  }
  Color max_color = max_color_property_->getOgreColor();
  Color min_color = min_color_property_->getOgreColor();

  if( use_rainbow_property_->getBool() )
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      float value = 1.0 - (val - min_intensity)/diff_intensity;
      getRainbowColor(value, points_out[i].color);
    }
  }
  else
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      float normalized_intensity = ( val - min_intensity ) / diff_intensity;
      normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
      points_out[i].color.r = max_color.r * normalized_intensity + min_color.r * (1.0f - normalized_intensity);
      points_out[i].color.g = max_color.g * normalized_intensity + min_color.g * (1.0f - normalized_intensity);
      points_out[i].color.b = max_color.b * normalized_intensity + min_color.b * (1.0f - normalized_intensity);
    }
  }

  return true;
}

void IntensityPCTransformer::createProperties( Property* parent_property, uint32_t mask, QList<Property*>& out_props )
{
  if( mask & Support_Color )
  {
    channel_name_property_ = new EditableEnumProperty( "Channel Name", "intensity",
                                                       "Select the channel to use to compute the intensity",
                                                       parent_property, SIGNAL( needRetransform() ), this );

    use_rainbow_property_ = new BoolProperty( "Use rainbow", true,
                                              "Whether to use a rainbow of colors or interpolate between two",
                                              parent_property, SLOT( updateUseRainbow() ), this );

    min_color_property_ = new ColorProperty( "Min Color", Qt::black,
                                             "Color to assign the points with the minimum intensity.  "
                                             "Actual color is interpolated between this and Max Color.",
                                             parent_property, SIGNAL( needRetransform() ), this );

    max_color_property_ = new ColorProperty( "Max Color", Qt::white,
                                             "Color to assign the points with the maximum intensity.  "
                                             "Actual color is interpolated between this and Min Color.",
                                             parent_property, SIGNAL( needRetransform() ), this );

    auto_compute_intensity_bounds_property_ = new BoolProperty( "Autocompute Intensity Bounds", true,
                                                                "Whether to automatically compute the intensity min/max values.",
                                                                parent_property, SLOT( updateAutoComputeIntensityBounds() ), this );

    min_intensity_property_ = new FloatProperty( "Min Intensity", 0,
                                                 "Minimum possible intensity value, used to interpolate from Min Color to Max Color for a point.",
                                                 parent_property, SIGNAL( needRetransform() ), this );

    max_intensity_property_ = new FloatProperty( "Max Intensity", 4096,
                                                 "Maximum possible intensity value, used to interpolate from Min Color to Max Color for a point.",
                                                 parent_property, SIGNAL( needRetransform() ), this );

    out_props.push_back( channel_name_property_ );
    out_props.push_back( use_rainbow_property_ );
    out_props.push_back( min_color_property_ );
    out_props.push_back( max_color_property_ );
    out_props.push_back( auto_compute_intensity_bounds_property_ );
    out_props.push_back( min_intensity_property_ );
    out_props.push_back( max_intensity_property_ );

    updateUseRainbow();
    updateAutoComputeIntensityBounds();
  }
}

void IntensityPCTransformer::updateChannels( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
  V_string channels;
  for(size_t i = 0; i < cloud->fields.size(); ++i )
  {
    channels.push_back(cloud->fields[i].name );
  }
  std::sort(channels.begin(), channels.end());

  if( channels != available_channels_ )
  {
    channel_name_property_->clear();
    for( V_string::const_iterator it = channels.begin(); it != channels.end(); ++it )
    {
      const std::string& channel = *it;
      if( channel.empty() )
      {
	continue;
      }
      channel_name_property_->addOptionStd( channel );
    }
    available_channels_ = channels;
  }
}

void IntensityPCTransformer::updateAutoComputeIntensityBounds()
{
  bool auto_compute = auto_compute_intensity_bounds_property_->getBool();
  min_intensity_property_->setHidden( auto_compute );
  max_intensity_property_->setHidden( auto_compute );
}

void IntensityPCTransformer::updateUseRainbow()
{
  bool use_rainbow = use_rainbow_property_->getBool();
  min_color_property_->setHidden( use_rainbow );
  max_color_property_->setHidden( use_rainbow );
}

uint8_t XYZPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return Support_None;
  }

  if (cloud->fields[xi].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_XYZ;
  }

  return Support_None;
}

bool XYZPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_XYZ))
  {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    float x = *reinterpret_cast<const float*>(point + xoff);
    float y = *reinterpret_cast<const float*>(point + yoff);
    float z = *reinterpret_cast<const float*>(point + zoff);

    Ogre::Vector3 pos(x, y, z);
    pos = transform * pos;
    points_out[i].position = pos;
  }

  return true;
}

uint8_t RGB8PCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t index = findChannelIndex(cloud, "rgb");
  if (index == -1)
  {
    return Support_None;
  }

  if (cloud->fields[index].datatype == sensor_msgs::PointField::INT32 ||
      cloud->fields[index].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGB8PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t index = findChannelIndex(cloud, "rgb");

  const uint32_t off = cloud->fields[index].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    uint32_t rgb = *reinterpret_cast<const uint32_t*>(point + off);
    float r = ((rgb >> 16) & 0xff) / 255.0f;
    float g = ((rgb >> 8) & 0xff) / 255.0f;
    float b = (rgb & 0xff) / 255.0f;
    points_out[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

uint8_t RGBF32PCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");
  if (ri == -1 || gi == -1 || bi == -1)
  {
    return Support_None;
  }

  if (cloud->fields[ri].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGBF32PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");

  const uint32_t roff = cloud->fields[ri].offset;
  const uint32_t goff = cloud->fields[gi].offset;
  const uint32_t boff = cloud->fields[bi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    float r = *reinterpret_cast<const float*>(point + roff);
    float g = *reinterpret_cast<const float*>(point + goff);
    float b = *reinterpret_cast<const float*>(point + boff);
    points_out[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

uint8_t FlatColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return Support_Color;
}

uint8_t FlatColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return 0;
}

bool FlatColorPCTransformer::transform( const sensor_msgs::PointCloud2ConstPtr& cloud,
                                        uint32_t mask,
                                        const Ogre::Matrix4& transform,
                                        V_PointCloudPoint& points_out )
{
  if( !( mask & Support_Color ))
  {
    return false;
  }

  Ogre::ColourValue color = color_property_->getOgreColor();

  const uint32_t num_points = cloud->width * cloud->height;
  for( uint32_t i = 0; i < num_points; ++i )
  {
    points_out[i].color = color;
  }

  return true;
}

void FlatColorPCTransformer::createProperties( Property* parent_property, uint32_t mask, QList<Property*>& out_props )
{
  if( mask & Support_Color )
  {
    color_property_ = new ColorProperty( "Color", Qt::white,
                                         "Color to assign to every point."
                                         parent_property, SIGNAL( needRetransform() ), this );
    out_props.push_back( color_property_ );
  }
}

uint8_t AxisColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return Support_Color;
}

uint8_t AxisColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return 255;
}

bool AxisColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();

  // compute bounds

  float min_value_current = 9999.0f;
  float max_value_current = -9999.0f;
  std::vector<float> values;
  values.reserve(num_points);

  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    float x = *reinterpret_cast<const float*>(point + xoff);
    float y = *reinterpret_cast<const float*>(point + yoff);
    float z = *reinterpret_cast<const float*>(point + zoff);

    Ogre::Vector3 pos(x, y, z);

    if (use_fixed_frame_)
    {
      pos = transform * pos;
    }

    float val = pos[axis_];
    min_value_current = std::min(min_value_current, val);
    max_value_current = std::max(max_value_current, val);

    values.push_back(val);
  }

  if (auto_compute_bounds_)
  {
    min_value_ = min_value_current;
    max_value_ = max_value_current;
  }

  for (uint32_t i = 0; i < num_points; ++i)
  {
    float range = std::max(max_value_ - min_value_, 0.001f);
    float value = 1.0 - (values[i] - min_value_)/range;
    getRainbowColor(value, points_out[i].color);
  }

  return true;
}

void AxisColorPCTransformer::createProperties( Property* parent_property, uint32_t mask, QList<Property*>& out_props )
{
  if (mask & Support_Color)
  {
    axis_property_ = new EnumProperty("Axis", prefix, boost::bind(&AxisColorPCTransformer::getAxis, this), boost::bind(&AxisColorPCTransformer::setAxis, this, _1),
                                                                parent, this);
    EnumPropertyPtr prop = axis_property_.lock();
    prop->addOption("X", AXIS_X);
    prop->addOption("Y", AXIS_Y);
    prop->addOption("Z", AXIS_Z);
    setPropertyHelpText(axis_property_, "The axis to interpolate the color along.");
    auto_compute_bounds_property_ = new BoolProperty( "Autocompute Value Bounds", prefix, boost::bind( &AxisColorPCTransformer::getAutoComputeBounds, this ),
                                                                              boost::bind( &AxisColorPCTransformer::setAutoComputeBounds, this, _1 ), parent, this );

    setPropertyHelpText(auto_compute_bounds_property_, "Whether to automatically compute the value min/max values.");
    min_value_property_ = new FloatProperty( "Min Value", prefix, boost::bind( &AxisColorPCTransformer::getMinValue, this ),
                                                                              boost::bind( &AxisColorPCTransformer::setMinValue, this, _1 ), parent, this );
    setPropertyHelpText(min_value_property_, "Minimum value value, used to interpolate the color of a point.");
    max_value_property_ = new FloatProperty( "Max Value", prefix, boost::bind( &AxisColorPCTransformer::getMaxValue, this ),
                                                                            boost::bind( &AxisColorPCTransformer::setMaxValue, this, _1 ), parent, this );
    setPropertyHelpText(max_value_property_, "Maximum value value, used to interpolate the color of a point.");

    use_fixed_frame_property_ = new BoolProperty( "Use Fixed Frame", prefix, boost::bind( &AxisColorPCTransformer::getUseFixedFrame, this ),
                                                                            boost::bind( &AxisColorPCTransformer::setUseFixedFrame, this, _1 ), parent, this );
    setPropertyHelpText(use_fixed_frame_property_, "Whether to color the cloud based on its fixed frame position or its local frame position.");

    out_props.push_back(axis_property_);
    out_props.push_back(auto_compute_bounds_property_);
    out_props.push_back(min_value_property_);
    out_props.push_back(max_value_property_);
    out_props.push_back(use_fixed_frame_property_);

    if (auto_compute_bounds_)
    {
      hideProperty(min_value_property_);
      hideProperty(max_value_property_);
    }
    else
    {
      showProperty(min_value_property_);
      showProperty(max_value_property_);
    }
  }
}

void AxisColorPCTransformer::setUseFixedFrame(bool use)
{
  use_fixed_frame_ = use;
  propertyChanged(use_fixed_frame_property_);
  causeRetransform();
}

void AxisColorPCTransformer::setAxis(int axis)
{
  axis_ = axis;
  propertyChanged(axis_property_);
  causeRetransform();
}

void AxisColorPCTransformer::setMinValue( float val )
{
  min_value_ = val;
  if (min_value_ > max_value_)
  {
    min_value_ = max_value_;
  }

  propertyChanged(min_value_property_);

  causeRetransform();
}

void AxisColorPCTransformer::setMaxValue( float val )
{
  max_value_ = val;
  if (max_value_ < min_value_)
  {
    max_value_ = min_value_;
  }

  propertyChanged(max_value_property_);

  causeRetransform();
}

void AxisColorPCTransformer::setAutoComputeBounds(bool compute)
{
  auto_compute_bounds_ = compute;

  if (auto_compute_bounds_)
  {
    hideProperty(min_value_property_);
    hideProperty(max_value_property_);
  }
  else
  {
    showProperty(min_value_property_);
    showProperty(max_value_property_);
  }

  propertyChanged(auto_compute_bounds_property_);

  causeRetransform();
}

}
