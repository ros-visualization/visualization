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

#ifndef RVIZ_RENDER_CONVERT_H
#define RVIZ_RENDER_CONVERT_H

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreMatrix3.h>

#include <boost/function.hpp>

#include <rviz/math/vector3.h>
#include <rviz/math/quaternion.h>

/**
 * \file
 */

namespace rviz
{
namespace render
{
namespace ogre
{

extern Ogre::Matrix3 g_ogre_to_robot_matrix;
extern Ogre::Matrix3 g_robot_to_ogre_matrix;

extern Ogre::Quaternion g_ogre_to_robot_quat;
extern Ogre::Quaternion g_robot_to_ogre_quat;

inline Ogre::Vector3 convert(const Vector3& v)
{
  return Ogre::Vector3(v.x, v.y, v.z);
}

inline Ogre::Quaternion convert(const Quaternion& q)
{
  return Ogre::Quaternion(q.w, q.x, q.y, q.z);
}

/**
 * \brief Convert a point from robot space to ogre space
 * @param point Converts this point in-place
 */
inline Ogre::Vector3 fromRobot( const Vector3& point )
{
  return g_robot_to_ogre_matrix * convert(point);
}

/**
 * \brief Convert a scale xyz from robot space to ogre space
 * @param scale Converts this scale xyz in-place
 */
inline Ogre::Vector3 scaleFromRobot( const Vector3& scale )
{
  Ogre::Vector3 s = g_robot_to_ogre_matrix * convert(scale);

  s.x = fabsf( s.x );
  s.y = fabsf( s.y );
  s.z = fabsf( s.z );

  return s;
}

/**
 * \brief Convert a quaternion from robot space to ogre space
 * @param quat Converts this quaternion in-place
 */
inline Ogre::Quaternion fromRobot( const Quaternion& quat )
{
  return g_robot_to_ogre_quat * convert(quat);
}

} // namespace ogre
} // namespace render
} // namespace rviz
#endif
