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

#include <rviz_mesh_loader/stl_parser.h>
#include <ros/console.h>

#include <rviz_msgs/Mesh.h>

#include <Eigen/Geometry>

namespace rviz_mesh_loader
{

void calculateUV(const Eigen::Vector3f& vec, float& u, float& v)
{
  Eigen::Vector3f pos(vec);
  pos.normalize();
  u = acos( pos.y() / pos.norm() );

  float val = pos.x() / ( sin( u ) );
  v = acos( val );

  u /= M_PI;
  v /= M_PI;
}

void parseSTL(uint8_t* buffer, size_t buffer_size, const std::string& extension, rviz_msgs::Mesh& out_mesh)
{
  out_mesh.submeshes.resize(1);
  rviz_msgs::SubMesh& submesh = out_mesh.submeshes[0];
  uint8_t* pos = buffer;
  pos += 80; // skip the 80 byte header

  uint32_t tri_count = *(uint32_t*)pos;
  pos += 4;

  submesh.positions.resize(tri_count * 3);
  submesh.normals.resize(tri_count * 3);
  submesh.indices.resize(tri_count * 3);
  submesh.tex_coords.resize(1);
  submesh.tex_coords[0].array.resize(tri_count * 3);
  submesh.tex_coords[0].dims = 2;
  for ( uint32_t tri = 0; tri < tri_count; ++tri )
  {
    // Read face normal
    Eigen::Vector3f normal;

    normal.x() = *(float*)pos;
    pos += 4;
    normal.y() = *(float*)pos;
    pos += 4;
    normal.z() = *(float*)pos;
    pos += 4;

    // Read 3 vertices
    for (uint32_t i = 0; i < 3; ++i)
    {
      uint32_t vert_index = tri * 3 + i;
      rviz_msgs::Vector3& p = submesh.positions[vert_index];
      p.x = *(float*)pos;
      pos += 4;
      p.y = *(float*)pos;
      pos += 4;
      p.z = *(float*)pos;
      pos += 4;

      rviz_msgs::TexCoord& uvw = submesh.tex_coords[0].array[vert_index];
      calculateUV( Eigen::Vector3f(p.x, p.y, p.z), uvw.uvw[0], uvw.uvw[1] );
    }

    // Blender was writing a large number into this short... am I misinterpreting what the attribute byte count is supposed to do?
    //unsigned short attributeByteCount = *(unsigned short*)pos;
    pos += 2;

    //pos += attributeByteCount;

    if (normal.squaredNorm() < 0.001)
    {
      rviz_msgs::Vector3& p0 = submesh.positions[0];
      rviz_msgs::Vector3& p1 = submesh.positions[1];
      rviz_msgs::Vector3& p2 = submesh.positions[2];
      Eigen::Vector3f side1 = Eigen::Vector3f(p0.x, p0.y, p0.z) - Eigen::Vector3f(p1.x, p1.y, p1.z);
      Eigen::Vector3f side2 = Eigen::Vector3f(p1.x, p1.y, p1.z) - Eigen::Vector3f(p2.x, p2.y, p2.z);
      normal = side1.cross(side2);
      normal.normalize();
    }

    // Assign normal
    for (uint32_t i = 0; i < 3; ++i)
    {
      uint32_t vert_index = tri * 3 + i;
      rviz_msgs::Vector3& n = submesh.normals[vert_index];
      n.x = normal.x();
      n.y = normal.y();
      n.z = normal.z();
    }
  }
}

}
