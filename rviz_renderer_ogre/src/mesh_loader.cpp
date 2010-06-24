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

#include <rviz_renderer_ogre/mesh_loader.h>
#include <rviz_renderer_ogre/mesh.h>
#include <rviz_renderer_ogre/submesh.h>
#include <rviz_renderer_ogre/material.h>
#include <rviz_renderer_ogre/simple_color_material.h>
#include <rviz_renderer_ogre/init.h>
#include <rviz_renderer_ogre/renderer.h>

#include <rviz_mesh_loader/loader.h>
#include <rviz_msgs/Mesh.h>

#include <resource_retriever/retriever.h>

#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreMeshSerializer.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreRoot.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace rviz_renderer_ogre
{

MeshPtr convertMesh(const std::string& name, const rviz_msgs::Mesh& input_mesh)
{
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, ROS_PACKAGE_NAME);

  Ogre::AxisAlignedBox aabb;
  float radius = 0.0;
  for (size_t i = 0; i < input_mesh.submeshes.size(); ++i)
  {
    const rviz_msgs::SubMesh& input_submesh = input_mesh.submeshes[i];

    bool has_normals = !input_submesh.normals.empty();

    Ogre::SubMesh* submesh = mesh->createSubMesh();
    submesh->useSharedVertices = false;
    submesh->vertexData = new Ogre::VertexData();
    Ogre::VertexData* vertex_data = submesh->vertexData;
    Ogre::VertexDeclaration* vertex_decl = vertex_data->vertexDeclaration;

    size_t offset = 0;
    // positions
    vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // normals
    if (has_normals)
    {
      ROS_ASSERT(input_submesh.normals.size() == input_submesh.positions.size());

      vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    }

    // vertex colors
    for (size_t j = 0; j < input_submesh.colors.size(); ++j)
    {
      ROS_ASSERT(input_submesh.colors[j].array.size() == input_submesh.positions.size());
      vertex_decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE, j);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
    }

    // Texture coordinates
    for (size_t j = 0; j < input_submesh.tex_coords.size(); ++j)
    {
     ROS_ASSERT(input_submesh.tex_coords[j].array.size() == input_submesh.positions.size());
     ROS_ASSERT(input_submesh.tex_coords[j].dims <= 3);

     Ogre::VertexElementType type = Ogre::VET_FLOAT1;
     if (input_submesh.tex_coords[j].dims == 2)
     {
       type = Ogre::VET_FLOAT2;
     }
     else if (input_submesh.tex_coords[j].dims == 3)
     {
       type = Ogre::VET_FLOAT3;
     }

     vertex_decl->addElement(0, offset, type, Ogre::VES_TEXTURE_COORDINATES, j);
     offset += Ogre::VertexElement::getTypeSize(type);
    }

    // allocate the vertex buffer
    vertex_data->vertexCount = input_submesh.positions.size();
    Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertex_decl->getVertexSize(0),
                                                                          vertex_data->vertexCount,
                                                                          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                                                                          false);

    vertex_data->vertexBufferBinding->setBinding(0, vbuf);
    float* vertices = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // Add the vertices
    for (size_t j = 0; j < input_submesh.positions.size(); j++)
    {
      const rviz_msgs::Vector3& p = input_submesh.positions[j];
      *vertices++ = p.x;
      *vertices++ = p.y;
      *vertices++ = p.z;

      Ogre::Vector3 ogre_pos(p.x, p.y, p.z);
      aabb.merge(ogre_pos);
      float dist = ogre_pos.length();
      if (dist > radius)
      {
        radius = dist;
      }


      if (has_normals)
      {
        const rviz_msgs::Vector3& n = input_submesh.normals[j];
        *vertices++ = n.x;
        *vertices++ = n.y;
        *vertices++ = n.z;
      }

      for (size_t j = 0; j < input_submesh.colors.size(); ++j)
      {
        const std_msgs::ColorRGBA& color = input_submesh.colors[j].array[i];
        Ogre::Root::getSingleton().convertColourValue(Ogre::ColourValue(color.r, color.g, color.b, color.a), reinterpret_cast<uint32_t*>(vertices));
        ++vertices;
      }

      for (size_t j = 0; j < input_submesh.tex_coords.size(); ++j)
      {
        const rviz_msgs::TexCoord& tex = input_submesh.tex_coords[j].array[i];
        for (uint32_t k = 0; k < input_submesh.tex_coords[j].dims; ++k)
        {
          *vertices++ = tex.uvw[k];
        }
      }
    }

    submesh->indexData->indexCount = input_submesh.indices.size();

    // allocate index buffer
    submesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
                                             Ogre::HardwareIndexBuffer::IT_16BIT,
                                             submesh->indexData->indexCount,
                                             Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                                             false);

    Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
    uint16_t* indices = static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // add the indices
    for (uint32_t j = 0; j < input_submesh.indices.size(); j++)
    {
      *indices++ = input_submesh.indices[j];
    }

    // Unlock
    vbuf->unlock();
    ibuf->unlock();
  }

  mesh->_setBounds(aabb);
  mesh->_setBoundingSphereRadius(radius);

  MeshPtr out_mesh(new Mesh(name, mesh));
  // assign materials
  for (size_t i = 0; i < input_mesh.submeshes.size(); ++i)
  {
    int8_t mat_index = input_mesh.submeshes[i].material_index;
    if (mat_index >= 0 && mat_index < (int8_t)input_mesh.materials.size())
    {
      out_mesh->getSubMesh(i)->setMaterialID(input_mesh.materials[mat_index].id);
    }
  }

  getRenderer()->addMesh(name, out_mesh);

  return out_mesh;
}

MaterialPtr createMaterial(const rviz_msgs::Material& mat)
{
  MaterialPtr out;

  ROS_ASSERT(!((rviz_uuid::UUID)mat.id).isNull());

  if (mat.has_color && mat.has_texture)
  {
    SimpleColorMaterial* scm = new SimpleColorMaterial(mat.id);
    out.reset(scm);

    //scm->setColor(Ogre::ColourValue(mat.color.r, mat.color.g, mat.color.b, mat.color.a));
    scm->setColor(Ogre::ColourValue(1, 0, 0, 1));
  }
  else if (mat.has_color)
  {
    SimpleColorMaterial* scm = new SimpleColorMaterial(mat.id);
    out.reset(scm);

    scm->setColor(Ogre::ColourValue(mat.color.r, mat.color.g, mat.color.b, mat.color.a));
  }
  else if (mat.has_texture)
  {
    SimpleColorMaterial* scm = new SimpleColorMaterial(mat.id);
    out.reset(scm);

    scm->setColor(Ogre::ColourValue(1, 0, 0, 1));
  }

  if (out)
  {
    getRenderer()->addMaterial(out->getID(), out);
  }

  return out;
}

void createMaterialsForMesh(const std::string& resource_path, const rviz_msgs::Mesh& input_mesh)
{
  for (uint32_t i = 0; i < input_mesh.materials.size(); i++)
  {
    const rviz_msgs::Material& in_mat = input_mesh.materials[i];
    createMaterial(in_mat);
  }
}

MeshPtr loadMesh(const std::string& resource_path)
{
  if (getRenderer()->meshExists(resource_path))
  {
    return getRenderer()->getMesh(resource_path);
  }
  else
  {
    fs::path model_path(resource_path);
    std::string ext = model_path.extension();
    // If it's an ogre .mesh, load it directly
    if (ext == ".mesh" || ext == ".MESH")
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(resource_path);
      }
      catch (resource_retriever::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return MeshPtr();
      }

      if (res.size == 0)
      {
        return MeshPtr();
      }

      Ogre::MeshSerializer ser;
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(resource_path, ROS_PACKAGE_NAME);
      ser.importMesh(stream, mesh.get());

      MeshPtr out_mesh(new Mesh(resource_path, mesh));
      getRenderer()->addMesh(resource_path, out_mesh);
      return out_mesh;
    }
    else
    {
      rviz_msgs::Mesh mesh_msg;
      rviz_mesh_loader::load(resource_path, mesh_msg);
      createMaterialsForMesh(resource_path, mesh_msg);
      return convertMesh(resource_path, mesh_msg);
    }
  }
}

} // namespace rviz_renderer_ogre
