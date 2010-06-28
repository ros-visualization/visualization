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

#include <rviz_renderer_ogre/ogre_material_generator.h>
#include <rviz_renderer_ogre/material.h>
#include <rviz_msgs/Material.h>

#include <resource_retriever/retriever.h>
#include <boost/filesystem.hpp>

#include <sstream>

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreGpuProgramManager.h>
#include <OGRE/OgreHighLevelGpuProgramManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>

namespace fs = boost::filesystem;

namespace rviz_renderer_ogre
{

void loadTexture(const std::string& resource_path)
{
  if (!Ogre::TextureManager::getSingleton().resourceExists(resource_path))
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
    }

    if (res.size != 0)
    {
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::Image image;
      std::string extension = fs::extension(fs::path(resource_path));

      if (extension[0] == '.')
      {
        extension = extension.substr(1, extension.size() - 1);
      }

      try
      {
        image.load(stream, extension);
        Ogre::TextureManager::getSingleton().loadImage(resource_path, ROS_PACKAGE_NAME, image);
      }
      catch (Ogre::Exception& e)
      {
        ROS_ERROR("Could not load texture [%s]: %s", resource_path.c_str(), e.what());
      }
    }
  }
}

std::string materialToStringID(const rviz_msgs::Material& input_mat)
{
  std::stringstream ss;
  ss << "Gen_";

  if (input_mat.has_color)
  {
    ss << "Color";
  }

  if (input_mat.has_texture)
  {
    ss << "Texture_" << input_mat.texture;
  }

  if (input_mat.has_normal_map)
  {
    ss << "NormalMap_" << input_mat.normal_map;
  }

  bool transparent = input_mat.opacity < 0.99;
  if (transparent)
  {
    ss << "Alpha";
  }

  return ss.str();
}

Ogre::GpuProgramPtr generateVertexShader(const rviz_msgs::Material& input_mat)
{
  std::stringstream ss;

  ss << "void vp(" << std::endl;
  ss << " float4 in_position : POSITION," << std::endl;
  ss << " float3 in_normal   : NORMAL," << std::endl;

  uint32_t num_tex_coords = input_mat.has_texture ? 1 : 0;
  for (uint32_t i = 0; i < num_tex_coords; i++)
  {
    ss << " float2 in_uv" << i << " : TEXCOORD" << i << ',' << std::endl;
  }

  if (input_mat.has_normal_map)
  {
    ss << " float3 in_tangent : TANGENT0,\n";
  }

  ss << std::endl;

  ss << " out float4 out_position : POSITION," << std::endl;
  ss << " out float3 out_view_pos : TEXCOORD0," << std::endl;
  ss << " out float3 out_normal : TEXCOORD1," << std::endl;
  uint32_t tex_coord_num = 2;
  for (uint32_t i = 0; i < num_tex_coords; i++)
  {
    ss << " out float2 out_uv" << i << " : TEXCOORD" << tex_coord_num++ << ',' << std::endl;
  }

  if (input_mat.has_normal_map)
  {
    ss << " out float3 out_tangent : TEXCOORD" << tex_coord_num++ << "," << std::endl;
    ss << " out float3 out_binormal : TEXCOORD" << tex_coord_num++ << "," << std::endl;
  }

  ss << std::endl;

  ss << " uniform float4x4 worldviewproj," << std::endl;
  ss << " uniform float4x4 worldview" << std::endl;

  ss << " )" << std::endl;

  ss << "{" << std::endl;
  ss << " out_position = mul(worldviewproj, in_position);" << std::endl;
  ss << " out_normal = mul(worldview, float4(in_normal,0)).xyz;" << std::endl;

  ss << " out_view_pos = mul(worldview, in_position).xyz;" << std::endl;

  if (input_mat.has_normal_map)
  {
    ss << " out_tangent = mul(worldview, float4(in_tangent, 0)).xyz;" << std::endl;
    ss << " out_binormal = cross(out_normal, out_tangent);" << std::endl;
  }

  for (uint32_t i = 0; i < num_tex_coords; i++)
  {
    ss << " out_uv" << i << " = in_uv" << i << ';' << std::endl;
  }

  ss << "}" << std::endl;

  std::string program_source = ss.str();
  std::string program_name = materialToStringID(input_mat) + "_VP";

  ROS_DEBUG("%s", program_source.c_str());

  // Create shader object
  Ogre::HighLevelGpuProgramPtr program =
      Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(program_name, ROS_PACKAGE_NAME, "cg",
                                                                     Ogre::GPT_VERTEX_PROGRAM);
  program->setSource(program_source);
  program->setParameter("entry_point", "vp");
  program->setParameter("profiles", "vs_1_1 arbvp1");

  const Ogre::GpuProgramParametersSharedPtr& params = program->getDefaultParameters();
  params->setNamedAutoConstant("worldviewproj", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
  params->setNamedAutoConstant("worldview", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
  program->load();

  return Ogre::GpuProgramPtr(program);
}

void generateWeightedAverageAlphaShaderCode(std::stringstream& ss, const rviz_msgs::Material& input_mat)
{
  if (input_mat.has_texture)
  {
    ss << "float4 color = tex2D(sampler_tex, in_uv0);\n";
    if (input_mat.has_color)
    {
      ss << "color *= in_color;\n";
    }
  }
  else
  {
    ss << "float4 color = in_color;\n";
  }

  ss << "float3 gooch_color = gooch98(color.rgb, normal, float3(1.0, 1.0, 1.0));\n";
  ss << "out_color0 = float4(gooch_color * color.a, color.a);\n";
  ss << "out_color1 = float4(1.0, 0.0, 0.0, 0.0);\n";
}

void generateGBufferShaderCode(std::stringstream& ss, const rviz_msgs::Material& input_mat)
{
  if (input_mat.has_texture)
  {
    ss << " out_color0.rgb = tex2D(sampler_tex, in_uv0);" << std::endl;
    if (input_mat.has_color)
    {
      ss << " out_color0.rgb *= in_color.rgb;" << std::endl;
    }
  }
  else
  {
    ss << " out_color0.rgb = in_color.rgb;" << std::endl;
  }

  ss << " out_color1.rgb = normal;" << std::endl;
  ss << "  out_color1.a = length(in_view_pos) / far_distance;" << std::endl;
}

Ogre::GpuProgramPtr generateFragmentShader(const rviz_msgs::Material& input_mat)
{
  std::stringstream ss;

  ss << "#include \"gooch_98.cg\"\n";

  if (input_mat.has_normal_map)
  {
    ss << "#include \"normal_mapping.cg\"\n";
  }

  ss << std::endl;

  ss << "void fp(" << std::endl;
  ss << " float3 in_view_pos : TEXCOORD0," << std::endl;
  ss << " float3 in_normal   : TEXCOORD1," << std::endl;

  uint32_t num_tex_coords = input_mat.has_texture ? 1 : 0;
  uint32_t tex_coord_num = 2;
  for (uint32_t i = 0; i < num_tex_coords; i++)
  {
    ss << " float2 in_uv" << i << " : TEXCOORD" << tex_coord_num++ << ',' << std::endl;
  }

  if (input_mat.has_normal_map)
  {
    ss << " float3 in_tangent : TEXCOORD" << tex_coord_num++ << "," << std::endl;
    ss << " float3 in_binormal : TEXCOORD" << tex_coord_num++ << "," << std::endl;
  }

  ss << std::endl;

  ss << " out float4 out_color0 : COLOR0," << std::endl;
  ss << " out float4 out_color1 : COLOR1," << std::endl;

  ss << std::endl;

  uint32_t sampler_num = 0;
  if (input_mat.has_texture)
  {
    ss << " uniform sampler sampler_tex : register(s" << sampler_num << ")," << std::endl;
    ++sampler_num;
  }

  if (input_mat.has_normal_map)
  {
    ss << " uniform sampler sampler_normal_map : register(s" << sampler_num << ")," << std::endl;
    ++sampler_num;
  }

  ss << " uniform float4 in_color," << std::endl;
  ss << " uniform float far_distance," << std::endl;
  ss << " uniform float4x4 worldview" << std::endl;

  ss << " )" << std::endl;

  ss << "{" << std::endl;

  if (input_mat.has_normal_map)
  {
    ss << " float3 normal = extractNormalFromMap(sampler_normal_map, in_uv0, in_normal, in_tangent, in_binormal);" << std::endl;
    //ss << " out_color0.rgb = out_color1.rgb;" << std::endl;
  }
  else
  {
    ss << " float3 normal = normalize(in_normal);" << std::endl;
  }

  bool transparent = input_mat.opacity < 0.99;
  if (transparent)
  {
    generateWeightedAverageAlphaShaderCode(ss, input_mat);
  }
  else
  {
    generateGBufferShaderCode(ss, input_mat);
  }

  ss << "}" << std::endl;

  Ogre::String program_source = ss.str();
  Ogre::String program_name = materialToStringID(input_mat) + "_FP";

  ROS_DEBUG("%s", program_source.c_str());

  // Create shader object
  Ogre::HighLevelGpuProgramPtr program = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
                                                                                                  program_name,
                                                                                                  ROS_PACKAGE_NAME,
                                                                                                  "cg",
                                                                                                  Ogre::GPT_FRAGMENT_PROGRAM);

  program->setSource(program_source);
  program->setParameter("entry_point", "fp");
  program->setParameter("profiles", "ps_2_0 arbfp1");

  const Ogre::GpuProgramParametersSharedPtr& params = program->getDefaultParameters();
  try
  {
    params->setNamedAutoConstant("in_color", Ogre::GpuProgramParameters::ACT_CUSTOM, Material::CustomParam_Color);
  }
  catch (Ogre::Exception&)
  {
  }

  //params->setNamedAutoConstant("object_id", Ogre::GpuProgramParameters::ACT_CUSTOM, Material::CustomParam_ObjectID);

  try
  {
    params->setNamedAutoConstant("far_distance", Ogre::GpuProgramParameters::ACT_FAR_CLIP_DISTANCE);
  }
  catch (Ogre::Exception&)
  {
  }

  try
  {
    params->setNamedAutoConstant("worldview", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
  }
  catch (Ogre::Exception&)
  {
  }

  program->load();
  return Ogre::GpuProgramPtr(program);
}

Ogre::MaterialPtr generateOgreMaterial(const rviz_msgs::Material& input_mat)
{
  std::string material_name = materialToStringID(input_mat);
  if (Ogre::MaterialManager::getSingleton().resourceExists(material_name))
  {
    return Ogre::MaterialManager::getSingleton().getByName(material_name);
  }

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(material_name, ROS_PACKAGE_NAME);

  bool transparent = input_mat.opacity < 0.99;
  if (transparent)
  {
    mat->getTechnique(0)->setSchemeName("WeightedAverageAlpha");
    mat->setSceneBlending(Ogre::SBT_ADD);
    mat->setDepthWriteEnabled(false);
  }
  else
  {
    mat->getTechnique(0)->setSchemeName("GBuffer");
  }
  Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  if (input_mat.has_texture)
  {
    Ogre::TextureUnitState* tu = pass->createTextureUnitState();
    loadTexture(input_mat.texture);
    tu->setTextureName(input_mat.texture);
  }

  if (input_mat.has_normal_map)
  {
    Ogre::TextureUnitState* tu = pass->createTextureUnitState();
    loadTexture(input_mat.normal_map);
    tu->setTextureName(input_mat.normal_map);
  }

  Ogre::GpuProgramPtr vertex_program = generateVertexShader(input_mat);
  Ogre::GpuProgramPtr fragment_program = generateFragmentShader(input_mat);
  pass->setVertexProgram(vertex_program->getName());
  pass->setFragmentProgram(fragment_program->getName());

  return mat;
}

} // namespace rviz_renderer_ogre
