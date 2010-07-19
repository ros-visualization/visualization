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

#include <rviz_renderer_ogre/points_material_generator.h>
#include <rviz_renderer_ogre/points_renderer_desc.h>
#include <rviz_renderer_ogre/init.h>
#include <rviz_renderer_ogre/renderer.h>

#include <rviz_msgs/Points.h>

#include <sstream>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreGpuProgramManager.h>
#include <OGRE/OgreHighLevelGpuProgramManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>

namespace rviz_renderer_ogre
{

std::string descToStringID(const PointsRendererDesc& desc, bool alpha)
{
  std::stringstream ss;
  ss << "GenPoints_" << desc.type;

  if (desc.has_orientation)
  {
    ss << "Orientation";
  }

  if (alpha)
  {
    ss << "Alpha";
  }

  return ss.str();
}

Ogre::GpuProgramPtr generateVertexShader(const PointsRendererDesc& desc, bool alpha)
{
  std::stringstream ss;
  if (desc.type == rviz_msgs::Points::TYPE_POINTS)
  {
    // nothing to do
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS
        || desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
  {
    ss << "#include <point_cloud_billboard_vp.cg>\n\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
  {
    ss << "#include <point_cloud_box_vp.cg>\n\n";
  }

  ss << "void vp(" << std::endl;

  ss << " float4 in_position : POSITION," << std::endl;
  ss << " float4 in_color   : COLOR," << std::endl;

  uint32_t tex_coord_index = 0;
  bool supports_geometry_programs = getRenderer()->useGeometryShaders();
  if (desc.type != rviz_msgs::Points::TYPE_POINTS)
  {
    if (!supports_geometry_programs)
    {
      ss << " float3 in_offset : TEXCOORD" << tex_coord_index++ << ",\n";
    }

    if (desc.has_orientation)
    {
      ss << " float4 in_orientation : TEXCOORD" << tex_coord_index++ << ",\n";
    }
  }

  ss << std::endl;

  ss << " out float4 out_position : POSITION," << std::endl;
  ss << " out float3 out_view_pos : TEXCOORD0," << std::endl;
  ss << " out float3 out_offset : TEXCOORD1," << std::endl;
  ss << " out float3 out_normal : TEXCOORD2," << std::endl;
  ss << " out float4 out_color : TEXCOORD3," << std::endl;

  ss << " uniform float4 size," << std::endl;
  ss << " uniform float4 camera_pos," << std::endl;
  ss << " uniform float4x4 worldviewproj," << std::endl;
  ss << " uniform float4x4 worldview" << std::endl;

  ss << " )" << std::endl;

  ss << "{" << std::endl;

  if (!supports_geometry_programs)
  {
    if (desc.type == rviz_msgs::Points::TYPE_POINTS)
    {
      ss << " float4 pos = in_position;\n";
      ss << " float3 normal = normalize(camera_pos.xyz - pos.xyz);\n";
    }
    else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS
          || desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
    {
      ss << " PosAndNormal posn = calculateBillboardVertexPositionAndNormal(in_position, in_offset.xy, camera_pos, size);\n";
      ss << " float4 pos = posn.pos;\n";
      ss << " float3 normal = posn.normal;\n";
    }
    else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
    {
      ss << " PosAndNormal posn = calculateBoxVertexPositionAndNormal(in_position, in_offset.xyz, worldviewproj, size);\n";
      ss << " float4 pos = posn.pos;\n";
      ss << " float3 normal = posn.normal;\n";
    }
  }
  else
  {
    ss << " float4 pos = in_position;\n";
    ss << " float3 normal = float3(1.0, 0.0, 0.0);\n";
  }

  ss << " out_position = mul(worldviewproj, pos);\n" << std::endl;
  ss << " out_normal = mul(worldview, float4(normal, 0)).xyz;\n" << std::endl;
  ss << " out_view_pos = mul(worldview, pos).xyz;\n" << std::endl;
  ss << " out_color = in_color;\n";

  ss << "}" << std::endl;

  std::string program_source = ss.str();
  std::string program_name = descToStringID(desc, alpha) + "_VP";

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
  params->setNamedAutoConstant("camera_pos", Ogre::GpuProgramParameters::ACT_CAMERA_POSITION_OBJECT_SPACE);
  try
  {
    params->setNamedAutoConstant("size", Ogre::GpuProgramParameters::ACT_CUSTOM, PointsRendererDesc::CustomParam_Size);
  }
  catch (Ogre::Exception&)
  {}

  program->load();

  return Ogre::GpuProgramPtr(program);
}

void generateWeightedAverageAlphaShaderCode(std::stringstream& ss, const PointsRendererDesc& desc)
{
  ss << "float4 color = in_color;\n";
  ss << "float3 gooch_color = gooch98(color.rgb, normal, float3(0.0, 0.0, 1.0));\n";
  ss << "out_color0 = float4(gooch_color * color.a, color.a);\n";
  ss << "out_color1 = float4(1.0, 0.0, 0.0, 0.0);\n";
}

void generateGBufferShaderCode(std::stringstream& ss, const PointsRendererDesc& desc)
{
  if (desc.type == rviz_msgs::Points::TYPE_POINTS)
  {
    ss << "float4 color = in_color;\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS)
  {
    ss << "float4 color = calculateBillboardColorWithEdge(in_color, in_offset);\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
  {
    ss << "float4 color = calculateBillboardSphereColor(in_color, in_offset);\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
  {
    ss << "float4 color = calculateBoxColorWithEdge(in_color, in_offset);\n";
  }

  ss << " out_color0.rgb = color.rgb;" << std::endl;
  ss << " out_color1.rgb = normal;" << std::endl;
  ss << " out_color1.a = length(in_view_pos) / far_distance;" << std::endl;
}

Ogre::GpuProgramPtr generateFragmentShader(const PointsRendererDesc& desc, bool alpha)
{
  std::stringstream ss;

  if (alpha)
  {
    ss << "#include <gooch_98.cg>\n";
  }

  if (desc.type == rviz_msgs::Points::TYPE_POINTS)
  {
    // nothing to do
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS
        || desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
  {
    ss << "#include <point_cloud_billboard_fp.cg>\n\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
  {
    ss << "#include <point_cloud_box_fp.cg>\n\n";
  }

  ss << std::endl;

  ss << "void fp(" << std::endl;
  ss << " float3 in_view_pos : TEXCOORD0," << std::endl;
  ss << " float3 in_offset : TEXCOORD1," << std::endl;
  ss << " float3 in_normal : TEXCOORD2," << std::endl;
  ss << " float4 in_color : TEXCOORD3," << std::endl;

  ss << std::endl;

  ss << " out float4 out_color0 : COLOR0," << std::endl;
  ss << " out float4 out_color1 : COLOR1," << std::endl;

  ss << std::endl;

  ss << " uniform float far_distance," << std::endl;
  ss << " uniform float4x4 worldview" << std::endl;

  ss << " )" << std::endl;

  ss << "{" << std::endl;

  ss << " float3 normal = in_normal;" << std::endl;

  if (alpha)
  {
    generateWeightedAverageAlphaShaderCode(ss, desc);
  }
  else
  {
    generateGBufferShaderCode(ss, desc);
  }

  ss << "}" << std::endl;

  Ogre::String program_source = ss.str();
  Ogre::String program_name = descToStringID(desc, alpha) + "_FP";

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
    //params->setNamedAutoConstant("object_id", Ogre::GpuProgramParameters::ACT_CUSTOM, Material::CustomParam_ObjectID);
  }
  catch (Ogre::Exception&)
  {
  }

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

std::pair<Ogre::MaterialPtr, Ogre::MaterialPtr> generateMaterialsForPoints(const PointsRendererDesc& desc)
{
  std::string material_name_opaque = descToStringID(desc, false);
  std::string material_name_alpha = descToStringID(desc, true);
  if (Ogre::MaterialManager::getSingleton().resourceExists(material_name_opaque))
  {
    return std::make_pair(Ogre::MaterialManager::getSingleton().getByName(material_name_opaque),
                          Ogre::MaterialManager::getSingleton().getByName(material_name_alpha));
  }

  Ogre::MaterialPtr mat_opaque = Ogre::MaterialManager::getSingleton().create(material_name_opaque, ROS_PACKAGE_NAME);
  {
    mat_opaque->getTechnique(0)->setSchemeName("GBuffer");
    mat_opaque->setPointSize(5);

    Ogre::Pass* pass = mat_opaque->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    Ogre::GpuProgramPtr vertex_program = generateVertexShader(desc, false);
    Ogre::GpuProgramPtr fragment_program = generateFragmentShader(desc, false);
    pass->setVertexProgram(vertex_program->getName());
    pass->setFragmentProgram(fragment_program->getName());
  }

  Ogre::MaterialPtr mat_alpha = Ogre::MaterialManager::getSingleton().create(material_name_alpha, ROS_PACKAGE_NAME);
  {
    mat_alpha->setPointSize(5);

    mat_alpha->getTechnique(0)->setSchemeName("WeightedAverageAlpha");
    mat_alpha->setSceneBlending(Ogre::SBT_ADD);
    mat_alpha->setDepthWriteEnabled(false);

    Ogre::Pass* pass = mat_alpha->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    Ogre::GpuProgramPtr vertex_program = generateVertexShader(desc, true);
    Ogre::GpuProgramPtr fragment_program = generateFragmentShader(desc, true);
    pass->setVertexProgram(vertex_program->getName());
    pass->setFragmentProgram(fragment_program->getName());
  }

  return std::make_pair(mat_opaque, mat_alpha);
}

} // namespace rviz_renderer_ogre

