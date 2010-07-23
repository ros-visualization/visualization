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
  ss << "GenPoints_" << (uint32_t)desc.type;

  if (desc.has_orientations)
  {
    ss << "Orientation";
  }

  if (desc.has_scales)
  {
    ss << "Scales";
  }
  else
  {
    ss << "Scale " << std::setprecision(3) << desc.scale.x << desc.scale.y << desc.scale.z;
  }

  if (alpha)
  {
    ss << "Alpha";
  }

  return ss.str();
}

void generateGenericGPVP(std::stringstream& ss, const PointsRendererDesc& desc)
{
  ss <<
"void vp(float4 position : POSITION,\n"
"        float4 color : COLOR,\n"
"        float4 t0 : TEXCOORD0,\n"
"        float4 t1 : TEXCOORD1,\n"
"        out float4 out_position : POSITION,\n"
"        out float4 out_color : COLOR,\n"
"        out float4 out_t0 : TEXCOORD0,\n"
"        out float4 out_t1 : TEXCOORD1\n"
")\n"
"{\n"
" out_position = position;\n"
" out_color = color;\n"
" out_t0 = t0;\n"
" out_t1 = t1;\n"
"}\n";
}

Ogre::GpuProgramPtr generateVertexShader(const PointsRendererDesc& desc, bool alpha)
{
  std::stringstream ss;
  bool supports_geometry_programs = getRenderer()->useGeometryShaders();

  if (supports_geometry_programs && desc.type != rviz_msgs::Points::TYPE_POINTS)
  {
    generateGenericGPVP(ss, desc);
  }
  else
  {
    ss << "#include <quaternion.cg>\n";

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
    ss << " float4 in_color    : COLOR," << std::endl;
    ss << " float3 in_normal   : NORMAL,\n";

    uint32_t tex_coord_index = 0;
    if (desc.type != rviz_msgs::Points::TYPE_POINTS)
    {
      if (!supports_geometry_programs)
      {
        ss << " float3 in_offset : TEXCOORD" << tex_coord_index++ << ",\n";
      }

      if (desc.has_normals)
      {
        ss << " float3 in_face_normal : TEXCOORD" << tex_coord_index++ << ",\n";
      }

      if (desc.has_orientations)
      {
        ss << " float4 in_orientation : TEXCOORD" << tex_coord_index++ << ",\n";
      }
    }
    else
    {
      if (desc.has_normals)
      {
        ss << " float3 in_face_normal : TEXCOORD" << tex_coord_index++ << ",\n";
      }
    }

    if (desc.has_scales)
    {
      ss << " float4 in_scale : TEXCOORD" << tex_coord_index++ << ",\n";
    }

    ss << std::endl;

    ss << " out float4 out_position : POSITION," << std::endl;
    ss << " out float3 out_view_pos : TEXCOORD0," << std::endl;
    ss << " out float3 out_offset : TEXCOORD1," << std::endl;
    ss << " out float3 out_normal : TEXCOORD2," << std::endl;
    ss << " out float4 out_color : TEXCOORD3," << std::endl;

    if (!desc.has_scales)
    {
      ss << " uniform float4 size," << std::endl;
    }

    ss << " uniform float4 camera_pos," << std::endl;
    ss << " uniform float4x4 worldviewproj," << std::endl;
    ss << " uniform float4x4 worldview" << std::endl;

    ss << " )" << std::endl;

    ss << "{" << std::endl;

    if (desc.has_scales)
    {
      ss << " float4 size = in_scale;\n";
    }

    if (desc.type == rviz_msgs::Points::TYPE_POINTS)
    {
      ss << " float4 pos = in_position;\n";
      if (desc.has_orientations)
      {
        ss << " float3 normal = quaternionRotate(in_orientation, float3(0.0, 0.0, -1.0));\n";
      }
      else if (desc.has_normals)
      {
        ss << " float3 normal = in_face_normal;\n";
      }
      else
      {
        ss << " float3 normal = normalize(camera_pos.xyz - pos.xyz);\n";
      }
    }
    else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS)
    {
      if (desc.has_orientations)
      {
        ss << " PosAndNormal posn = calculateBillboardVertexPositionAndNormalQuat(in_position, in_offset.xy, in_orientation, camera_pos, size);\n";
      }
      else if (desc.has_normals)
      {
        ss << " PosAndNormal posn = calculateBillboardVertexPositionAndNormal(in_position, in_offset.xy, in_face_normal, camera_pos, size);\n";
      }
      else
      {
        ss << " PosAndNormal posn = calculateBillboardVertexPositionAndNormal(in_position, in_offset.xy, camera_pos, size);\n";
      }
      ss << " float4 pos = posn.pos;\n";
      ss << " float3 normal = posn.normal;\n";
    }
    else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
    {
      if (desc.has_orientations)
      {
        ss << " PosAndNormal posn = calculateBillboardSpheresVertexPositionAndNormalQuat(in_position, in_offset.xy, in_orientation, camera_pos, size);\n";
      }
      else if (desc.has_normals)
      {
        ss << " PosAndNormal posn = calculateBillboardSpheresVertexPositionAndNormal(in_position, in_offset.xy, in_face_normal, camera_pos, size);\n";
      }
      else
      {
        ss << " PosAndNormal posn = calculateBillboardSpheresVertexPositionAndNormal(in_position, in_offset.xy, camera_pos, size);\n";
      }
      ss << " float4 pos = posn.pos;\n";
      ss << " float3 normal = posn.normal;\n";
    }
    else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
    {
      if (desc.has_orientations)
      {
        ss << " PosAndNormal posn = calculateBoxVertexPositionAndNormal(in_position, in_offset.xyz, in_normal, in_orientation, worldviewproj, size);\n";
      }
      else
      {
        ss << " PosAndNormal posn = calculateBoxVertexPositionAndNormal(in_position, in_offset.xyz, in_normal, worldviewproj, size);\n";
      }
      ss << " float4 pos = posn.pos;\n";
      ss << " float3 normal = posn.normal;\n";
    }

    ss << " out_position = mul(worldviewproj, pos);\n" << std::endl;
    ss << " out_normal = mul(worldview, float4(normal, 0)).xyz;\n" << std::endl;
    ss << " out_view_pos = mul(worldview, pos).xyz;\n" << std::endl;
    ss << " out_color = in_color;\n";
    if (!supports_geometry_programs && desc.type != rviz_msgs::Points::TYPE_POINTS)
    {
      ss << " out_offset = in_offset;\n";
    }

    ss << "}" << std::endl;
  }

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
  try
  {
    params->setNamedAutoConstant("worldviewproj", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
  }
  catch (Ogre::Exception&)
  {}

  try
  {
    params->setNamedAutoConstant("worldview", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
  }
  catch (Ogre::Exception&)
  {}

  try
  {
    params->setNamedAutoConstant("camera_pos", Ogre::GpuProgramParameters::ACT_CAMERA_POSITION_OBJECT_SPACE);
  }
  catch (Ogre::Exception&)
  {}

  try
  {
    params->setNamedAutoConstant("size", Ogre::GpuProgramParameters::ACT_CUSTOM, PointsRendererDesc::CustomParam_Size);
  }
  catch (Ogre::Exception&)
  {}

  program->load();

  return Ogre::GpuProgramPtr(program);
}

Ogre::GpuProgramPtr generateGeometryShader(const PointsRendererDesc& desc, bool alpha)
{
  std::stringstream ss;

  ss << " #include <quaternion.cg>\n";

  if (desc.type == rviz_msgs::Points::TYPE_POINTS)
  {
    ROS_ASSERT_MSG(false, "Shouldn't be generating geometry shaders for TYPE_POINTS");
    // nothing to do
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS
        || desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
  {
    ss << "#include <point_cloud_billboard_gp.cg>\n\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
  {
    ss << "#include <point_cloud_box_gp.cg>\n\n";
  }

  ss << std::endl;

  ss << "POINT TRIANGLE_OUT void gp(" << std::endl;
  ss << " AttribArray<float4> in_position : POSITION,\n";
  ss << " AttribArray<float4> in_color : COLOR,\n";

  uint32_t tex_coord_num = 0;

  if (desc.has_normals)
  {
    ss << " AttribArray<float3> in_normal : TEXCOORD" << tex_coord_num++ << ",\n";
  }
  else if (desc.has_orientations)
  {
    ss << " AttribArray<float4> in_orientation : TEXCOORD" << tex_coord_num++ << ",\n";
  }

  if (desc.has_scales)
  {
    ss << "AttribArray<float4> in_scale : TEXCOORD" << tex_coord_num++ << ",\n";
  }

  ss << std::endl;
  ss << std::endl;

  if (!desc.has_scales)
  {
    ss << " uniform float4 size," << std::endl;
  }

  ss << " uniform float4 camera_pos," << std::endl;
  ss << " uniform float4x4 worldviewproj," << std::endl;
  ss << " uniform float4x4 worldview" << std::endl;

  ss << " )" << std::endl;

  ss << "{" << std::endl;

  if (desc.has_scales)
  {
    ss << " float4 size = in_scale[0];\n";
  }

  if (desc.type == rviz_msgs::Points::TYPE_BILLBOARDS)
  {
    if (desc.has_orientations)
    {
      ss << " emitBillboardVerticesQuat(in_position[0], in_color[0], in_orientation[0], worldviewproj, worldview, camera_pos, size);\n";
    }
    else if (desc.has_normals)
    {
      ss << " emitBillboardVertices(in_position[0], in_color[0], in_normal[0], worldviewproj, worldview, camera_pos, size);\n";
    }
    else
    {
      ss << " emitBillboardVertices(in_position[0], in_color[0], worldviewproj, worldview, camera_pos, size);\n";
    }
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BILLBOARD_SPHERES)
  {
    if (desc.has_orientations)
    {
      ss << " emitBillboardSphereVerticesQuat(in_position[0], in_color[0], in_orientation[0], worldviewproj, worldview, camera_pos, size);\n";
    }
    else if (desc.has_normals)
    {
      ss << " emitBillboardSphereVertices(in_position[0], in_color[0], in_normal[0], worldviewproj, worldview, camera_pos, size);\n";
    }
    else
    {
      ss << " emitBillboardSphereVertices(in_position[0], in_color[0], worldviewproj, worldview, camera_pos, size);\n";
    }
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
  {
    if (desc.has_orientations)
    {
      ss << " emitBoxVertices(in_position[0], in_color[0], in_orientation[0], worldviewproj, worldview, size);\n";
    }
    else
    {
      ss << " emitBoxVertices(in_position[0], in_color[0], worldviewproj, worldview, size);\n";
    }
  }

  ss << "}" << std::endl;

  Ogre::String program_source = ss.str();
  Ogre::String program_name = descToStringID(desc, alpha) + "_GP";

  ROS_DEBUG("%s", program_source.c_str());

  // Create shader object
  Ogre::HighLevelGpuProgramPtr program = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
                                                                                                  program_name,
                                                                                                  ROS_PACKAGE_NAME,
                                                                                                  "cg",
                                                                                                  Ogre::GPT_GEOMETRY_PROGRAM);

  program->setSource(program_source);
  program->setParameter("entry_point", "gp");
  program->setParameter("profiles", "gpu_gp gp4_gp");

  const Ogre::GpuProgramParametersSharedPtr& params = program->getDefaultParameters();

  try
  {
    params->setNamedAutoConstant("worldview", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
  }
  catch (Ogre::Exception&)
  {
  }

  try
  {
    params->setNamedAutoConstant("worldviewproj", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
  }
  catch (Ogre::Exception&)
  {
  }

  try
  {
    params->setNamedAutoConstant("camera_pos", Ogre::GpuProgramParameters::ACT_CAMERA_POSITION_OBJECT_SPACE);
  }
  catch (Ogre::Exception&)
  {}

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
  ss << "float3 gooch_color = gooch98(color.rgb, normal, float3(0.0, 0.0, 1.0));\n";
  ss << "out_color0 = float4(gooch_color * color.a, color.a);\n";
  ss << "out_color1 = float4(1.0, 0.0, 0.0, 0.0);\n";
}

void generateGBufferShaderCode(std::stringstream& ss, const PointsRendererDesc& desc)
{
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
    ss << "ColorAndNormal can = calculateBillboardSphereColorAndNormal(in_color, in_offset);\n";
    ss << "float4 color = can.color;\n";
    ss << "normal = can.normal;\n";
  }
  else if (desc.type == rviz_msgs::Points::TYPE_BOXES)
  {
    ss << "float4 color = calculateBoxColorWithEdge(in_color, in_offset);\n";
  }

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

  bool supports_geometry_programs = getRenderer()->useGeometryShaders();
  Ogre::GpuProgramPtr geometry_program;
  if (supports_geometry_programs && desc.type != rviz_msgs::Points::TYPE_POINTS)
  {
    geometry_program = generateGeometryShader(desc, false);
  }

  Ogre::MaterialPtr mat_opaque = Ogre::MaterialManager::getSingleton().create(material_name_opaque, ROS_PACKAGE_NAME);
  {
    mat_opaque->getTechnique(0)->setSchemeName("GBuffer");
    mat_opaque->setPointSize(5);
    mat_opaque->setCullingMode(Ogre::CULL_NONE);

    Ogre::Pass* pass = mat_opaque->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    Ogre::GpuProgramPtr vertex_program = generateVertexShader(desc, false);
    Ogre::GpuProgramPtr fragment_program = generateFragmentShader(desc, false);
    pass->setVertexProgram(vertex_program->getName());
    pass->setFragmentProgram(fragment_program->getName());

    if (!geometry_program.isNull())
    {
      pass->setGeometryProgram(geometry_program->getName());
    }
  }

  Ogre::MaterialPtr mat_alpha = Ogre::MaterialManager::getSingleton().create(material_name_alpha, ROS_PACKAGE_NAME);
  {
    mat_alpha->setCullingMode(Ogre::CULL_NONE);
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

    if (!geometry_program.isNull())
    {
      pass->setGeometryProgram(geometry_program->getName());
    }
  }

  return std::make_pair(mat_opaque, mat_alpha);
}

} // namespace rviz_renderer_ogre

