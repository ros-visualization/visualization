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

#include <rviz_mesh_loader/loader.h>
#include <rviz_mesh_loader/assimp_parser.h>
#include <rviz_mesh_loader/stl_parser.h>
#include <rviz_mesh_loader/exception.h>

#include <resource_retriever/retriever.h>

#include <boost/filesystem.hpp>

#include <sstream>

namespace fs = boost::filesystem;

namespace rviz_mesh_loader
{

void load(const std::string& resource_path, rviz_msgs::Mesh& out_mesh)
{
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try
  {
    res = retriever.get(resource_path);
  }
  catch (resource_retriever::Exception& e)
  {
    std::stringstream ss;
    ss << "Failed to load resource [" << resource_path << "]: " << e.what();
    throw LoadException(ss.str());
  }

  if (res.size == 0)
  {
    throw LoadException("Resource [" + resource_path + "] has 0 size");
  }

  std::string ext = fs::extension(fs::path(resource_path)).substr(1);
  if (ext == "stl" || ext == "STL" || ext == "stlb" || ext == "STLB")
  {
    parseSTL(res.data.get(), res.size, resource_path, out_mesh);
  }
  else
  {
    parseWithAssimp(res.data.get(), res.size, resource_path, out_mesh);
  }
}

} // namespace rviz_mesh_loader
