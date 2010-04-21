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

#ifndef RVIZ_RPC_TRAITS_H
#define RVIZ_RPC_TRAITS_H

#include <rviz_uuid/uuid.h>
#include <rviz_msgs/UUID.h>
#include <ros/static_assert.h>

namespace rviz_rpc
{


namespace error_codes
{
enum ErrorCode
{
  Success,
  Exception,
};
} // namespace error_codes
typedef error_codes::ErrorCode ErrorCode;

namespace traits
{

template<typename M>
struct RequestID
{
  static rviz_msgs::UUID& reference(M& m)
  {
    return m.request_id;
  }

  static rviz_msgs::UUID value(const M& m)
  {
    return m.request_id;
  }
};

template<typename M>
struct ErrorCode
{
  static uint8_t& reference(M& m)
  {
    return m.error_code;
  }

  static uint8_t value(const M& m)
  {
    return m.error_code;
  }
};

template<typename M>
struct ErrorString
{
  static std::string& reference(M& m)
  {
    return m.error_string;
  }

  static const std::string& constReference(const M& m)
  {
    return m.error_string;
  }
};
} // namespace traits
} // namespace rviz_rpc

#endif // RVIZ_RPC_TRAITS_H
