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

#include "uuid.h"

#include <rviz_msgs/UUID.h>

#include <uuid/uuid.h>

#include <ros/assert.h>

namespace rviz
{

ROS_STATIC_ASSERT(sizeof(uuid_t) == 16);
ROS_STATIC_ASSERT(sizeof(UUID) == sizeof(uuid_t));

UUID UUID::Generate()
{
  uuid_t native;
  uuid_generate(native);

  UUID uuid;
  memcpy(&uuid, native, sizeof(uuid));

  return uuid;
}

UUID::UUID()
{
  data_.assign(0);
}

UUID::UUID(const rviz_msgs::UUID& rhs)
{
  *this = rhs;
}

bool UUID::operator<(const UUID& rhs) const
{
  return memcmp(this, &rhs, sizeof(rhs)) < 0;
}

bool UUID::operator==(const UUID& rhs) const
{
  return memcmp(this, &rhs, sizeof(rhs)) == 0;
}

UUID& UUID::operator=(const rviz_msgs::UUID& rhs)
{
  std::copy(rhs.data.begin(), rhs.data.end(), data_.begin());

  return *this;
}

std::string UUID::toString() const
{
  uuid_t native;
  memcpy(native, this, sizeof(native));
  char buf[37]; // UUID is 36 bytes + NULL terminator
  uuid_unparse(native, buf);
  return std::string(buf, 36);
}

UUID::operator rviz_msgs::UUID() const
{
  rviz_msgs::UUID msg;
  std::copy(data_.begin(), data_.end(), msg.data.begin());
  return msg;
}

std::ostream& operator<<(std::ostream& o, const UUID& u)
{
  o << u.toString();
  return o;
}

std::istream& operator>>(std::istream& i, UUID& u)
{
  char buf[37];
  i.get(buf, 36);
  buf[36] = 0;

  uuid_t native;
  uuid_parse(buf, native);
  memcpy(&u, native, sizeof(u));

  return i;
}

} // namespace rviz
