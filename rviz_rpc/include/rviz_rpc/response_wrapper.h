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

#ifndef RVIZ_RPC_RESPONSE_WRAPPER_H
#define RVIZ_RPC_RESPONSE_WRAPPER_H

#include "serializable_message.h"

#include <rviz_rpc/Response.h>

#include <rviz_uuid/uuid.h>

#include <ros/message_traits.h>
#include <ros/serialization.h>

#include <string>

#include <boost/make_shared.hpp>

namespace rviz_rpc
{

struct ResponseWrapper
{
  ResponseWrapper()
  : protocol(0)
  , error_code(0)
  {}

  rviz_uuid::UUID request_id;
  uint8_t protocol;
  uint8_t error_code;
  std::string error_string;

  SerializableMessage message;
  ros::SerializedMessage serialized_message;

  template<typename M>
  boost::shared_ptr<M const> instantiate() const
  {
    if (message.ti && *message.ti == typeid(M))
    {
      return boost::static_pointer_cast<M const>(message.message);
    }
    else if (serialized_message.buf)
    {
      boost::shared_ptr<M> m(boost::make_shared<M>());
      ros::serialization::deserializeMessage(serialized_message, *m);
      return m;
    }
    else
    {
      ros::SerializedMessage ser_m = message.serialize(message.message);
      boost::shared_ptr<M> m(boost::make_shared<M>());
      ros::serialization::deserializeMessage(ser_m, *m);
      return m;
    }
  }
};
typedef boost::shared_ptr<ResponseWrapper> ResponseWrapperPtr;
typedef boost::shared_ptr<ResponseWrapper const> ResponseWrapperConstPtr;

}

namespace ros
{

namespace message_traits
{

template<>
struct MD5Sum<rviz_rpc::ResponseWrapper> {
  static const char* value() { return MD5Sum<rviz_rpc::Response>::value(); }
  static const char* value(const rviz_rpc::ResponseWrapper& w) { return value(); }
};

template<>
struct DataType<rviz_rpc::ResponseWrapper> {
  static const char* value() { return DataType<rviz_rpc::Response>::value(); }
  static const char* value(const rviz_rpc::ResponseWrapper& w) { return value(); }
};

template<>
struct Definition<rviz_rpc::ResponseWrapper> {
  static const char* value() { return Definition<rviz_rpc::Response>::value(); }
  static const char* value(const rviz_rpc::ResponseWrapper& w) { return value(); }
};

} // namespace message_traits

namespace serialization
{

template<>
struct Serializer<rviz_rpc::ResponseWrapper>
{
  template<typename Stream>
  inline static void write(Stream& stream, const rviz_rpc::ResponseWrapper& r)
  {
    stream.next(rviz_msgs::UUID(r.request_id));
    stream.next(r.protocol);
    stream.next(r.error_code);
    stream.next(r.error_string);

    if (r.message.serialize)
    {
      SerializedMessage sm = r.message.serialize(r.message.message);
      uint32_t len = sm.num_bytes;
      memcpy(stream.advance(len), sm.buf.get(), len);
    }
    else
    {
      stream.next((uint32_t)0);
    }
  }

  template<typename Stream>
  inline static void read(Stream& stream, rviz_rpc::ResponseWrapper& r)
  {
    rviz_msgs::UUID req_id;
    stream.next(req_id);
    r.request_id = req_id;
    stream.next(r.protocol);
    stream.next(r.error_code);
    stream.next(r.error_string);
    uint32_t len = 0;
    stream.next(len);
    r.serialized_message.num_bytes = len;
    r.serialized_message.buf.reset(new uint8_t[len]);
    r.serialized_message.message_start = r.serialized_message.buf.get();
    memcpy(r.serialized_message.buf.get(), stream.advance(len), len);
  }

  inline static uint32_t serializedLength(const rviz_rpc::ResponseWrapper& r)
  {
    uint32_t size = 0;
    size += sizeof(r.request_id);
    size += serializationLength(r.protocol);
    size += serializationLength(r.error_code);
    size += serializationLength(r.error_string);
    size += 4; // message length field
    if (r.serialized_message.buf)
    {
      size += r.serialized_message.num_bytes;
    }
    else if (r.message.serialized_length)
    {
      size += r.message.serialized_length(r.message.message);
    }
    return size;
  }

};

} // namespace serialization

} // namespace ros

#endif // RVIZ_RPC_RESPONSE_WRAPPER_H
