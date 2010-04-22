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

#include <rviz_rpc/client.h>
#include <rviz_rpc/server.h>

#include <rviz_rpc/request_wrapper.h>
#include <rviz_rpc/response_wrapper.h>
#include <rviz_rpc/TestRequest.h>
#include <rviz_rpc/TestResponse.h>
#include <std_msgs/UInt32.h>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <boost/thread.hpp>

using namespace rviz_rpc;
namespace ser = ros::serialization;

RequestConstPtr g_request;
void requestMessageCallback(const RequestConstPtr& msg)
{
  g_request = msg;
}

RequestWrapperConstPtr g_request_wrapper;
void requestWrapperCallback(const RequestWrapperConstPtr& wrapper)
{
  g_request_wrapper = wrapper;
}

TEST(RequestWrapper, sendWrapperReceiveMessage)
{
  g_request.reset();
  RequestWrapperPtr wrap(new RequestWrapper);
  wrap->message.serialize = serialize<uint32_t>;
  wrap->message.serialized_length = serializedLength<uint32_t>;
  wrap->message.message = boost::shared_ptr<uint32_t>(new uint32_t(5));
  wrap->request_id = rviz_uuid::UUID::Generate();
  wrap->method = "foo";

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, requestMessageCallback);
  ros::Publisher pub = nh.advertise<RequestWrapper>("test", 0);
  pub.publish(wrap);

  while (!g_request)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  ASSERT_EQ(g_request->data.size(), 4U);

  ser::IStream stream((uint8_t*)&g_request->data.front(), (uint32_t)g_request->data.size());
  uint32_t val = 0;
  ser::deserialize(stream, val);
  EXPECT_EQ(val, 5U);
  EXPECT_EQ(wrap->request_id, g_request->request_id);
  EXPECT_STREQ(wrap->method.c_str(), g_request->method.c_str());
}

TEST(RequestWrapper, sendMessageReceiveWrapper)
{
  g_request_wrapper.reset();
  RequestPtr req(new Request);
  req->data.resize(4);
  ser::IStream stream((uint8_t*)&req->data.front(), (uint32_t)req->data.size());
  ros::serialization::serialize(stream, (uint32_t)5);
  req->request_id = rviz_uuid::UUID::Generate();
  req->method = "bar";

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, requestWrapperCallback);
  ros::Publisher pub = nh.advertise<RequestWrapper>("test", 0);
  pub.publish(req);

  while (!g_request_wrapper)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  ASSERT_EQ(g_request_wrapper->serialized_message.num_bytes, 4U);
  EXPECT_EQ(*g_request_wrapper->instantiate<uint32_t>(), 5U);
  EXPECT_EQ(g_request_wrapper->request_id, req->request_id);
  EXPECT_STREQ(g_request_wrapper->method.c_str(), req->method.c_str());
}

TEST(RequestWrapper, instantiate)
{
  RequestWrapper wrap;
  wrap.message.serialize = serialize<uint32_t>;
  wrap.message.serialized_length = serializedLength<uint32_t>;
  wrap.message.message = boost::shared_ptr<uint32_t>(new uint32_t(5));
  uint32_t val = *wrap.instantiate<uint32_t>();
  EXPECT_EQ(val, 5U);
  std_msgs::UInt32ConstPtr msg = wrap.instantiate<std_msgs::UInt32>();
  EXPECT_EQ(msg->data, 5U);
}

ResponseConstPtr g_response;
void responseMessageCallback(const ResponseConstPtr& msg)
{
  g_response = msg;
}

ResponseWrapperConstPtr g_response_wrapper;
void responseWrapperCallback(const ResponseWrapperConstPtr& wrapper)
{
  g_response_wrapper = wrapper;
}

TEST(ResponseWrapper, sendWrapperReceiveMessage)
{
  g_response.reset();
  ResponseWrapperPtr wrap(new ResponseWrapper);
  wrap->message.serialize = serialize<uint32_t>;
  wrap->message.serialized_length = serializedLength<uint32_t>;
  wrap->message.message = boost::shared_ptr<uint32_t>(new uint32_t(5));
  wrap->request_id = rviz_uuid::UUID::Generate();
  wrap->error_code = 5;
  wrap->error_string = "foo";

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, responseMessageCallback);
  ros::Publisher pub = nh.advertise<ResponseWrapper>("test", 0);
  pub.publish(wrap);

  while (!g_response)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  ASSERT_EQ(g_response->data.size(), 4U);

  ser::IStream stream((uint8_t*)&g_response->data.front(), (uint32_t)g_response->data.size());
  uint32_t val = 0;
  ser::deserialize(stream, val);
  EXPECT_EQ(val, 5U);
  EXPECT_EQ(wrap->request_id, g_response->request_id);
  EXPECT_EQ(wrap->error_code, g_response->error_code);
  EXPECT_STREQ(wrap->error_string.c_str(), g_response->error_string.c_str());
}

TEST(ResponseWrapper, sendMessageReceiveWrapper)
{
  g_response_wrapper.reset();
  ResponsePtr req(new Response);
  req->data.resize(4);
  ser::IStream stream((uint8_t*)&req->data.front(), (uint32_t)req->data.size());
  ros::serialization::serialize(stream, (uint32_t)5);
  req->request_id = rviz_uuid::UUID::Generate();
  req->error_code = 5;
  req->error_string = "bar";

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, responseWrapperCallback);
  ros::Publisher pub = nh.advertise<ResponseWrapper>("test", 0);
  pub.publish(req);

  while (!g_response_wrapper)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  ASSERT_EQ(g_response_wrapper->serialized_message.num_bytes, 4U);
  EXPECT_EQ(*g_response_wrapper->instantiate<uint32_t>(), 5U);
  EXPECT_EQ(g_response_wrapper->request_id, req->request_id);
  EXPECT_EQ(g_response_wrapper->error_code, req->error_code);
  EXPECT_STREQ(g_response_wrapper->error_string.c_str(), req->error_string.c_str());
}

TEST(ResponseWrapper, instantiate)
{
  ResponseWrapper wrap;
  wrap.message.serialize = serialize<uint32_t>;
  wrap.message.serialized_length = serializedLength<uint32_t>;
  wrap.message.message = boost::shared_ptr<uint32_t>(new uint32_t(5));
  uint32_t val = *wrap.instantiate<uint32_t>();
  EXPECT_EQ(val, 5U);
  std_msgs::UInt32ConstPtr msg = wrap.instantiate<std_msgs::UInt32>();
  EXPECT_EQ(msg->data, 5U);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_intra");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}

