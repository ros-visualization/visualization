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

#include <gtest/gtest.h>

#include <rviz_rpc/TestRequest.h>
#include <rviz_rpc/TestResponse.h>

using namespace rviz_rpc;

TestResponsePtr doubleCallback(const TestRequestConstPtr& req)
{
  TestResponsePtr res(new TestResponse);
  res->value_doubled = req->value * 2;
  return res;
}

TEST(Intra, callWithReturn)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  Client<TestRequest, TestResponse> c("test", nh);
  Server<TestRequest, TestResponse> s("test", nh, doubleCallback);

  c.waitForServer();

  TestRequestPtr req(new TestRequest);
  req->value = 5;
  TestResponseConstPtr res = c.call(req);
  EXPECT_EQ(res->value_doubled, 10U);
}

void callThread(volatile bool& done, bool& success, uint32_t start)
{
  ros::NodeHandle nh;
  Client<TestRequest, TestResponse> c("test", nh);
  c.waitForServer();

  while (!done)
  {
    TestRequestPtr req(new TestRequest);
    req->value = start++;
    TestResponseConstPtr res = c.call(req);
    if (res->value_doubled != req->value * 2)
    {
      ROS_ERROR("Expected %d but got %d", req->value, res->value_doubled);
      success = false;
    }
  }
}

TEST(Intra, multipleCallerThreads)
{
  ros::AsyncSpinner sp(4);
  sp.start();
  ros::NodeHandle nh;
  Server<TestRequest, TestResponse> s("test", nh, doubleCallback);

  bool done = false;
  bool success = true;
  boost::thread_group tg;
  for (uint32_t i = 0; i < 10; ++i)
  {
    tg.create_thread(boost::bind(callThread, boost::ref(done), boost::ref(success), i * 10000));
  }

  ros::WallDuration(5.0).sleep();
  done = true;
  tg.join_all();

  ASSERT_TRUE(success);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_intra");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
