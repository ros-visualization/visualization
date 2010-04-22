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

#include <rviz_rpc/TestRequest.h>
#include <rviz_rpc/TestResponse.h>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <boost/thread.hpp>

using namespace rviz_rpc;

TestResponsePtr doubleCallback(const ros::MessageEvent<TestRequest const>& req)
{
  TestResponsePtr res(new TestResponse);
  res->value = req.getMessage()->value * 2;
  return res;
}

TestResponsePtr tripleCallback(const ros::MessageEvent<TestRequest const>& req)
{
  TestResponsePtr res(new TestResponse);
  res->value = req.getMessage()->value * 3;
  return res;
}

TEST(Intra, callWithReturn)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  Server s("test", nh);
  s.addMethod<TestRequest, TestResponse>("double", doubleCallback);
  s.ready();

  Client c("test", nh);
  Method<TestRequest, TestResponse> m = c.addMethod<TestRequest, TestResponse>("double");
  c.connect();

  TestRequestPtr req(new TestRequest);
  req->value = 5;
  TestResponseConstPtr res = m.call(req);
  EXPECT_EQ(res->value, 10U);
}

TEST(Intra, multipleMethodsWithReturn)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  Server s("test", nh);
  s.addMethod<TestRequest, TestResponse>("double", doubleCallback);
  s.addMethod<TestRequest, TestResponse>("triple", tripleCallback);
  s.ready();

  Client c("test", nh);
  Method<TestRequest, TestResponse> d = c.addMethod<TestRequest, TestResponse>("double");
  Method<TestRequest, TestResponse> t = c.addMethod<TestRequest, TestResponse>("triple");
  c.connect();

  {
    TestRequestPtr req(new TestRequest);
    req->value = 5;
    TestResponseConstPtr res = d.call(req);
    EXPECT_EQ(res->value, 10U);
  }

  {
    TestRequestPtr req(new TestRequest);
    req->value = 20;
    TestResponseConstPtr res = t.call(req);
    EXPECT_EQ(res->value, 60U);
  }
}

TEST(Intra, callUnknownMethod)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  Server s("test", nh);
  s.addMethod<TestRequest, TestResponse>("double", doubleCallback);
  s.ready();

  Client c("test", nh);
  Method<TestRequest, TestResponse> m = c.addMethod<TestRequest, TestResponse>("triple");
  c.connect();

  TestRequestPtr req(new TestRequest);
  req->value = 5;

  try
  {
    TestResponseConstPtr res = m.call(req);
    FAIL();
  }
  catch (CallException&)
  {
    SUCCEED();
  }
}

void callThread(Method<TestRequest, TestResponse>& m, volatile bool& done, bool& success, uint32_t start)
{
  while (!done)
  {
    TestRequestPtr req(new TestRequest);
    req->value = start++;
    TestResponseConstPtr res = m.call(req);
    if (res->value != req->value * 2)
    {
      ROS_ERROR("Expected %d but got %d", req->value, res->value);
      success = false;
    }
  }
}

TEST(Intra, multipleCallerThreads)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  Server s("test", nh);
  s.addMethod<TestRequest, TestResponse>("double", doubleCallback);
  s.ready();

  Client c("test", nh);
  Method<TestRequest, TestResponse> m = c.addMethod<TestRequest, TestResponse>("double");
  c.connect();

  bool done = false;
  bool success = true;
  boost::thread_group tg;
  for (uint32_t i = 0; i < 10; ++i)
  {
    tg.create_thread(boost::bind(callThread, boost::ref(m), boost::ref(done), boost::ref(success), i * 10000));
  }

  ros::WallDuration(5.0).sleep();
  done = true;
  tg.join_all();

  ASSERT_TRUE(success);
}

struct Helper
{
  TestResponsePtr cb(const ros::MessageEvent<TestRequest const>& req)
  {
    req_ = req.getMessage();
    res_.reset(new TestResponse);
    res_->value = req.getMessage()->value * 2;
    return res_;
  }

  TestRequestConstPtr req_;
  TestResponsePtr res_;
};

TEST(Intra, noCopy)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  Server s("test", nh);
  Helper h;
  s.addMethod<TestRequest, TestResponse>("double", boost::bind(&Helper::cb, &h, _1));
  s.ready();

  Client c("test", nh);
  Method<TestRequest, TestResponse> m = c.addMethod<TestRequest, TestResponse>("double");
  c.connect();

  TestRequestPtr req(new TestRequest);
  req->value = 5;
  TestResponseConstPtr res = m.call(req);
  EXPECT_EQ(res->value, 10U);
  EXPECT_EQ(req, h.req_);
  EXPECT_EQ(res, h.res_);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_intra");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
