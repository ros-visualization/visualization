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

#include <rviz_interface_gen/Test.h>

#include <gtest/gtest.h>

#include <ros/ros.h>

using namespace rviz_interface_gen;

class MyTestServer : public TestServer
{
public:
  MyTestServer(const std::string& name, const ros::NodeHandle& nh)
  : TestServer(name, nh)
  {}

  virtual void triple( uint32_t val ,  uint32_t& out_val )
  {
    out_val = val * 3;
  }

  virtual void blockFor( const ros::Duration& d )
  {
    d.sleep();
  }

  virtual void noBlock( )
  {
    async_call_done = false;
    ros::Duration(1.0).sleep();
    async_call_done = true;
  }

  virtual void complex( const std_msgs::String& name ,  float val ,  std_msgs::String& out_ret )
  {
    std::stringstream ss;
    ss << name << val;
    out_ret.data = ss.str();
  }

  virtual void multipleReturn(uint32_t val, uint32_t& out_val, uint32_t& out_val_doubled)
  {
    out_val = val;
    out_val_doubled = val * 2;
  }

  virtual void withString(const std::string& str)
  {

  }

  bool async_call_done;
};

TEST(Interfaces, callWithReturn)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  MyTestServer s("test", nh);
  TestProxy p("test", nh);

  uint32_t ret = 0;
  p.triple(5, ret);
  EXPECT_EQ(ret, 15U);
}

TEST(Interfaces, blockingCall)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  MyTestServer s("test", nh);
  TestProxy p("test", nh);

  ros::Time start = ros::Time::now();
  p.blockFor(ros::Duration(1.0));
  ros::Time end = ros::Time::now();
  EXPECT_GE(end, start + ros::Duration(1.0));
}

TEST(Interfaces, asyncCall)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  MyTestServer s("test", nh);
  TestProxy p("test", nh);

  p.noBlock();
  EXPECT_FALSE(s.async_call_done);
}

TEST(Interfaces, multipleReturn)
{
  ros::AsyncSpinner sp(1);
  sp.start();
  ros::NodeHandle nh;
  MyTestServer s("test", nh);
  TestProxy p("test", nh);

  uint32_t ret1 = 0;
  uint32_t ret2 = 0;
  p.multipleReturn(5, ret1, ret2);
  EXPECT_EQ(ret1, 5U);
  EXPECT_EQ(ret2, 10U);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_intra");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}

