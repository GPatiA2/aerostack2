// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file state_interface_gtest.cpp
*
* A state interface gtest
*
* @authors Guillermo GP-Lenza
*/

#include <gtest/gtest.h>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executor.hpp"

#include "as2_state_interface/state_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::chrono_literals::operator""ms;

class StateInterface_Test : public ::testing::Test
{
protected:
  StateInterface_Test()
  {
    keys_to_register.push_back(as2_names::topics::self_localization::pose);
    keys_to_register.push_back(as2_names::topics::self_localization::twist);
    node_ = std::make_shared<rclcpp::Node>("drone0");
    executor_.add_node(node_);
    state_interface_ = StateInterface();
    state_interface_.configure(node_.get(), keys_to_register);
  }

  std::shared_ptr<rclcpp::Node> node_;
  StateInterface state_interface_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<std::string> keys_to_register;
};

TEST_F(StateInterface_Test, TestInterface1)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = rclcpp::Time(123456789);
  msg.pose.position.x = 1.0;
  msg.pose.position.y = 2.0;
  msg.pose.position.z = 3.0;
  auto publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::self_localization::pose, 10);

  rclcpp::WallRate rate(10ms);
  for (int i = 0; i < 10; ++i) {
    publisher->publish(msg);
    rate.sleep();
    executor_.spin_some();
    auto value = state_interface_.get_value<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose);
    RCLCPP_INFO(
      node_->get_logger(), "Received value: [%f, %f, %f]", value.pose.position.x,
      value.pose.position.y, value.pose.position.z);
    EXPECT_EQ(value.header.stamp, msg.header.stamp);
    EXPECT_EQ(value.pose.position.x, msg.pose.position.x);
    EXPECT_EQ(value.pose.position.y, msg.pose.position.y);
    EXPECT_EQ(value.pose.position.z, msg.pose.position.z);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
