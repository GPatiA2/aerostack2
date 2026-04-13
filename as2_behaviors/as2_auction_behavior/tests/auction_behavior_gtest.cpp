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

/*!*******************************************************************************************
 *  \file       auction_behavior_gtest.cpp
 *  \brief      Tests for coordinate_item plugin
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <as2_core/names/topics.hpp>

#include "as2_state_interface/state_interface.hpp"
#include "coordinate_item/coordinate_item.hpp"

using std::chrono_literals::operator""ms;
// ── coordinate_item tests ────────────────────────────────────────────────────

class CoordinateItemTest : public ::testing::Test
{
protected:
  CoordinateItemTest()
  {
    node_ = std::make_shared<rclcpp::Node>("test_coordinate_item");
    executor_.add_node(node_);
    state_interface_.configure(
      node_.get(),
      {as2_names::topics::self_localization::pose});
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, 10);
  }

  void publish_pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    rclcpp::WallRate rate(10ms);
    for (int i = 0; i < 15; ++i) {
      pub_->publish(pose);
      rate.sleep();
      executor_.spin_some();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  StateInterface state_interface_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

TEST_F(CoordinateItemTest, CreateStoresNameAndCoords)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target_A";
  item_msg.features = {3.0, 4.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  EXPECT_EQ(item->get_name(), "target_A");
  EXPECT_EQ(item->get_item().name, "target_A");
}

TEST_F(CoordinateItemTest, EvaluateDistanceFromOrigin)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target";
  item_msg.features = {3.0, 4.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  publish_pose(0.0, 0.0);

  float dist = item->evaluate(state_interface_);
  EXPECT_NEAR(dist, 5.0f, 1e-4f);
}

TEST_F(CoordinateItemTest, EvaluateDistanceFromNonOriginPose)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target";
  item_msg.features = {4.0, 4.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  publish_pose(1.0, 0.0);

  float dist = item->evaluate(state_interface_);
  EXPECT_NEAR(dist, 5.0f, 1e-4f);
}

TEST_F(CoordinateItemTest, EvaluateZeroDistanceWhenAtTarget)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target";
  item_msg.features = {2.0, 3.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  publish_pose(2.0, 3.0);

  float dist = item->evaluate(state_interface_);
  EXPECT_NEAR(dist, 0.0f, 1e-4f);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
