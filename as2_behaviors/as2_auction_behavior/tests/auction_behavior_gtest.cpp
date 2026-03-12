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
 *  \brief      Tests for greedy_sequential and coordinate_item plugins
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <as2_core/names/topics.hpp>

#include "as2_auction_behavior/auction_item_plugin_base.hpp"
#include "as2_state_interface/state_interface.hpp"
#include "greedy_sequential/greedy_sequential.hpp"
#include "coordinate_item/coordinate_item.hpp"
#include "as2_msgs/msg/bid.hpp"

using std::chrono_literals::operator""ms;

class MockItem : public as2_auction_behavior::AuctionItemPluginBase
{
public:
  MockItem(const std::string & name, float cost)
  : name_(name), cost_(cost)
  {
    item_.name = name;
  }

  std::shared_ptr<AuctionItemPluginBase> create(
    const as2_msgs::msg::AuctionItem &) const override
  {
    return std::make_shared<MockItem>(name_, cost_);
  }

  float evaluate(const StateInterface &) const override {return cost_;}
  std::string get_name() const override {return name_;}
  as2_msgs::msg::AuctionItem get_item() const override {return item_;}
  std::string to_string() const override
  {
    return "MockItem(name='" + name_ + "', cost=" + std::to_string(cost_) + ")";
  }

private:
  std::string name_;
  float cost_;
  as2_msgs::msg::AuctionItem item_;
};

class TestablePlugin : public greedy_sequential::Plugin
{
public:
  void add_item(const std::string & name, float cost)
  {
    auction_items_.push_back(std::make_shared<MockItem>(name, cost));
  }

  void set_bundle_size(int size) {bundle_size_ = size;}
};


class GreedySequentialTest : public ::testing::Test
{
protected:
  TestablePlugin plugin_;
};

TEST_F(GreedySequentialTest, SelectsLowestCost)
{
  plugin_.add_item("far", 10.0f);
  plugin_.add_item("close", 1.0f);
  plugin_.add_item("medium", 5.0f);

  auto bid = plugin_.compute_bid();

  ASSERT_EQ(bid.name.size(), 1u);
  EXPECT_EQ(bid.name[0], "close");
  EXPECT_NEAR(bid.amounts[0], 1.0, 1e-5);
}

TEST_F(GreedySequentialTest, SkipsTasksClaimedByOthers)
{
  plugin_.add_item("close", 1.0f);
  plugin_.add_item("far", 10.0f);

  as2_msgs::msg::Bid other_bid;
  other_bid.name.push_back("close");
  other_bid.amounts.push_back(1.0);
  plugin_.update(other_bid, "agent_other");

  auto bid = plugin_.compute_bid();

  ASSERT_EQ(bid.name.size(), 1u);
  EXPECT_EQ(bid.name[0], "far");
}

TEST_F(GreedySequentialTest, NoConvergenceBeforeItemsLoaded)
{
  // Empty auction_items_ → check_convergence must return false
  EXPECT_FALSE(plugin_.check_convergence());
}

TEST_F(GreedySequentialTest, ConvergesAfterBundleIsFull)
{
  plugin_.add_item("task_a", 2.0f);
  plugin_.add_item("task_b", 5.0f);

  EXPECT_FALSE(plugin_.check_convergence());

  // bundle_size defaults to 1; one compute_bid fills it
  plugin_.compute_bid();
  EXPECT_TRUE(plugin_.check_convergence());
}

TEST_F(GreedySequentialTest, ConvergesWhenAllTasksClaimedByOthers)
{
  plugin_.add_item("task_a", 1.0f);
  plugin_.add_item("task_b", 2.0f);

  as2_msgs::msg::Bid other_bid;
  other_bid.name = {"task_a", "task_b"};
  other_bid.amounts = {1.0, 2.0};
  plugin_.update(other_bid, "agent_other");

  EXPECT_TRUE(plugin_.check_convergence());
}

TEST_F(GreedySequentialTest, SecondComputeBidEmptyWhenBundleFull)
{
  plugin_.add_item("task_a", 1.0f);
  plugin_.add_item("task_b", 5.0f);

  plugin_.compute_bid();  // claims task_a; bundle now full (size=1)
  auto second_bid = plugin_.compute_bid();

  EXPECT_TRUE(second_bid.name.empty());
}

TEST_F(GreedySequentialTest, UpdateDoesNotOverwriteOwnClaims)
{
  plugin_.add_item("task_a", 1.0f);

  // Claim the task ourselves first
  plugin_.compute_bid();

  // A bid from another agent trying to claim the same task
  as2_msgs::msg::Bid other_bid;
  other_bid.name.push_back("task_a");
  other_bid.amounts.push_back(1.0);
  plugin_.update(other_bid, "agent_other");

  // We already claimed it; convergence should still hold
  EXPECT_TRUE(plugin_.check_convergence());
  // And get_result must still list our own claim
  auto result = plugin_.get_result();
  ASSERT_EQ(result.elements.size(), 1u);
  EXPECT_EQ(result.elements[0].name, "task_a");
}

TEST_F(GreedySequentialTest, BundleSizeTwoClaimsTwoBestTasks)
{
  plugin_.set_bundle_size(2);
  plugin_.add_item("a", 3.0f);
  plugin_.add_item("b", 1.0f);
  plugin_.add_item("c", 5.0f);

  // First call: claims "b" (cost 1)
  auto bid1 = plugin_.compute_bid();
  ASSERT_EQ(bid1.name.size(), 1u);
  EXPECT_EQ(bid1.name[0], "b");

  // Second call: bundle not full yet, claims "a" (cost 3)
  auto bid2 = plugin_.compute_bid();
  ASSERT_EQ(bid2.name.size(), 1u);
  EXPECT_EQ(bid2.name[0], "a");

  // Third call: bundle full → empty bid
  auto bid3 = plugin_.compute_bid();
  EXPECT_TRUE(bid3.name.empty());
}

TEST_F(GreedySequentialTest, BundleSizeTwoConvergesAfterTwoClaims)
{
  plugin_.set_bundle_size(2);
  plugin_.add_item("a", 1.0f);
  plugin_.add_item("b", 2.0f);
  plugin_.add_item("c", 3.0f);

  EXPECT_FALSE(plugin_.check_convergence());
  plugin_.compute_bid();
  EXPECT_FALSE(plugin_.check_convergence());  // only one claim so far
  plugin_.compute_bid();
  EXPECT_TRUE(plugin_.check_convergence());   // bundle full
}

TEST_F(GreedySequentialTest, BundleSizeTwoSkipsTasksClaimedByOthers)
{
  plugin_.set_bundle_size(2);
  plugin_.add_item("a", 1.0f);
  plugin_.add_item("b", 2.0f);
  plugin_.add_item("c", 3.0f);

  // Other agent claims "a" (cheapest) before we bid
  as2_msgs::msg::Bid other_bid;
  other_bid.name = {"a"};
  other_bid.amounts = {1.0};
  plugin_.update(other_bid, "agent_other");

  auto bid1 = plugin_.compute_bid();
  ASSERT_EQ(bid1.name.size(), 1u);
  EXPECT_EQ(bid1.name[0], "b");  // "a" is taken, next best is "b"

  auto bid2 = plugin_.compute_bid();
  ASSERT_EQ(bid2.name.size(), 1u);
  EXPECT_EQ(bid2.name[0], "c");
}

TEST_F(GreedySequentialTest, BundleSizeLargerThanAvailableTasksConvergesOnceAllClaimed)
{
  plugin_.set_bundle_size(5);
  plugin_.add_item("a", 1.0f);
  plugin_.add_item("b", 2.0f);

  plugin_.compute_bid();  // claims "a"
  EXPECT_FALSE(plugin_.check_convergence());
  plugin_.compute_bid();  // claims "b"
  // All tasks are claimed (by us); bundle not full but no free tasks remain
  EXPECT_TRUE(plugin_.check_convergence());
}

// ────────────────────────────────────────────────────────────────────────────
//  Multi-Agent Tests
// ────────────────────────────────────────────────────────────────────────────

class MultiAgentTest : public ::testing::Test
{
protected:
  void load_items(
    std::vector<TestablePlugin *> agents,
    const std::vector<std::pair<std::string, float>> & items)
  {
    for (auto * agent : agents) {
      for (const auto & [name, cost] : items) {
        agent->add_item(name, cost);
      }
    }
  }

  TestablePlugin agent1_;
  TestablePlugin agent2_;
  TestablePlugin agent3_;
};

// Agent 1 bids first; agent 2 sees the bid, skips the claimed task, and
// claims the next cheapest. Both converge with distinct tasks.
TEST_F(MultiAgentTest, TwoAgents_SequentialBidding_NoConflict)
{
  load_items({&agent1_, &agent2_}, {{"near", 1.0f}, {"far", 10.0f}});

  auto bid1 = agent1_.compute_bid();
  ASSERT_EQ(bid1.name.size(), 1u);
  EXPECT_EQ(bid1.name[0], "near");

  agent2_.update(bid1, "agent1");
  auto bid2 = agent2_.compute_bid();
  ASSERT_EQ(bid2.name.size(), 1u);
  EXPECT_EQ(bid2.name[0], "far");

  agent1_.update(bid2, "agent2");

  EXPECT_TRUE(agent1_.check_convergence());
  EXPECT_TRUE(agent2_.check_convergence());
}

// After a full round of sequential bids, each agent's result contains a
// distinct task with no overlap.
TEST_F(MultiAgentTest, TwoAgents_ResultsAreDisjoint)
{
  load_items({&agent1_, &agent2_}, {{"a", 1.0f}, {"b", 2.0f}, {"c", 3.0f}});

  auto bid1 = agent1_.compute_bid();  // claims "a"
  agent2_.update(bid1, "agent1");
  auto bid2 = agent2_.compute_bid();  // claims "b"
  agent1_.update(bid2, "agent2");

  auto res1 = agent1_.get_result();
  auto res2 = agent2_.get_result();

  ASSERT_EQ(res1.elements.size(), 1u);
  ASSERT_EQ(res2.elements.size(), 1u);
  EXPECT_EQ(res1.elements[0].name, "a");
  EXPECT_EQ(res2.elements[0].name, "b");
  EXPECT_NE(res1.elements[0].name, res2.elements[0].name);
}

// Three agents bid in sequence, each taking the next cheapest free task.
// All three converge, each owning exactly one distinct task.
TEST_F(MultiAgentTest, ThreeAgents_PartitionThreeTasks)
{
  load_items(
    {&agent1_, &agent2_, &agent3_},
    {{"task1", 1.0f}, {"task2", 2.0f}, {"task3", 3.0f}});

  auto bid1 = agent1_.compute_bid();
  EXPECT_EQ(bid1.name[0], "task1");
  agent2_.update(bid1, "agent1");
  agent3_.update(bid1, "agent1");

  auto bid2 = agent2_.compute_bid();
  EXPECT_EQ(bid2.name[0], "task2");
  agent1_.update(bid2, "agent2");
  agent3_.update(bid2, "agent2");

  auto bid3 = agent3_.compute_bid();
  EXPECT_EQ(bid3.name[0], "task3");
  agent1_.update(bid3, "agent3");
  agent2_.update(bid3, "agent3");

  EXPECT_TRUE(agent1_.check_convergence());
  EXPECT_TRUE(agent2_.check_convergence());
  EXPECT_TRUE(agent3_.check_convergence());
}

// With bundle_size=2, each agent claims its two cheapest free tasks after
// seeing the other's bids.  The four tasks are cleanly split 2-2.
TEST_F(MultiAgentTest, TwoAgents_BundleSize2_PartitionFourTasks)
{
  agent1_.set_bundle_size(2);
  agent2_.set_bundle_size(2);
  load_items(
    {&agent1_, &agent2_},
    {{"a", 1.0f}, {"b", 2.0f}, {"c", 3.0f}, {"d", 4.0f}});

  // Agent 1 fills its bundle before broadcasting
  auto bid1a = agent1_.compute_bid();  // claims "a"
  auto bid1b = agent1_.compute_bid();  // claims "b"

  agent2_.update(bid1a, "agent1");
  agent2_.update(bid1b, "agent1");

  // Agent 2 can only claim "c" and "d"
  auto bid2a = agent2_.compute_bid();
  auto bid2b = agent2_.compute_bid();

  ASSERT_EQ(bid2a.name.size(), 1u);
  ASSERT_EQ(bid2b.name.size(), 1u);
  EXPECT_EQ(bid2a.name[0], "c");
  EXPECT_EQ(bid2b.name[0], "d");

  agent1_.update(bid2a, "agent2");
  agent1_.update(bid2b, "agent2");

  EXPECT_TRUE(agent1_.check_convergence());
  EXPECT_TRUE(agent2_.check_convergence());
}

// Agent 2 has different costs for the same tasks (different robot positions).
// Each agent claims its own cheapest task, which may differ from the global cheapest.
TEST_F(MultiAgentTest, TwoAgents_DifferentCostsLeadToDifferentClaims)
{
  // agent1 is closer to "alpha", agent2 is closer to "beta"
  agent1_.add_item("alpha", 1.0f);
  agent1_.add_item("beta", 9.0f);
  agent2_.add_item("alpha", 9.0f);
  agent2_.add_item("beta", 1.0f);

  auto bid1 = agent1_.compute_bid();
  EXPECT_EQ(bid1.name[0], "alpha");

  agent2_.update(bid1, "agent1");
  auto bid2 = agent2_.compute_bid();
  // "alpha" is taken; agent2 claims "beta" which is also its personal cheapest
  EXPECT_EQ(bid2.name[0], "beta");

  agent1_.update(bid2, "agent2");

  EXPECT_TRUE(agent1_.check_convergence());
  EXPECT_TRUE(agent2_.check_convergence());
}

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

  // Publish a pose and spin until the StateInterface has received it.
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
  // Robot at (0,0), target at (3,4) -> distance = 5
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
  // Robot at (1,0), target at (4,4) -> distance = 5
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
