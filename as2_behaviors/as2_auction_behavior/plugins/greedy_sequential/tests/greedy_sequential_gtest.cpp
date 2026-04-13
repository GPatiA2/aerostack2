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
 *  \file       greedy_sequential_gtest.cpp
 *  \brief      Unit tests for the greedy_sequential auction plugin
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "as2_auction_behavior/auction_item_plugin_base.hpp"
#include "as2_state_interface/state_interface.hpp"
#include "greedy_sequential/greedy_sequential.hpp"
#include "as2_msgs/msg/bid.hpp"

// ── mock infrastructure ──────────────────────────────────────────────────────

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

// Exposes protected members for white-box testing.
// add_item_with_cost() populates both auction_items_ and my_costs_ without
// triggering send_bid() so tests run without a live ROS node.
class TestablePlugin : public greedy_sequential::Plugin
{
public:
  void add_item_with_cost(const std::string & name, float cost)
  {
    auction_items_.push_back(std::make_shared<MockItem>(name, cost));
    my_costs_[name] = static_cast<double>(cost);
  }

  void set_namespace(const std::string & ns) {namespace_ = ns;}
  void add_participant(const std::string & p) {participants_.push_back(p);}
};


// ── greedy_sequential tests ──────────────────────────────────────────────────

class GreedySequentialTest : public ::testing::Test
{
protected:
  TestablePlugin plugin_;
};

TEST_F(GreedySequentialTest, NoConvergenceWithoutItems)
{
  EXPECT_FALSE(plugin_.check_convergence());
}

TEST_F(GreedySequentialTest, NoConvergenceWithoutParticipants)
{
  plugin_.add_item_with_cost("a", 1.0f);
  EXPECT_FALSE(plugin_.check_convergence());
}

TEST_F(GreedySequentialTest, ConvergesImmediatelyWhenSoleParticipant)
{
  // participants_ contains only self — n_expected == 0, no peers to wait for.
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_item_with_cost("a", 1.0f);

  EXPECT_TRUE(plugin_.check_convergence());
}

TEST_F(GreedySequentialTest, NoConvergenceUntilAllPeersHeard)
{
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.add_participant("/drone2");
  plugin_.add_item_with_cost("a", 1.0f);

  EXPECT_FALSE(plugin_.check_convergence());

  as2_msgs::msg::Bid b1;
  b1.name = {"a"};
  b1.amounts = {2.0};
  plugin_.update(b1, "drone1");
  EXPECT_FALSE(plugin_.check_convergence());  // still missing drone2

  as2_msgs::msg::Bid b2;
  b2.name = {"a"};
  b2.amounts = {3.0};
  plugin_.update(b2, "drone2");
  EXPECT_TRUE(plugin_.check_convergence());   // all peers heard
}

TEST_F(GreedySequentialTest, ConflictResolution_CheapestAgentWins)
{
  plugin_.set_namespace("drone1");
  plugin_.add_item_with_cost("item_a", 5.0f);  // drone1 cost

  as2_msgs::msg::Bid peer_bid;
  peer_bid.name = {"item_a"};
  peer_bid.amounts = {2.0};  // drone0 is cheaper
  plugin_.update(peer_bid, "drone0");

  // drone0 (cost 2.0) beats drone1 (cost 5.0) — drone1 gets nothing
  auto result = plugin_.get_result();
  EXPECT_TRUE(result.elements.empty());
}

TEST_F(GreedySequentialTest, ConflictResolution_SelfWinsWhenCheaper)
{
  plugin_.set_namespace("drone0");
  plugin_.add_item_with_cost("item_a", 2.0f);  // drone0 cost

  as2_msgs::msg::Bid peer_bid;
  peer_bid.name = {"item_a"};
  peer_bid.amounts = {5.0};  // drone1 is more expensive
  plugin_.update(peer_bid, "drone1");

  auto result = plugin_.get_result();
  ASSERT_EQ(result.elements.size(), 1u);
  EXPECT_EQ(result.elements[0].name, "item_a");
}

TEST_F(GreedySequentialTest, ConflictResolution_TieBrokenByAgentName)
{
  // "drone0" < "drone1" lexicographically — drone0 should win on a tie.
  plugin_.set_namespace("drone1");
  plugin_.add_item_with_cost("item_a", 3.0f);

  as2_msgs::msg::Bid peer_bid;
  peer_bid.name = {"item_a"};
  peer_bid.amounts = {3.0};  // exactly equal cost
  plugin_.update(peer_bid, "drone0");

  // drone0 wins the tie (lexicographically smaller) — drone1 gets nothing
  auto result = plugin_.get_result();
  EXPECT_TRUE(result.elements.empty());
}

TEST_F(GreedySequentialTest, TwoAgents_EachWinsItsOwnItem)
{
  // drone0 is closer to item_A, drone1 is closer to item_B.
  // After exchanging bids, each should own its nearest item.
  TestablePlugin drone0, drone1;
  drone0.set_namespace("drone0");
  drone1.set_namespace("drone1");
  drone0.add_participant("/drone0");
  drone0.add_participant("/drone1");
  drone1.add_participant("/drone0");
  drone1.add_participant("/drone1");

  drone0.add_item_with_cost("item_A", 1.0f);
  drone0.add_item_with_cost("item_B", 9.0f);
  drone1.add_item_with_cost("item_A", 9.0f);
  drone1.add_item_with_cost("item_B", 1.0f);

  // Exchange bids
  as2_msgs::msg::Bid bid0;
  bid0.name = {"item_A", "item_B"};
  bid0.amounts = {1.0, 9.0};
  drone1.update(bid0, "drone0");

  as2_msgs::msg::Bid bid1;
  bid1.name = {"item_A", "item_B"};
  bid1.amounts = {9.0, 1.0};
  drone0.update(bid1, "drone1");

  EXPECT_TRUE(drone0.check_convergence());
  EXPECT_TRUE(drone1.check_convergence());

  auto res0 = drone0.get_result();
  auto res1 = drone1.get_result();

  ASSERT_EQ(res0.elements.size(), 1u);
  ASSERT_EQ(res1.elements.size(), 1u);
  EXPECT_EQ(res0.elements[0].name, "item_A");
  EXPECT_EQ(res1.elements[0].name, "item_B");
}

TEST_F(GreedySequentialTest, ThreeItems_TwoAgents_AssignmentCoversAll)
{
  // Three items, two agents. Each item must end up with exactly one owner.
  TestablePlugin drone0, drone1;
  drone0.set_namespace("drone0");
  drone1.set_namespace("drone1");
  drone0.add_participant("/drone0");
  drone0.add_participant("/drone1");
  drone1.add_participant("/drone0");
  drone1.add_participant("/drone1");

  drone0.add_item_with_cost("a", 1.0f);
  drone0.add_item_with_cost("b", 2.0f);
  drone0.add_item_with_cost("c", 10.0f);
  drone1.add_item_with_cost("a", 10.0f);
  drone1.add_item_with_cost("b", 10.0f);
  drone1.add_item_with_cost("c", 1.0f);

  as2_msgs::msg::Bid bid0;
  bid0.name = {"a", "b", "c"};
  bid0.amounts = {1.0, 2.0, 10.0};
  drone1.update(bid0, "drone0");

  as2_msgs::msg::Bid bid1;
  bid1.name = {"a", "b", "c"};
  bid1.amounts = {10.0, 10.0, 1.0};
  drone0.update(bid1, "drone1");

  auto res0 = drone0.get_result();
  auto res1 = drone1.get_result();

  // drone0 wins a(1<10) and b(2<10), drone1 wins c(1<10)
  EXPECT_EQ(res0.elements.size(), 2u);
  EXPECT_EQ(res1.elements.size(), 1u);
  EXPECT_EQ(res1.elements[0].name, "c");

  // No overlap and full coverage
  std::set<std::string> all;
  for (const auto & e : res0.elements) {all.insert(e.name);}
  for (const auto & e : res1.elements) {all.insert(e.name);}
  EXPECT_EQ(all.size(), 3u);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
