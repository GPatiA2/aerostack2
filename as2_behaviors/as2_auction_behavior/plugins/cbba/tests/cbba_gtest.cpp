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
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       cbba_gtest.cpp
 *  \brief      Unit tests for the CBBA auction plugin
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "as2_auction_behavior/auction_item_plugin_base.hpp"
#include "as2_core/state_interface.hpp"
#include "cbba/cbba.hpp"
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
// Allows direct manipulation of state without triggering send_bid.
class TestablePlugin : public cbba::Plugin
{
public:
  // Direct access to state for testing
  const std::vector<double> & y() const {return y_;}
  const std::vector<std::string> & z() const {return z_;}
  const std::vector<std::string> & bundle() const {return bundle_;}
  bool changed() const {return changed_;}

  // Trigger bundle building without state changes
  void trigger_build_bundle() {build_bundle();}

  // Direct state manipulation for setup
  void add_item_with_cost(const std::string & name, float cost)
  {
    auction_items_.push_back(std::make_shared<MockItem>(name, cost));
    item_names_.push_back(name);
    my_scores_[name] = -static_cast<double>(cost);  // score = -cost
    y_.push_back(-std::numeric_limits<double>::infinity());
    z_.push_back("");
  }

  void set_bundle_limit(int limit) {bundle_size_limit_ = limit;}
  void set_namespace(const std::string & ns) {namespace_ = ns;}
  void add_participant(const std::string & p) {participants_.push_back(p);}

  // Encode current state as a bid (for testing)
  as2_msgs::msg::Bid get_encoded_bid() const {return encode_bid();}
};

// ── CBBA tests ───────────────────────────────────────────────────────────────

class CBBATest : public ::testing::Test
{
protected:
  TestablePlugin plugin_;
};

// Test 1: No convergence without items
TEST_F(CBBATest, NoConvergenceWithoutItems)
{
  EXPECT_FALSE(plugin_.check_convergence());
}

// Test 2: No convergence without participants
TEST_F(CBBATest, NoConvergenceWithoutParticipants)
{
  plugin_.add_item_with_cost("a", 1.0f);
  EXPECT_FALSE(plugin_.check_convergence());
}

// Test 3: Sole participant converges immediately
TEST_F(CBBATest, SoleParticipantConvergesImmediately)
{
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_item_with_cost("a", 1.0f);
  plugin_.trigger_build_bundle();

  // No peers to wait for, last round caused no releases (changed_ = false)
  EXPECT_TRUE(plugin_.check_convergence());
}

// Test 4: Bundle building selects highest-score task (limit=1)
TEST_F(CBBATest, BundleBuildingSelectsHighestScore)
{
  plugin_.set_bundle_limit(1);
  plugin_.add_item_with_cost("cheap", 1.0f);   // score = -1.0
  plugin_.add_item_with_cost("expensive", 5.0f);  // score = -5.0
  plugin_.trigger_build_bundle();

  ASSERT_EQ(plugin_.bundle().size(), 1u);
  EXPECT_EQ(plugin_.bundle()[0], "cheap");
}

// Test 5: Bundle building respects size limit (limit=2)
TEST_F(CBBATest, BundleBuildingRespectsLimit)
{
  plugin_.set_bundle_limit(2);
  plugin_.add_item_with_cost("task1", 1.0f);
  plugin_.add_item_with_cost("task2", 2.0f);
  plugin_.add_item_with_cost("task3", 3.0f);
  plugin_.trigger_build_bundle();

  EXPECT_EQ(plugin_.bundle().size(), 2u);
  EXPECT_EQ(plugin_.bundle()[0], "task1");  // score = -1.0
  EXPECT_EQ(plugin_.bundle()[1], "task2");  // score = -2.0
}

// Test 6: Consensus update displaces a task (peer score > own score)
TEST_F(CBBATest, ConsensusDisplacesPeerWinsHigherScore)
{
  plugin_.set_namespace("drone0");
  plugin_.set_bundle_limit(2);
  plugin_.add_item_with_cost("t1", 2.0f);  // drone0 score = -2.0
  plugin_.add_item_with_cost("t2", 3.0f);
  plugin_.trigger_build_bundle();

  ASSERT_EQ(plugin_.bundle().size(), 2u);
  EXPECT_EQ(plugin_.z()[0], "drone0");  // drone0 won t1
  EXPECT_TRUE(plugin_.changed() == false);

  // Peer has higher score for t1 (score = -1.0 > -2.0)
  as2_msgs::msg::Bid peer_bid;
  peer_bid.name    = {"t1", "t2"};
  peer_bid.winners = {"drone1", "drone0"};
  peer_bid.amounts = {-1.0, -3.5};  // drone1 score better for t1
  plugin_.update(peer_bid, "drone1");

  EXPECT_EQ(plugin_.z()[0], "drone1");  // drone1 now owns t1
  EXPECT_TRUE(plugin_.changed());  // state changed
}

// Test 7: No displacement when peer score is lower
TEST_F(CBBATest, NoDisplacementPeerScoreLower)
{
  plugin_.set_namespace("drone0");
  plugin_.add_item_with_cost("t1", 2.0f);  // drone0 score = -2.0
  plugin_.trigger_build_bundle();

  ASSERT_EQ(plugin_.z()[0], "drone0");

  // Peer has lower score for t1 (score = -5.0 < -2.0)
  as2_msgs::msg::Bid peer_bid;
  peer_bid.name    = {"t1"};
  peer_bid.winners = {"drone1"};
  peer_bid.amounts = {-5.0};
  plugin_.update(peer_bid, "drone1");

  EXPECT_EQ(plugin_.z()[0], "drone0");  // drone0 still owns t1
  EXPECT_FALSE(plugin_.changed());  // no change
}

// Test 8: Tie-broken by lexicographically smaller agent name
TEST_F(CBBATest, TieBrokenByAgentName)
{
  plugin_.set_namespace("drone1");
  plugin_.add_item_with_cost("t1", 2.0f);  // drone1 score = -2.0
  plugin_.trigger_build_bundle();

  ASSERT_EQ(plugin_.z()[0], "drone1");

  // Peer has exactly same score; "drone0" < "drone1" lexicographically
  as2_msgs::msg::Bid peer_bid;
  peer_bid.name    = {"t1"};
  peer_bid.winners = {"drone0"};
  peer_bid.amounts = {-2.0};  // exactly equal
  plugin_.update(peer_bid, "drone0");

  EXPECT_EQ(plugin_.z()[0], "drone0");  // drone0 wins the tie
  EXPECT_TRUE(plugin_.changed());
}

// Test 9: Convergence requires all peers heard
TEST_F(CBBATest, ConvergenceRequiresAllPeersHeard)
{
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.add_participant("/drone2");
  plugin_.add_item_with_cost("t1", 1.0f);
  plugin_.trigger_build_bundle();

  EXPECT_FALSE(plugin_.check_convergence());  // missing drone1 and drone2

  // drone1 sends a bid
  as2_msgs::msg::Bid bid1;
  bid1.name    = {"t1"};
  bid1.winners = {"drone1"};
  bid1.amounts = {-1.5};
  plugin_.update(bid1, "drone1");
  EXPECT_FALSE(plugin_.check_convergence());  // still missing drone2

  // drone2 sends a bid
  as2_msgs::msg::Bid bid2;
  bid2.name    = {"t1"};
  bid2.winners = {"drone2"};
  bid2.amounts = {-2.0};
  plugin_.update(bid2, "drone2");
  EXPECT_TRUE(plugin_.check_convergence());  // all peers heard
}

// Test 10: Convergence requires changed_ to be false (stable)
TEST_F(CBBATest, ConvergenceRequiresStability)
{
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.add_item_with_cost("t1", 1.0f);
  plugin_.trigger_build_bundle();

  // First update causes a change (peer wins)
  as2_msgs::msg::Bid bid1;
  bid1.name    = {"t1"};
  bid1.winners = {"drone1"};
  bid1.amounts = {-0.5};  // drone1 has better score
  plugin_.update(bid1, "drone1");

  EXPECT_TRUE(plugin_.changed());  // changed_ is true from the update
  EXPECT_FALSE(plugin_.check_convergence());  // can't converge while changed_

  // Second update with same state should not change anything
  as2_msgs::msg::Bid bid2;
  bid2.name    = {"t1"};
  bid2.winners = {"drone1"};
  bid2.amounts = {-0.5};  // same score as before
  plugin_.update(bid2, "drone1");  // again from drone1

  // Since no new displacement, changed_ should be false
  EXPECT_FALSE(plugin_.changed());
  EXPECT_TRUE(plugin_.check_convergence());  // now can converge (all peers heard + stable)
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
