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
 *  \file       greedy_sequential.cpp
 *  \brief      Greedy sequential auction behavior plugin implementation
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "greedy_sequential/greedy_sequential.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace greedy_sequential
{

void Plugin::on_activate(std::shared_ptr<const GoalT> goal)
{
  // auction_items_ and participants_ are already set before this call.
  // Compute and broadcast the first bid to kick off the bidding chain.
  last_activity_time_ = std::chrono::steady_clock::now();
  send_bid(compute_bid());
  last_bid_time_ = std::chrono::steady_clock::now();
}

void Plugin::on_deactivate()
{
  reset();
}

void Plugin::on_execution_end()
{
  reset();
}

void Plugin::configure(rclcpp::Node * node)
{
  AuctionBehaviorPluginBase::configure(node);
  node->declare_parameter("bundle_size", 1);
  node->get_parameter("bundle_size", bundle_size_);
  node->declare_parameter("retransmit_timeout_s", retransmit_timeout_s_);
  node->get_parameter("retransmit_timeout_s", retransmit_timeout_s_);
  RCLCPP_INFO(
    rclcpp::get_logger("greedy_sequential"),
    "Configured with bundle_size=%d, retransmit_timeout_s=%.1f",
    bundle_size_, retransmit_timeout_s_);
}

as2_msgs::msg::Bid Plugin::compute_bid()
{
  as2_msgs::msg::Bid bid;

  if (static_cast<int>(my_claims_.size()) >= bundle_size_) {
    return bid;  // bundle full
  }

  double best_cost = std::numeric_limits<double>::max();
  std::string best_name;

  for (const auto & item : auction_items_) {
    RCLCPP_INFO(
      rclcpp::get_logger("greedy_sequential"),
      "Evaluating item '%s'", item->to_string().c_str());
    const std::string & name = item->get_name();
    if (claimed_by_others_.count(name) || my_claim_names_.count(name)) {
      continue;
    }
    double cost = static_cast<double>(item->evaluate(state_interface_));
    if (cost < best_cost) {
      best_cost = cost;
      best_name = name;
    }
  }

  if (!best_name.empty()) {
    bid.name.push_back(best_name);
    bid.amounts.push_back(best_cost);
    my_claim_names_.insert(best_name);
    my_claims_.emplace_back(best_name, best_cost);
    RCLCPP_INFO(
      rclcpp::get_logger("greedy_sequential"),
      "Claiming task '%s' with cost %.3f", best_name.c_str(), best_cost);
  }

  return bid;
}

void Plugin::update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id)
{
  // A bid just arrived: reset the stability clock. This is the only place
  // last_activity_time_ is touched; outgoing retransmits must not move it.
  last_activity_time_ = std::chrono::steady_clock::now();
  bool won_conflict = false;
  for (size_t i = 0; i < bid_msg.name.size(); ++i) {
    const std::string & name = bid_msg.name[i];

    if (!my_claim_names_.count(name)) {
      // No conflict — mark as claimed by the other agent.
      claimed_by_others_.insert(name);
      RCLCPP_INFO(
        rclcpp::get_logger("greedy_sequential"),
        "Task '%s' claimed by '%s'", name.c_str(), agent_id.c_str());
    } else {
      // Conflict: simultaneous bids on the same task.
      // Greedy sequential is first-come-first-serve: resolve by namespace
      // lexicographic order as a deterministic proxy for sequential turn order
      // (drone0 → drone1 → … → drone4).
      if (agent_id < namespace_) {
        // Other agent goes earlier in the sequence — yield.
        my_claim_names_.erase(name);
        my_claims_.erase(
          std::remove_if(
            my_claims_.begin(), my_claims_.end(),
            [&name](const auto & p) {return p.first == name;}),
          my_claims_.end());
        claimed_by_others_.insert(name);
        RCLCPP_INFO(
          rclcpp::get_logger("greedy_sequential"),
          "Task '%s' yielded to '%s' (FCFS: they go first in sequence)",
          name.c_str(), agent_id.c_str());
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("greedy_sequential"),
          "Task '%s' contested by '%s' — keeping claim (we go first in sequence)",
          name.c_str(), agent_id.c_str());
        won_conflict = true;
      }
    }
  }

  // The loser drops its duplicate claim only when it receives our bid.
  // compute_bid() returns empty when the bundle is full, so on_bid_received()
  // sends nothing through the normal chain.  If we converge before the
  // retransmit timeout fires, on_run() is never called again and the loser
  // keeps a stale duplicate.  Fix: broadcast all current claims immediately
  // so the conflict is resolved before check_convergence() can return true.
  if (won_conflict && !my_claims_.empty() && !participants_.empty()) {
    as2_msgs::msg::Bid full_bid;
    for (const auto & [n, cost] : my_claims_) {
      full_bid.name.push_back(n);
      full_bid.amounts.push_back(cost);
    }
    RCLCPP_INFO(
      rclcpp::get_logger("greedy_sequential"),
      "Won conflict — retransmitting all %zu claim(s) to resolve",
      full_bid.name.size());
    client_.forward_IA_msg<as2_msgs::msg::Bid>(full_bid, "bid", participants_);
    last_bid_time_ = std::chrono::steady_clock::now();
    // last_activity_time_ intentionally not updated; see check_convergence().
  }
}

bool Plugin::check_convergence()
{
  if (auction_items_.empty() || participants_.empty()) {
    return false;
  }

  // Two ways this drone can be done:
  //   1. Bundle is full — we claimed our target number of tasks.
  //   2. All items are accounted for — between our claims and what peers claimed,
  //      every item in the auction has an owner, so there is nothing left to bid on.
  //
  // We use a per-drone criterion rather than a global one (n_drones × bundle_size).
  // A global target would require knowing every peer's final claim count, but early
  // convergers call reset() and go silent. A late drone whose claimed_by_others_ is
  // incomplete from missed messages could then never reach the target and spin forever.
  const bool bundle_full =
    (static_cast<int>(my_claim_names_.size()) >= bundle_size_);
  const bool all_accounted =
    (claimed_by_others_.size() + my_claim_names_.size() >= auction_items_.size());

  if (!bundle_full && !all_accounted) {
    return false;
  }

  // Even though our local work is done, peers may still be resolving conflicts
  // and sending retransmits. We wait until no bid has been received for
  // stability_wait_s_ before declaring convergence.
  //
  // last_activity_time_ is only updated when a bid arrives (in update()), never
  // when we send or retransmit. If we also reset it on outgoing messages, a drone
  // that keeps retransmitting into silence (all peers already reset) would push the
  // clock forward indefinitely and never exit this wait.
  const double idle = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_activity_time_).count();
  if (idle < stability_wait_s_) {
    RCLCPP_INFO(
      rclcpp::get_logger("greedy_sequential"),
      "My work done (%zu claims, %zu others) — waiting for stability (%.2fs/%.2fs)",
      my_claim_names_.size(), claimed_by_others_.size(), idle, stability_wait_s_);
    return false;
  }
  return true;
}

Plugin::FeedbackT Plugin::get_feedback()
{
  FeedbackT feedback;
  for (const auto & [name, cost] : my_claims_) {
    feedback.asignees.push_back(namespace_);
    feedback.amounts.push_back(cost);
    for (const auto & item : auction_items_) {
      if (item->get_name() == name) {
        feedback.items.push_back(item->get_item());
        break;
      }
    }
  }
  return feedback;
}

Plugin::ResultT Plugin::get_result()
{
  ResultT result;
  for (const auto & [name, cost] : my_claims_) {
    result.winners.push_back(namespace_);
    for (const auto & item : auction_items_) {
      if (item->get_name() == name) {
        result.elements.push_back(item->get_item());
        break;
      }
    }
  }
  return result;
}

void Plugin::on_run()
{
  if (my_claims_.empty() || participants_.empty()) {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_bid_time_).count();
  if (elapsed <= retransmit_timeout_s_) {
    return;
  }

  // Build a single bid containing ALL current claims so that any peer that
  // missed an earlier individual bid can fill its claimed_by_others_ set and
  // eventually reach convergence.
  as2_msgs::msg::Bid full_bid;
  for (const auto & [name, cost] : my_claims_) {
    full_bid.name.push_back(name);
    full_bid.amounts.push_back(cost);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("greedy_sequential"),
    "Retransmitting all %zu claim(s) to %zu participant(s)",
    full_bid.name.size(), participants_.size());
  client_.forward_IA_msg<as2_msgs::msg::Bid>(full_bid, "bid", participants_);
  last_bid_time_ = std::chrono::steady_clock::now();
  // last_activity_time_ intentionally not updated — see check_convergence().
}

void Plugin::reset()
{
  claimed_by_others_.clear();
  my_claim_names_.clear();
  my_claims_.clear();
  auction_items_.clear();
}

}  // namespace greedy_sequential

PLUGINLIB_EXPORT_CLASS(greedy_sequential::Plugin, as2_auction_behavior::AuctionBehaviorPluginBase)
