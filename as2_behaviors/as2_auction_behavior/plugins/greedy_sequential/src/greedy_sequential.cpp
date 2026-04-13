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
 *  \brief      Greedy broadcast auction plugin implementation
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "greedy_sequential/greedy_sequential.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace greedy_sequential
{

// ── conflict resolution ──────────────────────────────────────────────────────

void Plugin::solve_conflicts()
{
  // Merge own costs and every received peer cost vector into a per-item table.
  std::map<std::string, std::map<std::string, double>> all_costs;
  for (const auto & [item, cost] : my_costs_) {
    all_costs[item][namespace_] = cost;
  }
  for (const auto & [agent, bids] : received_bids_) {
    for (const auto & [item, cost] : bids) {
      all_costs[item][agent] = cost;
    }
  }

  // For each item, the agent with the lowest cost wins.
  // Tie-break: lexicographically smallest agent name.
  // std::map iterates in sorted key order, so item processing is deterministic
  // across all drones — every drone with identical bid data reaches the same result.
  std::vector<std::pair<std::string, double>> result;
  for (const auto & [item, agent_costs] : all_costs) {
    std::string winner;
    double best = std::numeric_limits<double>::max();
    for (const auto & [agent, cost] : agent_costs) {
      if (cost < best || (cost == best && agent < winner)) {
        winner = agent;
        best = cost;
      }
    }
    if (winner == namespace_) {
      result.emplace_back(item, best);
    }
  }
  my_assignment_ = std::move(result);
}

// ── lifecycle ────────────────────────────────────────────────────────────────

void Plugin::on_auction_items_received(
  const as2_msgs::msg::AuctionItemArray & msg,
  const std::string & agent_id)
{
  AuctionBehaviorPluginBase::on_auction_items_received(msg, agent_id);

  // Compute this agent's cost for every item.
  as2_msgs::msg::Bid bid;
  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    const double cost = static_cast<double>(item->evaluate(state_interface_));
    my_costs_[name] = cost;
    bid.name.push_back(name);
    bid.amounts.push_back(cost);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("greedy_sequential"),
    "Broadcasting bid for %zu items", bid.name.size());

  // Broadcast to all participants (includes self, filtered by CA gateway).
  send_bid(bid);

  // Initial assignment based on own data only; updated as peer bids arrive.
  solve_conflicts();

  // Handle sole-participant case: if there are no peers, convergence is immediate.
  check_convergence();
}

void Plugin::on_activate(std::shared_ptr<const GoalT>/*goal*/)
{
  // Bid already sent in on_auction_items_received — nothing to do here.
}

void Plugin::on_deactivate() {reset();}
void Plugin::on_execution_end() {reset();}

void Plugin::reset()
{
  my_costs_.clear();
  received_bids_.clear();
  received_from_.clear();
  my_assignment_.clear();
  auction_items_.clear();
}

// ── bidding loop ─────────────────────────────────────────────────────────────

as2_msgs::msg::Bid Plugin::compute_bid()
{
  // Bids are sent once in on_auction_items_received, not reactively.
  // Return empty so the base class on_bid_received() chain is a no-op.
  return as2_msgs::msg::Bid{};
}

void Plugin::update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id)
{
  if (agent_id == namespace_) {return;}

  received_from_.insert(agent_id);

  auto & stored = received_bids_[agent_id];
  for (size_t i = 0; i < bid_msg.name.size(); ++i) {
    stored[bid_msg.name[i]] =
      (i < bid_msg.amounts.size()) ? bid_msg.amounts[i] :
      std::numeric_limits<double>::max();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("greedy_sequential"),
    "Received bid from '%s' — %zu/%zu peers",
    agent_id.c_str(), received_from_.size(),
    participants_.empty() ? 0u : participants_.size() - 1);

  solve_conflicts();
}

bool Plugin::check_convergence()
{
  if (auction_items_.empty() || participants_.empty()) {return false;}

  for (const auto & p : participants_) {
    const std::string p_id = (!p.empty() && p[0] == '/') ? p.substr(1) : p;
    if (p_id == namespace_) {continue;}
    if (!received_from_.count(p_id)) {return false;}
  }
  return true;
}

// ── result reporting ─────────────────────────────────────────────────────────

Plugin::FeedbackT Plugin::get_feedback()
{
  FeedbackT feedback;
  for (const auto & [name, cost] : my_assignment_) {
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
  for (const auto & [name, cost] : my_assignment_) {
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

}  // namespace greedy_sequential

PLUGINLIB_EXPORT_CLASS(greedy_sequential::Plugin, as2_auction_behavior::AuctionBehaviorPluginBase)
