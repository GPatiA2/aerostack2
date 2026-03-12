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

#include <limits>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace greedy_sequential
{

void Plugin::on_activate(std::shared_ptr<const GoalT> goal)
{
  // auction_items_ and participants_ are already set by on_modify before this call.
  // Compute and broadcast the first bid to kick off the bidding chain.
  as2_msgs::msg::Bid first_bid = compute_bid();
  if (!first_bid.name.empty()) {
    client_.forward_IA_msg<as2_msgs::msg::Bid>(first_bid, "bid", participants_);
  }
}

void Plugin::on_deactivate()
{
  reset();
}

void Plugin::on_execution_end()
{
  reset();
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
  for (const auto & name : bid_msg.name) {
    if (!my_claim_names_.count(name)) {
      claimed_by_others_.insert(name);
      RCLCPP_INFO(
        rclcpp::get_logger("greedy_sequential"),
        "Task '%s' claimed by '%s'", name.c_str(), agent_id.c_str());
    }
  }
}

bool Plugin::check_convergence()
{
  if (auction_items_.empty()) {
    return false;
  }
  const size_t total_claimed = claimed_by_others_.size() + my_claim_names_.size();
  return (total_claimed >= auction_items_.size()) ||
         (static_cast<int>(my_claims_.size()) >= bundle_size_);
}

Plugin::FeedbackT Plugin::get_feedback()
{
  FeedbackT feedback;
  for (const auto & [name, cost] : my_claims_) {
    feedback.asignees.push_back("self");
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
    result.winners.push_back("self");
    for (const auto & item : auction_items_) {
      if (item->get_name() == name) {
        result.elements.push_back(item->get_item());
        break;
      }
    }
  }
  return result;
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
