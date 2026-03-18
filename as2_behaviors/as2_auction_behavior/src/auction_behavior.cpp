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
 *  \file       auction_behavior.cpp
 *  \brief      auction behavior implementation file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <memory>
#include <string>
#include <vector>

#include "as2_ca/ca_gateway_client.hpp"
#include "as2_auction_behavior/auction_behavior.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "as2_msgs/msg/auction_item_array.hpp"
#include "as2_msgs/msg/bid.hpp"
#include "as2_msgs/msg/start_auction.hpp"

AuctionBehavior::~AuctionBehavior() {}

AuctionBehavior::AuctionBehavior(const rclcpp::NodeOptions & options)
: BehaviorServer("AuctionBehavior", options), client_(this), kb_interface_(this)
{
  started = false;
  behavior_name_ = "auction_behavior";
  RCLCPP_INFO(this->get_logger(), "Starting Auction Behavior Node...");
  try {
    std::string plugin_name_str;
    this->declare_parameter("plugin_name", "greedy_sequential");
    this->get_parameter("plugin_name", plugin_name_);
    plugin_name_ += "::Plugin";
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", e.what());
    this->~AuctionBehavior();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading auction behavior plugin: %s", plugin_name_.c_str());

  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_auction_behavior::AuctionBehaviorPluginBase>>(
    "as2_auction_behavior", "as2_auction_behavior::AuctionBehaviorPluginBase");
  try {
    auction_plugin_ = loader_->createSharedInstance(plugin_name_);
    auction_plugin_->initialize(this, client_);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load auction plugin: %s", e.what());
    this->~AuctionBehavior();
    return;
  }

  item_loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_auction_behavior::AuctionItemPluginBase>>(
    "as2_auction_behavior", "as2_auction_behavior::AuctionItemPluginBase");

  loaded_item_type_ = "";

  self_action_client_ = rclcpp_action::create_client<as2_msgs::action::Auction>(
    this, "AuctionBehavior");

  RCLCPP_INFO(this->get_logger(), "Auction behavior node created!");
}

void AuctionBehavior::configure()
{
// Need to use lambdas because bind needs to solve the exact function pointer at compile time,
// and the plugin functions are virtual and will be resolved at runtime
  RCLCPP_INFO(this->get_logger(), "Registering StartAuction handler...");
  client_.register_module<as2_msgs::msg::StartAuction>(
    "auction_item_array",
    behavior_name_,
    [this](const as2_msgs::msg::StartAuction & msg,
    const std::string & agent_id) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received StartAuction from auctioneer '%s' with %zu items and %zu participants",
        agent_id.c_str(), msg.items.list.size(), msg.participants.size());

      // Load plugin and items synchronously so they are ready before any bid arrives.
      // The self-goal below is async and on_activate may run after the first bid.
      const as2_msgs::msg::AuctionItemArray & elements = msg.items;
      if (loaded_item_type_ != elements.item_type) {
        try {
          item_plugin_ = item_loader_->createSharedInstance(elements.item_type + "::Plugin");
          auction_plugin_->set_item_plugin(item_plugin_);
          loaded_item_type_ = elements.item_type;
        } catch (const pluginlib::PluginlibException & e) {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to load item plugin '%s': %s",
            elements.item_type.c_str(), e.what());
          return;
        }
      }
      auction_plugin_->set_participans(msg.participants);
      auction_plugin_->on_auction_items_received(elements, agent_id);

      // Send a self-goal to activate the run timer so on_run() ticks for convergence detection.
      GoalT participant_goal;
      participant_goal.type     = elements.item_type;
      participant_goal.elements = elements.list;
      participant_goal.bidders  = msg.participants;

      is_participant_ = true;
      self_action_client_->async_send_goal(participant_goal);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Registering Bid handler...");
  client_.register_module<as2_msgs::msg::Bid>(
    "bid",
    behavior_name_,
    [this](const as2_msgs::msg::Bid & msg,
    const std::string & agent_id) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received bid from agent '%s' claiming %zu task(s)",
        agent_id.c_str(), msg.name.size());
      auction_plugin_->on_bid_received(msg, agent_id);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Configuring plugin");
  auction_plugin_->configure(this);
  RCLCPP_INFO(this->get_logger(), "Finished configuring plugin");
}
bool AuctionBehavior::on_activate(std::shared_ptr<const GoalT> goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Activating as %s with type '%s', %zu bidders",
    is_participant_ ? "participant" : "auctioneer", goal->type.c_str(), goal->bidders.size());

  std::string item_type = goal->type;

  // Load item plugin (same for both roles)
  if (loaded_item_type_ != item_type) {
    try {
      item_plugin_ = item_loader_->createSharedInstance(item_type + "::Plugin");
      auction_plugin_->set_item_plugin(item_plugin_);
      loaded_item_type_ = item_type;
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to load item plugin '%s': %s",
        item_type.c_str(), e.what());
      is_participant_ = false;
      return false;
    }
  }

  if (!is_participant_) {
    // Participants already loaded items/participants synchronously in the StartAuction handler.
    // Auctioneer loads them here, then broadcasts StartAuction and kicks off first bid.
    auction_plugin_->set_participans(goal->bidders);
    as2_msgs::msg::AuctionItemArray items_msg;
    items_msg.list = goal->elements;
    items_msg.item_type = item_type;
    auction_plugin_->on_auction_items_received(items_msg, this->get_namespace());

    as2_msgs::msg::StartAuction start_msg;
    start_msg.participants = goal->bidders;
    start_msg.items = items_msg;
    start_msg.itemtype = item_type;
    std::vector<std::string> other_bidders;
    const std::string my_ns = this->get_namespace();
    for (const auto & b : goal->bidders) {
      if (b != my_ns) {
        other_bidders.push_back(b);
      }
    }
    RCLCPP_INFO(
      this->get_logger(), "Forwarding StartAuction to %zu other bidders",
      other_bidders.size());
    client_.forward_IA_msg<as2_msgs::msg::StartAuction>(
      start_msg, "auction_item_array", other_bidders);
    auction_plugin_->on_activate(goal);
  }

  is_participant_ = false;
  goal_ = *goal;
  started = true;
  return true;
}

bool AuctionBehavior::on_modify(std::shared_ptr<const GoalT> goal)
{
  return true;
}

bool AuctionBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  auction_plugin_->on_deactivate();
  started = false;
  return true;
}

bool AuctionBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return true;
}

bool AuctionBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return true;
}

as2_behavior::ExecutionStatus AuctionBehavior::on_run(
  const std::shared_ptr<const GoalT> & goal,
  std::shared_ptr<FeedbackT> & feedback_msg,
  std::shared_ptr<ResultT> & result_msg)
{
  if (!started) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  if (!auction_plugin_->check_convergence()) {
    FeedbackT fb = auction_plugin_->get_feedback();
    feedback_msg->asignees = fb.asignees;
    feedback_msg->items = fb.items;
    feedback_msg->amounts = fb.amounts;
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  ResultT res = auction_plugin_->get_result();
  result_msg->winners = res.winners;
  result_msg->elements = res.elements;
  result_ = res;
  started = false;
  return as2_behavior::ExecutionStatus::SUCCESS;
}

void AuctionBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  auction_plugin_->on_execution_end();
  if (state == as2_behavior::ExecutionStatus::SUCCESS) {
    publish_results_to_kb(result_);
  }
}

void AuctionBehavior::publish_results_to_kb(const ResultT & result)
{
  for (size_t i = 0; i < result.elements.size(); ++i) {
    const auto & item = result.elements[i];
    const std::string & agent_id = result.winners[i];
    const std::string & id_point = item.name;

    // Find x and y coordinates from the item features
    std::string x_str, y_str;
    for (size_t j = 0; j < item.feature_names.size(); ++j) {
      if (item.feature_names[j] == "x") {
        x_str = std::to_string(item.features[j]);
      } else if (item.feature_names[j] == "y") {
        y_str = std::to_string(item.features[j]);
      }
    }

    kb_interface_.add_fact(id_point, "xCoord", x_str);
    kb_interface_.add_fact(id_point, "yCoord", y_str);
    kb_interface_.add_fact(id_point, "assignedTo", agent_id);
  }
}
