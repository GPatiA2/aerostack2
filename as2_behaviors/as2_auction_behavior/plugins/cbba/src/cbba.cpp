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
 *  \file       cbba.cpp
 *  \brief      Consensus-Based Bundle Algorithm (CBBA) plugin implementation
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "cbba/cbba.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <utility>
#include <unordered_set>
#include <map>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <as2_core/names/topics.hpp>

namespace cbba
{

// ── private helpers ─────────────────────────────────────────────────────────

void Plugin::reset()
{
  item_names_.clear();
  my_scores_.clear();
  item_coords_.clear();
  y_.clear();
  z_.clear();
  bundle_.clear();
  in_bundle_.clear();
  received_from_.clear();
  changed_ = false;
  global_assignment_.clear();
  last_send_time_ = 0.0;
}

static double euclidean(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  const double dx = a.first - b.first;
  const double dy = a.second - b.second;
  return std::sqrt(dx * dx + dy * dy);
}

double Plugin::compute_marginal_score(
  const std::string & task_name,
  const std::vector<std::pair<double, double>> & path,
  size_t & best_insert_pos) const
{
  auto coords_it = item_coords_.find(task_name);
  if (coords_it == item_coords_.end() || path.empty()) {
    // No coordinates: fall back to static score (used in unit tests)
    best_insert_pos = path.empty() ? 0 : path.size() - 1;
    auto score_it = my_scores_.find(task_name);
    return score_it != my_scores_.end()
      ? score_it->second
      : -std::numeric_limits<double>::infinity();
  }

  const auto & task_pos = coords_it->second;

  // Default: append at end of path
  double best_cost = euclidean(path.back(), task_pos);
  best_insert_pos = path.size() - 1;

  // Try inserting between every pair of consecutive path points
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    double cost = euclidean(path[i], task_pos)
      + euclidean(task_pos, path[i + 1])
      - euclidean(path[i], path[i + 1]);
    if (cost < best_cost) {
      best_cost = cost;
      best_insert_pos = i;
    }
  }

  return -best_cost;
}

void Plugin::rebuild_global_assignment()
{
  global_assignment_.clear();
  for (size_t j = 0; j < item_names_.size(); ++j) {
    global_assignment_[item_names_[j]] = z_[j];
  }
}

as2_msgs::msg::Bid Plugin::encode_bid() const
{
  as2_msgs::msg::Bid bid;
  for (const auto & name : item_names_) {
    bid.name.push_back(name);
  }
  for (const auto & winner : z_) {
    bid.winners.push_back(winner);
  }
  for (const auto & score : y_) {
    bid.amounts.push_back(score);
  }

  std::string bundle_str;
  for (const auto & task : bundle_) {
    for (size_t j = 0; j < item_names_.size(); ++j) {
      if (item_names_[j] == task) {
        bundle_str += task + "(score=" + std::to_string(y_[j]) + ") ";
        break;
      }
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("cbba"),
    "[%s] sending bid — bundle: [%s]",
    namespace_.c_str(), bundle_str.empty() ? "<empty>" : bundle_str.c_str());

  return bid;
}

void Plugin::build_bundle()
{
  const size_t effective_limit = (bundle_size_limit_ <= 0)
    ? item_names_.size()
    : static_cast<size_t>(bundle_size_limit_);

  // Build the agent's current path: drone start position + already-claimed task coords.
  // When 2D coordinates are available we use marginal best-insertion scoring (Choi 2009);
  // otherwise we fall back to the static my_scores_ table (preserves unit-test behaviour).
  std::vector<std::pair<double, double>> path;
  bool use_path = !item_coords_.empty();

  if (use_path) {
    try {
      const auto pose = state_interface_.get_value<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::self_localization::pose);
      path.push_back({pose.pose.position.x, pose.pose.position.y});
      for (const auto & task : bundle_) {
        auto it = item_coords_.find(task);
        if (it != item_coords_.end()) {
          path.push_back(it->second);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("cbba"),
        "Cannot get drone pose for path scoring (%s); using static scores", e.what());
      path.clear();
      use_path = false;
    }
  }

  while (bundle_.size() < effective_limit) {
    int best_task = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    size_t best_insert_pos = 0;

    for (size_t j = 0; j < item_names_.size(); ++j) {
      if (in_bundle_.count(item_names_[j])) {
        continue;
      }

      size_t insert_pos = 0;
      double score;
      if (use_path) {
        score = compute_marginal_score(item_names_[j], path, insert_pos);
      } else {
        auto it = my_scores_.find(item_names_[j]);
        score = it != my_scores_.end() ? it->second : -std::numeric_limits<double>::infinity();
      }

      if (score > y_[j] && score > best_score) {
        best_score = score;
        best_task = static_cast<int>(j);
        best_insert_pos = insert_pos;
      }
    }

    if (best_task == -1) {
      break;
    }

    const std::string & task_name = item_names_[best_task];
    bundle_.push_back(task_name);
    in_bundle_.insert(task_name);
    y_[best_task] = best_score;
    z_[best_task] = namespace_;

    // Extend path at the best insertion position for the next iteration
    if (use_path) {
      auto it = item_coords_.find(task_name);
      if (it != item_coords_.end()) {
        path.insert(path.begin() + static_cast<int>(best_insert_pos) + 1, it->second);
      }
    }
  }
}

void Plugin::release_from(int bundle_pos)
{
  if (bundle_pos < 0 || bundle_pos >= static_cast<int>(bundle_.size())) {
    return;
  }

  std::vector<std::string> released_names;
  for (int i = bundle_pos; i < static_cast<int>(bundle_.size()); ++i) {
    released_names.push_back(bundle_[i]);
  }

  bundle_.erase(bundle_.begin() + bundle_pos, bundle_.end());
  for (const auto & name : released_names) {
    in_bundle_.erase(name);
    // Find the index and reset if we were the winner
    for (size_t j = 0; j < item_names_.size(); ++j) {
      if (item_names_[j] == name && z_[j] == namespace_) {
        y_[j] = -std::numeric_limits<double>::infinity();
        z_[j] = "";
        break;
      }
    }
  }

  changed_ = true;
}

// ── lifecycle ────────────────────────────────────────────────────────────────

void Plugin::on_auction_items_received(
  const as2_msgs::msg::AuctionItemArray & msg,
  const std::string & agent_id)
{
  AuctionBehaviorPluginBase::on_auction_items_received(msg, agent_id);

  // Read configuration parameters (declare only on first auction)
  if (!node_ptr_->has_parameter("bundle_size_limit")) {
    node_ptr_->declare_parameter<int>("bundle_size_limit", 0);
  }
  if (!node_ptr_->has_parameter("retransmit_timeout_s")) {
    node_ptr_->declare_parameter<double>("retransmit_timeout_s", 2.0);
  }
  bundle_size_limit_ = node_ptr_->get_parameter("bundle_size_limit").as_int();
  retransmit_timeout_s_ = node_ptr_->get_parameter("retransmit_timeout_s").as_double();

  item_names_.clear();
  my_scores_.clear();
  item_coords_.clear();
  y_.clear();
  z_.clear();
  bundle_.clear();
  in_bundle_.clear();
  received_from_.clear();
  changed_ = false;

  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    const double cost = static_cast<double>(item->evaluate(state_interface_));
    const double score = -cost;

    item_names_.push_back(name);
    my_scores_[name] = score;
    y_.push_back(-std::numeric_limits<double>::infinity());
    z_.push_back("");

    // Extract 2D coordinates when available (features[0]=x, features[1]=y)
    const auto & raw = item->get_item();
    if (raw.features.size() >= 2) {
      item_coords_[name] = {raw.features[0], raw.features[1]};
    }
  }

  build_bundle();
  send_bid(encode_bid());
  last_send_time_ = node_ptr_->now().seconds();
  changed_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("cbba"),
    "Auction items received: %zu items, initial bundle size: %zu",
    item_names_.size(), bundle_.size());
}

void Plugin::on_activate(std::shared_ptr<const GoalT>/*goal*/)
{
  // Parameters are loaded in on_auction_items_received when needed.
}

void Plugin::on_deactivate() {}

void Plugin::on_execution_end()
{
  reset();
}

as2_msgs::msg::Bid Plugin::compute_bid()
{
  if (!changed_) {
    return as2_msgs::msg::Bid{};
  }
  changed_ = false;
  last_send_time_ = node_ptr_->now().seconds();
  return encode_bid();
}

void Plugin::update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id)
{
  if (agent_id == namespace_) {
    return;
  }

  // Validate CBBA bid format before counting this peer as heard-from.
  // An invalid bid (wrong sizes, e.g. arriving before items are loaded) must not
  // mark the peer as heard-from, otherwise check_convergence() could fire before
  // the peer's actual consensus data has been incorporated.
  size_t n = item_names_.size();
  if (bid_msg.name.size() != n || bid_msg.winners.size() != n || bid_msg.amounts.size() != n) {
    // n==0 means items not loaded yet — bid arrived before StartAuction, expected race on startup.
    if (n > 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("cbba"),
        "Invalid CBBA bid from %s: expected %zu items, got name.size()=%zu, winners.size()=%zu, amounts.size()=%zu",
        agent_id.c_str(), n, bid_msg.name.size(), bid_msg.winners.size(), bid_msg.amounts.size());
    }
    return;
  }

  received_from_.insert(agent_id);

  // Build name-to-index lookup from peer's item names
  std::map<std::string, size_t> peer_name_to_idx;
  for (size_t k = 0; k < n; ++k) {
    peer_name_to_idx[bid_msg.name[k]] = k;
  }

  changed_ = false;

  // Apply consensus: for each local task j, look up peer's claim and update if stronger
  for (size_t j = 0; j < n; ++j) {
    const auto & task_name = item_names_[j];
    auto it = peer_name_to_idx.find(task_name);
    if (it == peer_name_to_idx.end()) {
      continue;  // Peer doesn't have this task
    }

    size_t peer_idx = it->second;
    const double incoming_score = bid_msg.amounts[peer_idx];
    const std::string & incoming_winner = bid_msg.winners[peer_idx];

    // A bid with no winner means the peer has released this task — never propagate "nobody".
    // Accept if incoming score is higher, or equal score with lex-smaller (non-empty) agent name.
    bool peer_wins =
      !incoming_winner.empty() &&
      ((incoming_score > y_[j]) ||
       (incoming_score == y_[j] && incoming_winner < z_[j]));

    if (peer_wins) {
      // Check if we lose a task from our bundle
      if (z_[j] == namespace_ && in_bundle_.count(task_name)) {
        auto bundle_it = std::find(bundle_.begin(), bundle_.end(), task_name);
        if (bundle_it != bundle_.end()) {
          int pos = std::distance(bundle_.begin(), bundle_it);
          release_from(pos);
          changed_ = true;
        }
      }

      y_[j] = incoming_score;
      z_[j] = incoming_winner;
    }
  }

  // Rebuild bundle if we lost items and freed up slots
  if (changed_) {
    build_bundle();
  }
}

bool Plugin::check_convergence()
{
  if (item_names_.empty() || participants_.empty()) {
    return false;
  }

  // All N-1 peers must have sent at least one bid
  for (const auto & p : participants_) {
    const std::string p_id = (!p.empty() && p[0] == '/') ? p.substr(1) : p;
    if (p_id == namespace_) {
      continue;
    }
    if (!received_from_.count(p_id)) {
      return false;
    }
  }

  // Last round must have caused no task releases (stable state)
  return !changed_;
}

void Plugin::on_run()
{
  // Check if it's time to retransmit stale bids
  double current_time = node_ptr_->now().seconds();
  if (current_time - last_send_time_ >= retransmit_timeout_s_) {
    RCLCPP_WARN(
      rclcpp::get_logger("cbba"),
      "Retransmitting bid after %.1f seconds of inactivity",
      current_time - last_send_time_);
    send_bid(encode_bid());
    last_send_time_ = current_time;
  }
}

Plugin::FeedbackT Plugin::get_feedback()
{
  FeedbackT feedback;
  for (const auto & task_name : bundle_) {
    feedback.asignees.push_back(namespace_);
    for (size_t j = 0; j < item_names_.size(); ++j) {
      if (item_names_[j] == task_name) {
        feedback.amounts.push_back(y_[j]);
        break;
      }
    }
    for (const auto & item : auction_items_) {
      if (item->get_name() == task_name) {
        feedback.items.push_back(item->get_item());
        break;
      }
    }
  }
  return feedback;
}

Plugin::ResultT Plugin::get_result()
{
  rebuild_global_assignment();
  ResultT result;
  for (const auto & item_ptr : auction_items_) {
    const std::string & item_name = item_ptr->get_name();
    result.elements.push_back(item_ptr->get_item());
    auto it = global_assignment_.find(item_name);
    result.winners.push_back((it != global_assignment_.end()) ? it->second : "");
  }
  return result;
}

std::map<std::string, std::string> Plugin::get_global_assignment() const
{
  return global_assignment_;
}

}  // namespace cbba

PLUGINLIB_EXPORT_CLASS(cbba::Plugin, as2_auction_behavior::AuctionBehaviorPluginBase)
