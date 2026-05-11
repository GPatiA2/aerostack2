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
 *  \file       cbba.hpp
 *  \brief      Consensus-Based Bundle Algorithm (CBBA) plugin.
 *
 *  Protocol:
 *    Phase 1 — Bundle Building: Each agent greedily builds a bundle of tasks up to
 *              bundle_size_limit, broadcasting (y, z) tables after each claim.
 *    Phase 2 — Consensus: Agents accept stronger claims from peers, release displaced
 *              tasks, and rebuild bundles. Iterates until convergence.
 *
 *  Convergence: All N-1 peers have sent at least one bid AND the last round caused no
 *               task releases (stable state).
 *
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef CBBA__CBBA_HPP_
#define CBBA__CBBA_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "as2_auction_behavior/auction_behavior_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/bid.hpp"

namespace cbba
{

class Plugin : public as2_auction_behavior::AuctionBehaviorPluginBase
{
  using GoalT = as2_msgs::action::Auction::Goal;
  using FeedbackT = as2_msgs::action::Auction::Feedback;
  using ResultT = as2_msgs::action::Auction::Result;

public:
  Plugin() = default;

  // Initialization: compute personal scores and start bundle building.
  void on_auction_items_received(
    const as2_msgs::msg::AuctionItemArray & msg,
    const std::string & agent_id) override;

  // Activate: prepare state structures and load configuration parameters.
  void on_activate(std::shared_ptr<const GoalT> goal) override;
  void on_deactivate() override;
  void on_execution_end() override;

  // Compute and return the next bid if the bundle has changed since last send.
  as2_msgs::msg::Bid compute_bid() override;

  // Handle incoming bid: decode (y, z) tables and run consensus.
  void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) override;

  // True when all peers have sent at least one bid and last round caused no releases.
  bool check_convergence() override;

  // Periodic task: retransmit bid if timeout has elapsed.
  void on_run() override;

  FeedbackT get_feedback() override;
  ResultT get_result() override;
  std::map<std::string, std::string> get_global_assignment() const override;

protected:
  std::vector<std::string> item_names_;       // all task names
  std::map<std::string, double> my_scores_;   // static personal score (fallback when no coords)
  // XY coordinates of each task; populated when items carry 2D features.
  std::map<std::string, std::pair<double, double>> item_coords_;
  std::vector<double> y_;                     // best winning score per item
  std::vector<std::string> z_;                // winning agent per item
  std::vector<std::string> bundle_;           // ordered list of tasks claimed by this agent
  std::unordered_set<std::string> in_bundle_;  // quick membership check
  std::unordered_set<std::string> received_from_;  // peers from which we've heard
  int bundle_size_limit_;                     // max tasks per agent
  bool changed_;                              // true if bundle changed since last send
  std::map<std::string, std::string> global_assignment_;  // final item → agent mapping
  double retransmit_timeout_s_;               // timeout before retransmitting stale bids
  double last_send_time_;                     // timestamp of last broadcast

  // Phase 1: greedily add tasks to bundle; scores are marginal best-insertion costs
  // into the agent's current path (Choi et al. 2009, CBBA paper).
  void build_bundle();

  // Remove task at index and all tasks added after it; mark as changed.
  void release_from(int index);

  // Rebuild global_assignment_ from current y_ and z_ vectors.
  void rebuild_global_assignment();

  // Encode y_ and z_ vectors into Bid.name and Bid.amounts for transmission.
  as2_msgs::msg::Bid encode_bid() const;

  // Reset internal state for a new auction.
  void reset();

  // Compute marginal best-insertion score for task_name into path.
  // Returns score = -(min insertion cost). Also sets best_insert_pos to the
  // path index after which the task should be inserted.
  // Falls back to my_scores_ when coordinates for task_name are unavailable.
  double compute_marginal_score(
    const std::string & task_name,
    const std::vector<std::pair<double, double>> & path,
    size_t & best_insert_pos) const;
};

}  // namespace cbba

#endif  // CBBA__CBBA_HPP_
