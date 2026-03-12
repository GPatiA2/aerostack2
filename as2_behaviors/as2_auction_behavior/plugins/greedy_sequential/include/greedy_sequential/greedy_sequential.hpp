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
 *  \file       greedy_sequential.hpp
 *  \brief      Greedy sequential auction behavior plugin header
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef GREEDY_SEQUENTIAL__GREEDY_SEQUENTIAL_HPP_
#define GREEDY_SEQUENTIAL__GREEDY_SEQUENTIAL_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "as2_auction_behavior/auction_behavior_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/bid.hpp"

namespace greedy_sequential
{

class Plugin : public as2_auction_behavior::AuctionBehaviorPluginBase
{
  using GoalT = as2_msgs::action::Auction::Goal;
  using FeedbackT = as2_msgs::action::Auction::Feedback;
  using ResultT = as2_msgs::action::Auction::Result;

public:
  Plugin() = default;

  void on_activate(std::shared_ptr<const GoalT> goal) override;
  void on_deactivate() override;
  void on_execution_end() override;

  as2_msgs::msg::Bid compute_bid() override;
  void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) override;
  bool check_convergence() override;

  FeedbackT get_feedback() override;
  ResultT get_result() override;

protected:
  // Tasks claimed by other agents (received via update())
  std::unordered_set<std::string> claimed_by_others_;

  // Tasks claimed by this agent (used to skip in compute_bid())
  std::unordered_set<std::string> my_claim_names_;

  // Ordered record of this agent's claims: (task_name, cost)
  std::vector<std::pair<std::string, double>> my_claims_;

  // Maximum number of tasks this agent will claim
  int bundle_size_{1};

private:
  void reset();
};

}  // namespace greedy_sequential

#endif  // GREEDY_SEQUENTIAL__GREEDY_SEQUENTIAL_HPP_
