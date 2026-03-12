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
 *  \file       ca_gateway_client.hpp
 *  \brief      Ca_gateway_client implementation file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "as2_ca/ca_gateway_client.hpp"
#include <string>
#include <memory>
#include <vector>

using std::placeholders::_1;

namespace as2_ca
{
CAGatewayClient::CAGatewayClient(rclcpp::Node * parent)
{
  parent_ = parent;

  agent_id_ = parent_->get_namespace();

  // Add node namespace and register module
  std::string register_module_service = agent_id_ + "/register_module";
  std::string forward_generic_topic = agent_id_ + "/gateway_out";
  // Create a client for the module registration service
  register_module_client_ = parent_->create_client<as2_msgs::srv::RegisterModule>(
    register_module_service);

  // Wait for the service to be available
  while (!register_module_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(parent->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(parent->get_logger(), "Service not available, waiting again...");
  }

  RCLCPP_INFO(parent->get_logger(), "Connected to register_module service");

  forwarder_pub_ = parent_->create_publisher<as2_msgs::msg::LocalGenericMessage>(
    forward_generic_topic, 10);
}


int CAGatewayClient::get_subscriber_count()
{
  return local_generic_subscribers_.size();
}

void CAGatewayClient::clear()
{
  this->local_generic_subscribers_.clear();
}

CAGatewayClient::~CAGatewayClient()
{
  clear();
}

}  // namespace as2_ca
