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

std::vector<std::string> CAGatewayClient::get_known_peers()
{
  std::vector<std::string> peers;

  // Get all nodes in the system
  auto node_names = parent_->get_node_names();

  for (const auto & node_name : node_names) {
    // Extract namespace from node name (node name format is "/namespace/node_name")
    std::string namespace_str = node_name;
    size_t last_slash = node_name.find_last_of('/');
    if (last_slash != std::string::npos && last_slash > 0) {
      namespace_str = node_name.substr(0, last_slash);
    }

    // Strip leading slash to normalize namespace format (e.g. "/drone0" → "drone0")
    if (!namespace_str.empty() && namespace_str.front() == '/') {
      namespace_str = namespace_str.substr(1);
    }

    // Skip empty namespaces and own agent_id
    std::string own_id = agent_id_;
    if (!own_id.empty() && own_id.front() == '/') {own_id = own_id.substr(1);}
    if (!namespace_str.empty() && namespace_str != own_id) {
      // Check if this namespace is already in the peers list
      if (std::find(peers.begin(), peers.end(), namespace_str) == peers.end()) {
        peers.push_back(namespace_str);
      }
    }
  }

  return peers;
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
