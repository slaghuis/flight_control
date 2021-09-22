// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flight_control/tracker_snap_action.h"



namespace DroneNodes
{

 
BT::NodeStatus TrackerSnapAction::tick()
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;
  
  BT::Optional<std::string> msg = getInput<std::string>("filename");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
  }

  auto request = std::make_shared<lander_interfaces::srv::Snap::Request>();
  request->name = msg.value();

  _halt_requested.store(false);
  
  while ((!_halt_requested) && (!client_ptr_->wait_for_service(1s))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  if (_halt_requested) {
    return BT::NodeStatus::FAILURE;
  }
  
  auto result = client_ptr_->async_send_request(request);
 
  bool service_result = false;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    service_result = result.get()->result;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service [tracker/Snap]");
  }

  return (service_result) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
      
void TrackerSnapAction::halt() 
{  
  _halt_requested.store(true); 
}
  
  
}  // namespace