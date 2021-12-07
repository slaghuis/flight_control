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

#include "flight_control/camera_save_action.h"

namespace DroneNodes
{
     

BT::NodeStatus CameraSaveAction::tick()
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;
  BT::Optional<std::string> msg = getInput<std::string>("filename");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [filename]: ", msg.error() );
  }
  
  auto request = std::make_shared<camera_lite_interfaces::srv::Save::Request>();
  request->name = msg.value();

  _halt_requested.store(false);
   
  while ((!_halt_requested) && (!client_ptr_->wait_for_service(250ms))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  if (_halt_requested) {
    return BT::NodeStatus::FAILURE;
  }
  
  auto future = client_ptr_->async_send_request(request);

  std::future_status status;
  do {
    status = future.wait_for(250ms);  // Not spinning here!  We are in a thread, and the spinning is taken cared of elsewhere.
    if (status == std::future_status::deferred) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future deferred");
    } else if (status == std::future_status::timeout) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future timeout");
    } else if (status == std::future_status::ready) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future ready!");
    }
  } while ((status != std::future_status::ready) && (!_halt_requested)); 
   
  bool service_result = false;
  if (status == std::future_status::ready)
  {
    auto result = future.get();
    service_result = result->result; 
  }
  
  return (service_result) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
      
void CameraSaveAction::halt() 
{  
  _halt_requested.store(true); 
}
  
  
}  // namespace