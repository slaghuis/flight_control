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

#include "flight_control/drone_takeoff_action.h"

namespace DroneNodes
{

 
BT::NodeStatus DroneTakeoffAction::tick()
{
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
  
  
  BT::Optional<float> msg = getInput<float>("altitude");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
  }

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot move the drone.  Failing.");
  }
  
  // Call the action server
  auto goal_msg = Takeoff::Goal();  //drone_interfaces::action::Takeoff::Goal
  goal_msg.target_altitude = msg.value(); 
    
  auto send_goal_options = rclcpp_action::Client<Takeoff>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&DroneTakeoffAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&DroneTakeoffAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&DroneTakeoffAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleTakeoff::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));

  while( (action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING) )
  {   
    setStatusRunningAndYield();
  }  
  
  cleanup(false);
  
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void DroneTakeoffAction::cleanup(bool halted) 
{
  if( halted )
  {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after a halt()", name().c_str());
    goal_handle_ = future_goal_handle_->get();
    this->client_ptr_->async_cancel_goal(goal_handle_); // Request a cancellation.
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after SUCCESS", name().c_str());
    // The Action Server Request completed as per normal.  Nothng to do.
  }
}
  
void DroneTakeoffAction::halt() 
{
  std::cout << name() << ": halted." << std::endl;
  cleanup(true);
  action_status = ActionStatus::CANCELED;
  
  CoroActionNode::halt();
}
  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void DroneTakeoffAction::goal_response_callback(std::shared_future<GoalHandleTakeoff::SharedPtr> future)
  {
    //future_goal_handle_ = future;
  
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void DroneTakeoffAction::feedback_callback(
    GoalHandleTakeoff::SharedPtr,
    const std::shared_ptr<const Takeoff::Feedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "Current Altitude: %.2fm", feedback->current_altitude);
  }

  void DroneTakeoffAction::result_callback(const GoalHandleTakeoff::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        action_status = ActionStatus::SUCCEEDED;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
        action_status = ActionStatus::ABORTED;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
        action_status = ActionStatus::CANCELED;
        return;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        action_status = ActionStatus::UNKNOWN;
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Navigation complete");
    // rclcpp::shutdown();
  }  
  
}  // namespace