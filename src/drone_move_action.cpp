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

// Some reference to https://github.com/ros-planning/navigation2/blob/main/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp

#include "flight_control/drone_move_action.h"

namespace DroneNodes
{
  
BT::NodeStatus DroneMoveAction::tick()
{  
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
  Pose3D goal;
  if ( !getInput<Pose3D>("goal", goal))
  {
        throw BT::RuntimeError("missing required input [goal]");
  }
  
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot move the drone.  Failing.");
  }
  
  RCLCPP_INFO(node_->get_logger(), "Navigating to [%.1f, %.1f, %.1f].", goal.x, goal.y, goal.z);
  
  // Call the action server
  auto goal_msg = NavigateToPose::Goal();  //navigation_interfaces::action::NavigateToPose::Goal
  goal_msg.pose.pose.position.x = goal.x; 
  goal_msg.pose.pose.position.y = goal.y;
  goal_msg.pose.pose.position.z = goal.z;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, goal.theta);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();
  
  node_->get_parameter("navigation_bt_file", goal_msg.behavior_tree);
    
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&DroneMoveAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&DroneMoveAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&DroneMoveAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleNavigateToPose::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));

  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  cleanup();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void DroneMoveAction::cleanup() 
{

  if( _halt_requested )
  {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after a halt()", name().c_str());
    try {
      goal_handle_ = future_goal_handle_->get();
      this->client_ptr_->async_cancel_goal(goal_handle_); // Request a cancellation.
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "[%s] - Exception caught");
    }
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after SUCCESS", name().c_str());
    // The Action Server Request completed as per normal.  Nothng to do.
  }
} 
  
void DroneMoveAction::halt() 
{  
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
}

  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void DroneMoveAction::goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Drone Move Action Goal accepted by server, waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void DroneMoveAction::feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {

    RCLCPP_DEBUG(node_->get_logger(), "Distance remaining %.2f", feedback->distance_remaining);
  }

  void DroneMoveAction::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
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
  }  
  
}  // namespace