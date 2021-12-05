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

#include "flight_control/drone_target_land_action.h"

namespace DroneNodes
{
  
BT::NodeStatus DroneTargetLandAction::tick()
{  
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
  Pose3D goal;
  if ( !getInput<Pose3D>("goal", goal))
  {
        throw BT::RuntimeError("missing required input [goal]");
  }

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Lander Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot land the drone.  Failing.");
  }
  
  RCLCPP_INFO(node_->get_logger(), "Attempting to land on a Aruco market at [%.1f, %.1f, %.1f].", goal.x, goal.y, goal.z);
  
  // Call the action server
  auto goal_msg = TargetLand::Goal();  //navigation_interfaces::action::Land::Goal
  goal_msg.pose.pose.position.x = goal.x; 
  goal_msg.pose.pose.position.y = goal.y;
  goal_msg.pose.pose.position.z = goal.z;

  tf2::Quaternion q;
  q.setRPY(0, 0, goal.theta);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();
  
  goal_msg.target = 0;   // Land on any target  TODO Read this from the XML, optional 
        
  auto send_goal_options = rclcpp_action::Client<TargetLand>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&DroneTargetLandAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&DroneTargetLandAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&DroneTargetLandAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleTargetLand::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));

  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  cleanup();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void DroneTargetLandAction::cleanup() 
{

  if( _halt_requested )
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] - Cleaning up after a halt()", name().c_str());
    try {
      goal_handle_ = future_goal_handle_->get();
      this->client_ptr_->async_cancel_goal(goal_handle_); // Request a cancellation.
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "[%s] - Exception caught");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s] - Cleaning up after SUCCESS", name().c_str());
    // The Action Server Request completed as per normal.  Nothng to do.
  }
} 
  
void DroneTargetLandAction::halt() 
{  
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
}

  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void DroneTargetLandAction::goal_response_callback(std::shared_future<GoalHandleTargetLand::SharedPtr> future)
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

  void DroneTargetLandAction::feedback_callback(
    GoalHandleTargetLand::SharedPtr,
    const std::shared_ptr<const TargetLand::Feedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "Distance remaining %.2f", feedback->distance_to_target);
  }

  void DroneTargetLandAction::result_callback(const GoalHandleTargetLand::WrappedResult & result)
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
    
    RCLCPP_DEBUG(node_->get_logger(), "Landing complete");
  }  
  
}  // namespace