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

#include "flight_control/photogrammetry_action.h"

namespace DroneNodes
{
  
BT::NodeStatus PhotogrammetryAction::tick()
{  
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
  BT::Optional<std::vector<Pose3D>> msg = getInput<std::vector<Pose3D>>("path");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [goal]: ", 
                             msg.error() );
  }
  RCLCPP_INFO(node_->get_logger(), "ACTION: Received goal request to execute over %d waypoints", msg.value().size());
  
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot move the drone.  Failing.");
  }
  
  // Call the action server
  auto goal_msg = ExecuteAlongPath::Goal();
  goal_msg.path.header.stamp = node_->now();
  goal_msg.path.header.frame_id = "map";
  
  for(size_t i = 0; i < msg.value().size(); i++) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = msg.value()[i].x;
    pose.position.y = msg.value()[i].y;
    pose.position.z = msg.value()[i].z;
  
    tf2::Quaternion q;
    q.setRPY( 0, 0, msg.value()[i].theta );  // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
  
    pose.orientation.x = q[0];
    pose.orientation.y = q[1];
    pose.orientation.z = q[2];
    pose.orientation.w = q[3];
  
    goal_msg.path.poses.push_back(pose);
  }
  node_->get_parameter("navigation_bt_file", goal_msg.behavior_tree);
    
  auto send_goal_options = rclcpp_action::Client<ExecuteAlongPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&PhotogrammetryAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&PhotogrammetryAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&PhotogrammetryAction::result_callback, this, _1);

  RCLCPP_INFO(node_->get_logger(), "PHOTOGRAMMETRY ACTION: Sending goal to execute photgrammetry.  Should happen once.", msg.value().size());
  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleExecuteAlongPath::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));

  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  cleanup();
  RCLCPP_INFO(node_->get_logger(), "PHOTOGRAMMETRY ACTION: Done executing over %d wayoints.", msg.value().size());
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void PhotogrammetryAction::cleanup() 
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
  
void PhotogrammetryAction::halt() 
{  
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
}

  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void PhotogrammetryAction::goal_response_callback(std::shared_future<GoalHandleExecuteAlongPath::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "ACTION: Photogrammetry goal accepted by server, waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void PhotogrammetryAction::feedback_callback(
    GoalHandleExecuteAlongPath::SharedPtr,
    const std::shared_ptr<const ExecuteAlongPath::Feedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "ACTION: Feedback received Current Waypoint %d", feedback->current_waypoint);
  }

  void PhotogrammetryAction::result_callback(const GoalHandleExecuteAlongPath::WrappedResult & result)
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
    
    RCLCPP_INFO(node_->get_logger(), "PHOTOGRAMMETRY_ACTION complete");
  }  
  
}  // namespace