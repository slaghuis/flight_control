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

#include "flight_control/coverage_action.h"

namespace DroneNodes
{
  
BT::NodeStatus CoverageAction::tick()
{  
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
  BT::Optional<std::vector<Point2D>> msg = getInput<std::vector<Point2D>>("area");
  BT::Optional<float> height = getInput<float>("height");
  BT::Optional<float> p_area_w = getInput<float>("projected_width");
  BT::Optional<float> p_area_h = getInput<float>("projected_height");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [goal]: ", 
                             msg.error() );
  }
  RCLCPP_INFO(node_->get_logger(), "COVERAGE ACTION: Received goal request to calculte a path over an area defined by %d points", msg.value().size() );
  
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot move the drone.  Failing.");
  }
  
  // Call the action server
  auto goal_msg = ComputeCoveragePath::Goal();
  
  goal_msg.height = height.value();
  goal_msg.projected_area_w = p_area_w.value();
  goal_msg.projected_area_h = p_area_h.value();
  
  //goal_msg.area.header.stamp = node_->now();
  //goal_msg.area.header.frame_id = "map";
  
  for(size_t i = 0; i < msg.value().size(); i++) {
    geometry_msgs::msg::Point32 point;
    point.x = msg.value()[i].x;
    point.y = msg.value()[i].y;
    point.z = 0.0;                     // The Z value is not used
  
    goal_msg.area.points.push_back(point);
  }
      
  auto send_goal_options = rclcpp_action::Client<ComputeCoveragePath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&CoverageAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&CoverageAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&CoverageAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleComputeCoveragePath::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));

  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  setOutput("path", returned_path.str().c_str());
            
  cleanup();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void CoverageAction::cleanup() 
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
  
void CoverageAction::halt() 
{  
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
}

  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void CoverageAction::goal_response_callback(std::shared_future<GoalHandleComputeCoveragePath::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Drone Coverage Action Goal accepted by server, waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void CoverageAction::feedback_callback(
    GoalHandleComputeCoveragePath::SharedPtr,
    const std::shared_ptr<const ComputeCoveragePath::Feedback> )
  {

    // RCLCPP_DEBUG(node_->get_logger(), "Distance remaining %d", feedback->current_waypoint);
  }

  void CoverageAction::result_callback(const GoalHandleComputeCoveragePath::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result->path.poses.size() == 0) {
          action_status = ActionStatus::FAILED;   // No waypoints could be calculated..  Is the area too small or the camera insufficient?
        } else {
          // Build out a response x;y,z,theta float combinations seperated by |
          for(size_t i=0; i<result.result->path.poses.size(); i++) {
            if ( i > 0) {
              returned_path << "|";
            }
            returned_path << result.result->path.poses[i].position.x << ";";
            returned_path << result.result->path.poses[i].position.y << ";";
            returned_path << result.result->path.poses[i].position.z << ";";
            
            tf2::Quaternion q(
              result.result->path.poses[i].orientation.x,
              result.result->path.poses[i].orientation.y,
              result.result->path.poses[i].orientation.z,
              result.result->path.poses[i].orientation.w
            );
            tf2::Matrix3x3 m(q);
            double roll,pitch,yaw;
            m.getRPY(roll, pitch, yaw);
            
            returned_path << yaw;
          }
          RCLCPP_INFO(node_->get_logger(), "Compute Coverage Path complete");
        }
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
    
  }  
  
}  // namespace