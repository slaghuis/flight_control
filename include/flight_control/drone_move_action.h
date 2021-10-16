#ifndef MOVE_ACTION_H
#define MOVE_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
    
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "navigation_interfaces/action/navigate_to_pose.hpp"

#include "flight_control/pose_3D.h"
#include "flight_control/flight_control_node.h"

namespace DroneNodes
{

class DroneMoveAction : public BT::AsyncActionNode
{
  public:  
    using NavigateToPose = navigation_interfaces::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    DroneMoveAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
            
      this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
           node_,
          "nav_lite/navigate_to_pose");
          
      if (!node_->has_parameter("navigation_bt_file")) {
        node_->declare_parameter<std::string>("navigation_bt_file", "navigate.xml");
      }
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose3D>("goal") };
    }

    BT::NodeStatus tick() override;
    void halt() override;
    void cleanup();

  private:
    std::atomic_bool _halt_requested;
    ActionStatus action_status;

    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleNavigateToPose::SharedPtr>> future_goal_handle_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
    void feedback_callback( 
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
};
     
} // Namespace

#endif // MOVE_ACTION_H
