#ifndef PHOTOGRAMMETRY_ACTION_H
#define PHOTOGRAMMETRY_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
    
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "coverage_planner_interfaces/action/execute_along_path.hpp"

#include "flight_control/pose_3D.h"
#include "flight_control/flight_control_node.h"

namespace DroneNodes
{

class PhotogrammetryAction : public BT::AsyncActionNode
{
  public:  
    using ExecuteAlongPath = coverage_planner_interfaces::action::ExecuteAlongPath;
    using GoalHandleExecuteAlongPath = rclcpp_action::ClientGoalHandle<ExecuteAlongPath>;
    
    PhotogrammetryAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
            
      this->client_ptr_ = rclcpp_action::create_client<ExecuteAlongPath>(
           node_,
          "coverage_server/photogrammetry");
          
      if (!node_->has_parameter("navigation_bt_file")) {
        node_->declare_parameter<std::string>("navigation_bt_file", "navigate.xml");
      }
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::vector<Pose3D>>("path") };
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
    
    rclcpp_action::Client<ExecuteAlongPath>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleExecuteAlongPath::SharedPtr>> future_goal_handle_;
    GoalHandleExecuteAlongPath::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleExecuteAlongPath::SharedPtr> future);
    void feedback_callback( 
      GoalHandleExecuteAlongPath::SharedPtr,
      const std::shared_ptr<const ExecuteAlongPath::Feedback> feedback);
    void result_callback(const GoalHandleExecuteAlongPath::WrappedResult & result);
};
     
} // Namespace

#endif // PHOTOGRAMMETRY_ACTION_H
