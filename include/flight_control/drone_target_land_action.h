#ifndef TARGET_LAND_ACTION_H
#define TARGET_LAND_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
    
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "lander_interfaces/action/land.hpp"

#include "flight_control/pose_3D.h"
#include "flight_control/flight_control_node.h"

namespace DroneNodes
{

class DroneTargetLandAction : public BT::AsyncActionNode
{
  public:  
    using TargetLand = lander_interfaces::action::Land;
    using GoalHandleTargetLand = rclcpp_action::ClientGoalHandle<TargetLand>;
    
    DroneTargetLandAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
            
      this->client_ptr_ = rclcpp_action::create_client<TargetLand>(
       node_,
      "lander/land");
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
    
    rclcpp_action::Client<TargetLand>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleTargetLand::SharedPtr>> future_goal_handle_;
    GoalHandleTargetLand::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleTargetLand::SharedPtr> future);
    void feedback_callback( 
      GoalHandleTargetLand::SharedPtr,
      const std::shared_ptr<const TargetLand::Feedback> feedback);
    void result_callback(const GoalHandleTargetLand::WrappedResult & result);
};
     
} // Namespace

#endif // TARGET_LAND_ACTION_H
