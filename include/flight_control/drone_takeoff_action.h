#ifndef TAKEOFF_ACTION_H
#define TAKEOFF_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "drone_interfaces/action/takeoff.hpp"

#include "flight_control/flight_control_node.h"

#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{

class DroneTakeoffAction : public BT::CoroActionNode
{
  public:
    using Takeoff = drone_interfaces::action::Takeoff;
    using GoalHandleTakeoff = rclcpp_action::ClientGoalHandle<Takeoff>;

    DroneTakeoffAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<Takeoff>(
       node_,
      "Takeoff");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<float>("altitude") };
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
    void cleanup(bool halted);

  private:
    std::atomic_bool _halt_requested;
    ActionStatus action_status;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    
    rclcpp_action::Client<Takeoff>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleTakeoff::SharedPtr>> future_goal_handle_;
    GoalHandleTakeoff::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleTakeoff::SharedPtr> future);
    void feedback_callback( 
      GoalHandleTakeoff::SharedPtr,
      const std::shared_ptr<const Takeoff::Feedback> feedback);
    void result_callback(const GoalHandleTakeoff::WrappedResult & result);

};
     
} // Namespace

#endif // TAKEOFF_ACTION_H
