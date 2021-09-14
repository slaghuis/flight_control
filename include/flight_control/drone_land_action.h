#ifndef LAND_ACTION_H
#define LAND_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "drone_interfaces/action/land.hpp"

#include "flight_control/flight_control_node.h"

#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{

class DroneLandAction : public BT::CoroActionNode
{
  public:
    using Land = drone_interfaces::action::Land;
    using GoalHandleLand = rclcpp_action::ClientGoalHandle<Land>;

    DroneLandAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<Land>(
       node_,
      "Land");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<bool>("gear_down") };
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
    
    rclcpp_action::Client<Land>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleLand::SharedPtr>> future_goal_handle_;
    GoalHandleLand::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleLand::SharedPtr> future);
    void feedback_callback( 
      GoalHandleLand::SharedPtr,
      const std::shared_ptr<const Land::Feedback> feedback);
    void result_callback(const GoalHandleLand::WrappedResult & result);

};
     
} // Namespace

#endif // LAND_ACTION_H

