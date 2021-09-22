#ifndef SNAP_ACTION_H
#define SNAP_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "lander_interfaces/srv/snap.hpp"

#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{

class TrackerSnapAction : public BT::AsyncActionNode
{
  public:

    TrackerSnapAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
      this->client_ptr_ = node_->create_client<lander_interfaces::srv::Snap>("tracker/snap");
      
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("filename") };
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
  private:
    std::atomic_bool _halt_requested;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Client<lander_interfaces::srv::Snap>::SharedPtr client_ptr_;
};
     
} // Namespace

#endif // SNAP_ACTION_H
