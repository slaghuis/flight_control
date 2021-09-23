#ifndef MAP_LOAD_ACTION_H
#define MAP_LOAD_ACTION_H

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

#include "navigation_interfaces/srv/load_map.hpp"

#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{

class MapLoadAction : public BT::AsyncActionNode
{
  public:

    MapLoadAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
      this->client_ptr_ = node_->create_client<navigation_interfaces::srv::LoadMap>("nav_lite/load_map");
      
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
    
    rclcpp::Client<navigation_interfaces::srv::LoadMap>::SharedPtr client_ptr_;
};
     
} // Namespace

#endif // MAP_LOAD_ACTION_H
