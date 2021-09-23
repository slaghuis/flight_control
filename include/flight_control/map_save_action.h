#ifndef MAP_SAVE_ACTION_H
#define MAP_SAVE_ACTION_H

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

#include "navigation_interfaces/srv/save_map.hpp"

#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{

class MapSaveAction : public BT::AsyncActionNode
{
  public:

    MapSaveAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
      this->client_ptr_ = node_->create_client<navigation_interfaces::srv::SaveMap>("nav_lite/save_map");
      
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("filename") };  // Also <int>Depth and <bool>compress and Bounding Volume
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
  private:
    std::atomic_bool _halt_requested;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Client<navigation_interfaces::srv::SaveMap>::SharedPtr client_ptr_;
};
     
} // Namespace

#endif // MAP_SAVE_ACTION_H
