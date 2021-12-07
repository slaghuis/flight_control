#ifndef PICTURE_ACTION_H
#define PICTURE_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "camera_lite_interfaces/srv/save.hpp"

namespace DroneNodes
{

class CameraSaveAction : public BT::AsyncActionNode
{
  public:

    CameraSaveAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;   
      this->client_ptr_ = node_->create_client<camera_lite_interfaces::srv::Save>("camera/save_picture");    
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("path"), 
                BT::InputPort<std::string>("filename") };
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
  private:
    std::atomic_bool _halt_requested;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Client<camera_lite_interfaces::srv::Save>::SharedPtr client_ptr_;
};
     
} // Namespace

#endif // PICTURE_ACTION_H
