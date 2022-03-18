#ifndef CAMERA_MODEL_ACTION_H
#define CAMERA_MODEL_ACTION_H

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

#include "coverage_planner_interfaces/srv/camera_model.hpp"

#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{

class CameraModelAction : public BT::AsyncActionNode
{
  public:

    CameraModelAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
      this->client_ptr_ = node_->create_client<coverage_planner_interfaces::srv::CameraModel>("coverage_planner/camera_model");
      
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<float>("spatial_resolution"), 
                BT::OutputPort<float>("height"), 
                BT::OutputPort<float>("projected_width"),
                BT::OutputPort<float>("projected_height") };
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
  private:
    std::atomic_bool _halt_requested;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Client<coverage_planner_interfaces::srv::CameraModel>::SharedPtr client_ptr_;
};
     
} // Namespace

#endif // CAMERA_MODEL_ACTION_H
