#ifndef UTILITY_ACTION_H
#define UTILITY_ACTION_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include <time.h>

#define FILENAME_FORMAT "%Y%m%d_%H%M%S.jpg"
#define FILENAME_SIZE 20

namespace DroneNodes
{
     
class SaySomething : public BT::SyncActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }
};  


class GenerateFilename : public BT::SyncActionNode
{
  public:
    GenerateFilename(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("filename") };
    }

};
  
} // Namespace

#endif // UTILITY_ACTION_H
