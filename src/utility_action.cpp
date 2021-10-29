#include "flight_control/utility_action.h"
#include "rclcpp/rclcpp.hpp"

namespace DroneNodes
{
    
BT::NodeStatus SaySomething::tick()
{
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot says: " + msg.value());
    return BT::NodeStatus::SUCCESS;
}
  
  
}  // namespace