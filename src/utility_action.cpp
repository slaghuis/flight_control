#include "flight_control/utility_action.h"

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
  
BT::NodeStatus GenerateFilename::tick()
{
  
  // the output may change at each tick(). Here we keep it simple.
  
  static char name[FILENAME_SIZE];
  time_t now = time(0);
  strftime(name, sizeof(name), FILENAME_FORMAT, localtime(&now));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Generated name : %s", name);
  setOutput("filename", name );
  return BT::NodeStatus::SUCCESS;
}  
  
}  // namespace