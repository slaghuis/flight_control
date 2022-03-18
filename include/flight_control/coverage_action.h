#ifndef COVERAGE_ACTION_H
#define COVERAGE_ACTION_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Quaternion.h>      //tf2::Quaternion
#include <tf2_ros/buffer.h>                 //tf2::Matrix3x3

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "coverage_planner_interfaces/action/compute_coverage_path.hpp"

#include "flight_control/point_2D.h"
#include "flight_control/pose_3D.h"
#include "flight_control/flight_control_node.h"

namespace DroneNodes
{

class CoverageAction : public BT::AsyncActionNode
{
  public:  
    using ComputeCoveragePath = coverage_planner_interfaces::action::ComputeCoveragePath;
    using GoalHandleComputeCoveragePath = rclcpp_action::ClientGoalHandle<ComputeCoveragePath>;
    
    CoverageAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
            
      this->client_ptr_ = rclcpp_action::create_client<ComputeCoveragePath>(
           node_,
          "coverage_server/compute_coverage_path");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::vector<Point2D>>("area"), 
                BT::InputPort<float>("height"), 
                BT::InputPort<float>("projected_width"), 
                BT::InputPort<float>("projected_height"), 
                BT::OutputPort<std::vector<Pose3D>>("path")};        
    }

    BT::NodeStatus tick() override;
    void halt() override;
    void cleanup();

  private:
    std::atomic_bool _halt_requested;
    ActionStatus action_status;
    
    // Saved Results for feedback
    std::stringstream returned_path;

    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    
    rclcpp_action::Client<ComputeCoveragePath>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleComputeCoveragePath::SharedPtr>> future_goal_handle_;
    GoalHandleComputeCoveragePath::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleComputeCoveragePath::SharedPtr> future);
    void feedback_callback( 
      GoalHandleComputeCoveragePath::SharedPtr,
      const std::shared_ptr<const ComputeCoveragePath::Feedback> feedback);
    void result_callback(const GoalHandleComputeCoveragePath::WrappedResult & result);
};
     
} // Namespace

#endif // COVERAGE_ACTION_H
