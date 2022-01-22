// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * Reads a behavior tree xml file and calls various action servers and 
 * simple services to complete the mission.
 * **********************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "flight_control/utility_action.h"
#include "flight_control/drone_move_action.h"
#include "flight_control/drone_takeoff_action.h"
#include "flight_control/drone_land_action.h"
#include "flight_control/drone_target_land_action.h"
#include "flight_control/camera_save_action.h"
#include "flight_control/map_load_action.h"
#include "flight_control/map_save_action.h"

#include "navigation_interfaces/srv/mission.hpp"

using Mission = navigation_interfaces::srv::Mission;
using namespace std::chrono_literals;
using namespace BT;

class FlightControlNode : public rclcpp::Node
{
  public:
    FlightControlNode()
    : Node("flight_control_node"), current_battery_voltage_(14.8), clear_to_fly_(false)
    {
      
      one_off_timer_ = this->create_wall_timer(
        1000ms, std::bind(&FlightControlNode::init, this));
        
    }

  private:
  
    void init ()
    {
      // Only run this once
      this->one_off_timer_->cancel();
      
      // Declare and get parameters
      behaviour_tree_file_ = this->declare_parameter<std::string>("mission_bt_file", "mission.xml");
      minimum_battery_voltage_ = this->declare_parameter<float>("minimum_battery_voltage", 13.6);
      clear_to_fly_ = ! this->declare_parameter<bool>("use_ground_control", false);
      drone_code_ = this->declare_parameter<int>("drone_code", 42);

      // Setup the behavior tree
      using namespace DroneNodes;
      factory.registerSimpleCondition("BatteryOK", std::bind(&FlightControlNode::CheckBattery, this));
 
      factory.registerNodeType<DroneTakeoffAction>("TakeoffDrone");
      factory.registerNodeType<DroneLandAction>("LandDrone");
      factory.registerNodeType<DroneTargetLandAction>("TargetLandDrone");
      factory.registerNodeType<DroneMoveAction>("MoveDrone");
      factory.registerNodeType<SaySomething>("SaySomething");
      factory.registerNodeType<GenerateFilename>("GenerateFilename");
      factory.registerNodeType<CameraSaveAction>("SavePicture");
      factory.registerNodeType<MapLoadAction>("LoadMap");
      factory.registerNodeType<MapSaveAction>("SaveMap");

      tree = factory.createTreeFromFile(behaviour_tree_file_);
        
      auto node_ptr = shared_from_this();                
      // Iterate through all the nodes and call init() if it is an Action_B
      for( auto& node: tree.nodes )
      {
        // Not a typo: it is "=", not "=="
        if( auto takeoff_action = dynamic_cast<DroneTakeoffAction*>( node.get() )) {
          takeoff_action->init( node_ptr );
        } else if( auto land_action = dynamic_cast<DroneLandAction*>( node.get() )) {
          land_action->init( node_ptr );
        } else if( auto target_land_action = dynamic_cast<DroneTargetLandAction*>( node.get() )) {
          target_land_action->init( node_ptr );
        } else if( auto move_action = dynamic_cast<DroneMoveAction*>( node.get() )) {
          move_action->init( node_ptr );
        } else if( auto save_picture_action = dynamic_cast<CameraSaveAction*>( node.get() )) {
          save_picture_action->init( node_ptr );
        } else if( auto load_action = dynamic_cast<MapLoadAction*>( node.get() )) {
          load_action->init( node_ptr );
        } else if( auto save_action = dynamic_cast<MapSaveAction*>( node.get() )) {
          save_action->init( node_ptr );
        }
      }

      subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "drone/battery", 5, std::bind(&FlightControlNode::battery_callback, this, std::placeholders::_1));
      
      mission_service_ = this->create_service<Mission>("drone/mission", 
        std::bind(&FlightControlNode::start_mission, this, std::placeholders::_1, std::placeholders::_2));
      
      timer_ = this->create_wall_timer(
        1000ms, std::bind(&FlightControlNode::timer_callback, this));
    }

    void start_mission(const std::shared_ptr<Mission::Request> request,
                          std::shared_ptr<Mission::Response> response)
    {
      if (request->drone_code == drone_code_) {
        behaviour_tree_file_ = request->mission_file;
        clear_to_fly_ = true;
        response->accepted = true;
      } else {
        clear_to_fly_ = false;
        response->accepted = false;
      }
    }                      
    
    BT::NodeStatus CheckBattery()
    {      
      return (current_battery_voltage_ >= minimum_battery_voltage_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }  
  
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) 
    {
      current_battery_voltage_ = msg->voltage;
    }
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
    
    void timer_callback()
    {      
      using namespace DroneNodes;
      
      if( !clear_to_fly_ ) {
        return;
      }
      
      if( tree.tickRoot() != NodeStatus::RUNNING) {
        this->timer_->cancel();  
        rclcpp::shutdown();
      }
    }
    
    BehaviorTreeFactory factory;
    Tree tree;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr one_off_timer_;
    float minimum_battery_voltage_;
    float current_battery_voltage_;
    int drone_code_;
    
    std::string behaviour_tree_file_;
    bool clear_to_fly_;
    rclcpp::Service<Mission>::SharedPtr mission_service_;    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightControlNode>());
  rclcpp::shutdown();
  return 0;
}
