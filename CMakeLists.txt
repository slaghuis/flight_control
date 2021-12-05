cmake_minimum_required(VERSION 3.5)
project(flight_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(lander_interfaces REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(BehaviorTreeV3)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(flight_control_node 
  src/flight_control_node.cpp
  src/utility_action.cpp
  src/drone_move_action.cpp
  src/drone_takeoff_action.cpp
  src/drone_land_action.cpp
  src/drone_target_land_action.cpp
  src/map_load_action.cpp 
  src/map_save_action.cpp )
  
ament_target_dependencies(flight_control_node
  "rclcpp"
  "rclcpp_action"
  "navigation_interfaces"
  "drone_interfaces"
  "lander_interfaces"
  "sensor_msgs"
  "tf2" 
  "tf2_ros" )
target_include_directories(flight_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flight_control_node BT::behaviortree_cpp_v3)  

install(TARGETS
  flight_control_node
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
   
ament_package()
