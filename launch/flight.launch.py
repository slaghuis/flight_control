from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    flight_control_node = Node(
        package="flight_control",
        executable="flight_control_node",
        parameters=[
            {"mission_bt_file"        : "./src/flight_control/behaviour_trees/sample.xml"},
            {"navigation_bt_file"     : "./src/navigation_lite/behavior_trees/navigate.xml"},            
            {"minimum_battery_voltage": 13.6},
            {"use_ground_control"     : False},
            {"drone_code"             : 42}
        ]
    )
    
    ld.add_action(flight_control_node)

    return ld
