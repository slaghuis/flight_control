'from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="flight_control",
            executable="flight_control_node",
            name="flight_control_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"mission_bt_file": "./src/flight_control/behaviour_trees/sample.xml"},
                {"minimum_battery_voltage": 13.6}
            ]
        )
    ])
