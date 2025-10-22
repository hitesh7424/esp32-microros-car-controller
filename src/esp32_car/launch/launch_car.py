from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import time
import requests



def generate_launch_description():
    rosbridge_launch_file = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )

    # Start ngrok to expose the WebSocket server
    ngrok_process = ExecuteProcess(
        cmd=["ngrok", "http", "9090"],  # Replace '9090' with your WebSocket server port
        name="ngrok",
        output="screen",
    )


    return LaunchDescription(
        [
            # Start the micro-ROS agent
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "micro_ros_agent",
                    "micro_ros_agent",
                    "udp4",
                    "--port",
                    "8888",
                ],
                name="micro_ros_agent",
                output="screen",
            ),
            # Start the wheel speed controller node
            Node(
                package="esp32_car",
                executable="wheel_speed_controller",
                name="wheel_speed_controller",
                output="screen",
            ),
            # Include the rosbridge_websocket launch file
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(rosbridge_launch_file), launch_arguments={}
            ),
            # Start ngrok
            ngrok_process,

        ]
    )
