#!/usr/bin/env python3

"""
ROS 2 launch file: Turtlesim + Rosbridge (+ optional compressed image republisher).

Based on examples/5_docker_turtlesim/launch_turtlesim.launch.py from
robotmcp/ros-mcp-server (Apache-2.0).

Nodes:
  - rosbridge_websocket : WebSocket bridge on :9090 that ros-mcp connects to
  - rosapi              : introspection services (topics, services, params)
  - turtlesim_node      : the simulated robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for turtlesim with rosbridge."""

    port_arg = DeclareLaunchArgument(
        "port", default_value="9090", description="Port for rosbridge websocket server"
    )

    address_arg = DeclareLaunchArgument(
        "address",
        default_value="",
        description="Address for rosbridge websocket server (empty for all interfaces)",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="Log level for nodes"
    )

    log_info = LogInfo(
        msg=[
            "Starting Turtlesim + Rosbridge:",
            "  - rosbridge port: ",
            LaunchConfiguration("port"),
        ]
    )

    # The simulated robot.
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # WebSocket bridge: what ros-mcp / any MCP client speaks to.
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                "use_compression": False,
                "max_message_size": 10000000,
                "send_action_goals_in_new_thread": True,
                "call_services_in_new_thread": True,
                "default_call_service_timeout": 5.0,
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # Introspection services used by ros-mcp's discovery tools.
    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            port_arg,
            address_arg,
            log_level_arg,
            log_info,
            rosbridge_node,
            rosapi_node,
            turtlesim_node,
        ]
    )
