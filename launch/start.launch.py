from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
# Load Params Yaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="component_manager_node",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2-components",
                plugin="component::component_talker",
                name="launch_component_talker_node",
                parameters=[os.path.join(get_package_share_directory('ros2-components'), 'config', 'params.yaml')]
                # parameters=[{"param1" : 122, "param2": 89}]
            ),
            ComposableNode(
                package="ros2-components",
                plugin="component::component_listener",
                name="launch_component_listener_node",
            )
        ],
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(container)

    return ld