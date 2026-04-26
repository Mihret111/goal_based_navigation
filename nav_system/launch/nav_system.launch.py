from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # launch server component alone first to see if it works
    nav_server_component = ComposableNode(
        package='nav_system',
        plugin='nav_system::NavActionServerComponent',
        name='nav_action_server_component',
        parameters=[{'controller_mode': 'simultaneous'}]
    )

    # launch client component alone first to see if it works
    nav_client_component = ComposableNode(
        package='nav_system',
        plugin='nav_system::NavActionClientComponent',
        name='nav_action_client_component'
    )

    # launch both in a container
    container = ComposableNodeContainer(
        name='nav_system_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',       # multithreaded
        composable_node_descriptions=[
            nav_server_component,
            nav_client_component,
        ],
        output='screen'
    )

    return LaunchDescription([
        container
    ])