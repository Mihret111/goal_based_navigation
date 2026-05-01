from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    controller_mode = LaunchConfiguration('controller_mode')

    declare_controller_mode = DeclareLaunchArgument(
        'controller_mode',
        default_value='staged',
        description='Controller mode: staged or simultaneous'
    )

    # --- Include robot simulation launch ---
    robot_pkg_share = get_package_share_directory('bme_gazebo_sensors')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'spawn_robot.launch.py')
        )
    )

    # --- Server component ---
    nav_server_component = ComposableNode(
        package='nav_system',
        plugin='nav_system::NavActionServerComponent',
        name='nav_action_server_component',
        parameters=[{
            'controller_mode': controller_mode
        }]
    )

    # --- Client component ---
    nav_client_component = ComposableNode(
        package='nav_system',
        plugin='nav_system::NavActionClientComponent',
        name='nav_action_client_component'
    )

    # --- Component container ---
    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            nav_server_component,
            nav_client_component,
        ],
        output='screen'
    )

    # --- Tkinter UI ---
    tk_ui = Node(
        package='nav_ui',
        executable='ui_tkinter',
        name='nav_ui_tk',
        output='screen'
    )

    return LaunchDescription([
        declare_controller_mode,
        robot_launch,
        container,
        tk_ui,
    ])