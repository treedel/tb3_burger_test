from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler

import os
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'ros2_control_demo_example_2'
    package_share = FindPackageShare(package=package_name).find(package_name)

    urdf_filename = 'tb3_hw.urdf.xacro'

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    
    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'rviz_config.rviz'),
        description='Location of RViz config file'
    )

    urdf_path = os.path.join(package_share, 'urdf', urdf_filename)
    robot_description = xacro.process_file(urdf_path).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    robot_controllers = os.path.join(package_share, "config", "controllers.yaml")
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tb3_base_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('/cmd_vel_in','/cmd_vel_out'),
            ('/cmd_vel_out','/tb3_base_controller/cmd_vel')
        ]
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)

    ld.add_action(controller_manager)
    ld.add_action(robot_state_publisher)
    ld.add_action(delayed_joint_state_broadcaster)
    ld.add_action(robot_controller_spawner)
    ld.add_action(twist_stamper)
    ld.add_action(rviz)

    return ld