from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'tb3_description'
    urdf_filename = 'turtlebot3_burger.urdf'
    package_share = FindPackageShare(package=package_name).find(package_name)

    urdf_path = os.path.join(package_share, 'urdf', urdf_filename)

    robot_description = xacro.process_file(urdf_path).toxml()

    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp = LaunchConfiguration('use_jsp')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_jsp = DeclareLaunchArgument(
        name='use_jsp',
        default_value='true',
        description='Whether to launch joint state publisher'
    )

    declare_use_jsp_gui = DeclareLaunchArgument(
        name='use_jsp_gui',
        default_value='true',
        description='Whether to launch jsp in gui mode'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'model.rviz'),
        description='Location of RViz config file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    joint_state_publisher = Node(
        condition=IfCondition(PythonExpression(
            ["'", use_jsp, "' == 'true' and '", use_jsp_gui, "' == 'false'"]
        )),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        condition=IfCondition(PythonExpression(
            ["'", use_jsp, "' == 'true' and '", use_jsp_gui, "' == 'true'"]
        )),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(declare_use_jsp)
    ld.add_action(declare_use_jsp_gui)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)

    return ld