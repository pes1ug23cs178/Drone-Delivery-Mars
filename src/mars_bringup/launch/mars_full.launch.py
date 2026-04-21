#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:
    use_graph_planner = LaunchConfiguration('use_graph_planner')
    declare_use_graph_planner = DeclareLaunchArgument(
        'use_graph_planner',
        default_value='false',
        description='Enable graph-based planner and optional mission manager path following.',
    )

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    mars_world_share = get_package_share_directory('mars_world')
    mars_drone_share = get_package_share_directory('mars_drone_description')
    mars_bringup_share = get_package_share_directory('mars_bringup')

    world_file = os.path.join(mars_world_share, 'worlds', 'mars_delivery.world')
    drone_xacro = os.path.join(mars_drone_share, 'urdf', 'drone.urdf.xacro')
    model_path = os.path.join(mars_world_share, 'models')
    rviz_config = os.path.join(mars_bringup_share, 'rviz', 'mars_drone.rviz')

    gazebo_launch_file = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    robot_description = xacro.process_file(drone_xacro).toxml()

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[model_path, os.pathsep, EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity',
            'mars_drone',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.2',
        ],
    )

    mission_manager = Node(
        package='mars_mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'use_graph_planner': use_graph_planner}],
    )

    graph_path_planner = Node(
        package='mars_graph_planner',
        executable='graph_path_planner',
        name='graph_path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_graph_planner),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    delayed_spawn = TimerAction(period=4.0, actions=[spawn_drone])
    delayed_graph_planner = TimerAction(period=5.5, actions=[graph_path_planner])
    delayed_mission_manager = TimerAction(period=6.0, actions=[mission_manager])
    delayed_rviz = TimerAction(period=7.0, actions=[rviz])

    return LaunchDescription(
        [
            declare_use_graph_planner,
            set_gazebo_model_path,
            gazebo,
            robot_state_publisher,
            delayed_spawn,
            delayed_graph_planner,
            delayed_mission_manager,
            delayed_rviz,
        ]
    )
