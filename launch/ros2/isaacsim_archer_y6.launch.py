#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-20
################################################################

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "hex_ros_isaacsim_arm"
    urdf_pkg_path = FindPackageShare("hex_ros_urdf_archer_y6")

    # args
    test_arg = DeclareLaunchArgument(
        name='test',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to turn on test ctrl node')
    mock_arg = DeclareLaunchArgument(
        name='mock',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to turn on the mock bridge node')
    robot_grip_type_arg = DeclareLaunchArgument(
        name='robot_grip_type',
        default_value='empty',
        choices=['gr100', 'empty'],
        description='Grip type: gr100 (1-DoF) or empty (0-DoF)')
    bridge_state_topic_arg = DeclareLaunchArgument(
        name='bridge_state_topic',
        default_value='/joint_states',
        description='Bridge joint_states topic (must be a param)')
    bridge_cmd_topic_arg = DeclareLaunchArgument(
        name='bridge_cmd_topic',
        default_value='/hex_arm_cmd',
        description='Bridge torque command topic (must be a param)')
    time_source_arg = DeclareLaunchArgument(
        name='time_source',
        default_value='sim',
        choices=['sim', 'ros'],
        description='Forwarded header time source: sim (bridge) or ros (now)')
    mock_joint_names_arg = DeclareLaunchArgument(
        name='mock_joint_names',
        default_value='',
        description='Mock bridge joint names (comma-separated); empty = learn from cmd')
    mock_fixed_stamp_arg = DeclareLaunchArgument(
        name='mock_fixed_stamp',
        default_value='0.0',
        description='Mock bridge fixed header stamp (sec); 0 = use real clock (test time_source)')
    control_mode_arg = DeclareLaunchArgument(
        name='control_mode',
        default_value='jnt',
        choices=['mit', 'jnt', 'ee'],
        description='Test ctrl mode (arm): MIT/JNT position presets or EE pose presets')

    # robot node
    robot_param_path = FindPackageShare(package_name).find(
        package_name) + '/config/ros2/archer_params.yaml'
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "empty.urdf"])
    robot_node = Node(package=package_name,
                      executable='isaacsim_archer_y6',
                      name='hex_ros_isaacsim_archer_y6',
                      output="screen",
                      emulate_tty=True,
                      parameters=[
                          robot_param_path,
                          {
                              'robot_grip_type':
                              LaunchConfiguration('robot_grip_type'),
                              'urdf_path':
                              ParameterValue(urdf_file_path, value_type=str),
                              'bridge_state_topic':
                              LaunchConfiguration('bridge_state_topic'),
                              'bridge_cmd_topic':
                              LaunchConfiguration('bridge_cmd_topic'),
                              'time_source':
                              LaunchConfiguration('time_source'),
                          },
                      ])

    # mock bridge group
    mock_group = GroupAction(
        [
            Node(
                package=package_name,
                executable='mock_bridge',
                name='mock_bridge',
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'cmd_topic': LaunchConfiguration('bridge_cmd_topic'),
                    'state_topic': LaunchConfiguration('bridge_state_topic'),
                    'joint_names': LaunchConfiguration('mock_joint_names'),
                    'mass': 1.0,
                    'damping': 1.0,
                    'pub_rate': 500.0,
                    'fixed_stamp_sec':
                    ParameterValue(
                        LaunchConfiguration('mock_fixed_stamp'),
                        value_type=str),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('mock')),
    )

    # test group
    test_group = GroupAction(
        [
            Node(
                package=package_name,
                executable='test_ctrl',
                name='test_ctrl',
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'use_sim_time': False,
                    'control_mode': LaunchConfiguration('control_mode'),
                }],
                remappings=[
                    ('manip_ctrl', 'manip_ctrl'),
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration('test')),
    )

    return LaunchDescription([
        test_arg,
        mock_arg,
        robot_grip_type_arg,
        bridge_state_topic_arg,
        bridge_cmd_topic_arg,
        time_source_arg,
        mock_joint_names_arg,
        mock_fixed_stamp_arg,
        control_mode_arg,
        robot_node,
        mock_group,
        test_group,
    ])
