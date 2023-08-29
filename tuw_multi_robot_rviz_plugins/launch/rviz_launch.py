#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution 
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node

# Add string commands if not empty

def generate_launch_description():


    use_sim_time     = LaunchConfiguration('use_sim_time',  default='true')
    this_pgk = get_package_share_directory('tuw_graph_rviz_plugins')

    def rviz_launch_configuration(context):
        file = os.path.join(this_pgk, 'config', 'rviz', context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    namespace_arg    = DeclareLaunchArgument('namespace',   default_value=TextSubstitution(text=''))
    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)
    rviz_config_arg = DeclareLaunchArgument('config', 
                default_value=TextSubstitution(text='graph'), 
                description='Use empty, cave or roblab to load a TUW enviroment')

    return LaunchDescription([
        namespace_arg,
        rviz_config_arg,
        rviz_launch_configuration_arg,
        Node(
            package='rviz2',
            namespace=LaunchConfiguration('namespace'),
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [LaunchConfiguration('config')]],
            parameters=[{
                "use_sim_time": use_sim_time}],
        )
    ])