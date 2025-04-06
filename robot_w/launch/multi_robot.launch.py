# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.descriptions import ParameterFile


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    ld = LaunchDescription()

    # Set namespace
    namespace = LaunchConfiguration('namespace', default='robot')
    declare_enable_rviz = DeclareLaunchArgument(
        name='namespace', default_value=namespace, description='Namespace for navigation'
    )

    #Get robot_w package directory
    robot_w = get_package_share_directory('robot_w')

    # Own nav2 launch directory
    nav_launch_dir = os.path.join(robot_w, 'launch', 'nav2_bringup')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))
    declare_map_dir = DeclareLaunchArgument(
        name='map', default_value=map_dir, description='Map used for robots'
    )


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=use_rviz, description='Enable rviz launch'
    )

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))
    declare_map_dir = DeclareLaunchArgument(
        name='map', default_value=map_dir, description='Map used for robots'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(robot_w, 'param', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_dir}


    
    # Rviz multi robot config
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            robot_w, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use')

    
    # Launch commands
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_dir)

    #Tf remappings to use in map server
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Map server node with remappings for tf
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_dir,
                     },]
        )

    # Lifecycle node for map server
    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    

    # Add map nodes to launch description
    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    #Nav bringup
    bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )
    
    # # Collision Monitor Bringup
    # collision_bringup = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(nav_launch_dir, 'collision_monitor_node.launch.py')),
    #                 launch_arguments={  
    #                                 'namespace': namespace,
    #                                 'use_sim_time': use_sim_time,
    #                                 'params_file': os.path.join(robot_w, "param", "collision_monitor_params.yaml")
    #                                 }.items()
    #                                 )

    collision_bringup = Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'namespace': namespace,
                         'params_file': os.path.join(robot_w, "param", "collision_monitor_params.yaml")
                         }]
        )

    # Add nav2 to launch description
    ld.add_action(bringup_cmd)
    ld.add_action(collision_bringup)


    # Rviz launch
    rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  #'namespace': TextSubstitution(text=namespace)
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(use_rviz)
                                    )

    # Add rviz launch to ld (and optionally initial pose)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_params_file_cmd)

    return ld

