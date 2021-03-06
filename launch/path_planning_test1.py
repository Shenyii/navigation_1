from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
    #    Node(
    #        package='rviz2',
    #        #node_namespace='turtlesim1',
    #        node_executable='rviz2',
    #        #node_name='sim'
    #    ),
       Node(
          package='navigation_1',
          #node_namespace='turtlesim2',
          node_executable='generate_map',
          #output='screen',
          #node_name='sim'
       ),
       Node(
          package='navigation_1',
          #node_namespace='turtlesim2',
          node_executable='a_star',
          output='screen',
          #remappings=[
          #   ('/initialpose', '/move_base_simple/goal'),
          #]
       ),
       Node(
          package='navigation_1',
          #node_namespace='turtlesim2',
          node_executable='opt_path_in_gallerys',
          output='screen',
          #node_name='sim'
       ),
    ])
