from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      return LaunchDescription([
            Node(
                  package='my_package',
                  node_executable='talker',
                  node_name='talker',
                  output='screen'
            ),
            Node(
                  package='my_package',
                  node_executable='listener',
                  node_name='listener',
                  output='screen'
            )
      ])