from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  background_chart = LaunchConfiguration('background_chart')
  background_chart_arg = DeclareLaunchArgument(
    "background_chart",
      default_value=PathJoinSubstitution([
          FindPackageShare('camp'),
            'workspace/13283/13283_2.KAP'
      ])
  )

  return LaunchDescription([
    background_chart_arg,
    Node(
      package='camp',
      executable='CCOMAutonomousMissionPlanner',
      name='camp',
      arguments=[
        PathJoinSubstitution([
          FindPackageShare('camp'),
          'workspace/'
        ]),
        background_chart
      ],
    )
  ])
