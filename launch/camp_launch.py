from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # Workspace directory only. CAMP is not given a background raster at launch:
  # it draws an OpenStreetMap backdrop of its own, and any charts the operator
  # opens are app state that CAMP persists and restores by itself. Forcing one
  # here also made it stick -- the command-line chart is loaded through
  # openBackground(), which persists it, so it came back on subsequent starts
  # even once the argument was removed.
  return LaunchDescription([
    Node(
      package='camp',
      executable='CCOMAutonomousMissionPlanner',
      name='camp',
      arguments=[
        PathJoinSubstitution([
          FindPackageShare('camp'),
          'workspace/'
        ])
      ],
      emulate_tty=True
    )
  ])
