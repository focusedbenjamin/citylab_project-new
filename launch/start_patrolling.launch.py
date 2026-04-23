from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Load the RViz config file
    rviz_config_file = os.path.join(get_package_share_directory('robot_patrol'),
                                    'rviz', 'tb3_default.rviz')

    # Launch the 'patrol_node' from the 'robot_patrol' package
    robot_patrol_node = Node(
        package='robot_patrol',
        executable='patrol_node',
        output='screen'
    )

    # Launch the 'rviz2' from the 'rviz2' package
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_patrol_node,
        # rviz2_node,
    ])
