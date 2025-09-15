import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('quadro_gz'),
        'urdf',
        'Demo.urdf'
    )

    # spawn robot
    spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-file', urdf_file, '-name', 'demo_robot']
        )

    ld = LaunchDescription()
    # ld.add_action(gazebo_node)
    ld.add_action(spawn_node)

    return ld
