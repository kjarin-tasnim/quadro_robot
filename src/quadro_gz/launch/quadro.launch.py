from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ign_pkg_share = get_package_share_directory('ros_gz_sim')
    quadro_desc_pkg_share = get_package_share_directory('quadro_description')

    gz_sim_launch = PathJoinSubstitution(
        [ign_pkg_share, 'launch', 'gz_sim.launch.py']
    )

    spawn_entity_launch = PathJoinSubstitution(
        [quadro_desc_pkg_share, 'launch', 'spawn.launch.py']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity_launch),
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])