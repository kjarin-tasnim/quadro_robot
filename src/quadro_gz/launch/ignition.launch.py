from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
#launch ignition from ros_ign_gazebo
    pkg_share = get_package_share_directory('ros_gz_sim')

    gz_sim_launch = PathJoinSubstitution(
        [pkg_share, 'launch', 'gz_sim.launch.py']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )


    return LaunchDescription([
        gazebo
    ])