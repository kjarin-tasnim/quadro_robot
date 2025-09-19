from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    ign_pkg_share = get_package_share_directory('ros_gz_sim')
    quadro_desc_pkg_share = get_package_share_directory('quadro_description')
    control_pkg_share = get_package_share_directory('quadro_control')
    gz_args = LaunchConfiguration('gz_args', default='')

    gz_sim_launch = PathJoinSubstitution(
        [ign_pkg_share, 'launch', 'gz_sim.launch.py']
    )

    spawn_entity_launch = PathJoinSubstitution(
        [quadro_desc_pkg_share, 'launch', 'spawn.launch.py']
    )


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])])



    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity_launch),
    )

    controllers = PathJoinSubstitution(
        [control_pkg_share, 'config', 'controllers.yaml']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description')
        ]
        
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    

    # leg1_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['leg_1_controller', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )

    # leg2_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['leg_2_controller', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )
    # leg3_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['leg_3_controller', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )
    # leg4_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',     
    #     arguments=['leg_4_controller', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )

    # base_to_actuator_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['base_to_actuator_controller', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )







    return LaunchDescription([
        bridge,
        gazebo,
        spawn_entity,
        # control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        # leg1_controller_spawner,
        # leg2_controller_spawner,
        # leg3_controller_spawner,
        # leg4_controller_spawner,
        # base_to_actuator_controller_spawner,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])