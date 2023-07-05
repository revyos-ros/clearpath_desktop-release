from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Configurations
    platform_model = LaunchConfiguration('platform_model')
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    arg_platform_model = DeclareLaunchArgument(
        'platform_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='model.rviz',
    )

    pkg_clearpath_platform_description = FindPackageShare('clearpath_platform_description')
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    config_rviz = PathJoinSubstitution(
        [pkg_clearpath_viz, 'rviz', LaunchConfiguration('config')]
    )


    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_rviz],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
               ('/tf', 'tf'),
               ('/tf_static', 'tf_static')
            ],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ("joint_states", "platform/joint_states")
            ]
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                pkg_clearpath_platform_description,
                'launch',
                'description.launch.py'])
        )
    ])


    ld = LaunchDescription()
    # Args
    ld.add_action(arg_platform_model)
    ld.add_action(arg_namespace)
    ld.add_action(arg_rviz_config)
    ld.add_action(arg_use_sim_time)
    # Nodes
    ld.add_action(group_view_model)
    return ld
