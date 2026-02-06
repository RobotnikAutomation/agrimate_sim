from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import TimerAction

def generate_launch_description():

    declared_arguments = []

    set_nvidia_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_nvidia_glx = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    # World file
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_path",
            default_value=PathJoinSubstitution([
                FindPackageShare('robotnik_agrimate'),
                'worlds',
                'test.world'
            ]),
            description=""
        )
    )
    world_path = LaunchConfiguration("world_path")

    # Spawn World
    spawn_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_gazebo_ignition'), 'launch/spawn_world.launch.py'
            ])
        ),
        launch_arguments={
            'world_path': world_path,
        }.items(),
    )

    # Spawn robot (delayed)
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_gazebo_ignition'), 'launch/spawn_robot.launch.py'
            ])
        ),
        launch_arguments={
            'robot': 'rbfiqus',
            'robot_model': 'rbfiqus_agrimate',
            'use_sim_time': 'true',
            'x': '4.0',
            'y': '0.0',
            'z': '0.6',
        }.items(),
    )
    delayed_spawn_robot = TimerAction(
        period=10.0,
        actions=[spawn_robot]
    )


    group = GroupAction([
        set_nvidia_offload,
        set_nvidia_glx,
        spawn_world,
        delayed_spawn_robot,
    ])

    return LaunchDescription(declared_arguments + [group])

