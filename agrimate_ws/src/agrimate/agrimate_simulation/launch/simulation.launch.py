from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import TimerAction

def generate_launch_description():

    set_nvidia_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_nvidia_glx = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    declared_arguments = [
        DeclareLaunchArgument("robot_id", default_value="robot"),
        DeclareLaunchArgument("robot", default_value="rbfiqus_agrimate"),
        DeclareLaunchArgument("robot_model", default_value="rbfiqus_agrimate"),
        DeclareLaunchArgument("x", default_value="4.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.6"),
        DeclareLaunchArgument("run_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("low_performance_simulation", default_value="false"),
        DeclareLaunchArgument("robot_spawn_delay", default_value="10.0"),
        DeclareLaunchArgument(
            "world_path",
            default_value=PathJoinSubstitution([
                FindPackageShare('agrimate_simulation'),
                'worlds',
                'test.world'
            ]),
        ),
    ]

    spawn_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_gazebo_ignition'), 'launch/spawn_world.launch.py'
            ])
        ),
        launch_arguments={
            'world_path': LaunchConfiguration("world_path"),
            'gui': LaunchConfiguration("gazebo_gui"),
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_gazebo_ignition'), 'launch/spawn_robot.launch.py'
            ])
        ),
        launch_arguments={
            'robot_id': LaunchConfiguration("robot_id"),
            'robot': LaunchConfiguration("robot"),
            'robot_model': LaunchConfiguration("robot_model"),
            'x': LaunchConfiguration("x"),
            'y': LaunchConfiguration("y"),
            'z': LaunchConfiguration("z"),
            'run_rviz': LaunchConfiguration("run_rviz"),
            'rviz_config': LaunchConfiguration("rviz_config"),
            'use_sim_time': LaunchConfiguration("use_sim_time"),
            'low_performance_simulation': LaunchConfiguration("low_performance_simulation"),
        }.items(),
    )
    delayed_spawn_robot = TimerAction(
        period=LaunchConfiguration("robot_spawn_delay"),
        actions=[spawn_robot]
    )


    group = GroupAction([
        set_nvidia_offload,
        set_nvidia_glx,
        spawn_world,
        delayed_spawn_robot,
    ])

    return LaunchDescription(declared_arguments + [group])
