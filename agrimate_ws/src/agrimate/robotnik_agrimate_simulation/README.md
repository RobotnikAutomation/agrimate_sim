# robotnik_agrimate_simulation

Gazebo simulation package for the AgriMate project. It contains the simulation
worlds, vineyard models, resource hooks, and base launch file for spawning the
RB-Fiqus AgriMate robot.

## Launch

```bash
ros2 launch robotnik_agrimate_simulation simulation.launch.py
```

## Contents

- `launch/simulation.launch.py`: starts the world and spawns the robot.
- `worlds/`: Gazebo world files.
- `models/`: simulation assets used by the worlds.
- `hooks/`: Gazebo resource path setup.

## Build

From the workspace root:

```bash
colcon build --packages-select robotnik_agrimate_simulation
source install/setup.bash
```

## Troubleshooting

If Gazebo crashes when importing a Blender-generated model, check whether the
OBJ export was created with "Selection Only" and no selected object.

If Gazebo cannot find worlds or models after sourcing the workspace, check the
generated resource paths first:

```bash
env | grep -E 'GZ_SIM_RESOURCE_PATH|IGN_GAZEBO_RESOURCE_PATH|GAZEBO_RESOURCE_PATH'
```

This package uses a Gazebo resource hook. A previous package name
(`agrimate_simulation`) could make the hook run before `ament_prefix_path`,
which left the resource variables incomplete. The package was renamed to
`robotnik_agrimate_simulation` to keep the hook ordering stable.

For the full analysis and root cause, see
[doc/gazebo_resource_hook_bug_report.md](./doc/gazebo_resource_hook_bug_report.md).
