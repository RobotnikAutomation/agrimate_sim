# agrimate_sim

Workspace repository for the Gazebo simulation of the RB-Fiqus AgriMate robot.
The base simulation package lives under
`agrimate_ws/src/agrimate/agrimate_simulation`.

The Robotnik private stack under development is intentionally external to this
repository. If you have access to it, install it through
`dependencies/repos/robotnik.repos` as the multi-package repository
`agrimate/robotnik_agrimate`.

## What This Repository Provides

- `agrimate_simulation`: Gazebo world, simulation assets, resource hooks, and
  the base launch file for spawning the robot in simulation.
- A workspace layout that can run the simulation on its own.
- A `robotnik.repos` file for importing the private Robotnik stack when access
  is available.

## Requirements

- ROS 2 Jazzy installed and configured.
- Gazebo Ignition environment available through ROS 2.
- The robotnik_simulation instalation guide completed.
- Workspace dependencies installed with `rosdep`.
- The specific `robotnik_controllers` package for RB-Fiqus Ackermann available
  through the packaged Debian artifact included in this repository. The
  conventional `robotnik_controllers` package shipped with `simulation` is not
  valid for this setup.

The repos files use HTTPS URLs by default. If you prefer SSH, you can
rewrite GitHub HTTPS URLs globally:

```bash
git config --global url."git@github.com:".insteadOf "https://github.com/"
```

To remove that rewrite later:

```bash
git config --global --unset url.git@github.com:.insteadOf
```

## Clone

```bash
git clone https://github.com/RobotnikAutomation/agrimate_sim.git
```

## Robotnik Controllers Debian Package

This repository also includes the packaged `robotnik_controllers` binary needed
for the RB-Fiqus Ackermann simulation to work correctly when the source
repository is not being used directly. This specific package is required for
correct behavior of the Ackermann platform; the conventional
`robotnik_controllers` package from `simulation` does not work for this robot.

The current package is:

```text
debs/ros-jazzy-robotnik-controllers_1.3.0-20260505.113947-639e4c7_amd64.deb
```

Install this package before building the workspace. Do not rely on the
conventional `robotnik_controllers` package from `simulation` for this setup.

## Commands to find, uninstall, and install the .deb package

If you need to check whether the package is installed, remove it, or (re)install the version included in this repository, use the following commands.

- Check if the package is installed (shows name and version):

```bash
dpkg -l | grep robotnik
```

- Remove the package if present:

```bash
sudo dpkg -r ros-jazzy-robotnik-controllers
sudo apt autoremove -y
sudo rm -f /var/cache/apt/archives/ros-jazzy-robotnik-controllers*.deb
```

- Install the correct version included in this repository (from the repo root):

```bash
cd agrimate_sim
sudo apt update
sudo apt install ./debs/ros-jazzy-robotnik-controllers_1.3.0-20260505.113947-639e4c7_amd64.deb
```

If `apt` fails due to dependencies, use:

```bash
sudo dpkg -i debs/ros-jazzy-robotnik-controllers_1.3.0-20260505.113947-639e4c7_amd64.deb
sudo apt -f install -y
```

## Temporary GPS Patch

Until the RTK node is available, apply a temporary patch to
[gps_plugin.urdf.xacro](/home/jlgalan/repos/agrimate_github_prep/agrimate_sim/agrimate_ws/src/robotnik/robotnik_sensors/robotnik_sensors/urdf/gps/gps_plugin.urdf.xacro)
to reduce the simulated NavSat positioning error.

File to edit:

```text
agrimate_sim/agrimate_ws/src/robotnik/robotnik_sensors/robotnik_sensors/urdf/gps/gps_plugin.urdf.xacro
```

Change these values in `position_sensing`:

```text
1.75e-6  -> 1.75e-8
1.35e-6  -> 1.35e-8
```

This patch is only a workaround to stabilize the GPS behavior while the RTK
solution is not yet integrated.


## Build And Run Base Simulation

All commands below are executed from `agrimate_ws`:

```bash
cd agrimate_sim/agrimate_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to agrimate_simulation
source install/setup.bash
ros2 launch agrimate_simulation simulation.launch.py
```

To inspect the available launch arguments:

```bash
ros2 launch agrimate_simulation simulation.launch.py --show-args
```

## Private Robotnik Stack

If you have access to the private Robotnik AgriMate stack under development,
import it into the workspace with:

```bash
cd agrimate_sim
vcs import agrimate_ws/src < dependencies/repos/robotnik.repos
```

## Notes

- The base simulation remains usable without the private `robotnik_agrimate`
  stack.
- If the build fails, ensure `source /opt/ros/jazzy/setup.bash` has been run in
  the current shell.
- If `colcon build` fails on `gz_ros2_control`, rerun the build once before
  investigating further.
