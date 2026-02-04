# agrimate_sim

Repository to load the Gazebo simulation of the RB-Fiqus robot for the AGRIMATE project.

## Requirements
- ROS 2 installed and configured.
- Gazebo Ignition.
- Workspace dependencies installed.

Installation:
```
git clone --recursive https://github.com/RobotnikAutomation/agrimate_sim.git
```

## Steps to run the simulation (all inside `agrimate_ws`, which is under `agrimate_sim`)
1. Open a terminal.
2. Go to the workspace:
   ```
   cd agrimate_sim/agrimate_ws
   ```
3. Build the workspace:
   ```
   colcon build
   ```
4. Source the workspace packages into the current environment:
   ```
   source install/setup.bash
   ```
5. Launch the simulation:
   ```
   ros2 launch robotnik_agrimate simulation.launch.py
   ```

## Notes
- If the build fails, ensure you have sourced the ROS 2 setup (`source /opt/ros/jazzy/setup.bash`) and installed all dependencies.
- Run the commands from the `agrimate_ws` directory mentioned above.
- If `colcon build` fails on gz_control, try running it again.

# Don't Work. Dependency error :(
