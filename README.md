# Robotic-Arm-Control

This workspace contains the current six degree of freedom arm simulation, MoveIt configuration, GUI pose teleop, workspace visualizer, and the one-shot launcher that starts the full stack cleanly.

## Packages

- `sixdof_pkg`: robot URDF, meshes, Gazebo launch
- `sixdof_moveit`: MoveIt config, RViz config, complete launch
- `src/scripts/teleop.py`: GUI pose teleop for the arm and gripper
- `src/scripts/workspace.py`: RViz workspace cloud visualizer
- `src/scripts/start_complete_stack.py`: clean launcher for the complete stack plus helper scripts

## Build

From the workspace root:

```bash
cd /home/roar/6dof_arm/Robotic-Arm-Control
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

After the build:

```bash
cd /home/roar/6dof_arm/Robotic-Arm-Control
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Recommended Launch

The recommended entrypoint is the unified launcher:

```bash
cd /home/roar/6dof_arm/Robotic-Arm-Control
python3 src/scripts/start_complete_stack.py
```

What it does:

- Kills lingering Gazebo, RViz, MoveIt, controller spawner, and helper-script processes
- Syncs the current RViz config from `src/sixdof_moveit/config/moveit.rviz` into the installed config path
- Launches `ros2 launch sixdof_moveit complete.launch.py`
- Starts `src/scripts/teleop.py`
- Starts `src/scripts/workspace.py`

## Direct Launch Options

Full stack:

```bash
ros2 launch sixdof_moveit complete.launch.py
```

Gazebo only:

```bash
ros2 launch sixdof_pkg gazebo.launch.py
```

Display / RViz only:

```bash
ros2 launch sixdof_pkg display.launch.py
```

## Teleop GUI

Run manually if needed:

```bash
python3 src/scripts/teleop.py
```

The teleop GUI is built around full 6DOF pose control.

### What it does

- Moves the end effector in world-frame `+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`
- Can solve with `fixed orientation` on or with `position only` mode
- Does not lock the last joints anymore
- Uses full pose IK when fixed orientation is enabled
- Lets you capture the current tool orientation and reuse it as the pose target
- Lets you apply suggested orientation presets and then jog XYZ while maintaining that orientation
- Includes gripper controls with one slider plus `Open` and `Close` buttons

### Orientation controls

The GUI includes:

- `Maintain fixed orientation`
- `Capture Current`
- `Apply Here`
- `Look Forward`
- `Look Down`
- `Look Up`
- `Look Right`
- `Look Left`
- editable `Roll`, `Pitch`, `Yaw`
- `Apply RPY`

Notes:

- The preset buttons are intentionally approximate, not mathematically rigid canonical poses. They are meant to give you useful starting orientations for testing 6DOF fixed-orientation planning.
- The kinematics config is set to `position_only_ik: false`, so fixed-orientation pose solving is enabled.

### Gripper controls

The GUI includes a single gripper slider that commands both fingers together.

- Slider range: `0.0 m` to `0.07 m`
- `Open` button uses a buffered near-max opening to avoid pushing exactly to the hard limit
- `Close` button uses a buffered near-min opening to avoid pushing exactly to the hard limit

The URDF currently uses:

- `right_gripper` upper limit: `0.07`
- `left_gripper` upper limit: `0.07`
- mirrored axis directions so both fingers move correctly in Gazebo under the same command

## Workspace Visualizer

Run manually if needed:

```bash
python3 src/scripts/workspace.py
```

Default behavior:

- Samples `4000` random valid FK configurations by default
- Publishes a denser workspace cloud than before
- Publishes the current end-effector marker live

Topics:

- `/workspace_cloud`
- `/workspace_ee_pos`

## RViz Defaults

The default RViz config lives at:

- `src/sixdof_moveit/config/moveit.rviz`

It now includes the workspace marker displays by default:

- `Workspace Cloud` from `/workspace_cloud`
- `Workspace EE` from `/workspace_ee_pos`

So when you launch through `start_complete_stack.py`, those marker topics are enabled automatically in RViz.

## Troubleshooting

### Stale colcon prefix warnings

If you see warnings about missing paths in `AMENT_PREFIX_PATH` or `CMAKE_PREFIX_PATH`, rebuild from a clean ROS environment:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Gazebo or controller warnings on rerun

Use the unified launcher instead of manually rerunning multiple launch files. It clears stale Gazebo and ROS processes before startup.

### Fixed-orientation planning fails

If fixed-orientation motion cannot be planned for a target:

- capture the current orientation first
- try a nearby preset
- slightly relax the requested pose by changing XYZ step size
- temporarily switch `Maintain fixed orientation` off to verify the position is reachable

### RViz does not show the workspace markers

If you launched through `start_complete_stack.py`, the RViz config should already contain the marker displays.

If you launched manually, ensure:

- `workspace.py` is running
- RViz fixed frame is `world`
- the marker topics are enabled in the Displays panel
