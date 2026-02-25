# Robotic-Arm-Control

This repository contains the ROAR arm MoveIt setup and three teleoperation scripts (legacy workspace).

## Build & install (single-terminal: RViz + Gazebo)

Open Terminal 1 and run:

```bash
# from repo root
cd /home/roar/new_arm/Robotic-Arm-Control
colcon build --symlink-install
source install/setup.bash

# Launch the full MoveIt / simulation stack (RViz & Gazebo)
ros2 launch ROAR_MoveIT complete.launch.py
```

Notes:
- If `colcon build` is already done, just `source install/setup.bash` then run the `ros2 launch` line.
- Use Ctrl+C to stop the launch.

You can also launch the display or Gazebo-only for the package that contains the robot model:

```bash
ros2 launch ROAR_pkg display.launch.py
ros2 launch ROAR_pkg gazebo.launch.py
```

## Teleop scripts (Terminal 2)

Open Terminal 2, source the workspace, then run any of the scripts:

```bash
cd /home/roar/new_arm/Robotic-Arm-Control
source install/setup.bash

# Simple mover (position-only XYZ teleop)
python3 src/scripts/simple_mover.py

# Wrist-lock teleop (lock pitch + twist joints, then move XYZ)
python3 src/scripts/wrist_lock_teleop.py

# Macro orientation teleop (save/apply orientation macros + XYZ teleop)
python3 src/scripts/macro_orientation_teleop.py
```

### simple_mover.py
- Purpose: Move the end effector in X/Y/Z by small increments using MoveIt position constraints.
- Usage:
	- Single-letter commands: `w` `a` `s` `d` `q` `e` move 1 cm by default along axes.
	- Provide a distance in cm: `w 5` moves 5 cm forward. Also accepts `w5`.
	- `x` quits.

### wrist_lock_teleop.py
- Purpose: Lock `Joint_4` (pitch) and `Joint_5` (twist) to current values, then move XYZ while holding wrist orientation.
- Usage:
	- `c` locks the current pitch + twist (reads from `joint_states`).
	- `u` unlocks wrist (free orientation).
	- Movement commands same as `simple_mover.py` (`w`, `w 5`, etc.).
	- `x` quits.

### macro_orientation_teleop.py
- Purpose: Save orientation macros (pitch+twist) to slots and lock to them while moving XYZ.
- Usage:
	- `save 1` / `save 2` / `save 3` — save current orientation to slot.
	- `lock 1` / `lock 2` / `lock 3` — lock to a saved slot.
	- `u` unlocks (free movement).
	- `p` prints all saved macros.
	- Movement commands same as above (`w`, `w 5`, etc.).
	- `x` quits.

## Notes & troubleshooting
- Ensure TF frames are available (for example `world` and `Link_5`) and `joint_states` are being published.
- If TF lookup fails, the scripts will log an error and ignore the command.
- If MoveIt rejects a goal, check the planner/scene and joint limits.

---
Generated for this workspace; scripts live in `src/scripts` and MoveIt/launch files are in `src/ROAR_MoveIT` and `src/ROAR_pkg`.

