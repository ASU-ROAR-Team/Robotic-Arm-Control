# New_ROAR_Arm

ROAR/ROVER robotic arm control workspace (MoveIt + Gazebo + operator scripts).

## What is new

This workspace now includes two new rover-focused packages for the arm mounted on the rover platform:

- `rover_pkg`:
  - URDF + meshes for the arm-on-rover model
  - Gazebo launch for spawning the rover arm system
- `rover_moveit`:
  - MoveIt configuration (SRDF, controllers, kinematics, planning config)
  - Full demo/complete launch integration

These packages model and run the arm strapped onto the rover.

---

## Main launch (rover stack)

Use this stack for all rover arm tests and scripts:

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
ros2 launch rover_moveit complete.launch.py
```

If you changed package config/URDF/SRDF, rebuild first:

```bash
cd /home/roar/new_arm/New_ROAR_Arm
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_moveit complete.launch.py
```

---

## rover_scripts overview

All rover operator/testing scripts are in `src/rover_scripts/`.

- `teleop.py` — full XYZ + joint rotation + orientation lock teleop
- `wrist_lock_teleop.py` — XYZ teleop while locking wrist joints
- `macro_orientation_teleop.py` — save/recall orientation macros + XYZ moves
- `simple_mover.py` — simple interactive relative XYZ mover
- `test_runner.py` — automated reliability trial runner
- `workspace.py` — RViz workspace cloud visualizer
- `workspace_checker.py` — numeric workspace bounds/checker
- `collision_guard.py` — shared collision pre-check helper used by motion scripts

---

## How the new collision guard works

`collision_guard.py` is not a standalone CLI script.
It is imported by rover motion scripts and checks current state validity through MoveIt (`/check_state_validity`) before sending goals.

If current state is colliding (or cannot be validated), motion commands are blocked.

---

## How to run the new rover scripts (with collision guard)

### 1) Start rover stack

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
ros2 launch rover_moveit complete.launch.py
```

### 2) In a second terminal, run any rover script

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
python3 src/rover_scripts/teleop.py
```

Other examples:

```bash
python3 src/rover_scripts/wrist_lock_teleop.py
python3 src/rover_scripts/macro_orientation_teleop.py
python3 src/rover_scripts/simple_mover.py
python3 src/rover_scripts/test_runner.py --trials 10 --modes locked --steps 5,8
python3 src/rover_scripts/workspace.py --samples 2000
python3 src/rover_scripts/workspace_checker.py --samples 2000 --lock
```

---

## Notes

- If a script logs `Blocked by collision guard`, MoveIt reported the current state as invalid/colliding for the active planning group.
- Ensure every terminal runs `source install/setup.bash` before launching scripts.
- If you changed SRDF/URDF collision settings, rebuild and relaunch before testing again.
