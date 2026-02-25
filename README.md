# New_ROAR_Arm

ROAR Arm control and analysis workspace (MoveIt + Gazebo + operator scripts).

This repo includes:
- Motion stack launch (`ROAR_MoveIT`)
- Interactive teleoperation scripts (`src/scripts`)
- Workspace/reachability analysis scripts (`src/scripts/workspace.py`, `src/scripts/workspace_checker.py`)

---

## 1) Build and launch

### Terminal 1 — Build (once) + launch stack

```bash
cd /home/roar/new_arm/New_ROAR_Arm
colcon build --symlink-install
source install/setup.bash
ros2 launch ROAR_MoveIT complete.launch.py
```

If already built, skip `colcon build` and just run:

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
ros2 launch ROAR_MoveIT complete.launch.py
```

This launch brings up MoveIt and simulation components required by the scripts.

---

## 2) Script overview (what each script does)

All scripts below are in `src/scripts/`.

### `teleop.py`
- Main interactive teleop terminal.
- Supports world-frame XYZ translation and joint-space rotation shortcuts.
- Orientation lock mode:
  - `c` locks `Joint_4` + `Joint_5` at current values.
  - `u` unlocks orientation.

### `workspace.py` (RViz reachability cloud)
- Generates and publishes reachable EE positions as a marker cloud to RViz.
- Static compute at startup, then republishes cached markers.
- Also publishes a live yellow sphere for current EE position.

### `workspace_checker.py` (terminal report)
- Computes FK samples and prints workspace bounds + reachable translation from current EE pose.
- Supports lock-aware mode with:
  - `--lock` to hold `Joint_4` and `Joint_5` fixed while sampling.
- Reports rotation headroom (remaining degrees to limits), including teleop mapping:
  - `rz -> Joint_1`, `ry -> Joint_2`, `rx -> Joint_3`.

---

## 3) Recommended runtime workflow (by terminal)

### Terminal 1 — ROS/MoveIt stack

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
ros2 launch ROAR_MoveIT complete.launch.py
```

### Terminal 2 — Teleop control

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
python3 src/scripts/teleop.py
```

### Terminal 3 — RViz reachability cloud

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash
python3 src/scripts/workspace.py --samples 2000 --point-size 0.008
```

### Terminal 4 — Numeric workspace checker (optional)

```bash
cd /home/roar/new_arm/New_ROAR_Arm
source install/setup.bash

# Normal mode (all joints free)
python3 src/scripts/workspace_checker.py --samples 2000

# Orientation-lock aware mode (J4/J5 fixed)
python3 src/scripts/workspace_checker.py --samples 2000 --lock
```

---

## 4) RViz reachability setup

When `workspace.py` is running:

1. Open RViz (from the launch).
2. Set **Fixed Frame** to `world`.
3. Add marker topics:
   - Add → By topic → `/workspace_cloud` → `MarkerArray`
   - Add → By topic → `/workspace_ee_pos` → `MarkerArray`

What you should see:
- Reachability point cloud of EE positions (color by height).
- White bounding box around sampled workspace.
- Yellow sphere tracking live EE position.

---

## 5) Teleop command quick reference

Inside `teleop.py` terminal:

- XYZ movement:
  - `w/s` for ±X
  - `a/d` for ±Y
  - `q/e` for ±Z
  - Optional magnitude in cm: e.g. `w 5`

- Joint-space rotation shortcuts:
  - `rz` / `rz-` → rotate via `Joint_1`
  - `ry` / `ry-` → rotate via `Joint_2`
  - `rx` / `rx-` → rotate via `Joint_3`
  - Optional magnitude in degrees: e.g. `rz 10`

- Orientation lock:
  - `c` lock `Joint_4` + `Joint_5`
  - `u` unlock

- Utility:
  - `p` print current status
  - `x` quit

---

## 6) Troubleshooting

- If a script cannot move or solve goals:
  - Confirm Terminal 1 launch is running.
  - Confirm `source install/setup.bash` was run in every terminal.

- If checker reports TF lookup failures (e.g., `world` frame missing):
  - Wait a few seconds after launch and retry.
  - Verify your RViz/TF tree includes `world` and `Link_5`.

- If workspace sampling is slow:
  - Reduce samples (`--samples 500` or `--samples 1000`) for faster feedback.

---

## 7) Practical tips

- Use `workspace.py` for visual intuition in RViz.
- Use `workspace_checker.py --lock` to get the realistic translational envelope while holding orientation joints.
- Use smaller sample counts during tuning, then increase for final measurement quality.

