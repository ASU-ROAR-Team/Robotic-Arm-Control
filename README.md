# Robotic-Arm-Control (legacy)

This branch contains the legacy ROAR arm workspace source files. The repository intentionally tracks only source code; build artifacts and generated install/log files are excluded.

Ignored by `.gitignore`:
- `build/` — colcon build outputs
- `install/` — colcon install outputs
- `log/` — build logs

Quick start
1. Clone the repo and checkout `legacy`:

```bash
git clone https://github.com/ASU-ROAR-Team/Robotic-Arm-Control.git
cd Robotic-Arm-Control
git checkout legacy
```

2. Build (recommended in a clean workspace):

```bash
colcon build --merge-install
```

3. Source and run:

```bash
source install/setup.bash
ros2 launch legacy_pkg display.launch.py
```

Notes
- The `build/`, `install/`, and `log/` directories are ignored and will not be committed.
- If you want to re-add any of those directories, remove the matching lines from `.gitignore`.

