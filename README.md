# Robotrol

Robotrol is a Python-based control center for 6-DoF robotic arms running FluidNC/GRBL. It combines serial control, a live queue, gamepad jogging, DH-based kinematics (FK/IK + TCP pose), vision tools, and optional OTA configuration.

## Features
- FluidNC/GRBL serial control with queue and live status
- Gamepad jogging with configurable profiles
- DH6 FK/IK and TCP pose monitor
- TCP sequence generator (retreat/target/retreat)
- OpenCV-based camera and board-vision tools
- UDP mirror to 3D visualizer
- Profile system (Moveo, Red15/EB15, EB300)

## Requirements
- Windows 11
- Python 3.10+
- Python packages: pyserial, pygame, Pillow, numpy, opencv-python

## Run
```powershell
python Robotrol_FluidNC_v6_2.py
```

## Profiles
Profiles live in `Moveo.json`, `Red15.json`, and `EB300.json`. Use the profile selector in the UI to switch.

## Repo layout
- `Robotrol_FluidNC_v6_2.py` main UI
- `tcp_pose_module_v3.py` FK/TCP panel
- `tcp_world_kinematics_frame.py` TCP sequence generator
- `robotik_chess_stack/` edge/agent chess stack and specs

## Notes
- Build artifacts (`build/`, `dist/`) and caches are not tracked in Git.
- No license is specified yet.
