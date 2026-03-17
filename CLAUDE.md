# CLAUDE.md — Robotrol v2.0

## Project Structure

```
robotrol_v2/
  robotrol/                  Python package
    __main__.py              Entry point: python -m robotrol
    config/                  Constants, AppConfig, ProfileManager
    serial/                  SerialClient, GRBL/FluidNC protocol parser
    kinematics/              DH model, FK, IK, transforms
    queue/                   G-code queue, CLI history
    visualizer/              UDP mirror for 3D visualizer
    gui/                     tkinter GUI (app coordinator, theme, toolbar)
      panels/                Persistent panels (queue, TCP pose, status)
      tabs/                  Notebook tabs (12 tabs)
    chess/                   Chess vision pipeline
    pickplace/               Pick & Place subsystem
      control/               Robot control, FSM, gripper
      perception/            Camera, markers, pose estimation
      planning/              Grasp/place planning
      simulation/            Mock world, simulation loop
      learning/              TNT self-learning
    tools/                   Calibration tools, math utils
    vision/                  Camera capture, board pose, autocalib
  profiles/                  Robot profiles (EB300.json, Moveo.json, ...)
  configs/                   Application config files
  model/                     DH model data
  data/                      Runtime data (captures, calibration)
  tests/                     pytest test suite
```

## Critical: Robot Profile Protection

**NEVER modify profile JSON files** (profiles/*.json) without explicit user request.
These contain calibrated DH parameters, endstop limits, and gamepad configs.
A wrong value can cause physical damage to the robot.

## Development Rules

1. **Start:** `python -m robotrol` (from robotrol_v2/ directory)
2. **Tests:** `pytest tests/ -v`
3. **Syntax check:** `python -m py_compile <file>.py`
4. **Backend MUST NOT import tkinter** — only gui/ modules may use tkinter
5. **FK verification after kinematics changes:** EB300 TCP=(179,0,860), Moveo TCP=(0,0,920) at MPos=0
6. **All UI text in English**
7. **No bare `except:`** — always use specific exception types
8. **Safety-critical code:** Preserve all limit checks, homing guards, and clamp functions

## Architecture

- **Backend** (config/, serial/, kinematics/, queue/, visualizer/): Pure Python, no GUI dependency, fully testable
- **GUI** (gui/): tkinter-based, references backend via `self.app` (RobotrolApp coordinator)
- **Tabs** are lazy-loaded via `_TAB_REGISTRY` in gui/app.py — failures are logged, not fatal
- **Profiles** drive robot-specific behavior (DH model, endstops, gamepad config)

## Robot Profiles

| Profile | Endstops | Notes |
|---------|----------|-------|
| Moveo | Yes | Default profile, 6-DOF educational arm |
| EB300 | No | Industrial arm, no homing allowed |
| EB15_red | No | Small arm, no homing allowed |

## Governance

Zentrale Standards via `.claude/rules/` (Symlinks nach `Lead/standards/`). Level: **Minimal**.

**Kostenstelle:** `ROBOTROL` (Lead-Controlling, MS-08)

## Inbox-System

| Adresse | Pfad |
|---------|------|
| `robotrol` | `/mnt/NAS5/Projekte/robotrol_v2/data/inbox` |

## QM-Compliance-Matrix

| Standard | Status | Nachweis |
|----------|--------|----------|
| MS-04 Selbstverifikation | Erfuellt | pytest-Tests in `tests/` |
| MS-06 Kein Code im Root | Erfuellt | Code in `robotrol/`, Scripts in `scripts/` |
| MS-08 Kostenstelle | Erfuellt | `ROBOTROL` in CLAUDE.md |
| MS-11 Inbox-Check | Erfuellt | `data/inbox/` vollstaendig |
| MS-15 Selbstheilungs-Mandat | Erfuellt | Grundregeln referenziert |
| SEC-01 Keine Secrets im Code | Erfuellt | Keine Secrets |
