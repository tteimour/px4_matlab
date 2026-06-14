# PX4 Multicopter Simulation — Windows + WSL2 setup (`main_windows_novio`)

MATLAB replication of the PX4 multicopter stack (controllers, sensor models,
EKF2) with a ground-control GUI and a Unity/Cesium 3D world.

This branch adapts the simulation to a **Windows PC**: MATLAB and Unity run
natively on Windows, the ROS/Ubuntu side (rosbridge) runs inside **WSL2**, and
the two sides talk over the rosbridge WebSocket. Unlike the Linux setup — where
MATLAB auto-launches rosbridge — on Windows **you start rosbridge manually in a
WSL terminal** (the GUI prints the exact command when needed).

```
WINDOWS                                        WSL2 (Ubuntu 22.04)
─────────────────────────                      ─────────────────────────────
MATLAB  run_interactive.m  ──ws://localhost:9090──►  rosbridge_server
  • pose   (CesiumBridgeWs)                              │ (DDS stays in WSL)
                                                          ▲
Unity (Cesium world)  ─────ws://localhost:9090──────────┘
  reads vehicle pose
```

Native DDS **cannot** cross the Windows/WSL2 boundary — do not try to make
MATLAB's ROS Toolbox see WSL topics directly. Every MATLAB↔ROS link on this
branch goes through rosbridge instead (no MATLAB ROS Toolbox required on
Windows).

---

## 1. Prerequisites

**Windows side**
- MATLAB R2024b (no ROS Toolbox needed on this branch; Mapping Toolbox is
  optional — it only provides the satellite basemap on the Flight tab).
- A CPython 3.9–3.12 install (e.g. python.org installer).
- The Unity project with the Cesium world (same project as on Linux); its
  rosbridge endpoint must point at `ws://localhost:9090`.

**WSL side**
- Windows 10 21H2+ / Windows 11 with WSL2:
  ```powershell
  wsl --install -d Ubuntu-22.04
  ```
  Keep the default **NAT** networking (do not switch to mirrored mode);
  Windows→WSL `localhost` forwarding is what the WebSocket relies on.
- ROS 2 Humble + rosbridge inside WSL:
  ```bash
  sudo apt install ros-humble-desktop ros-humble-rosbridge-suite
  ```

## 2. One-time MATLAB configuration

Point MATLAB at your Python and install roslibpy **into that Python**:
```matlab
pyenv('Version', 'C:\Path\To\Python311\python.exe')   % then restart MATLAB
```
```powershell
C:\Path\To\Python311\python.exe -m pip install roslibpy
```

## 3. Run — flight

| Where | What |
|---|---|
| WSL terminal | `source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| Unity | press **Play** |
| MATLAB | `run_interactive` |

In the GUI: **ARM** (or just hit **TAKEOFF**, which auto-arms), fly with the
stick console, switch modes from the bottom bar. The vehicle pose streams to
Unity automatically while the Cesium toggle is on (default).

## 4. Differences vs the Linux setup

| | Linux | Windows (this branch) |
|---|---|---|
| rosbridge | auto-launched & stopped by MATLAB | started manually in a WSL terminal |
| Pose to ROS | native DDS node | rosbridge WebSocket (`CesiumBridgeWs`) |

## 5. Troubleshooting

- **`roslibpy not found`** — `pyenv` points at the wrong Python, or pip
  installed into another one. The Cesium bridge prints this check first.
- **Bridge connect timeout** — rosbridge isn't running in WSL, or Unity/
  firewall grabbed port 9090. `wsl -e bash -lc "ss -ltnp | grep 9090"`.
- **`ReactorNotRestartable`** — Python's Twisted reactor survived a previous
  session in a bad state; restart MATLAB once.

> Status note: the simulation itself is exercised heavily on Linux; the Windows
> WebSocket transport in this branch (`CesiumBridgeWs`, WSL paths) is new and
> has not yet been validated end-to-end on a Windows machine.
