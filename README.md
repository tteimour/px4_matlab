# PX4 Multicopter Simulation — Windows + WSL2 setup (`main_windows`)

MATLAB replication of the PX4 multicopter stack (controllers, sensor models,
EKF2) with a ground-control GUI, a Unity/Cesium 3D world, and an optional
OpenVINS visual-inertial odometry pipeline that can replace GNSS aiding.

This branch adapts the simulation to a **Windows PC**: MATLAB and Unity run
natively on Windows, everything ROS/Ubuntu-related runs inside **WSL2**, and
the two sides talk exclusively over the rosbridge WebSocket. Unlike the Linux
setup — where MATLAB auto-launches rosbridge and OpenVINS itself — on Windows
**you start the Ubuntu-side processes manually in WSL terminals** (the GUI
prints the exact commands when needed).

```
WINDOWS                                        WSL2 (Ubuntu 22.04)
─────────────────────────                      ─────────────────────────────
MATLAB  run_interactive.m  ──ws://localhost:9090──►  rosbridge_server
  • pose   (CesiumBridgeWs)                              │ (DDS stays in WSL)
  • IMU    (ImuBridgeWs)                                 ▼
  • VIO odom poll (VioOdomSubWs) ◄──────────  OpenVINS (ov_msckf)
                                                          ▲
Unity (Cesium world)  ─────ws://localhost:9090──►  unity_image_bridge
  camera images, reads vehicle pose
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
- The OpenVINS pipeline built inside WSL at `~/git/openVins/`
  (copy the two workspaces from the Linux machine or rebuild them):
  ```bash
  cd ~/git/openVins/open_vins   && colcon build
  cd ~/git/openVins/openvins_ws && colcon build
  ```

## 2. One-time MATLAB configuration

1. Point MATLAB at your Python and install roslibpy **into that Python**:
   ```matlab
   pyenv('Version', 'C:\Path\To\Python311\python.exe')   % then restart MATLAB
   ```
   ```powershell
   C:\Path\To\Python311\python.exe -m pip install roslibpy
   ```
2. If your WSL distro name or Linux username differ from the defaults,
   adjust the config path in `sim/run_interactive.m` → `vioCfgPaths()`:
   ```
   \\wsl$\Ubuntu-22.04\home\synapgnc\git\openVins\open_vins\config\matlab_unity
   ```
   (You can check the right UNC path in Explorer under `\\wsl$\`.)

## 3. Run — basic flight (no VIO)

| Where | What |
|---|---|
| WSL terminal 1 | `source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| Unity | press **Play** |
| MATLAB | `run_interactive` |

In the GUI: **ARM** (or just hit **TAKEOFF**, which auto-arms), fly with the
stick console, switch modes from the bottom bar. The vehicle pose streams to
Unity automatically while the Cesium toggle is on (default).

## 4. Run — with VIO (OpenVINS)

Do the three steps above first, then:

1. **MATLAB, VIO tab** — set the OpenVINS parameters if needed and tick
   **Enable VIO**. This writes the parameters into the WSL config files
   through `\\wsl$` and prints the launch command. (The in-GUI camera /
   feature image views stay disabled on Windows; OpenVINS gets its images
   from Unity inside WSL, not through MATLAB.)
2. **WSL terminal 2** — launch OpenVINS:
   ```bash
   source /opt/ros/humble/setup.bash && \
   source ~/git/openVins/open_vins/install/setup.bash && \
   source ~/git/openVins/openvins_ws/install/setup.bash && \
   ros2 launch openvins_matlab_bridge openvins_matlab_unity.launch.py
   ```
3. Fly up to ~30 m before/while enabling — a wider ground footprint gives the
   tracker far more features. `/ov_msckf/odomimu` then feeds the live map
   trails and the comparison plots; tick **Fuse VIO → EKF** to replace GNSS
   aiding with the odometry.

Changing VIO parameters later: untick + retick **Enable VIO** in MATLAB
(rewrites the configs), then restart the OpenVINS launch in its WSL terminal
(Ctrl-C, run again).

## 5. Differences vs the Linux setup

| | Linux | Windows (this branch) |
|---|---|---|
| rosbridge / OpenVINS | auto-launched & stopped by MATLAB | started manually in WSL terminals |
| Pose / IMU to ROS | native DDS nodes | rosbridge WebSocket (`CesiumBridgeWs`, `ImuBridgeWs`) |
| VIO odometry into MATLAB | DDS subscriber callback | WebSocket, polled once per frame (`VioOdomSubWs`) |
| VIO-tab image views | live | disabled (diagnostic only; not needed by the pipeline) |
| OpenVINS config edits | direct file path | through the `\\wsl$` share |

## 6. Troubleshooting

- **`roslibpy not found`** — `pyenv` points at the wrong Python, or pip
  installed into another one. Both bridges print this check first.
- **Bridge connect timeout** — rosbridge isn't running in WSL, or Unity/
  firewall grabbed port 9090. `wsl -e bash -lc "ss -ltnp | grep 9090"`.
- **`ReactorNotRestartable`** — Python's Twisted reactor survived a previous
  session in a bad state; restart MATLAB once.
- **VIO enable errors about the config path** — fix `vioCfgPaths()` (step 2.2).
- **No `/ov_msckf/odomimu`** — OpenVINS needs the IMU stream (Cesium toggle
  on), Unity playing, motion, and enough features; check its WSL terminal
  output.
- **IMU stream stutter** — 200 Hz JSON over the WebSocket is untested on
  every machine; if needed lower `imu_pub_hz` in `sim/run_interactive.m`.

> Status note: the simulation itself is exercised heavily on Linux; the
> Windows WebSocket transport in this branch (`ImuBridgeWs`, `VioOdomSubWs`,
> WSL paths) is new and has not yet been validated end-to-end on a Windows
> machine — expect to touch step 2.2 and report anything that misbehaves.
