# arm-control

Teach-and-repeat control for a **Franka Research 3** arm with exchangeable passive grippers,
running on a **Raspberry Pi 4** with a realtime kernel and operated from a web page.

- Choose the mounted gripper; every gripper has a **pick** and a **place** sequence that start and
  end above the object (e.g. forklift style: move to the side, slide under the part, lift).
- Record sequences by moving the arm by hand (**free drive**) or by jogging, with waypoints, waits,
  **guarded moves** (move until contact) and **presses** (force controlled, optionally rotating).
- Teach named **positions** on a desk map by moving the arm there. Clicking a position picks there
  if the gripper is empty and places there if it holds a part. Sequences are replayed relative to
  the position; by default they turn with the direction as seen from the robot base.
- Robot access from the UI: unlock the brakes and activate FCI, or hand the robot back to Desk.

## Architecture

```
browser ──HTTP──► teach_server.py (Flask, port 8080) ──UDP──► franka_teach_rt (C++, libfranka) ──► FR3
                    application logic, UI, storage            1 kHz realtime loop on core 3
```

| Path | What |
|---|---|
| `app/rt/franka_teach_rt.cpp` | Realtime process: modes *idle* (robot holds itself), *follow* (smooth Cartesian target following + force mode), *free* (gravity compensation with damping). Joint-velocity governor, automatic recovery after reflexes. |
| `app/scripts/teach_server.py` | Web server and application logic (modes, jogging, recording, sequence execution, positions, Desk actions). |
| `app/scripts/teach_core.py` | ROS-free geometry and storage: sequence transforms, desk frame, JSON files in `~/franka_teach_data`. |
| `app/scripts/robot_link.py` | UDP protocol to `franka_teach_rt` (commands to 127.0.0.1:5555, state at 50 Hz to :5556). |
| `app/scripts/desk.py` | Minimal client for the Desk web API (take control, brakes, FCI). Undocumented by Franka. |
| `app/web/index.html` | The UI (tabs Operate / Grippers / Positions / Manual). |
| `app/test/` | `test_teach_core.py` (geometry) and `sim_test.py` (whole server against a simulated robot; `--serve` runs the UI on :8090 for UI work without a robot). |
| `pi-setup/` | Pi setup scripts, systemd services, realtime tuning, libfranka test tools. |

ROS is not used at runtime: on the Pi, `ros2_control`/franka_ros2 lost ~0.2 % of control packets
(plus a mode-switch race in franka_hardware), and every lost command during motion triggered
acceleration-discontinuity reflexes. libfranka directly runs at 100 %. ROS 2 Jazzy is only
installed for libfranka's dependencies (pinocchio).

## Setup on a new Pi

Ubuntu Server 24.04 (64-bit) via Raspberry Pi Imager, then on the Pi (user `anton`, scripts in
`pi-setup/`):

1. `sudo bash 01_base_ros.sh` – updates, swap, ROS 2 Jazzy base, realtime limits
2. Realtime kernel (Ubuntu Pro, free for personal use):
   `sudo pro attach <token> && sudo pro enable realtime-kernel --variant=raspi`
3. `bash 02_build_libfranka.sh` – libfranka 0.20.5 (needs FR3 system ≥ 5.9.0)
4. `sudo bash 03_realtime_tuning.sh` and reboot – isolated core 3, eth0 IRQs on it at FIFO 98
5. `sudo bash 04_connect_robot.sh <robot-ip>` – static address on eth0 in the robot's subnet
6. `bash 05_install_app.sh` – builds `franka_teach_rt`, installs and starts the services
   `franka-teach-rt` and `franka-teach-ui` (robot IP in `/etc/default/franka-teach`)

Notes from our setup: the FR3 is at 192.168.5.10, the Pi's eth0 at 192.168.5.12/24, the laptop
reaches the Pi over WiFi (a network without client isolation; the Pi 4 needs
`key-management: psk-sha256` in netplan for WPA2/PMF networks). The Pi has no RTC, so NTP must stay
enabled. Desk is not reachable from the laptop directly; use the buttons in the UI or an SSH tunnel
(`ssh -L 8443:192.168.5.10:443 <pi>` → https://127.0.0.1:8443/desk/).

## Using it

Open `http://<pi>:8080`.

1. **Positions → Robot access**: save the Desk login once, then **Unlock & activate FCI** (also in
   the header while the robot is not connected). If someone else holds control in Desk, confirm with
   the circle button on the Pilot.
2. **Grippers**: add a gripper and set its mass (the robot's force estimate needs it), record pick
   and place.
3. **Positions**: teach positions by moving the arm above them (free drive) and saving; optionally
   set up the desk coordinate system from two points.
4. **Operate**: select the mounted gripper, click a position → *Pick here* / *Place here*. Clicking
   an empty spot offers *Go here* (drives there at the current height).

Modes (header): **Hold** – robot holds itself; **Free drive** – push the arm by hand (Pilot-button
guiding is disabled while FCI is active); **Control** – the arm is stiff and follows jog and sequence
commands.

## Development

Tests run on any machine with Python 3 and Flask (no robot, no ROS):

```
python3 app/test/test_teach_core.py
python3 app/test/sim_test.py            # all scenarios against a simulated robot
python3 app/test/sim_test.py --serve    # UI with demo data on http://127.0.0.1:8090
```

`pi-setup/tools/` has two standalone libfranka checks: `hold_test` (connection quality while the
robot holds still) and `move_test` (2 cm up and down).

## Lessons from the real robot

- The first Cartesian command must equal `O_T_EE_c`, and commands must be jerk-limited
  (planner + two 15 ms filters + libfranka's rate limiter).
- The FR3's joint velocity limits depend on the joint position (zero at the ends of the range);
  near them Cartesian moves cause `joint_velocity_violation`. Prefer positions in the middle of the
  workspace; the automatic angle mode keeps the wrist from twisting.
- `O_F_ext_hat_K` is the force the robot exerts (pressing down reads negative z) and has a few N of
  drifting offset; forces are tared before guarded moves and presses.
