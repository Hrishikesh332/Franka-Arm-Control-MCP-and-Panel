# Autonomous-Gripper

Tools for a Franka Research 3 (FR3) arm: an MCP server that lets an AI client observe and
operate it, and a teach-and-repeat web panel. The two projects are independent and share no code.

| Folder | What it is | Runs on |
|---|---|---|
| [`Franka_Arm_MCP/`](Franka_Arm_MCP/README.md) | MCP server (`franka-real`): read robot status, preview and run taught Desk tasks, with local safety checks and a certificate-pinned connection | Your laptop |
| [`arm-control/`](arm-control/README.md) | Teach-and-repeat: record pick/place sequences per gripper and replay them from a web UI | Raspberry Pi 4 + libfranka |

## How they fit together

- Both talk to the same robot, which sits on its own network (default `192.168.5.10`).
  Its Desk web interface is reached through the Raspberry Pi, which acts as the SSH bridge.
- The MCP reaches Desk either directly (Ethernet) or through the Pi's SSH tunnel; see
  [`Franka_Arm_MCP/CONNECTION.md`](Franka_Arm_MCP/CONNECTION.md).
- The robot has one control owner at a time. The MCP never forces a takeover, so release control
  in one tool before using the other.
- The MCP drives tasks you taught in Desk. The panel drives the arm itself through libfranka.

## Where to start

- **AI control of the arm:** `Franka_Arm_MCP/README.md`, then `Franka_Arm_MCP/SETUP.md`.
- **Teach-and-repeat panel:** `arm-control/README.md`.

## Before you commit to Git

- Secrets stay local: the MCP's Desk login (`Franka_Arm_MCP/.session.json`), certificates and
  virtual environments. `Franka_Arm_MCP/.gitignore` covers them; there is no `.gitignore` at this
  level yet, so add one for anything new you create here.
- Check before the first commit: `git status --short --ignored`.
- Keep passwords, tokens and device MAC or IP details out of READMEs and scripts.
