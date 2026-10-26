# Franka MCP (`franka-real`)

An MCP server that lets an AI client (Codex, Claude Code, Claude Desktop) work with a physical
Franka FR3: read its status, preview taught tasks, and run tasks you have explicitly enabled,
under local safety checks. The Desk password is typed locally in a terminal and never goes
through chat or to disk.

## How it works

`server.py` is our own MCP server, built on the official `mcp` Python SDK (stdio). Its 26 tools
call the robot's Desk web API over HTTPS, after checking the robot's pinned certificate hash.

| Tools | Count | API used |
|---|---|---|
| Read: status, brakes, safety, robot state, tasks, events | 9 | Documented Desk API (`/api/...`), read-only |
| Control: take/release control, brakes, execute, stop, resume, workflow speed | 8 | Desk web endpoints (`/desk/api`, `/admin/api`) |
| Authoring: `franka_real_prepare_*` upload checked task revisions to Desk | 9 | Desk web endpoints |

The control and authoring endpoints are the ones Desk's own browser UI uses. They are not part of
Franka's documented API, so a firmware update could change them. Execution only starts taught
tasks. There are no arbitrary joint or Cartesian commands, no safety resets and no forced control takeover.

## Quick start

```sh
bash setup.sh      # Python 3.10+: builds .venv, installs dependencies, checks config, prints the MCP registration
```

Paste the registration it prints into your client and restart it. Then each session:

| Step | Double-click | Purpose |
|---|---|---|
| 1 | `Connect Desk over WiFi.command` | `tunnel` mode only: SSH tunnel to Desk via the Pi. Keep it open. |
| 2 | `Connect Franka.command` | Desk login (token valid 1 hour locally). |
| 3 | `Enable Franka Tasks.command` | Optional: enable exact task versions. Blank enables nothing. |

Two connection modes, switched by `"connection"` in `config.json`: `direct` (machine on the
robot's Ethernet; skip step 1) or `tunnel` (Wi-Fi, through the Raspberry Pi). Details and error
guidance for agents: **[CONNECTION.md](CONNECTION.md)**.

Full guide, config reference and troubleshooting: **[SETUP.md](SETUP.md)**.

## Files

```text
server.py              MCP server: the tool definitions
web_control.py         Desk client and all safety guards
connect.py             Login and task-enablement script (used by the launchers)
tunnel_desk_reader.py  Pinned connection through the local tunnel (tunnel mode)
franka_config.py       Reads and validates config.json
config.json            Robot address, certificate pin, tunnel defaults   <- edit this
task-policy.json       Enabled tasks, default workflow, required Ready Home
CONNECTION.md          How the machine reaches the robot (direct or via the Raspberry Pi)
setup.sh               Setup; `bash setup.sh pack` writes a clean zip for another machine
AGENTS.md              Workflow rules for agents (Ready Home, default workflow)

Local only, never in Git:  plans/  tomato_versioning.py  robot-cert.pem  .session.json  .venv/  archive/
```

`plans/` holds the checked task data that execution reads before starting a task and that the
`prepare_*` tools upload. `tomato_versioning.py` backs one authoring tool. Both stay on disk and
in packed bundles.

## Safety in short

- Execution needs a local login, owned control, an exact task hash enabled locally, and the arm
  matching the checked Ready Home. Errors and singularity warnings block it.
- `.session.json` is a live admin token. It is Git-ignored and left out of packed bundles. Delete
  it when you finish; the next login overwrites it.
- Do not move the robot just to test the integration. The read tools and `server.py --probe` never move it.
