# Setup guide

How to get this folder working on a machine, connect to the robot, and fix the usual
problems. For a short overview see [README.md](README.md).

## 1. Requirements

- Python **3.10 or newer** (`python3 --version`).
- A way to reach the robot. Set `"connection"` in `config.json`: `direct` if the machine is on the
  robot's Ethernet, `tunnel` if it is on Wi-Fi and goes through the Raspberry Pi (see CONNECTION.md).
- For the login: SSH access to the bridge host (default `10.23.60.102`) and your Desk
  username and password.
- macOS for the double-click launchers. On Linux the server works the same; use the
  equivalent commands in section 5.

## 2. Install

Move the folder to the new machine. On the old one, `bash setup.sh pack` writes
`../franka_hardware_mcp-portable.zip` (about 100 KB) without the venv, the login token
or the archive. Unzip it anywhere, then in that folder:

```sh
bash setup.sh
```

It will:

1. check for Python 3.10+;
2. create `.venv` (and rebuild it if it was copied from another machine);
3. install the pinned dependencies from `requirements.lock.txt`;
4. validate `config.json` and confirm its pin matches `robot-cert.pem`;
5. make the launchers executable and clear the macOS quarantine flag;
6. print the MCP registration with this folder's real paths.

A different Python: `PYTHON=/path/to/python3.12 bash setup.sh`.

## 3. Register the MCP

Use the block `setup.sh` printed. Templates:

**Codex**: add to `~/.codex/config.toml`:

```toml
[mcp_servers.franka-real]
command = "/path/to/franka_hardware_mcp/.venv/bin/python"
args = ["/path/to/franka_hardware_mcp/server.py"]
startup_timeout_sec = 20
tool_timeout_sec = 40
```

**Claude Code**:

```sh
claude mcp add franka-real -- /path/to/franka_hardware_mcp/.venv/bin/python /path/to/franka_hardware_mcp/server.py
```

**Claude Desktop** (`claude_desktop_config.json`):

```json
{ "mcpServers": { "franka-real": {
    "command": "/path/to/franka_hardware_mcp/.venv/bin/python",
    "args": ["/path/to/franka_hardware_mcp/server.py"] } } }
```

Restart the client afterwards. The server name is `franka-real`; the tools start with
`franka_real_`.

## 4. Check it works (nothing moves)

```sh
python3 franka_config.py check        # config + certificate pin, offline
.venv/bin/python server.py --probe    # one read-only status read from the robot
```

The probe makes read-only requests and needs the robot reachable. In your client you can
also call `franka_real_capabilities`, which also reports whether you are logged in.

## 5. Connect each session

1. **`Connect Desk over WiFi.command`**: opens the tunnel (`tunnel` mode only; skip it when `connection` is `direct`). It shows the defaults from
   `config.json`; press Enter to accept or type another bridge host or user. Enter the
   SSH password when asked and **keep the window open**.
2. **`Connect Franka.command`**: asks for the Desk username and password (hidden) and
   saves a session token. It then offers to enable tasks.
3. **`Enable Franka Tasks.command`** (optional): lists taught tasks, shows each one you
   pick, and enables it only if you type `ENABLE`. Each run replaces the approvals, and
   blank means none, so nothing can execute until you do this.

Without the launchers (Linux, or a blocked file):

```sh
ssh -N -L 127.0.0.1:8443:<robot_host>:443 <user>@<bridge_host>   # terminal 1, keep open
.venv/bin/python connect.py                                       # terminal 2
.venv/bin/python connect.py --enable-tasks                        # optional
```

## 6. Configuration reference

`config.json`:

| Key | Meaning |
|---|---|
| `robot_host` | Robot IP address. |
| `connection` | `direct` (robot_host:443) or `tunnel` (local SSH tunnel via the Pi). Default `direct`; applies to the MCP and the login. |
| `certificate_sha256` | SHA-256 of the robot's TLS certificate (64 lowercase hex characters). Checked before any request. |
| `timeout_seconds` | Network timeout, above 0 and at most 10. Default 5. |
| `desk_tunnel.bridge_host` | Default SSH bridge for the launcher. |
| `desk_tunnel.bridge_user` | Default SSH user. Default `pi`. |
| `desk_tunnel.local_port` | Local tunnel port, 1024 to 65535. Default 8443. Always bound to 127.0.0.1. |

`task-policy.json` is the execution policy: `approved_tasks` (task id mapped to the exact
content hash; empty means nothing can run), `default_workflow`, and `required_home`
(Ready Home joint angles and tolerances). The enable step edits only `approved_tasks`.

### Using a different robot or replacing its certificate

Only after you have verified it is the right robot:

```sh
openssl s_client -connect <robot_ip>:443 </dev/null 2>/dev/null | openssl x509 > robot-cert.pem
openssl x509 -in robot-cert.pem -outform der | shasum -a 256
```

Put the hash in `certificate_sha256`, the IP in `robot_host`, then run
`python3 franka_config.py check`. Task ids and names in `web_control.py` and `plans/` belong to
the original robot's taught tasks; a different robot needs those tasks created first.

## 7. Keeping secrets out of Git

`.gitignore` excludes: the Desk session (`.session.json`, which holds an admin token),
`.franka-*` temp files, `.env*`, all keys and certificates (`*.pem`, including
`robot-cert.pem`, `*.crt`, `*.key`), Codex and other client registration files
(`codex-config*.toml`, `.codex/`, `.mcp.json`), anything named like credentials, password
or token, `.venv/`, `diagnostics/`, `plans/`, `archive/` and zip bundles.

`robot-cert.pem` and `plans/` stay on your disk and in `bash setup.sh pack` bundles, but
never reach GitHub. `plans/` holds the checked task data that `web_control.py` reads before
starting a task and in the `franka_real_prepare_*` tools; a clone without it refuses to
execute (it fails closed), so copy `plans/` over from your packed bundle. A fresh clone also has no
`robot-cert.pem`: `config.json` still works, and `franka_config.py check` simply skips
the certificate comparison. Recreate the file with the commands in section 6 if you want it.

Before pushing, check what would be committed:

```sh
git status --short --ignored      # .session.json must appear with "!!"
git ls-files | grep -i -E "session|token|secret|password|\.key"   # expect no output
```

`.gitignore` only protects files that are not already tracked. If any of these were
committed earlier, stop tracking them (this keeps the files on disk):

```sh
git rm --cached --ignore-unmatch robot-cert.pem plans codex-config.toml .session.json
git commit -m "Stop tracking local-only files"
```

That does not erase them from past commits. Check with
`git log --all --oneline -- .session.json robot-cert.pem plans 'codex-config*'`. If
`.session.json` appears, remove it from history, treat that token as leaked, and log out of
Desk or restart the session on the robot. The certificate and the Codex template hold no
credentials, so for them a clean-up commit is enough.

`config.json` is tracked and contains private network addresses (robot, bridge), the SSH
user name and the public certificate pin. They are not credentials, but keep the repository
private if you do not want your lab network layout public.

## 8. Troubleshooting

| Message or symptom | Fix |
|---|---|
| `Could not open the Desk tunnel at 127.0.0.1:8443` | Run step 1 and keep its window open. Check `desk_tunnel.local_port` matches. |
| `Desk login required` or `Desk login expired` | Run step 2 again. |
| `Desk tunnel presented a different robot certificate` | The tunnel points at the wrong machine. Check `robot_host` and the bridge host. |
| `Robot TLS certificate changed; verify the robot…` | Do not just update the pin. Verify the robot, then follow section 6. |
| `Robot read failed … check Ethernet and local network access` | This machine cannot reach `robot_host:443`. Check the cable, IP and routing. |
| `Task is not locally enabled with this exact content` | Run `Enable Franka Tasks.command`; any change to the task changes its hash. |
| Launcher "cannot be opened" on macOS | Run `bash setup.sh` (clears quarantine), or right-click then Open. |
| `.venv` errors after copying the folder | Run `bash setup.sh`; it rebuilds the venv. |
| `config.json … is missing` or invalid | Restore it from the zip, or compare with the table in section 6. |
| `Python 3.10 or newer is required` | Install one and run `PYTHON=/path/to/python3.12 bash setup.sh`. |

## 9. Maintenance

- Add or upgrade a dependency: edit `requirements.txt`, then
  `.venv/bin/python -m pip install -r requirements.txt` and
  `.venv/bin/python -m pip freeze > requirements.lock.txt`.
- Remove the install: delete `.venv/` and `.session.json`; remove the registration from
  your client.
- Old material lives in `archive/`; `archive/portability-cleanup-2026-10-26/README.md`
  lists what changed in the latest cleanup and how to revert each file.
