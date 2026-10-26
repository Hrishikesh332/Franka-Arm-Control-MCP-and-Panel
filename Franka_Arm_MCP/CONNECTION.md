# Connecting to the robot

How this MCP reaches the FR3. Agents: read this when a tool says it cannot reach the robot or
asks for a login. Humans: SETUP.md has the install steps.

## Two modes, one switch

`config.json` → `"connection"`. It applies to the MCP tools **and** the login, and takes effect
on the next tool call (no restart).

```text
direct : this machine ──Ethernet──────────────────────────────► FR3  192.168.5.10:443
tunnel : this machine ──127.0.0.1:8443 (ssh -L)──► Raspberry Pi ──eth0──► FR3  192.168.5.10:443
                                                  (Wi-Fi, 10.23.60.102)   (Pi = 192.168.5.12)
```

Addresses are the current lab setup; the real values live in `config.json`.

| | `direct` (default if the key is absent) | `tunnel` (set in this repo's `config.json`) |
|---|---|---|
| Use when | The machine is on the robot's own network (Ethernet) | The machine is on Wi-Fi; Desk is reachable only through the Pi |
| Before each session | Step 2 only | Step 1, then step 2 |
| Step 1 | not needed | `Connect Desk over WiFi.command`: SSH tunnel, **keep it open** |
| Step 2 | `Connect Franka.command`: Desk login | same |
| MCP traffic goes to | `robot_host:443` | `127.0.0.1:<desk_tunnel.local_port>` |
| Config used | `robot_host`, `certificate_sha256` | plus `desk_tunnel.bridge_host`, `bridge_user`, `local_port` |

Both modes verify the robot's pinned certificate hash before sending anything, and use the same
login session (`.session.json`). A tunnel that points at the wrong machine is rejected.

Without the launchers (Linux): `ssh -N -L 127.0.0.1:8443:<robot_host>:443 <bridge_user>@<bridge_host>`,
then `.venv/bin/python connect.py`.

## What the agent can check

- `franka_real_capabilities` returns `connection` (the mode), `robot_host` and `authenticated`.
- `python3 franka_config.py get connection` / `check` read and validate the config.

## Symptom → cause → action

| Message | Meaning | Action |
|---|---|---|
| `Could not open the Desk tunnel at 127.0.0.1:…` | Tunnel mode, tunnel not running | Ask the user to run step 1 and keep it open. |
| `Robot read failed … check Ethernet and local network access` | Direct mode, robot not reachable | If the user is on Wi-Fi, set `"connection": "tunnel"`; otherwise check the cable and IP. |
| `Desk login required` / `expired` | No valid session (lasts 1 hour) | Ask the user to run step 2. |
| `Desk tunnel presented a different robot certificate` | Tunnel points at the wrong device | Check `robot_host` and `bridge_host`. Never just update the pin. |
| `Robot TLS certificate changed` | Direct mode, certificate differs from the pin | Verify the robot first; see SETUP.md section 6. |
| `Robot control is already held by …` | Another client owns control | Release it in Desk or stop the other client. The MCP never forces takeover. |

## Rules for agents

- Never type, request or store the Desk or SSH password. The user enters them in the launcher windows.
- Do not edit `.session.json` or the certificate pin to get around an error.
- The Raspberry Pi also runs the `arm-control` panel (libfranka). Only one client can own control
  of the arm; do not start a task while the panel is moving it.
- The Pi's address comes from DHCP and can change; update `desk_tunnel.bridge_host` if step 1 cannot connect.
