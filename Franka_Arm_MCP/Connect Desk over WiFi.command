#!/bin/zsh
# STEP 1 — open the SSH tunnel to the robot's Desk. Keep this window open while you work.
# Defaults come from config.json ("robot_host" and the "desk_tunnel" section).
set -u
TASK_ROOT="${0:A:h}"
cd "$TASK_ROOT" || exit 1

PY="$TASK_ROOT/.venv/bin/python"
[[ -x "$PY" ]] || PY="$(command -v python3 || true)"
if [[ -z "$PY" ]]; then
  echo "python3 not found. Run 'bash setup.sh' in this folder first."
  read "?Press Return to close."; exit 1
fi
cfg() { "$PY" "$TASK_ROOT/franka_config.py" get "$1"; }

ROBOT_HOST="$(cfg robot_host)" && LOCAL_PORT="$(cfg desk_tunnel.local_port)" \
  && DEFAULT_HOST="$(cfg desk_tunnel.bridge_host)" && DEFAULT_USER="$(cfg desk_tunnel.bridge_user)" || {
  echo "Could not read config.json (see the message above)."
  read "?Press Return to close."; exit 1
}

echo "Franka Desk Wi-Fi tunnel"
[[ "$(cfg connection 2>/dev/null)" == "direct" ]] && echo "Note: config.json has connection=direct, so the MCP does not use this tunnel."
echo "Robot:  ${ROBOT_HOST}:443   (set in config.json)"
echo "Local:  https://127.0.0.1:${LOCAL_PORT}/desk/"
echo
read "BRIDGE_HOST?SSH bridge host [${DEFAULT_HOST}]: "
BRIDGE_HOST="${BRIDGE_HOST:-$DEFAULT_HOST}"
read "BRIDGE_USER?SSH username for ${BRIDGE_HOST} [${DEFAULT_USER}]: "
BRIDGE_USER="${BRIDGE_USER:-$DEFAULT_USER}"
if [[ -z "$BRIDGE_HOST" ]]; then
  echo "No bridge host. Enter one, or set desk_tunnel.bridge_host in config.json."
  read "?Press Return to close."; exit 1
fi
echo
echo "Enter the SSH password locally when prompted. Keep this window open."
echo "Next: run 'Connect Franka.command' (step 2)."
echo

exec ssh \
  -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -o StrictHostKeyChecking=accept-new \
  -L "127.0.0.1:${LOCAL_PORT}:${ROBOT_HOST}:443" \
  "${BRIDGE_USER}@${BRIDGE_HOST}"
