#!/bin/zsh
# STEP 2 — create the local Desk login (needs the tunnel from step 1).
# The password is typed here, hidden, and never stored; only the robot's session token is kept.
TASK_ROOT="${0:A:h}"
cd "$TASK_ROOT" || exit 1
if [[ -x "$TASK_ROOT/.venv/bin/python" ]]; then
  "$TASK_ROOT/.venv/bin/python" "$TASK_ROOT/connect.py"
else
  echo "No Python environment here yet. Run 'bash setup.sh' in this folder first."
fi
read "?Press Return to close."
