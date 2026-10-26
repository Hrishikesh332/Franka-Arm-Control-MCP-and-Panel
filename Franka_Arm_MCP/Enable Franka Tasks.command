#!/bin/zsh
# OPTIONAL — review taught tasks and enable exact task versions for execution from chat.
# Needs the tunnel (step 1) and a login (step 2). Leaving the list blank enables nothing.
TASK_ROOT="${0:A:h}"
cd "$TASK_ROOT" || exit 1
if [[ -x "$TASK_ROOT/.venv/bin/python" ]]; then
  "$TASK_ROOT/.venv/bin/python" "$TASK_ROOT/connect.py" --enable-tasks
else
  echo "No Python environment here yet. Run 'bash setup.sh' in this folder first."
fi
read "?Press Return to close."
