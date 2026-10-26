#!/usr/bin/env bash
# Set up this folder on a new machine.
#
#   bash setup.sh        create .venv, install the pinned dependencies, validate
#                        config.json, and print the MCP registration for this location
#   bash setup.sh pack   write a clean zip for another machine (no .venv, no login
#                        session, no archive); default: ../franka_hardware_mcp-portable.zip
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

if [[ "${1:-}" == "pack" ]]; then
  name="$(basename "$HERE")"
  out="${2:-$(dirname "$HERE")/${name}-portable.zip}"
  rm -f "$out"
  (cd "$(dirname "$HERE")" && zip -q -r "$out" "$name" \
      -x "$name/.venv/*" "$name/archive/*" "$name/diagnostics/*" \
         "$name/.session.json" "$name/.franka-*" "*/__pycache__/*" "*.pyc" "*.DS_Store" "*.zip")
  echo "Wrote $out ($(du -h "$out" | cut -f1)). It excludes .venv, .session.json (login token), archive/ and diagnostics/."
  exit 0
fi

PYTHON="${PYTHON:-python3}"
if ! "$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
  echo "Python 3.10 or newer is required (found: $("$PYTHON" --version 2>&1 || echo none))." >&2
  echo "Install one, or run:  PYTHON=/path/to/python3.12 bash setup.sh" >&2
  exit 1
fi

# A .venv copied from another machine points at that machine's Python; rebuild it.
if [[ -e .venv/bin/python ]] && ! .venv/bin/python -c 'pass' 2>/dev/null; then
  echo "Existing .venv does not run on this machine; rebuilding it."
  "$PYTHON" -m venv --clear .venv
elif [[ ! -e .venv/bin/python ]]; then
  "$PYTHON" -m venv .venv
fi
.venv/bin/python -m pip install -q --disable-pip-version-check -r requirements.lock.txt
.venv/bin/python -c 'import mcp'
.venv/bin/python franka_config.py check

chmod +x ./*.command setup.sh 2>/dev/null || true
if command -v xattr >/dev/null 2>&1; then xattr -dr com.apple.quarantine . 2>/dev/null || true; fi

cat <<MSG

Environment ready in $HERE/.venv

Register the MCP (pick your client):

  Codex  (~/.codex/config.toml)
    [mcp_servers.franka-real]
    command = "$HERE/.venv/bin/python"
    args = ["$HERE/server.py"]
    startup_timeout_sec = 20
    tool_timeout_sec = 40

  Claude Code
    claude mcp add franka-real -- "$HERE/.venv/bin/python" "$HERE/server.py"

  Claude Desktop (claude_desktop_config.json)
    "mcpServers": { "franka-real": { "command": "$HERE/.venv/bin/python", "args": ["$HERE/server.py"] } }

Then, each session:  1) Connect Desk over WiFi.command (skip if config.json connection is "direct")   2) Connect Franka.command
MSG
