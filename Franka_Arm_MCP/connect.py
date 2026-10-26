"""Interactive Desk authentication. Never sends a password through chat."""
import argparse
from getpass import getpass
import json

from server import make_reader
from web_control import DeskWebClient, POLICY, task_id, write_private


def enable_tasks(client):
    print(json.dumps(client.list_tasks(), indent=2))
    selected = input("Exact taught task IDs checked for this workcell to enable (comma-separated; blank = none): ").strip()
    approved = {}
    for ident in filter(None, (item.strip() for item in selected.split(","))):
        preview = client.preview(task_id(ident))
        print(json.dumps(preview, indent=2))
        if input(f"Enable this exact task {ident} for physical execution from chat? Type ENABLE: ") == "ENABLE":
            approved[ident] = preview["task_sha256"]
    # Update only the approvals. default_workflow, required_home and the other
    # policy fields must survive an enablement run.
    policy = json.loads(POLICY.read_text()) if POLICY.exists() else {}
    policy["approved_tasks"] = approved
    write_private(POLICY, policy)
    print(f"Enabled {len(approved)} exact task version(s). No robot movement or control takeover was requested.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--enable-tasks", action="store_true")
    options = parser.parse_args()
    # config.json "connection" decides the route: "tunnel" reaches Desk through the
    # SSH bridge on localhost, "direct" connects to robot_host. Either way the robot's
    # pinned certificate is checked before any credentials are sent.
    reader = make_reader()
    client = DeskWebClient(reader)
    try:
        if options.enable_tasks:
            enable_tasks(client)
        else:
            route = f"through the tunnel at https://127.0.0.1:{reader.tunnel_port}" if hasattr(reader, "tunnel_port") else f"directly at https://{reader.host}"
            print(f"Dedicated Franka MCP — connecting {route}")
            print("Password is hidden. Only the robot-issued session token is stored locally with permissions 600.")
            username = input("Franka Desk username: ").strip()
            password = getpass("Franka Desk password: ")
            result = client.login(username, password)
            del password
            print(json.dumps(result, indent=2))
            print("Connected. You can now ask chat to list tasks, request control or operate the brakes.")
            enable_tasks(client)
    except (RuntimeError, ValueError) as exc:
        print("Connection not completed:", exc)
        raise SystemExit(1)
