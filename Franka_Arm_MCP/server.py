"""Dedicated MCP for the physical FR3's Desk HTTPS API.

Separate from the franka-sim MCP. Web control needs a local login.
"""
from __future__ import annotations

import argparse
import anyio
from datetime import datetime, timezone
import hashlib
import hmac
import http.client
import json
from pathlib import Path
import platform
import ssl
from typing import Any

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from franka_config import connection_mode, load_config, robot_settings
from web_control import DeskWebClient, POLICY, STOPPED

ROOT = Path(__file__).resolve().parent
READ_PATHS = frozenset({
    "/api/system", "/api/arm", "/api/arm/joints", "/api/fci",
    "/api/arm/warnings", "/api/safety/recovery", "/api/safety/self-tests",
    "/api/system/control-token",
})
MAX_RESPONSE_BYTES = 262144


class DeskReader:
    def __init__(self, config: dict[str, Any] | None = None):
        self.config = config if config is not None else load_config()
        self.host, self.pin, self.timeout = robot_settings(self.config)

    def pinned_connection(self):
        # Trust the exact certificate captured from the user's robot, including
        # self-signed certificates. Check its DER hash BEFORE any HTTP request.
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        connection = http.client.HTTPSConnection(self.host, port=443, timeout=self.timeout, context=context)
        try:
            connection.connect()
            actual = hashlib.sha256(connection.sock.getpeercert(binary_form=True)).hexdigest()
            if not hmac.compare_digest(actual, self.pin):
                raise RuntimeError("Robot TLS certificate changed; verify the robot before updating the local pin")
            return connection
        except Exception:
            connection.close()
            raise

    def get(self, path: str) -> dict[str, Any] | list[Any]:
        if path not in READ_PATHS:
            raise ValueError("Only the fixed read-only Desk endpoints are permitted")
        connection = self.pinned_connection()
        try:
            connection.request("GET", path, headers={"Accept": "application/json"})
            response = connection.getresponse()
            body = response.read(MAX_RESPONSE_BYTES + 1)
            if len(body) > MAX_RESPONSE_BYTES:
                raise RuntimeError("Desk response exceeded the size limit")
            if response.status != 200:
                raise RuntimeError(f"Desk GET {path} returned HTTP {response.status}; no robot changes were made")
            if response.getheader("Content-Type", "").split(";", 1)[0].strip() != "application/json":
                raise RuntimeError("Desk returned a non-JSON response")
            result = json.loads(body)
            if not isinstance(result, (dict, list)):
                raise RuntimeError("Unexpected Desk response shape")
            return result
        except (OSError, http.client.HTTPException) as exc:
            raise RuntimeError(f"Robot read failed at {self.host}: {type(exc).__name__}; check Ethernet and local network access") from None
        finally:
            connection.close()


def make_reader(config: dict[str, Any] | None = None) -> DeskReader:
    """Reader for the configured connection mode (config.json "connection").

    "direct" -> robot_host:443.  "tunnel" -> the local SSH tunnel to the robot via the
    Raspberry Pi. Both check the robot's certificate pin before any request.
    """
    config = config if config is not None else load_config()
    if connection_mode(config) == "tunnel":
        from tunnel_desk_reader import TunnelDeskReader  # imported late: it imports this module
        return TunnelDeskReader(config)
    return DeskReader(config)


def read_group(paths: dict[str, str]) -> dict[str, Any]:
    reader = make_reader()
    started = datetime.now(timezone.utc).isoformat()
    data = {name: reader.get(path) for name, path in paths.items()}
    return {
        "source": "physical_franka_desk_api",
        "robot_host": reader.host,
        "read_only": True,
        "read_started_at_utc": started,
        "read_finished_at_utc": datetime.now(timezone.utc).isoformat(),
        "snapshot_is_atomic": False,
        **data,
    }


def configured_host() -> str:
    """Robot address from config.json for the instructions text; never blocks server start-up."""
    try:
        return robot_settings(load_config())[0]
    except (RuntimeError, ValueError, KeyError):
        return "the address set in config.json"


mcp = FastMCP(
    "franka-real",
    instructions=(
        f"Connect to the PHYSICAL Franka FR3 at {configured_host()} through its Desk HTTPS APIs. "
        "Control tools require a local Desk login. Execute only an explicitly requested, locally enabled taught task; inspect it first. "
        "Do not move the robot merely to test this integration. Never force control takeover or clear safety errors. "
        "franka_real_joint_brakes describes brakes; franka_real_robot_state supplies measured joint angles and Cartesian pose. "
        "Always use Franka standard Ready Home, joint degrees [0,-45,0,-135,0,90,45], as workflow start and return. "
        "The current Home-to-Home workflow is chat_backward_x_from_franka_ready_v06 (0_move2dest8). "
        "Check all seven joints, position and complete flange orientation; the earlier workcell start is an intermediate pose. "
        "No table geometry or physical tool verification is available: do not infer safe clearances from TCP height alone. "
        "Never infer motion, a grasp, end-effector pose or hardware qualification from these reads. "
        "FCI motion needs a separately configured real-time Linux control workstation. "
        "The existing franka-sim MCP is a different robot in simulation."
    ),
)
READ_ANNOTATIONS = ToolAnnotations(readOnlyHint=True, destructiveHint=False, idempotentHint=True, openWorldHint=True)
CONTROL_ANNOTATIONS = ToolAnnotations(readOnlyHint=False, destructiveHint=True, idempotentHint=False, openWorldHint=True)


def web_client():
    return DeskWebClient(make_reader())


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_observe() -> dict[str, Any]:
    """Read fresh physical robot system, arm type, operating mode, FCI and warnings. No motion or control acquisition."""
    return await anyio.to_thread.run_sync(read_group, {"system": "/api/system", "arm": "/api/arm", "fci": "/api/fci", "arm_warnings": "/api/arm/warnings"})


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_joint_brakes() -> dict[str, Any]:
    """Read the seven physical joint BRAKE states. This endpoint does not report joint angles or tool pose."""
    return await anyio.to_thread.run_sync(read_group, {"joint_brakes": "/api/arm/joints"})


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_safety_status() -> dict[str, Any]:
    """Read recovery indications and periodic self-test status. Does not clear errors or execute tests."""
    return await anyio.to_thread.run_sync(read_group, {"recovery": "/api/safety/recovery", "self_tests": "/api/safety/self-tests"})


@mcp.tool(annotations=READ_ANNOTATIONS)
def franka_real_capabilities() -> dict[str, Any]:
    """Describe this connector's capabilities and the requirements for later physical motion. Local configuration only."""
    reader = make_reader()
    try:
        web_client().session()
        authenticated = True
    except RuntimeError:
        authenticated = False
    task_policy = json.loads(POLICY.read_text())
    approved = task_policy.get("approved_tasks", {})
    return {
        "robot_host": reader.host,
        "connection": connection_mode(reader.config),
        "desk_url": f"https://{reader.host}/desk/",
        "api_reference": f"https://{reader.host}/deskapi/",
        "host_platform": platform.system(),
        "read_only": False,
        "authenticated": authenticated,
        "physical_task_execution_configured": authenticated and bool(approved),
        "enabled_task_ids": sorted(approved),
        "default_workflow": task_policy.get("default_workflow"),
        "required_home": task_policy.get("required_home"),
        "task_execution_latched": STOPPED.is_set(),
        "raw_password_stored": False,
        "tls_authentication": "pinned SHA-256 of the robot certificate",
        "robot_state_is_live": False,
        "available": ["system status", "arm identity", "FCI status", "brake status", "warnings", "safety status", "measured joint angles and Cartesian pose", "configured end-effector profile", "taught task listing/preview/start/stop", "control token acquisition/release", "brake lock/unlock"],
        "not_available": ["arbitrary joint/Cartesian targets", "table geometry", "camera", "direct gripper commands", "safety reset", "FCI activation"],
        "motion_path": "Chat -> franka-real MCP -> Desk HTTPS task API -> robot's onboard task controller",
    }


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_robot_state() -> dict[str, Any]:
    """Read fresh physical joint angles, Cartesian pose, estimated forces, robot errors and configured tool profile. No table map or physical tool validation is supplied."""
    return await anyio.to_thread.run_sync(web_client().robot_state)


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_list_tasks() -> dict[str, Any]:
    """List existing taught Desk tasks/groups from the physical robot. Requires local Desk login."""
    return await anyio.to_thread.run_sync(web_client().list_tasks)


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_preview_task(task_id: str) -> dict[str, Any]:
    """Read a saved task's actual payload, SHA-256 and local execution eligibility. No motion."""
    return await anyio.to_thread.run_sync(web_client().preview, task_id)


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_task_status() -> dict[str, Any]:
    """Read fresh Desk execution status. Never infer grasp or task success from a start response."""
    result = await anyio.to_thread.run_sync(web_client().request, "GET", "/desk/api/execution")
    return {"execution": result, "source": "physical_franka_desk_web_api"}


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_ground_relative_transfer() -> dict[str, Any]:
    """Save the checked Ground-relative revision: 150 mm virtual tool, 50 mm tip gap, source +180 deg, X-only transfer, -180 deg, lift and exact start return. Workflow speed 40%, per-app settings 5%. Does not change tool calibration, enable or run it."""
    return await anyio.to_thread.run_sync(web_client().prepare_ground_relative_transfer)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_ground_y_transfer() -> dict[str, Any]:
    """Save only the checked v02 revision: 100 mm virtual tool, 10 mm tip gap, +180 degree source rotation, +Y 100 mm, lift 50 mm, X-only transfer, descend, -180 degrees, lift and return. Workflow slider 70%, per-motion settings 5%. Does not enable, execute or change the physical TCP/load."""
    return await anyio.to_thread.run_sync(web_client().prepare_ground_y_transfer)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_no_rotation_transfer() -> dict[str, Any]:
    """Save only checked v03: remove both rotations, retain constant flange orientation and elbow joint 3, +Y 100 mm, lift, X transfer, descend, lift and return. Individual motion speed 50%, acceleration/deceleration 5%, workflow slider 70%. Never enable or execute motion."""
    return await anyio.to_thread.run_sync(web_client().prepare_no_rotation_transfer)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_no_rotation_speed30() -> dict[str, Any]:
    """Save only checked v04 with no rotations, fixed orientation/elbow, 30% component speeds, 5% acceleration/deceleration and the 70% workflow slider. Never enable or execute motion."""
    return await anyio.to_thread.run_sync(web_client().prepare_no_rotation_speed30)


@mcp.tool(annotations=READ_ANNOTATIONS)
async def franka_real_diagnostic_events() -> dict[str, Any]:
    """Read and preserve the documented event log and return recent singularity/IK/motion-planning entries. No motion, control takeover, error acknowledgement or recovery."""
    return await anyio.to_thread.run_sync(web_client().diagnostic_events)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_backwards_x_first() -> dict[str, Any]:
    """Save checked v05: source lift, backward base X 573 to 319 mm first, Y shift only at destination X, destination descent, lift and return. All component speeds 30%, workflow 70%, no rotations. Does not enable or execute motion."""
    return await anyio.to_thread.run_sync(web_client().prepare_backwards_x_first)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_home_tasks() -> dict[str, Any]:
    """Save standard Franka Ready Home as a separate native Joint Motion task and v06 backward-X workflow with explicit Home prerequisite, workcell transition and Home return. Components 30%. Does not enable, execute, clear singularity or change safety settings."""
    return await anyio.to_thread.run_sync(web_client().prepare_home_tasks)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_flange_inward_test() -> dict[str, Any]:
    """Save only checked 10 mm flange-local -Z inward free-space test and exact Ready Home return. Fixed orientation; component and workflow speeds 30%. Does not enable or start motion."""
    return await anyio.to_thread.run_sync(web_client().prepare_flange_inward_test)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_tomato_source_fix() -> dict[str, Any]:
    """Save only checked Pick & Place Tomato v2 - Source Approach Fix as a new task. Repairs pose/joint inconsistencies, uses native Joint Motion above source then vertical Cartesian descent, preserves taught targets and safety thresholds, exact Ready Home and 30% speeds. Never enable, run, acknowledge faults or change the original."""
    try:
        from tomato_versioning import prepare_tomato_source_fix
    except ImportError:
        raise RuntimeError("tomato_versioning.py is not installed here (a local-only, one-off authoring helper); copy it from your packed bundle to use this tool") from None
    return await anyio.to_thread.run_sync(prepare_tomato_source_fix, web_client())


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_set_workflow_speed(percent: float) -> dict[str, Any]:
    """Set the user-created table-transfer workflow's saved speed to 1–40 percent while stopped. Preserves waypoints and per-app speed settings, verifies read-back, and never restarts the arm."""
    return await anyio.to_thread.run_sync(web_client().set_workflow_speed, percent)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_prepare_table_transfer() -> dict[str, Any]:
    """Create the separately named, checked v04 bare-flange routine in Desk: Z>=225 mm, positive 180-degree rotation, X-only transfer, 5% speed settings. Does not enable or start it; checks original tasks remain unchanged."""
    return await anyio.to_thread.run_sync(web_client().prepare_table_transfer)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_take_control() -> dict[str, Any]:
    """Request robot control without moving it. Fails if another session owns control; never forces takeover."""
    return await anyio.to_thread.run_sync(web_client().take_control)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_release_control() -> dict[str, Any]:
    """Release the control token owned by this MCP. Does not execute motion."""
    return await anyio.to_thread.run_sync(web_client().release_control)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_unlock_brakes() -> dict[str, Any]:
    """Unlock physical arm brakes only on the user's request, then read measured brake status. Requires local login and owned control."""
    return await anyio.to_thread.run_sync(web_client().brakes, True)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_lock_brakes() -> dict[str, Any]:
    """Request physical arm brake locking, then read measured brake status. Requires local login and owned control."""
    return await anyio.to_thread.run_sync(web_client().brakes, False)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_execute_task(task_id: str, expected_task_sha256: str) -> dict[str, Any]:
    """Start an explicitly requested taught task on the PHYSICAL arm. Requires local task enablement, matching preview hash, login, control and robot readiness. The robot may MOVE. Does not claim completion."""
    return await anyio.to_thread.run_sync(web_client().execute, task_id, expected_task_sha256)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_stop() -> dict[str, Any]:
    """Stop Desk task execution and latch new MCP task starts. Software task stop only; the physical emergency stop remains independent."""
    return await anyio.to_thread.run_sync(web_client().stop)


@mcp.tool(annotations=CONTROL_ANNOTATIONS)
async def franka_real_resume() -> dict[str, Any]:
    """Explicitly clear this MCP's task-stop latch. Never restarts or replays an interrupted task."""
    return await anyio.to_thread.run_sync(web_client().resume)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", action="store_true", help="Read current physical robot status once and exit")
    options = parser.parse_args()
    if options.probe:
        print(json.dumps(read_group({"system": "/api/system", "arm": "/api/arm", "fci": "/api/fci", "joint_brakes": "/api/arm/joints", "recovery": "/api/safety/recovery", "self_tests": "/api/safety/self-tests"}), indent=2))
    else:
        mcp.run(transport="stdio")
