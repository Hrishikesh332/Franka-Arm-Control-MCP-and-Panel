# Physical Franka workflow defaults

The user requires Franka standard Ready Home as every new workflow's start
and return. Use all seven joint angles in degrees:
`[0, -45, 0, -135, 0, 90, 45]`.

The default workflow is `chat_backward_x_from_franka_ready_v06`
(`0_move2dest8`). The separate Home target is `Home_Franka_Ready_30_v01`
(`0_move2start2`). The previous Z=446.7 mm workcell start is an intermediate
pose. Retain 30% component speeds unless the user changes them.

Before execution, check measured position, complete flange orientation, and
all seven joints against Ready Home: within 5 mm position, 1 degree
orientation, and 1 degree per joint. Require no onboard singularity
warning, no robot errors, Execution mode and normally owned control. Verify
the final return against the same complete Home target.

Home is a joint posture, not base/table calibration or automatic singularity
recovery. Do not claim the arm is currently Home without a fresh read. The
separate Home task's approach from an arbitrary current pose is unqualified.

Preserve old task artifacts and evidence; create new revision folders.
Do not modify backend MCP settings or environments.

For connection modes (direct Ethernet vs. tunnel via the Raspberry Pi) and connection errors,
read CONNECTION.md.
