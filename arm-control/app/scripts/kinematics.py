"""Franka Research 3 kinematics: forward kinematics and a seeded inverse kinematics.

Used to replay sequences in joint space: a recorded joint configuration is reachable for sure,
and for a sequence moved to another position the joint angles are computed here, starting from
the recorded ones, so the arm keeps the same configuration (no elbow flips) and every joint is
checked against its range before anything moves.

DH parameters (Craig's modified convention) and joint ranges from Franka's documentation
(https://frankarobotics.github.io/docs/robot_specifications.html). The end effector is the
flange (no end effector configured in Desk; our grippers are passive and modelled only by mass).
"""

import math

import numpy as np

# a, d, alpha per joint, then the flange
DH = [(0.0, 0.333, 0.0), (0.0, 0.0, -math.pi / 2), (0.0, 0.316, math.pi / 2),
      (0.0825, 0.0, math.pi / 2), (-0.0825, 0.384, -math.pi / 2), (0.0, 0.0, math.pi / 2),
      (0.088, 0.0, math.pi / 2)]
FLANGE = (0.0, 0.107, 0.0)
Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
Q_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
MARGIN = 0.15  # rad kept free at both ends of every range (as the controller's soft limits)


def _dh(a, d, alpha, theta):
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0, a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0, 0, 0, 1]])


def fk(q):
    """Flange pose in the base frame (4x4) for joint angles q."""
    T = np.eye(4)
    for (a, d, alpha), theta in zip(DH, q):
        T = T @ _dh(a, d, alpha, theta)
    return T @ _dh(*FLANGE, 0.0)


def pose_to_matrix(p, quat):
    """Position + quaternion [x, y, z, w] -> 4x4."""
    x, y, z, w = quat
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = p
    return T


def _rot_error(R_target, R):
    """Rotation vector (base frame) from R to R_target."""
    E = R_target @ R.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1) / 2)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-9:  # angle close to pi
        w, v = np.linalg.eigh((E + E.T) / 2)
        axis = v[:, np.argmax(w)]
        return axis * angle
    return axis / n * angle


def _pose_error(T_target, T):
    return np.concatenate([T_target[:3, 3] - T[:3, 3], _rot_error(T_target[:3, :3], T[:3, :3])])


def jacobian(q, eps=1e-6):
    """Numerical 6x7 Jacobian (position, rotation vector) in the base frame."""
    T0 = fk(q)
    J = np.zeros((6, 7))
    for i in range(7):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        J[:, i] = _pose_error(fk(dq), T0) / eps
    return J


def ik(T_target, seed, iterations=200, tol_pos=1e-4, tol_rot=1e-3):
    """Joint angles reaching T_target, starting from seed (damped least squares, staying inside
    the joint ranges minus MARGIN). Returns (q, error_text) - q is None if it failed."""
    q = np.clip(np.array(seed, dtype=float), Q_MIN + MARGIN, Q_MAX - MARGIN)
    lam = 0.05
    for _ in range(iterations):
        e = _pose_error(T_target, fk(q))
        if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
            return q, ''
        J = jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), e)
        # pull gently towards the seed in the null space (keep the recorded arm configuration)
        N = np.eye(7) - np.linalg.pinv(J) @ J
        dq += 0.1 * N @ (np.array(seed) - q)
        step = np.max(np.abs(dq))
        if step > 0.2:
            dq *= 0.2 / step
        q = np.clip(q + dq, Q_MIN + MARGIN, Q_MAX - MARGIN)
    e = _pose_error(T_target, fk(q))
    return None, f'{np.linalg.norm(e[:3]) * 1000:.0f} mm / ' \
                 f'{math.degrees(np.linalg.norm(e[3:])):.0f}° short with all joints inside their ranges'


def limit_problem(q):
    """Text describing the first joint outside its range minus MARGIN, or ''."""
    for i, v in enumerate(q):
        if v < Q_MIN[i] + MARGIN or v > Q_MAX[i] - MARGIN:
            return f'joint {i + 1} would be at {v:.2f} rad (range {Q_MIN[i]:.2f}..{Q_MAX[i]:.2f})'
    return ''
