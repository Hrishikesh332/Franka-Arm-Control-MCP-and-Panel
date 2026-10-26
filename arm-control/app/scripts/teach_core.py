"""ROS-free core of franka_teach: geometry, storage and motion planning.

Coordinates are in the robot base frame (metres, quaternions as [x, y, z, w]) unless noted.

A gripper has a 'pick' and a 'place' sequence. A sequence is recorded starting (and ending)
with the end effector above the object. It is stored relative to that start pose: positions
are rotated into the start pose's yaw frame and taken relative to its x/y, while the height
stays absolute (the table height is the same everywhere). Replaying it at a position moves
the start to the position's x/y and, if the position has an orientation, rotates the whole
sequence about the vertical axis to it.

Angle modes of a position (rotation about the vertical axis only; tilt stays as recorded):
  auto     - the sequence approaches from the same direction as seen from the robot base as when
             it was recorded, so the arm looks the same at every position (mostly joint 1 turns,
             the wrist does not twist towards its limits)
  recorded - the absolute angle of the recording
  taught   - the angle stored with the position
"""

import json
import math
import re
import time
from pathlib import Path

KINDS = ('pick', 'place')

DEFAULT_SETTINGS = {
    'safe_z': 0.30,          # m, minimum travel height between positions
    'transit_speed': 0.25,   # fraction of the controller's max velocity for travel moves
    'max_force': 25.0,       # N, abort a sequence if the external force exceeds this
}


# ---------------------------------------------------------------- geometry
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def quat_about(axis, angle):
    s = math.sin(angle / 2.0)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)]


def quat_z(angle):
    return quat_about([0.0, 0.0, 1.0], angle)


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q]


def quat_angle(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def yaw_of(q):
    """Heading of the end effector's x axis projected onto the table plane."""
    x, y, z, w = q
    return math.atan2(2.0 * (x * y + z * w), 1.0 - 2.0 * (y * y + z * z))


def rot2(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c * v[0] - s * v[1], s * v[0] + c * v[1]]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------- desk frame
def desk_to_base(desk, xy):
    """Desk coordinates (from the calibrated origin along the calibrated x axis) -> base x/y."""
    if not desk:
        return list(xy)
    d = rot2(xy, desk['angle'])
    return [desk['origin'][0] + d[0], desk['origin'][1] + d[1]]


def base_to_desk(desk, xy):
    if not desk:
        return list(xy)
    return rot2([xy[0] - desk['origin'][0], xy[1] - desk['origin'][1]], -desk['angle'])


def make_desk(origin_xy, xaxis_xy):
    dx, dy = xaxis_xy[0] - origin_xy[0], xaxis_xy[1] - origin_xy[1]
    if math.hypot(dx, dy) < 0.05:
        raise ValueError('the x-axis point must be at least 5 cm from the origin')
    return {'origin': list(origin_xy), 'angle': math.atan2(dy, dx)}


# ---------------------------------------------------------------- sequences
def to_local(steps, start_p, start_q):
    """Convert absolute recorded steps into the start pose's frame (see module docstring)."""
    yaw = yaw_of(start_q)
    qinv = quat_z(-yaw)
    out = []
    for s in steps:
        s = dict(s)
        if s['type'] == 'pose':
            d = rot2([s['p'][0] - start_p[0], s['p'][1] - start_p[1]], -yaw)
            s['p'] = [d[0], d[1], s['p'][2]]
            s['q'] = quat_normalize(quat_mul(qinv, s['q']))
        elif s['type'] == 'guarded':
            d = rot2(s['dir'][:2], -yaw)
            s['dir'] = [d[0], d[1], s['dir'][2]]
        out.append(s)
    return out


def instantiate(seq, xy, yaw=None):
    """Absolute steps of a stored sequence with its start placed at xy (and rotated to yaw).

    yaw=None keeps the orientation the sequence was recorded with.
    """
    yaw = seq['recorded_yaw'] if yaw is None else yaw
    qz = quat_z(yaw)
    turn = wrap(yaw - seq['recorded_yaw'])
    out = []
    for s in seq['steps']:
        s = dict(s)
        if s['type'] == 'pose':
            d = rot2(s['p'][:2], yaw)
            s['p'] = [xy[0] + d[0], xy[1] + d[1], s['p'][2]]
            s['q'] = quat_normalize(quat_mul(qz, s['q']))
            if s.get('joints'):
                # seed for the IK: the recorded configuration, turned with the base joint
                s['joints'] = [s['joints'][0] + turn] + list(s['joints'][1:])
        elif s['type'] == 'guarded':
            d = rot2(s['dir'][:2], yaw)
            s['dir'] = [d[0], d[1], s['dir'][2]]
        out.append(s)
    return out


def bearing(xy):
    """Direction of a point as seen from the robot base."""
    return math.atan2(xy[1], xy[0])


def position_yaw(seq, pos):
    """Start yaw to use for a stored sequence at a position (None = as recorded)."""
    mode = position_angle_mode(pos)
    if mode == 'taught':
        return pos['yaw']
    if mode == 'auto':
        return seq['recorded_yaw'] + wrap(bearing(pos['xy']) - bearing(seq['recorded_xy']))
    return None


def position_angle_mode(pos):
    if pos.get('angle') in ('auto', 'recorded', 'taught'):
        return pos['angle']
    return 'taught' if pos.get('yaw') is not None else 'auto'  # positions saved before modes


def slerp(a, b, t):
    d = sum(x * y for x, y in zip(a, b))
    if d < 0:
        b, d = [-x for x in b], -d
    if d > 0.9995:
        return quat_normalize([x + t * (y - x) for x, y in zip(a, b)])
    th = math.acos(d)
    return quat_normalize([(math.sin((1 - t) * th) * x + math.sin(t * th) * y) / math.sin(th)
                           for x, y in zip(a, b)])


def densify(p0, q0, step, max_dist, max_angle):
    """Intermediate pass-through poses on the straight line from (p0, q0) to a pose step, so a
    joint-space replay follows the Cartesian line (joint interpolation alone bends it)."""
    n = max(math.ceil(math.dist(p0, step['p']) / max_dist), math.ceil(quat_angle(q0, step['q']) / max_angle))
    out = []
    for k in range(1, n):
        t = k / n
        out.append({'type': 'pose', 'blend': True, 'speed': step.get('speed'), 'dense': True,
                    'p': [a + t * (b - a) for a, b in zip(p0, step['p'])], 'q': slerp(q0, step['q'], t)})
    return out


def start_pose(steps):
    first = next((s for s in steps if s['type'] == 'pose'), None)
    if first is None:
        raise ValueError('sequence has no poses')
    return first['p'], first['q']


def transit(cur_p, target_p, target_q, safe_z, speed):
    """Rise to a safe height, travel above the target, then descend onto it."""
    z = max(cur_p[2], target_p[2], safe_z)
    # 'seed' tells the joint planner where to start its IK: from the current configuration, or
    # from the configuration of the sequence that follows
    return [
        {'type': 'pose', 'p': [cur_p[0], cur_p[1], z], 'q': target_q, 'blend': True,
         'speed': speed, 'seed': 'current'},
        {'type': 'pose', 'p': [target_p[0], target_p[1], z], 'q': target_q, 'blend': True,
         'speed': speed, 'seed': 'next'},
        {'type': 'pose', 'p': list(target_p), 'q': target_q, 'blend': False, 'speed': speed,
         'seed': 'next'},
    ]


def thin_continuous(steps, min_dist=0.005, min_angle=math.radians(3.0)):
    """Drop continuously recorded poses that are closer than min_dist / min_angle together."""
    out, last = [], None
    for s in steps:
        if s['type'] != 'pose' or not s.get('blend'):
            out.append(s)
            if s['type'] == 'pose':
                last = s
            continue
        if last is None or (math.dist(last['p'], s['p']) > min_dist
                            or quat_angle(last['q'], s['q']) > min_angle):
            out.append(s)
            last = s
    return out


# ---------------------------------------------------------------- storage
def safe_name(name):
    name = re.sub(r'[^\w\- ]', '', str(name), flags=re.UNICODE).strip()
    if not name or len(name) > 60:
        raise ValueError('invalid name')
    return name


class Store:
    """JSON files under data_dir: grippers/<name>.json, positions.json, desk.json, state.json."""

    def __init__(self, data_dir):
        self.dir = Path(data_dir)
        (self.dir / 'grippers').mkdir(parents=True, exist_ok=True)

    def _read(self, name, default):
        f = self.dir / name
        try:
            return json.loads(f.read_text())
        except (FileNotFoundError, ValueError):
            return default

    def _write(self, name, data):
        f = self.dir / name
        tmp = f.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, indent=1))
        tmp.replace(f)

    # grippers
    def gripper_names(self):
        return sorted(f.stem for f in (self.dir / 'grippers').glob('*.json'))

    def gripper(self, name):
        g = self._read(f'grippers/{safe_name(name)}.json', None)
        if g is None:
            raise KeyError(f'unknown gripper "{name}"')
        return g

    def add_gripper(self, name):
        name = safe_name(name)
        if name in self.gripper_names():
            raise ValueError(f'gripper "{name}" already exists')
        self._write(f'grippers/{name}.json',
                    {'name': name, 'created': time.strftime('%Y-%m-%d %H:%M:%S'),
                     'sequences': {k: None for k in KINDS}})
        return name

    def delete_gripper(self, name):
        (self.dir / f'grippers/{safe_name(name)}.json').unlink()
        st = self.state()
        if st.get('gripper') == name:
            st['gripper'] = None
            self.save_state(st)

    def set_gripper_load(self, gripper, mass, com):
        g = self.gripper(gripper)
        g['mass'], g['com'] = mass, list(com)
        self._write(f'grippers/{g["name"]}.json', g)

    def save_sequence(self, gripper, kind, steps, speed):
        if kind not in KINDS:
            raise ValueError('kind must be pick or place')
        g = self.gripper(gripper)
        sp, sq = start_pose(steps)
        g['sequences'][kind] = {
            'recorded': time.strftime('%Y-%m-%d %H:%M:%S'),
            'recorded_xy': sp[:2],
            'recorded_yaw': yaw_of(sq),
            'speed': speed,
            'steps': to_local(steps, sp, sq),
        }
        self._write(f'grippers/{g["name"]}.json', g)

    def update_sequence(self, gripper, kind, seq):
        g = self.gripper(gripper)
        g['sequences'][kind] = seq
        self._write(f'grippers/{g["name"]}.json', g)

    # positions
    def positions(self):
        return self._read('positions.json', {})

    def save_position(self, name, xy, yaw=None, angle='auto'):
        name = safe_name(name)
        if angle not in ('auto', 'recorded', 'taught') or (angle == 'taught' and yaw is None):
            raise ValueError('angle must be auto, recorded or taught (with a yaw)')
        pos = self.positions()
        pos[name] = {'xy': [float(xy[0]), float(xy[1])], 'yaw': yaw, 'angle': angle}
        self._write('positions.json', pos)
        return name

    def delete_position(self, name):
        pos = self.positions()
        pos.pop(name, None)
        self._write('positions.json', pos)

    # desk frame
    def desk(self):
        return self._read('desk.json', None)

    def save_desk(self, desk):
        self._write('desk.json', desk)

    def desk_points(self):
        return self._read('desk_points.json', {})

    def save_desk_points(self, pts):
        self._write('desk_points.json', pts)

    # settings and state (active gripper, whether it holds a part)
    def settings(self):
        return {**DEFAULT_SETTINGS, **self._read('settings.json', {})}

    def save_settings(self, values):
        s = self.settings()
        for k in DEFAULT_SETTINGS:
            if k in values:
                s[k] = float(values[k])
        self._write('settings.json', s)
        return s

    def state(self):
        return {'gripper': None, 'holding': False, **self._read('state.json', {})}

    def save_state(self, st):
        self._write('state.json', st)
