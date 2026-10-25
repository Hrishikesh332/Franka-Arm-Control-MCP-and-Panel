"""Connection to franka_teach_rt, the realtime robot process (see rt/franka_teach_rt.cpp).

Commands go as one-line text datagrams to 127.0.0.1:5555; the process broadcasts its state at
50 Hz to 127.0.0.1:5556.
"""

import json
import socket
import threading
import time

COMMAND_ADDR = ('127.0.0.1', 5555)
STATE_ADDR = ('127.0.0.1', 5556)

# franka_teach_rt mode names -> modes of the teach server
RT_MODES = {'idle': 'off', 'free': 'guide', 'follow': 'control'}
SERVER_MODES = {v: k for k, v in RT_MODES.items()}


def parse_state(datagram):
    """S <mode> <robot_mode> <success> <O_T_EE x16> <wrench x6> <follower x8> <counter>|<errors>|<last error>
    or E <reason> while franka_teach_rt has no connection to the robot."""
    if datagram.startswith('E'):
        return {'connected': False, 'last_error': datagram[1:].strip()}
    head, errors, last_error = datagram.split('|', 2)
    f = head.split()
    if f[0] != 'S' or len(f) != 35:
        raise ValueError('bad state message')
    T = [float(v) for v in f[4:20]]  # column major 4x4
    rot = [[T[0], T[4], T[8]], [T[1], T[5], T[9]], [T[2], T[6], T[10]]]
    try:
        errors = json.loads(errors)
    except ValueError:
        errors = [errors] if errors.strip('[] ') else []
    return {
        'connected': True,
        'mode': RT_MODES.get(f[1], 'off'),
        'robot_mode': int(f[2]),
        'success_rate': float(f[3]),
        'p': [T[12], T[13], T[14]],
        'q': quat_from_matrix(rot),
        'wrench': [float(v) for v in f[20:26]],
        'follower': [float(v) for v in f[26:34]],
        'counter': int(f[34]),
        'errors': errors,
        'last_error': last_error.strip(),
    }


def quat_from_matrix(m):
    """Rotation matrix -> quaternion [x, y, z, w]."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = (tr + 1.0) ** 0.5 * 2
        return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = (1.0 + m[0][0] - m[1][1] - m[2][2]) ** 0.5 * 2
        return [s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    if m[1][1] > m[2][2]:
        s = (1.0 + m[1][1] - m[0][0] - m[2][2]) ** 0.5 * 2
        return [(m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    s = (1.0 + m[2][2] - m[0][0] - m[1][1]) ** 0.5 * 2
    return [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s]


class RobotLink:
    """Sends commands to franka_teach_rt and calls on_state(dict) for every state message."""

    def __init__(self, on_state):
        self.on_state = on_state
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(STATE_ADDR)
        self.rx.settimeout(0.5)
        threading.Thread(target=self._receive, daemon=True).start()

    def send(self, command):
        self.tx.sendto(command.encode(), COMMAND_ADDR)

    def _receive(self):
        while True:
            try:
                data = self.rx.recv(4096).decode()
            except socket.timeout:
                continue
            try:
                self.on_state(parse_state(data))
            except (ValueError, IndexError):
                time.sleep(0.01)
