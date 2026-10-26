"""Run teach_server against a simulated franka_teach_rt (no robot or ROS needed).

The simulated follower moves the end effector towards the latest target at the commanded
speed; a table at TABLE_Z pushes back like a stiff spring, which exercises guarded moves,
presses and the force limit. Usage: python sim_test.py [--serve]  (needs flask)
"""

import math
import os
import sys
import tempfile
import threading
import time
import types
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
TABLE_Z = 0.10
TABLE_K = 2000.0      # N/m
V_MAX = 0.25          # m/s at speed scale 1 (as in cartesian_follower.yaml)


# ---------------------------------------------------------------- stub ROS modules
def quat_angle(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def slerp(a, b, t):
    d = sum(x * y for x, y in zip(a, b))
    if d < 0:
        b, d = [-x for x in b], -d
    if d > 0.9995:
        r = [x + t * (y - x) for x, y in zip(a, b)]
    else:
        th = math.acos(d)
        r = [(math.sin((1 - t) * th) * x + math.sin(t * th) * y) / math.sin(th) for x, y in zip(a, b)]
    n = math.sqrt(sum(x * x for x in r))
    return [x / n for x in r]


# ---------------------------------------------------------------- simulated robot
class SimRobot:
    def __init__(self):
        self.cb = None
        self.mode = 'off'
        self.lock = threading.Lock()
        self.p = [0.4, 0.0, 0.3]
        self.q = [1.0, 0.0, 0.0, 0.0]  # pointing down
        self.target = None
        self.scale = 0.3
        self.active = False
        self.min_z_seen = 1.0
        self.trace = []
        self.force = None   # active force mode command (mirrors the C++ controller)
        self.force_fault = 0  # kept after force mode ends, like the controller
        self.max_press = 0.0
        self.load = None
        threading.Thread(target=self.loop, daemon=True).start()

    def link(self, on_state):
        """RobotLink replacement handed to TeachServer."""
        self.cb = on_state
        return NS(send=self.command)

    def command(self, line):
        f = line.split()
        with self.lock:
            if f[0] == 'mode':
                self.mode = {'idle': 'off', 'free': 'guide', 'follow': 'control'}[f[1]]
                self.active, self.target = self.mode == 'control', None
                if self.mode != 'control':
                    self.force = None
            elif f[0] == 'load':
                self.load = (float(f[1]), [float(v) for v in f[2:5]])
            elif f[0] == 'target':
                v = [float(x) for x in f[1:8]]
                self.target = (v[:3], v[3:])
            elif f[0] == 'stop':
                self.target = None
            elif f[0] == 'speed':
                self.scale = float(f[1])
            elif f[0] == 'force':
                d = [float(x) for x in f[1:]]
                if d[0] and self.force is None:
                    self.force_fault = 0
                    self.force = {'F': d[1], 'gain': d[2], 'va': d[3], 'vc': d[4], 'depth': d[5],
                                  'abort': d[6], 'z0': self.p[2], 'touched': False, 'fault': 0,
                                  'tare': TABLE_K * max(0.0, TABLE_Z - self.p[2])}
                elif d[0] and self.force is not None:
                    self.force['F'] = d[1]  # target changed while pressing (like the controller)
                elif not d[0]:
                    self.force = None
                    if self.target:
                        self.target = ([self.target[0][0], self.target[0][1], self.p[2]], self.target[1])

    def hand_guide(self, p, q=None):
        """Teleport the arm as if moved by hand (guide mode)."""
        with self.lock:
            self.p = list(p)
            if q is not None:
                self.q = list(q)
        time.sleep(0.03)  # let a few pose messages reach the server

    def loop(self):
        dt = 0.005
        while True:
            time.sleep(dt)
            with self.lock:
                v, w, ang, fm = 0.0, 0.0, 0.0, self.force
                contact = TABLE_K * max(0.0, TABLE_Z - self.p[2])
                if self.active and self.target is not None:
                    tp, tq = self.target
                    e = [tp[i] - self.p[i] for i in range(3)]
                    if fm:
                        e[2] = 0.0
                    d = math.sqrt(sum(c * c for c in e))
                    step = min(d, V_MAX * self.scale * dt)
                    if d > 1e-12:
                        self.p = [self.p[i] + e[i] / d * step for i in range(3)]
                    v = step / dt if d > 1e-9 else 0.0
                    dist = max(0.0, d - step)
                    # rotate towards the target at the angular speed limit (0.6 rad/s * scale)
                    ang = quat_angle(self.q, tq)
                    rot = min(ang, 0.6 * self.scale * dt)
                    if ang > 1e-9:
                        self.q = slerp(self.q, tq, rot / ang)
                    w = rot / dt if ang > 1e-9 else 0.0
                    ang = max(0.0, ang - rot)
                else:
                    dist = 0.0
                if fm and self.active:
                    c = contact - fm['tare']
                    fm['touched'] = fm['touched'] or c > min(2.0, 0.3 * fm['F'])
                    lim = fm['vc'] if fm['touched'] else fm['va']
                    vz = -max(-lim, min(lim, fm['gain'] * (fm['F'] - c)))
                    if c > fm['abort']:
                        fm['fault'] = 1
                    elif fm['z0'] - self.p[2] > fm['depth'] and vz < 0:
                        fm['fault'] = 2
                    if fm['fault']:
                        self.force_fault = fm['fault']
                        self.force = None
                    else:
                        self.p[2] += vz * dt
                        self.max_press = max(self.max_press, c)
                contact = TABLE_K * max(0.0, TABLE_Z - self.p[2])
                self.min_z_seen = min(self.min_z_seen, self.p[2])
                self.trace.append(list(self.p))
                fz = contact
                p, q, active = list(self.p), list(self.q), self.active
                fstat = [1.0 if self.force else 0.0,
                         (contact - fm['tare']) if fm else 0.0, float(self.force_fault)]
            if self.cb:
                self.cb({'connected': True, 'mode': self.mode, 'robot_mode': 2 if active else 1, 'success_rate': 1.0,
                         'p': p, 'q': q, 'wrench': [0.0, 0.0, -fz, 0.0, 0.0, 0.0],
                         'follower': [dist, ang, v, w, 1.0 if active else 0.0] + fstat,
                         'counter': 0, 'errors': [], 'last_error': ''})


# ---------------------------------------------------------------- scenario
def main():
    sim = SimRobot()
    sys.path.insert(0, os.path.join(PKG, 'scripts'))
    import teach_server as ts

    data = tempfile.mkdtemp(prefix='teach_sim_')
    node = ts.TeachServer(data, link_factory=sim.link)
    c = ts.make_app(node).test_client()
    time.sleep(0.3)

    def api(path, body=None, method=None, expect_ok=True):
        if method == 'DELETE':
            r = c.delete(path)
        elif body is None and method != 'POST':
            r = c.get(path)
        else:
            r = c.post(path, json=body or {})
        j = r.get_json()
        if expect_ok and isinstance(j, dict) and j.get('ok') is False:
            raise AssertionError(f'{path}: {j["error"]}')
        return j

    def wait_job(timeout=60):
        time.sleep(0.2)
        end = time.time() + timeout
        while time.time() < end:
            job = api('/api/state')['job']
            if not job['running']:
                return job['message']
            time.sleep(0.05)
        raise AssertionError('job did not finish')

    # gripper + record a forklift-like pick at (0.5, 0.0): side, down, guarded down, slide, up
    api('/api/grippers', {'name': 'fork'})
    api('/api/active', {'gripper': 'fork'})
    sim.hand_guide([0.5, 0.0, 0.20])
    api('/api/record/start', {'gripper': 'fork', 'kind': 'pick', 'speed': 0.5})
    sim.hand_guide([0.5, -0.04, 0.20]); api('/api/record/waypoint', {})
    sim.hand_guide([0.5, -0.04, 0.13]); api('/api/record/waypoint', {})
    api('/api/mode', {'mode': 'control'})
    r = api('/api/record/guarded', {'axis': '-z', 'dist': 0.06, 'force': 5.0, 'speed': 0.1})
    assert r['executed'], r
    z_contact = api('/api/state')['pose']['p'][2]
    assert TABLE_Z - 0.004 < z_contact < TABLE_Z, z_contact
    api('/api/mode', {'mode': 'guide'})
    sim.hand_guide([0.5, 0.0, z_contact + 0.002]); api('/api/record/waypoint', {})
    sim.hand_guide([0.5, 0.0, 0.20])
    n = api('/api/record/finish', {})['steps']
    print('recorded pick with', n, 'steps')
    g = api('/api/grippers/fork')
    kinds = [s['type'] for s in g['sequences']['pick']['steps']]
    assert kinds == ['pose', 'pose', 'pose', 'guarded', 'pose', 'pose'], kinds

    # teach a position by moving the arm there, then pick there
    sim.hand_guide([0.3, 0.35, 0.25])
    api('/api/positions', {'name': 'shelf A', 'angle': 'recorded'})
    sim.hand_guide([0.45, -0.2, 0.30])
    sim.min_z_seen = 1.0
    sim.trace.clear()
    assert api('/api/positions/shelf A/auto', method='POST')['kind'] == 'pick'
    msg = wait_job()
    assert msg == 'done', (msg, 'min z', sim.min_z_seen, 'last', sim.trace[-1])
    st = api('/api/state')
    assert st['holding'] is True
    end = st['pose']['p']
    assert abs(end[0] - 0.3) < 1e-3 and abs(end[1] - 0.35) < 1e-3 and abs(end[2] - 0.20) < 1e-3, end
    assert TABLE_Z - 0.004 < sim.min_z_seen < TABLE_Z, sim.min_z_seen
    side = min(sim.trace, key=lambda p: p[2])
    assert abs(side[0] - 0.3) < 2e-3 and abs(side[1] - 0.31) < 2e-3, side  # -4 cm in y, moved
    assert max(p[2] for p in sim.trace) >= 0.30 - 1e-6  # travelled at safe height
    print('pick at shelf A ok, deepest point', [round(v, 4) for v in side])

    # automatic angle: at a position 90 deg further round the base the whole pick turns with it
    api('/api/active', {'holding': False})
    sim.hand_guide([0.0, 0.5, 0.25])
    api('/api/positions', {'name': 'left', 'angle': 'auto'})
    sim.trace.clear()
    api('/api/positions/left/pick', method='POST')
    assert wait_job() == 'done'
    yaw = math.degrees(api('/api/state')['pose']['yaw'])
    side = min(sim.trace, key=lambda p: p[2])
    # recorded: side step -4 cm in y at bearing 0 -> now +4 cm in x at bearing 90 deg
    assert abs(yaw - 90) < 1 and abs(side[0] - 0.04) < 2e-3 and abs(side[1] - 0.5) < 2e-3, (yaw, side)
    print('auto angle ok: yaw %.1f deg, side step at %s' % (yaw, [round(v, 3) for v in side]))

    # picking again while holding is refused, placing without a sequence is refused
    r = api('/api/positions/shelf A/pick', method='POST', expect_ok=False)
    assert r['ok'] is False and 'already holds' in r['error'], r
    r = api('/api/positions/shelf A/auto', method='POST', expect_ok=False)
    assert r['ok'] is False and 'no place sequence' in r['error'], r

    # force limit: lower the table limit so the guarded contact itself exceeds it
    api('/api/settings', {'max_force': 3.0})
    api('/api/active', {'holding': False})
    api('/api/positions/shelf A/pick', method='POST')
    msg = wait_job()
    assert msg.startswith('aborted: force limit'), msg
    assert api('/api/state')['holding'] is False
    print('force limit abort ok:', msg)

    # desk frame from two taught points; positions report desk coordinates
    api('/api/settings', {'max_force': 25.0})
    api('/api/mode', {'mode': 'guide'})
    sim.hand_guide([0.2, -0.3, 0.2]); api('/api/desk/origin', method='POST')
    sim.hand_guide([0.2, 0.3, 0.2]); api('/api/desk/xaxis', method='POST')
    pos = api('/api/positions')['shelf A']['desk_xy']
    assert abs(pos[0] - 0.65) < 1e-9 and abs(pos[1] - (-0.1)) < 1e-9, pos
    api('/api/positions', {'name': 'B', 'desk_xy': [100, 0]})
    assert abs(api('/api/positions')['B']['xy'][1] - (-0.2)) < 1e-9

    # gripper load is sent to the robot while none of our controllers runs
    api('/api/mode', {'mode': 'off'})
    api('/api/grippers/fork/load', {'mass': 0.4, 'com_mm': [0, 0, 50]})
    assert sim.load == (0.4, [0.0, 0.0, 0.05]), sim.load

    # record a place sequence with a live press: 15 N while rotating 90 deg
    sim.hand_guide([0.5, 0.0, 0.20])
    api('/api/record/start', {'gripper': 'fork', 'kind': 'place', 'speed': 0.5})
    api('/api/mode', {'mode': 'control'})
    api('/api/jog', {'axis': 'z', 'delta': -0.08})
    time.sleep(1.5)
    api('/api/record/waypoint', {})
    sim.max_press = 0.0
    yaw0 = api('/api/state')['pose']['yaw']
    r = api('/api/record/press', {'force': 15, 'contact': 'springy', 'rotate_deg': 90,
                                  'rot_speed_deg': 45, 'hold_s': 0.5, 'max_depth': 0.04})
    assert r['executed'], r
    st = api('/api/state')
    turned = math.degrees(ts.tc.wrap(st['pose']['yaw'] - yaw0))
    assert 13.5 < sim.max_press < 16.5 and abs(abs(turned) - 90) < 1, (sim.max_press, turned)
    print(f'live press ok: peak {sim.max_press:.1f} N, rotated {turned:.1f} deg')
    api('/api/jog', {'axis': 'z', 'delta': 0.08})
    time.sleep(1.5)
    api('/api/record/finish', {})
    kinds = [s_['type'] for s_ in api('/api/grippers/fork')['sequences']['place']['steps']]
    assert kinds == ['pose', 'pose', 'press', 'pose'], kinds

    # replay the place at shelf A
    api('/api/active', {'holding': True})
    sim.max_press = 0.0
    assert api('/api/positions/shelf A/auto', method='POST')['kind'] == 'place'
    msg = wait_job()
    assert msg == 'done', msg
    st = api('/api/state')
    assert st['holding'] is False and 13.5 < sim.max_press < 16.5, (st['holding'], sim.max_press)
    print(f'place with press at shelf A ok: peak {sim.max_press:.1f} N')

    # press that cannot reach the force within its depth is aborted by the controller
    api('/api/grippers/fork/place/edit', {'op': 'set', 'index': 2, 'max_depth': 0.005})
    api('/api/active', {'holding': True})
    api('/api/positions/shelf A/place', method='POST')
    msg = wait_job()
    assert 'maximum press depth' in msg and api('/api/state')['holding'] is True, msg
    print('press depth abort ok:', msg)

    # continuous recording by hand: path is sampled and thinned, can be paused for stop points
    api('/api/mode', {'mode': 'guide'})
    sim.hand_guide([0.45, -0.10, 0.25])
    api('/api/record/start', {'gripper': 'fork', 'kind': 'pick', 'continuous': True, 'speed': 0.3})
    for i in range(1, 31):                       # 15 cm along +y in 5 mm steps, by "hand"
        sim.hand_guide([0.45, -0.10 + 0.005 * i, 0.25])
        time.sleep(0.11)
    rec = api('/api/state')['recording']
    assert rec['continuous'] and len(rec['path']) > 10, rec
    api('/api/record/continuous', {'on': False})
    sim.hand_guide([0.45, 0.10, 0.20]); time.sleep(0.3)
    n_paused = api('/api/state')['recording']['steps']
    api('/api/record/waypoint', {})
    api('/api/record/continuous', {'on': True})
    for i in range(1, 11):
        sim.hand_guide([0.45 + 0.005 * i, 0.10, 0.20])
        time.sleep(0.11)
    api('/api/record/finish', {})
    steps = api('/api/grippers/fork')['sequences']['pick']['steps']
    stops = [x for x in steps if x['type'] == 'pose' and not x['blend']]
    passes = [x for x in steps if x['type'] == 'pose' and x['blend']]
    assert len(stops) == 3 and 25 <= len(passes) <= 45, (len(stops), len(passes))
    print(f'continuous recording ok: {len(passes)} path points, {len(stops)} stop points '
          f'(nothing sampled while paused: {n_paused} steps then)')

    # measuring the force a part needs: rising force until "take"
    sim.hand_guide([0.5, 0.0, 0.20])
    api('/api/record/start', {'gripper': 'fork', 'kind': 'place', 'speed': 0.5})
    api('/api/mode', {'mode': 'control'})
    api('/api/jog', {'axis': 'z', 'delta': -0.095})          # 5 mm above the table
    time.sleep(2.5)
    api('/api/record/measure', {'action': 'start', 'force': 10, 'contact': 'springy',
                                'rotate_deg': 45, 'max_depth': 0.03})
    end = time.time() + 20
    while time.time() < end and api('/api/state')['measure']['contact'] < 6.0:
        time.sleep(0.1)
    m = api('/api/state')['measure']
    assert m['running'] and m['contact'] >= 6.0, m
    api('/api/record/measure', {'action': 'take'})
    time.sleep(0.5)
    m = api('/api/state')['measure']
    assert not m['running'] and 'recorded' in m['message'], m
    api('/api/jog', {'axis': 'z', 'delta': 0.09})
    time.sleep(2.0)
    api('/api/record/finish', {})
    press = [x for x in api('/api/grippers/fork')['sequences']['place']['steps'] if x['type'] == 'press']
    assert len(press) == 1 and 5.5 <= press[0]['force'] <= 8.0 and press[0]['rotate_deg'] == 45, press
    print(f'measure force ok: {m["message"]}')

    # stop during a move
    api('/api/positions/shelf A/goto', method='POST')
    time.sleep(0.3)
    api('/api/stop', {})
    assert wait_job().startswith('aborted: stopped')
    print('all simulated scenarios passed')


def serve(port=8090):
    """Serve the UI against the simulated robot with some demo data (for manual UI testing)."""
    sim = SimRobot()
    sys.path.insert(0, os.path.join(PKG, 'scripts'))
    import teach_server as ts
    data = tempfile.mkdtemp(prefix='teach_sim_')
    node = ts.TeachServer(data, link_factory=sim.link)
    st = node.store
    for name in ('fork', 'hook'):
        st.add_gripper(name)
    q = [1.0, 0.0, 0.0, 0.0]
    st.save_sequence('fork', 'pick', [
        {'type': 'pose', 'p': [0.5, 0.0, 0.2], 'q': q, 'blend': False},
        {'type': 'pose', 'p': [0.5, -0.04, 0.2], 'q': q, 'blend': False},
        {'type': 'pose', 'p': [0.5, -0.04, 0.13], 'q': q, 'blend': False},
        {'type': 'guarded', 'dir': [0, 0, -1], 'dist': 0.06, 'force': 5.0, 'speed': 0.1},
        {'type': 'pose', 'p': [0.5, 0.0, 0.1], 'q': q, 'blend': False},
        {'type': 'pose', 'p': [0.5, 0.0, 0.2], 'q': q, 'blend': False}], 0.5)
    st.save_sequence('fork', 'place', [
        {'type': 'pose', 'p': [0.5, 0.0, 0.2], 'q': q, 'blend': False},
        {'type': 'pose', 'p': [0.5, 0.0, 0.13], 'q': q, 'blend': False},
        ts.press_step({'force': 12, 'contact': 'springy', 'rotate_deg': 90, 'rot_speed_deg': 30,
                       'hold_s': 1.0, 'max_depth': 0.04}),
        {'type': 'pose', 'p': [0.5, 0.0, 0.2], 'q': q, 'blend': False}], 0.4)
    st.set_gripper_load('fork', 0.35, [0.0, 0.0, 0.045])
    st.save_state({'gripper': 'fork', 'holding': False})
    st.save_desk(ts.tc.make_desk([0.2, -0.4], [0.2, 0.4]))
    for n, xy, yaw in (('shelf A', [0.35, 0.3], None), ('shelf B', [0.55, 0.3], 0.5),
                       ('station', [0.5, -0.25], None)):
        st.save_position(n, xy, yaw)
    ts.make_app(node).run(host='127.0.0.1', port=port, threaded=True)


if __name__ == '__main__':
    serve() if '--serve' in sys.argv else main()
