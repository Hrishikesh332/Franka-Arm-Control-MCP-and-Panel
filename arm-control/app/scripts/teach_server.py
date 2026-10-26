#!/usr/bin/env python3
"""Teach-and-repeat web server for the Franka FR3 with exchangeable passive grippers.

The robot itself is driven by franka_teach_rt (libfranka, realtime); this server talks to it
over a local socket (robot_link) and provides the web UI and all application logic.

Modes:
  off     - the robot holds itself (franka_teach_rt only reads the state)
  guide   - free drive: gravity compensation with damping, move the arm by hand
  control - the arm follows Cartesian targets: jogging, sequences, presses

Every gripper has a pick and a place sequence that start and end above the object
(see teach_core). Positions are taught by moving the arm above them; clicking a position
picks there if the active gripper is empty, and places there if it holds a part.
"""

import argparse
import logging
import math
import os
import sys
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import teach_core as tc  # noqa: E402
from robot_link import RobotLink, SERVER_MODES  # noqa: E402
from desk import Desk, DeskError  # noqa: E402

log = logging.getLogger('franka_teach')
MODES = ('off', 'guide', 'control')
MAX_ANGULAR_VELOCITY = 0.6  # rad/s at speed 1, as in franka_teach_rt
ROBOT_MODES = {0: 'other', 1: 'idle', 2: 'move', 3: 'guiding', 4: 'reflex',
               5: 'user stopped', 6: 'automatic error recovery'}
AXES = {'+x': [1, 0, 0], '-x': [-1, 0, 0], '+y': [0, 1, 0], '-y': [0, -1, 0],
        '+z': [0, 0, 1], '-z': [0, 0, -1]}

BLEND_RADIUS = 0.01                   # m, pass-through waypoints are switched this early
REACHED_DIST = 2e-4                   # m, stop waypoints count as reached below this
REACHED_ANGLE = math.radians(0.2)

# Force mode settings per contact type: gain (m/s)/N, approach speed, speed after contact (m/s).
# Tuned in simulation: 'springy' is precise for mechanisms of ~1-20 N/mm, 'hard' for the bare desk.
PRESS_SETTINGS = {'springy': (0.002, 0.010, 0.005), 'hard': (0.0005, 0.005, 0.001)}
PRESS_ABORT_MARGIN = 15.0   # N above the target force
FORCE_FAULTS = {1: 'contact force too high', 2: 'maximum press depth reached without the force'}


class Aborted(RuntimeError):
    pass


def press_step(b):
    """Validated press step. The rotation is about the vertical axis, so it is the same in the
    sequence's local frame and needs no transformation when a sequence is moved or rotated."""
    contact = b.get('contact', 'springy')
    if contact not in PRESS_SETTINGS:
        raise ValueError('contact must be springy or hard')
    force = float(b.get('force', 10.0))
    if not 0.5 <= force <= 50.0:
        raise ValueError('press force must be between 0.5 and 50 N')
    return {'type': 'press', 'force': force, 'contact': contact,
            'rotate_deg': float(b.get('rotate_deg', 0.0)),
            'rot_speed_deg': min(90.0, max(1.0, float(b.get('rot_speed_deg', 30.0)))),
            'hold_s': min(60.0, max(0.0, float(b.get('hold_s', 0.5)))),
            'max_depth': min(0.1, max(0.001, float(b.get('max_depth', 0.03))))}


class TeachServer:

    def __init__(self, data_dir, link_factory=RobotLink, robot_ip=None):
        self.store = tc.Store(data_dir)
        self.robot_ip = robot_ip or os.environ.get('ROBOT_IP', '192.168.5.10')
        self.desk = None                 # Desk session holding control, see desk_start()
        self.desk_info = {'running': False, 'message': ''}

        self.lock = threading.Lock()
        self.pose = None            # (p, q) measured, q = [x, y, z, w]
        self.pose_stamp = 0.0
        self.wrench = [0.0] * 6     # external force/torque in the base frame
        self.robot_mode = None
        self.success_rate = None
        self.errors = []
        self.last_error = ''
        self.connected = False
        self.connection_error = 'robot process not running'
        self.connect_stamp = 0.0
        self.follower_status = None  # [dist, angle, v, w, active]
        self.active_mode = 'off'      # which of our controllers runs, see MODES
        self.target = None           # last target sent to the follower
        self.speed = 0.3             # manual (jog) speed fraction
        self.rec = None              # active recording, see record_start()
        self.job = None              # running motion thread
        self.abort = threading.Event()
        self.job_info = {'running': False, 'label': '', 'step': 0, 'steps': 0, 'message': ''}
        self.collision_set = False
        self.load_applied = None     # (mass, com) last sent to the robot

        self.link = link_factory(self.on_robot_state)
        threading.Thread(target=self.record_loop, daemon=True).start()

    # ---------------------------------------------------------------- robot state
    def on_robot_state(self, st):
        with self.lock:
            self.connected = st['connected']
            self.connect_stamp = time.time()
            if not st['connected']:
                self.connection_error = st['last_error']
                self.active_mode = 'off'
                self.follower_status = None
                return
            self.connection_error = ''
            self.pose = (st['p'], st['q'])
            self.pose_stamp = time.time()
            self.wrench = st['wrench']
            self.robot_mode = st['robot_mode']
            self.success_rate = st['success_rate']
            self.errors = st['errors']
            self.last_error = st['last_error']
            self.active_mode = st['mode']
            self.follower_status = st['follower'] if st['mode'] == 'control' else None

    def record_loop(self):
        while True:
            time.sleep(0.1)
            self.record_tick()

    def record_tick(self):
        with self.lock:
            if (self.rec is None or not self.rec['continuous'] or self.rec.get('paused')
                    or self.pose is None):
                return
            p, q = self.pose
            self.rec['steps'].append({'type': 'pose', 'p': list(p), 'q': list(q), 'blend': True})
            self.rec['steps'] = tc.thin_continuous(self.rec['steps'])

    # ---------------------------------------------------------------- helpers
    def current_pose(self):
        with self.lock:
            if self.pose is None or time.time() - self.pose_stamp > 1.0:
                raise RuntimeError('no robot pose received (is the robot connected?)')
            return list(self.pose[0]), list(self.pose[1])

    @property
    def follower_active(self):
        return self.active_mode == 'control'

    def robot_connected(self):
        """(connected, reason) - call with self.lock held."""
        if time.time() - self.connect_stamp > 1.0:
            return False, 'robot process (franka_teach_rt) not running'
        if not self.connected:
            reason = self.connection_error or 'no connection'
            if 'FCI' in reason:
                reason = 'FCI is not active - activate FCI in Desk'
            return False, reason
        return True, ''

    def wait_mode(self, mode, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.active_mode == mode:
                    return
                error = self.last_error
            time.sleep(0.02)
        raise RuntimeError(f'robot did not switch to {mode}' + (f': {error}' if error else
                           ' (robot in error state? try Recover errors)'))

    def set_mode(self, mode):
        """off: the robot holds itself; guide: free drive by hand; control: jog / sequences."""
        if mode not in MODES:
            raise ValueError('mode must be off, guide or control')
        if mode != 'control':
            self.stop()
        with self.lock:
            current = self.active_mode
            in_error = self.robot_mode == 4 or bool(self.errors)
            connected, why = self.robot_connected()
        if not connected:
            raise RuntimeError(f'robot not connected: {why}')
        if current == mode:
            return
        if in_error:
            raise RuntimeError('the robot is in an error state - press Recover errors first')
        if current != 'off':
            self.link.send('mode idle')
            self.wait_mode('off')
        self.apply_load()  # robot parameters can only be changed while no motion runs
        with self.lock:
            self.target = None
        self.publish_speed(self.speed)
        if mode != 'off':
            self.link.send(f'mode {SERVER_MODES[mode]}')
            self.wait_mode(mode)

    def apply_load(self):
        """Tell the robot the mass of the mounted gripper (executed by franka_teach_rt in idle)."""
        st = self.store.state()
        g = self.store.gripper(st['gripper']) if st['gripper'] else {}
        load = (float(g.get('mass', 0.0)), tuple(g.get('com', [0.0, 0.0, 0.0])))
        if load != self.load_applied:
            self.link.send('load {} {} {} {}'.format(load[0], *load[1]))
            self.load_applied = load

    def recover(self):
        """Clear a robot reflex (automatic error recovery in franka_teach_rt)."""
        self.stop()
        self.link.send('mode idle')
        self.link.send('recover')
        end = time.time() + 10.0
        time.sleep(0.5)
        while time.time() < end:
            with self.lock:
                if not self.errors and self.robot_mode != 4:
                    self.target = None
                    return
            time.sleep(0.1)
        raise RuntimeError('error recovery did not clear the robot errors')

    # ---------------------------------------------------------------- Desk (FCI, brakes)
    def desk_login_data(self):
        user, password = os.environ.get('DESK_USER'), os.environ.get('DESK_PASSWORD')
        if user and password:
            return user, password
        login = self.store._read('desk_login.json', {})
        if not login.get('user'):
            raise RuntimeError('no Desk login stored - enter it in the Positions tab under Settings')
        return login['user'], login['password']

    def save_desk_login(self, user, password):
        path = self.store.dir / 'desk_login.json'
        self.store._write('desk_login.json', {'user': user, 'password': password})
        os.chmod(path, 0o600)

    def desk_job(self, label, steps):
        if self.desk_info['running']:
            raise RuntimeError('a Desk action is already running')

        def run():
            self.desk_info.update(running=True, message=label + '...')
            try:
                steps()
                self.desk_info['message'] = label + ': done'
            except (DeskError, RuntimeError, OSError) as e:
                self.desk_info['message'] = f'{label} failed: {e}'
                log.warning(self.desk_info['message'])
            finally:
                self.desk_info['running'] = False
        threading.Thread(target=run, daemon=True).start()

    def desk_start(self):
        """Log in to Desk, take control (Pilot confirmation if needed), unlock, activate FCI."""
        user, password = self.desk_login_data()

        def steps():
            desk = Desk(self.robot_ip, user, password)
            say = lambda m: self.desk_info.update(message=m)  # noqa: E731
            say('logging in to Desk')
            desk.login()
            say('taking control of the robot')
            desk.take_control(on_waiting=say)
            self.desk = desk
            say('unlocking the brakes')
            try:
                desk.open_brakes()
            except DeskError as e:
                log.info('unlock: %s (continuing, the brakes may already be open)', e)
            say('switching to execution mode')
            try:
                desk.set_operating_mode('execution')  # FCI is refused in programming mode
            except DeskError as e:
                log.info('execution mode: %s (continuing, it may already be active)', e)
            say('activating FCI')
            end = time.time() + 10.0  # unlocking takes a few seconds; FCI waits for it
            while True:
                try:
                    desk.activate_fci()
                    break
                except DeskError:
                    if time.time() > end:
                        raise
                    time.sleep(1.0)
        self.desk_job('unlock & activate FCI', steps)

    def desk_stop(self):
        """Deactivate FCI, lock the brakes and give control back (e.g. for colleagues using Desk)."""
        user, password = self.desk_login_data()

        def steps():
            with self.lock:
                connected = self.robot_connected()[0]
            if connected:
                self.set_mode('off')
            desk = self.desk
            if desk is None:
                desk = Desk(self.robot_ip, user, password)
                desk.login()
                desk.take_control(on_waiting=lambda m: self.desk_info.update(message=m))
            self.desk_info['message'] = 'deactivating FCI'
            try:
                desk.deactivate_fci()
            except DeskError:
                pass  # was not active
            self.desk_info['message'] = 'locking the brakes'
            desk.close_brakes()
            try:
                desk.set_operating_mode('programming')  # back to normal Desk use
            except DeskError as e:
                log.info('programming mode: %s', e)
            desk.release_control()
            self.desk = None
        self.desk_job('deactivate FCI & lock', steps)

    def publish_speed(self, speed):
        self.link.send(f'speed {min(1.0, max(0.01, float(speed)))}')

    def current_target(self):
        with self.lock:
            if self.target is not None:
                return [list(self.target[0]), list(self.target[1])]
        return list(self.current_pose())

    def send_target(self, p, q):
        with self.lock:
            if not self.follower_active:
                raise RuntimeError('switch to control mode first')
            self.target = (list(p), list(q))
        self.link.send('target ' + ' '.join(repr(float(v)) for v in list(p) + list(q)))

    def stop_motion(self):
        self.link.send('stop')
        with self.lock:
            self.target = None

    def stop(self):
        self.abort.set()
        self.stop_motion()
        if self.job is not None and self.job is not threading.current_thread():
            self.job.join(timeout=3.0)

    # ---------------------------------------------------------------- execution
    def check(self, tare, max_force):
        """Raise if the job was stopped, the robot faulted or the external force is too high."""
        if self.abort.is_set():
            raise Aborted('stopped')
        with self.lock:
            active, mode, w = self.follower_active, self.robot_mode, list(self.wrench)
        if not active or mode == 4:
            with self.lock:
                error = self.last_error
            raise Aborted('robot stopped' + (f': {error}' if error else ' (reflex / error)'))
        force = math.dist(w[:3], tare[:3])
        if force > max_force:
            raise Aborted(f'force limit exceeded ({force:.1f} N > {max_force:.0f} N)')

    def wait_reached(self, blend, tare, max_force, timeout=120.0):
        time.sleep(0.05)  # let the status reflect the new target
        end = time.time() + timeout
        while True:
            self.check(tare, max_force)
            with self.lock:
                st = self.follower_status
            if st is not None:
                dist, ang, v = st[0], st[1], st[2]
                if blend and dist < BLEND_RADIUS and ang < math.radians(5):
                    return
                if not blend and dist < REACHED_DIST and ang < REACHED_ANGLE and v < 1e-3:
                    return
            if time.time() > end:
                raise Aborted('timeout waiting for motion')
            time.sleep(0.01)

    def guarded_move(self, step, max_force):
        """Move along step['dir'] until the contact force along it reaches step['force']."""
        d = step['dir']
        n = math.sqrt(sum(c * c for c in d))
        d = [c / n for c in d]
        time.sleep(0.2)  # settle before taring the force estimate
        with self.lock:
            tare = list(self.wrench)
        p, q = self.current_target()
        self.publish_speed(step.get('speed', 0.05))
        self.send_target([p[i] + d[i] * step['dist'] for i in range(3)], q)
        time.sleep(0.05)
        while True:
            self.check(tare, max_force)
            with self.lock:
                w, st = list(self.wrench), self.follower_status
            along = abs(sum((w[i] - tare[i]) * d[i] for i in range(3)))
            if along >= step['force']:
                self.stop_motion()
                time.sleep(0.3)
                return True
            if st is not None and st[0] < REACHED_DIST and st[2] < 1e-3:
                if step.get('require_contact', True):
                    raise Aborted(f'no contact within {step["dist"] * 1000:.0f} mm')
                return False
            time.sleep(0.005)

    def press(self, step, tare):
        """Force-controlled press along -z, optionally rotating about the vertical axis meanwhile."""
        gain, v_approach, v_contact = PRESS_SETTINGS[step.get('contact', 'springy')]
        force = float(step['force'])
        abort_force = force + PRESS_ABORT_MARGIN
        limit = abort_force + 5.0  # the controller aborts first; this is the server-side backstop
        time.sleep(0.2)  # settle: the controller tares the force estimate when force mode starts
        self.link.send('force 1 {} {} {} {} {} {}'.format(
            force, gain, v_approach, v_contact, float(step.get('max_depth', 0.03)), abort_force))
        try:
            def wait(cond, timeout, what):
                end, since = time.time() + timeout, None
                time.sleep(0.1)
                while True:
                    self.check(tare, limit)
                    with self.lock:
                        st = self.follower_status
                    if st is None or len(st) < 8:
                        raise Aborted('controller does not support force mode (rebuild needed)')
                    if st[7]:
                        raise Aborted(f'press aborted: {FORCE_FAULTS.get(int(st[7]), "fault")}')
                    if not st[5]:
                        raise Aborted('press aborted: force mode ended')
                    if cond(st):
                        since = since or time.time()
                        if time.time() - since >= 0.3:
                            return
                    else:
                        since = None
                    if time.time() > end:
                        raise Aborted(f'press: timeout while {what}')
                    time.sleep(0.01)

            tol = max(1.0, 0.1 * force)
            wait(lambda st: abs(st[6] - force) < tol, 30.0, 'building up the force')
            angle = math.radians(float(step.get('rotate_deg', 0.0)))
            if abs(angle) > 1e-4:
                p, q = self.current_target()
                rate = math.radians(float(step.get('rot_speed_deg', 30.0)))
                self.publish_speed(rate / MAX_ANGULAR_VELOCITY)
                self.send_target(p, tc.quat_mul(tc.quat_z(angle), q))
                wait(lambda st: st[1] < REACHED_ANGLE and st[3] < 1e-3, 60.0, 'rotating')
            hold = float(step.get('hold_s', 0.0))
            if hold > 0:
                end = time.time() + hold
                while time.time() < end:
                    wait(lambda st: True, 5.0, 'holding')
        finally:
            self.link.send('force 0')
            time.sleep(0.2)

    def execute(self, steps, default_speed, max_force, tare):
        info = self.job_info
        info['steps'] = len(steps)
        for i, s in enumerate(steps):
            info.update(step=i + 1, message=f'step {i + 1}/{len(steps)}: {s["type"]}')
            if s['type'] == 'pose':
                self.publish_speed(s.get('speed', default_speed))
                self.send_target(s['p'], s['q'])
                self.wait_reached(s.get('blend', False), tare, max_force)
            elif s['type'] == 'wait':
                end = time.time() + float(s['s'])
                while time.time() < end:
                    self.check(tare, max_force)
                    time.sleep(0.02)
            elif s['type'] == 'guarded':
                self.guarded_move(s, max_force)
            elif s['type'] == 'press':
                self.publish_speed(default_speed)
                self.press(s, tare)
        last = next((s for s in reversed(steps) if s['type'] == 'pose'), None)
        if last is not None and last.get('blend'):
            self.wait_reached(False, tare, max_force)

    def start_job(self, label, steps, default_speed, on_success=None):
        if self.job is not None and self.job.is_alive():
            raise RuntimeError('the robot is busy')
        with self.lock:
            if self.rec is not None:
                raise RuntimeError('finish or discard the recording first')
        self.set_mode('control')
        max_force = self.store.settings()['max_force']
        self.abort.clear()

        def run():
            self.job_info.update(running=True, label=label, step=0, message='starting')
            try:
                time.sleep(0.2)
                with self.lock:
                    tare = list(self.wrench)
                self.execute(steps, default_speed, max_force, tare)
                if on_success:
                    on_success()
                self.job_info['message'] = 'done'
            except Exception as e:  # noqa: BLE001
                self.stop_motion()
                self.job_info['message'] = f'aborted: {e}'
                log.warning(f'{label}: {e}')
            finally:
                self.publish_speed(self.speed)
                self.job_info['running'] = False

        self.job = threading.Thread(target=run, daemon=True)
        self.job.start()

    def plan_at(self, kind, pos):
        """Transit + sequence for the active gripper at a position."""
        st = self.store.state()
        if not st['gripper']:
            raise RuntimeError('select the mounted gripper first')
        seq = self.store.gripper(st['gripper'])['sequences'].get(kind)
        if not seq:
            raise RuntimeError(f'gripper "{st["gripper"]}" has no {kind} sequence yet')
        steps = tc.instantiate(seq, pos['xy'], tc.position_yaw(seq, pos))
        sp, sq = tc.start_pose(steps)
        settings = self.store.settings()
        cur_p, _ = self.current_target()
        travel = tc.transit(cur_p, sp, sq, settings['safe_z'], settings['transit_speed'])
        return travel + steps, seq.get('speed', 0.2)

    def do_action(self, kind, position):
        st = self.store.state()
        if kind == 'auto':
            kind = 'place' if st['holding'] else 'pick'
        pos = self.store.positions().get(position)
        if pos is None:
            raise KeyError(f'unknown position "{position}"')
        if kind == 'goto':
            seq_kind = 'place' if st['holding'] else 'pick'
            try:
                plan, _ = self.plan_at(seq_kind, pos)
                plan = plan[:3]  # transit only, ends above the object
            except RuntimeError:
                p, q = self.current_target()
                s = self.store.settings()
                plan = tc.transit(p, [pos['xy'][0], pos['xy'][1], max(p[2], s['safe_z'])], q,
                                  s['safe_z'], s['transit_speed'])
            self.start_job(f'go to {position}', plan, 0.3)
            return kind
        if kind == 'pick' and st['holding']:
            raise RuntimeError('the gripper already holds a part - place it first')
        if kind == 'place' and not st['holding']:
            raise RuntimeError('the gripper is empty - pick a part first')
        plan, speed = self.plan_at(kind, pos)

        def done():
            s = self.store.state()
            s['holding'] = kind == 'pick'
            self.store.save_state(s)
        self.start_job(f'{kind} at {position}', plan, speed, done)
        return kind

    # ---------------------------------------------------------------- recording
    def record_start(self, gripper, kind, continuous, speed):
        if kind not in tc.KINDS:
            raise ValueError('kind must be pick or place')
        self.store.gripper(gripper)  # must exist
        if self.job is not None and self.job.is_alive():
            raise RuntimeError('the robot is busy')
        p, q = self.current_pose()
        with self.lock:
            self.rec = {'gripper': gripper, 'kind': kind, 'continuous': bool(continuous),
                        'speed': float(speed),
                        'steps': [{'type': 'pose', 'p': p, 'q': q, 'blend': False}]}

    def record_append(self, step):
        with self.lock:
            if self.rec is None:
                raise RuntimeError('not recording')
            self.rec['steps'].append(step)
            return len(self.rec['steps'])

    def record_stop_point(self):
        """Record the current pose as a stop point unless the last recorded pose is already here."""
        p, q = self.current_pose()
        with self.lock:
            if self.rec is None:
                raise RuntimeError('not recording')
            last = next((s for s in reversed(self.rec['steps']) if s['type'] == 'pose'), None)
            if last and math.dist(last['p'], p) < 1e-3 and tc.quat_angle(last['q'], q) < math.radians(1):
                last['blend'] = False
                return len(self.rec['steps'])
            self.rec['steps'].append({'type': 'pose', 'p': p, 'q': q, 'blend': False})
            return len(self.rec['steps'])

    def record_live(self, step, run):
        """Add a guarded move / press: stop point where it starts, then run it live in control
        mode (continuous sampling paused meanwhile) or only record it otherwise."""
        with self.lock:
            if self.rec is None:
                raise RuntimeError('not recording')
            live = self.follower_active
            self.rec['paused'] = True
        try:
            self.record_stop_point()
            if live:
                self.abort.clear()
                try:
                    run()
                finally:
                    self.publish_speed(self.speed)
            return self.record_append(step), live
        finally:
            with self.lock:
                if self.rec is not None:
                    self.rec['paused'] = False

    def record_guarded(self, axis, dist, force, speed):
        step = {'type': 'guarded', 'dir': AXES[axis], 'dist': float(dist), 'force': float(force),
                'speed': float(speed)}
        return self.record_live(step, lambda: self.guarded_move(step, self.store.settings()['max_force']))

    def record_press(self, params):
        step = press_step(params)

        def run():
            with self.lock:
                tare = list(self.wrench)
            self.press(step, tare)
        return self.record_live(step, run)

    def record_finish(self):
        p, q = self.current_pose()
        with self.lock:
            rec, self.rec = self.rec, None
        if rec is None:
            raise RuntimeError('not recording')
        rec['steps'].append({'type': 'pose', 'p': p, 'q': q, 'blend': False})
        steps = tc.thin_continuous(rec['steps'])
        self.store.save_sequence(rec['gripper'], rec['kind'], steps, rec['speed'])
        return len(steps)

    # ---------------------------------------------------------------- state
    def state(self):
        st = self.store.state()
        with self.lock:
            pose = self.pose
            rec = None if self.rec is None else {
                'gripper': self.rec['gripper'], 'kind': self.rec['kind'],
                'continuous': self.rec['continuous'], 'paused': bool(self.rec.get('paused')),
                'steps': len(self.rec['steps']),
                'stops': sum(1 for x in self.rec['steps'] if x['type'] == 'pose' and not x.get('blend')),
                # recorded path for the map (base frame x/y)
                'path': [x['p'][:2] for x in self.rec['steps'] if x['type'] == 'pose'][-500:]}
            return {
                'pose': {'p': pose[0], 'q': pose[1], 'yaw': tc.yaw_of(pose[1])} if pose else None,
                'pose_age': time.time() - self.pose_stamp if pose else None,
                'force': math.sqrt(sum(c * c for c in self.wrench[:3])),
                'force_z': self.wrench[2],
                'press_force': self.follower_status[6]
                if self.follower_status and len(self.follower_status) >= 8
                and self.follower_status[5] else None,
                'robot_mode': ROBOT_MODES.get(self.robot_mode, 'unknown')
                if self.robot_mode is not None else 'no data',
                'errors': self.errors,
                'last_error': self.last_error,
                'connected': self.robot_connected()[0],
                'desk': dict(self.desk_info),
                'connection_problem': self.robot_connected()[1],
                'success_rate': self.success_rate,
                'mode': self.active_mode,
                'speed': self.speed,
                'recording': rec,
                'job': dict(self.job_info),
                'gripper': st['gripper'],
                'holding': st['holding'],
            }


def make_app(node):
    web_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'web')
    app = Flask(__name__, static_folder=None)
    store = node.store

    def ok(**kw):
        return jsonify(ok=True, **kw)

    def body():
        return request.get_json(silent=True) or {}

    @app.errorhandler(Exception)
    def on_error(e):
        msg = e.args[0] if isinstance(e, KeyError) and e.args else str(e)
        return jsonify(ok=False, error=msg), 400

    @app.get('/')
    def index():
        return send_from_directory(web_dir, 'index.html')

    # -- robot
    @app.get('/api/state')
    def get_state():
        return jsonify(node.state())

    @app.post('/api/mode')
    def post_mode():
        node.set_mode(body()['mode'])
        return ok()

    @app.post('/api/speed')
    def post_speed():
        node.speed = min(1.0, max(0.01, float(body()['speed'])))
        if not node.job_info['running']:
            node.publish_speed(node.speed)
        return ok()

    @app.post('/api/jog')
    def post_jog():
        b = body()
        axis, delta = b['axis'], float(b['delta'])
        if abs(delta) > 0.1:
            raise ValueError('jog step too large')
        if node.job is not None and node.job.is_alive():
            raise RuntimeError('the robot is busy')
        p, q = node.current_target()
        if axis in ('x', 'y', 'z'):
            p['xyz'.index(axis)] += delta
        elif axis in ('rx', 'ry', 'rz'):
            unit = [0.0, 0.0, 0.0]
            unit[['rx', 'ry', 'rz'].index(axis)] = 1.0
            q = tc.quat_mul(tc.quat_about(unit, delta), q)  # about the base frame axis
        else:
            raise ValueError('bad axis')
        node.send_target(p, q)
        return ok()

    @app.post('/api/stop')
    def post_stop():
        node.stop()
        return ok()

    @app.post('/api/recover')
    def post_recover():
        node.recover()
        return ok()

    # -- grippers
    @app.get('/api/grippers')
    def get_grippers():
        out = []
        for name in store.gripper_names():
            g = store.gripper(name)
            out.append({'name': name, 'mass': g.get('mass', 0.0), 'com': g.get('com', [0, 0, 0]),
                        'sequences': {
                k: None if not s else {'steps': len(s['steps']), 'recorded': s['recorded'],
                                       'speed': s.get('speed', 0.2)}
                for k, s in g['sequences'].items()}})
        return jsonify(out)

    @app.get('/api/grippers/<name>')
    def get_gripper(name):
        return jsonify(store.gripper(name))

    @app.post('/api/grippers')
    def post_gripper():
        return ok(name=store.add_gripper(body()['name']))

    @app.delete('/api/grippers/<name>')
    def delete_gripper(name):
        store.delete_gripper(name)
        return ok()

    @app.post('/api/active')
    def post_active():
        b = body()
        st = store.state()
        if 'gripper' in b and (b['gripper'] or None) != st['gripper']:
            if b['gripper']:
                store.gripper(b['gripper'])
            if st['holding']:
                raise RuntimeError('place the part before changing the gripper')
            node.set_mode('off')  # a gripper change is done by hand; the load is set next
            st['gripper'] = b['gripper'] or None
        if 'holding' in b:
            st['holding'] = bool(b['holding'])
        store.save_state(st)
        node.apply_load()
        return ok()

    @app.post('/api/grippers/<name>/load')
    def post_load(name):
        b = body()
        mass = float(b['mass'])
        com = [float(v) / 1000 for v in b.get('com_mm', [0, 0, 0])]
        if not 0.0 <= mass <= 3.0 or any(abs(c) > 0.3 for c in com):
            raise ValueError('mass must be 0-3 kg and the centre of mass within 300 mm')
        store.set_gripper_load(name, mass, com)
        if store.state()['gripper'] == name and node.active_mode == 'off':
            node.apply_load()
        return ok()

    @app.post('/api/grippers/<name>/<kind>/edit')
    def edit_sequence(name, kind):
        b = body()
        seq = store.gripper(name)['sequences'].get(kind)
        if not seq:
            raise RuntimeError('no such sequence')
        steps, i, op = seq['steps'], int(b.get('index', 0)), b['op']
        if op == 'delete':
            if sum(s['type'] == 'pose' for j, s in enumerate(steps) if j != i) == 0:
                raise ValueError('a sequence needs at least one pose')
            steps.pop(i)
        elif op == 'toggle_blend' and steps[i]['type'] == 'pose':
            steps[i]['blend'] = not steps[i].get('blend', False)
        elif op in ('up', 'down'):
            j = i - 1 if op == 'up' else i + 1
            if 0 <= j < len(steps):
                steps[i], steps[j] = steps[j], steps[i]
        elif op == 'insert_wait':
            steps.insert(i + 1, {'type': 'wait', 's': float(b.get('s', 1.0))})
        elif op == 'insert_guarded':
            # direction given in the base frame at the recorded orientation
            d = tc.rot2(AXES[b['axis']][:2], -seq['recorded_yaw'])
            steps.insert(i + 1, {'type': 'guarded', 'dir': [d[0], d[1], AXES[b['axis']][2]],
                                 'dist': float(b.get('dist', 0.03)),
                                 'force': float(b.get('force', 5.0)),
                                 'speed': float(b.get('speed', 0.05))})
        elif op == 'insert_press':
            steps.insert(i + 1, press_step(b))
        elif op == 'set' and steps[i]['type'] == 'press':
            steps[i] = press_step({**steps[i], **b})
        elif op == 'set':
            for k in ('s', 'dist', 'force', 'speed'):
                if k in b and k in steps[i]:
                    steps[i][k] = float(b[k])
            if steps[i]['type'] == 'guarded' and b.get('axis') in AXES:
                d = tc.rot2(AXES[b['axis']][:2], -seq['recorded_yaw'])
                steps[i]['dir'] = [d[0], d[1], AXES[b['axis']][2]]
        elif op == 'speed':
            seq['speed'] = min(1.0, max(0.01, float(b['speed'])))
        elif op == 'clear':
            seq = None
        else:
            raise ValueError('bad edit operation')
        store.update_sequence(name, kind, seq)
        return ok()

    @app.post('/api/grippers/<name>/<kind>/test')
    def test_sequence(name, kind):
        """Run a sequence where it was recorded (ignores and keeps the holding state)."""
        seq = store.gripper(name)['sequences'].get(kind)
        if not seq:
            raise RuntimeError('no such sequence')
        steps = tc.instantiate(seq, seq['recorded_xy'])
        sp, sq = tc.start_pose(steps)
        settings = store.settings()
        cur_p, _ = node.current_target()
        plan = tc.transit(cur_p, sp, sq, settings['safe_z'], settings['transit_speed']) + steps
        node.start_job(f'test {kind} of {name}', plan, seq.get('speed', 0.2))
        return ok()

    # -- recording
    @app.post('/api/record/start')
    def record_start():
        b = body()
        node.record_start(b['gripper'], b['kind'], b.get('continuous', False),
                          b.get('speed', 0.2))
        return ok()

    @app.post('/api/record/waypoint')
    def record_waypoint():
        return ok(steps=node.record_stop_point())

    @app.post('/api/record/continuous')
    def record_continuous():
        with node.lock:
            if node.rec is None:
                raise RuntimeError('not recording')
            node.rec['continuous'] = bool(body().get('on', True))
        return ok()

    @app.post('/api/record/wait')
    def record_wait():
        return ok(steps=node.record_append({'type': 'wait', 's': float(body().get('s', 1.0))}))

    @app.post('/api/record/guarded')
    def record_guarded():
        b = body()
        n, live = node.record_guarded(b['axis'], b.get('dist', 0.03), b.get('force', 5.0),
                                      b.get('speed', 0.05))
        return ok(steps=n, executed=live)

    @app.post('/api/record/press')
    def record_press():
        n, live = node.record_press(body())
        return ok(steps=n, executed=live)

    @app.post('/api/record/finish')
    def record_finish():
        return ok(steps=node.record_finish())

    @app.post('/api/record/cancel')
    def record_cancel():
        with node.lock:
            node.rec = None
        return ok()

    # -- positions and desk frame
    @app.get('/api/positions')
    def get_positions():
        desk = store.desk()
        return jsonify({n: {**p, 'angle': tc.position_angle_mode(p),
                            'desk_xy': tc.base_to_desk(desk, p['xy'])}
                        for n, p in store.positions().items()})

    @app.post('/api/positions')
    def post_position():
        b = body()
        angle = b.get('angle', 'auto')
        if b.get('desk_xy') is not None:
            xy = tc.desk_to_base(store.desk(), [float(v) / 1000 for v in b['desk_xy']])
            yaw = math.radians(float(b['yaw_deg'])) if b.get('yaw_deg') is not None else None
            if yaw is not None:
                angle = 'taught'
        else:
            p, q = node.current_pose()
            xy, yaw = p[:2], tc.yaw_of(q)
        return ok(name=store.save_position(b['name'], xy, yaw if angle == 'taught' else None, angle))

    @app.delete('/api/positions/<name>')
    def delete_position(name):
        store.delete_position(name)
        return ok()

    @app.post('/api/positions/<name>/<kind>')
    def position_action(name, kind):
        if kind not in ('pick', 'place', 'goto', 'auto'):
            raise ValueError('bad action')
        return ok(kind=node.do_action(kind, name))

    @app.post('/api/goto')
    def post_goto():
        """Drive to a point clicked on the map: rise to the safe height, travel, descend again to
        the current height (orientation unchanged)."""
        b = body()
        xy = tc.desk_to_base(store.desk(), [float(v) / 1000 for v in b['desk_xy']])
        p, q = node.current_target()
        if math.hypot(*xy) > 0.8 or math.hypot(*xy) < 0.2:
            raise ValueError('that point is outside the reachable ring (0.2-0.8 m from the robot base)')
        s = store.settings()
        # turn with the robot (like the automatic angle mode) instead of keeping the absolute angle
        q = tc.quat_normalize(tc.quat_mul(tc.quat_z(tc.wrap(tc.bearing(xy) - tc.bearing(p))), q))
        node.start_job(f'go to {b["desk_xy"][0]:.0f}, {b["desk_xy"][1]:.0f}',
                       tc.transit(p, [xy[0], xy[1], p[2]], q, s['safe_z'], s['transit_speed']), 0.3)
        return ok()

    @app.get('/api/desk')
    def get_desk():
        return jsonify({'desk': store.desk(), 'points': store.desk_points()})

    @app.post('/api/desk/<which>')
    def post_desk_point(which):
        if which not in ('origin', 'xaxis'):
            raise ValueError('point must be origin or xaxis')
        p, _ = node.current_pose()
        pts = store.desk_points()
        pts[which] = p[:2]
        store.save_desk_points(pts)
        if 'origin' in pts and 'xaxis' in pts:
            store.save_desk(tc.make_desk(pts['origin'], pts['xaxis']))
        return ok()

    @app.delete('/api/desk')
    def delete_desk():
        store.save_desk(None)
        store.save_desk_points({})
        return ok()

    @app.post('/api/robot/<action>')
    def post_robot(action):
        if action == 'start':
            node.desk_start()
        elif action == 'stop':
            node.desk_stop()
        else:
            raise ValueError('action must be start or stop')
        return ok()

    @app.post('/api/desk_login')
    def post_desk_login():
        b = body()
        if not b.get('user') or not b.get('password'):
            raise ValueError('user and password are required')
        node.save_desk_login(str(b['user']), str(b['password']))
        return ok()

    @app.get('/api/settings')
    def get_settings():
        return jsonify(store.settings())

    @app.post('/api/settings')
    def post_settings():
        return ok(settings=store.save_settings(body()))

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--data-dir', default=os.path.expanduser('~/franka_teach_data'))
    parser.add_argument('--robot-ip', default=os.environ.get('ROBOT_IP', '192.168.5.10'))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    logging.getLogger('werkzeug').setLevel(logging.WARNING)  # no line per request
    node = TeachServer(args.data_dir, robot_ip=args.robot_ip)
    log.info('Web UI on http://0.0.0.0:%d', args.port)
    try:
        make_app(node).run(host='0.0.0.0', port=args.port, threaded=True)
    finally:
        node.stop()


if __name__ == '__main__':
    main()
