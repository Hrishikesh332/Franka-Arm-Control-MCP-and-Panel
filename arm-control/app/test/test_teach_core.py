import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import teach_core as tc  # noqa: E402

DOWN = [1.0, 0.0, 0.0, 0.0]  # gripper pointing down (180 deg about x)


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def same_rot(a, b, tol=1e-9):
    return tc.quat_angle(a, b) < tol


def recorded_sequence(yaw):
    q = tc.quat_mul(tc.quat_z(yaw), DOWN)
    start = [0.5, 0.1, 0.20]
    side = [start[0] + 0.03 * math.cos(yaw + math.pi / 2),
            start[1] + 0.03 * math.sin(yaw + math.pi / 2), 0.20]
    return [
        {'type': 'pose', 'p': start, 'q': q, 'blend': False},
        {'type': 'pose', 'p': side, 'q': q, 'blend': False},
        {'type': 'pose', 'p': [side[0], side[1], 0.12], 'q': q, 'blend': False},
        {'type': 'guarded', 'dir': [math.cos(yaw), math.sin(yaw), 0.0], 'dist': 0.05,
         'force': 5.0, 'speed': 0.05},
        {'type': 'wait', 's': 0.5},
        {'type': 'pose', 'p': start, 'q': q, 'blend': False},
    ]


def test_yaw_of():
    for yaw in (-2.5, -0.3, 0.0, 0.7, 3.0):
        assert abs(tc.wrap(tc.yaw_of(tc.quat_mul(tc.quat_z(yaw), DOWN)) - yaw)) < 1e-9


def test_replay_at_recorded_place_is_identity():
    yaw = math.radians(30)
    steps = recorded_sequence(yaw)
    with tempfile.TemporaryDirectory() as d:
        st = tc.Store(d)
        st.add_gripper('fork A')
        st.save_sequence('fork A', 'pick', steps, 0.2)
        seq = st.gripper('fork A')['sequences']['pick']
    out = tc.instantiate(seq, seq['recorded_xy'])
    for a, b in zip(steps, out):
        assert a['type'] == b['type']
        if a['type'] == 'pose':
            assert close(a['p'], b['p']) and same_rot(a['q'], b['q'])
        if a['type'] == 'guarded':
            assert close(a['dir'], b['dir'])


def test_replay_moved_keeps_height_and_orientation():
    steps = recorded_sequence(math.radians(30))
    sp, sq = tc.start_pose(steps)
    seq = {'recorded_yaw': tc.yaw_of(sq), 'steps': tc.to_local(steps, sp, sq)}
    out = tc.instantiate(seq, [0.2, -0.4])
    for a, b in zip(steps, out):
        if a['type'] == 'pose':
            assert close([b['p'][0] - 0.2, b['p'][1] + 0.4],
                         [a['p'][0] - sp[0], a['p'][1] - sp[1]])
            assert abs(a['p'][2] - b['p'][2]) < 1e-12
            assert same_rot(a['q'], b['q'])


def test_replay_rotated():
    steps = recorded_sequence(0.0)
    sp, sq = tc.start_pose(steps)
    seq = {'recorded_yaw': 0.0, 'steps': tc.to_local(steps, sp, sq)}
    out = tc.instantiate(seq, [0.0, 0.5], yaw=math.pi / 2)
    side = out[1]['p']           # recorded +y side move becomes -x after +90 deg
    assert close(side, [-0.03, 0.5, 0.20])
    assert close(out[3]['dir'], [0.0, 1.0, 0.0])  # recorded +x push becomes +y
    assert abs(tc.wrap(tc.yaw_of(out[0]['q']) - math.pi / 2)) < 1e-9


def test_auto_angle_follows_bearing():
    seq = {'recorded_xy': [0.5, 0.0], 'recorded_yaw': 0.3}
    pos = {'xy': [0.0, 0.5], 'yaw': None, 'angle': 'auto'}
    assert abs(tc.position_yaw(seq, pos) - (0.3 + math.pi / 2)) < 1e-12
    assert tc.position_yaw(seq, {**pos, 'angle': 'recorded'}) is None
    assert tc.position_yaw(seq, {'xy': [0, 0.5], 'yaw': 1.0, 'angle': 'taught'}) == 1.0
    assert tc.position_angle_mode({'xy': [0, 0], 'yaw': None}) == 'auto'  # old files


def test_desk_roundtrip():
    desk = tc.make_desk([0.3, -0.2], [0.3 + 0.5 * math.cos(0.4), -0.2 + 0.5 * math.sin(0.4)])
    for xy in ([0, 0], [0.1, 0.2], [-0.3, 0.05]):
        assert close(tc.base_to_desk(desk, tc.desk_to_base(desk, xy)), xy)
    assert close(tc.base_to_desk(desk, [0.3, -0.2]), [0, 0])


def test_transit_goes_over_safe_height():
    plan = tc.transit([0.4, 0.0, 0.15], [0.5, 0.3, 0.2], DOWN, 0.3, 0.4)
    assert [s['p'][2] for s in plan] == [0.3, 0.3, 0.2]
    assert plan[-1]['blend'] is False


def test_thin_continuous():
    steps = [{'type': 'pose', 'p': [0.001 * i, 0, 0], 'q': DOWN, 'blend': True}
             for i in range(100)]
    assert len(tc.thin_continuous(steps)) < 20


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
