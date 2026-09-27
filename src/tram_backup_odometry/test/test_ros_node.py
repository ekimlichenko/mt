"""ROS 2 node tests: parameters, exact stamp propagation, GNSS release, message content.

Needs rclpy and tram_vehicle_msgs (a sourced Humble workspace, e.g. the Docker
image); skipped otherwise.  Most tests call the subscription callbacks directly
with synthetic messages and capture what the node publishes, so they do not
depend on DDS timing; ``test_dds_end_to_end`` goes through real topics.

Synthetic run: a tram stands still on route T2S at s0 with RTK fixes (master on
the centreline, rover 12.44 m ahead) for 5 s, then accelerates at 1 m/s^2 for
8 s, with wheel speeds in km/h scaled by k (the topic convention).
"""
import math
import os
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tram_vehicle_msgs.msg')

from builtin_interfaces.msg import Time  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tram_backup_odometry.core.config import Params  # noqa: E402
from tram_backup_odometry.core.frames import latlon_to_map  # noqa: E402
from tram_backup_odometry.core.track_map import load_routes  # noqa: E402
from tram_backup_odometry.node import (BackupOdometryNode, CallbackTimes,  # noqa: E402
                                       default_maps_dir, float_to_time)

MAPS = default_maps_dir()
T0_SEC = 1785151800          # an epoch in the dataset's range
S0 = 300.0                   # start on T2S (on the map)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Capture:
    """Stands in for a publisher; keeps plain tuples (the node reuses message objects)."""

    def __init__(self):
        self.msgs = []

    def publish(self, m):
        if isinstance(m, VelocitySensor):
            self.msgs.append(((m.header.stamp.sec, m.header.stamp.nanosec), m.header.frame_id, m.velocity))
        elif isinstance(m, Odometry):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            self.msgs.append(((m.header.stamp.sec, m.header.stamp.nanosec), m.header.frame_id, m.child_frame_id,
                              (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), list(m.pose.covariance),
                              m.twist.twist.linear.x, m.twist.covariance[0]))
        else:
            self.msgs.append(m.data)


def make_node(**overrides):
    params = [Parameter(k, value=v) for k, v in dict({'maps_dir': MAPS}, **overrides).items()]
    node = BackupOdometryNode(parameter_overrides=params)
    node.pub_vel, node.pub_pos, node.pub_slip = Capture(), Capture(), Capture()
    return node


def map_to_latlon(x, y):
    """Inverse of frames.latlon_to_map by Newton iteration (test helper)."""
    lat, lon = 55.8, 37.5
    for _ in range(30):
        e, n = latlon_to_map(lat, lon)
        d = 1e-6
        e1, n1 = latlon_to_map(lat + d, lon)
        e2, n2 = latlon_to_map(lat, lon + d)
        a, b, c, dd = (e1 - e) / d, (e2 - e) / d, (n1 - n) / d, (n2 - n) / d
        rx, ry = x - e, y - n
        det = a * dd - b * c
        lat += (dd * rx - b * ry) / det
        lon += (a * ry - c * rx) / det
    return lat, lon


def stamp(t_ns):
    return Time(sec=t_ns // 1000000000, nanosec=t_ns % 1000000000)


def synthetic_run(node, still_s=5.0, move_s=8.0, acc=1.0, with_gnss=True):
    """Feed the node in stamp order; returns (input stamps set, expected travelled distance)."""
    p = node.params
    k = p.k_for_vehicle()
    route = load_routes(MAPS)['T2S']
    fixes = {}
    for ant, s in (('master', S0), ('rover', S0 + p.rover_ahead_m)):
        x, y, z, _ = route.pose(s)
        fixes[ant] = map_to_latlon(x, y) + (z + p.z_offset_m,)
    stamps = set()
    base = T0_SEC * 1000000000 + 12345          # deliberately not on a round nanosecond value
    step_ns = 50000000                          # 20 Hz grid
    n_steps = int((still_s + move_s) / 0.05)
    for i in range(n_steps + 1):
        t_ns = base + i * step_ns
        t = i * 0.05
        v = 0.0 if t <= still_s else acc * (t - still_s)
        notch = 0 if t < still_s else 5
        st = stamp(t_ns)
        cmd = DriverControllerCommand()
        cmd.header.stamp = st
        cmd.position = notch
        node._on_cmd(cmd)
        stamps.add((st.sec, st.nanosec))
        if i % 2 == 0:
            wst = stamp(t_ns + 7000000)         # wheels on their own phase
            for cb in (node._on_front, node._on_rear):
                m = VelocitySensor()
                m.header.stamp = wst
                m.header.frame_id = 'base_link'
                m.velocity = v * k
                cb(m)
            stamps.add((wst.sec, wst.nanosec))
            if with_gnss and t <= still_s:
                for ant, cb in (('master', node._on_fix_master), ('rover', node._on_fix_rover)):
                    if not node.fix_subs:
                        break
                    lat, lon, alt = fixes[ant]
                    f = NavSatFix()
                    f.header.stamp = stamp(t_ns)
                    f.status.status = 2
                    f.latitude, f.longitude, f.altitude = lat, lon, alt
                    cb(f)
    return stamps, 0.5 * acc * move_s ** 2


# ---------------------------------------------------------------------------------- unit bits
def test_float_to_time_roundtrip():
    for t in (0.0, 1.5, 1785151800.123456789, 1785151800.9999999999):
        tm = float_to_time(t)
        assert 0 <= tm.nanosec < 1000000000
        assert abs(tm.sec + tm.nanosec * 1e-9 - t) < 1e-6


def test_callback_times_percentiles():
    c = CallbackTimes()
    for i in range(1, 1001):
        c.add(i * 1e-5)                         # 10 us .. 10 ms uniformly
    assert c.count == 1000 and abs(c.max - 1e-2) < 1e-12
    assert 4.9e-3 <= c.percentile(0.5) <= 5.0e-3 * 1.07
    assert 9.4e-3 <= c.percentile(0.95) <= 9.5e-3 * 1.07
    assert len(c.take_window()) == 1000 and c.window == []


# ---------------------------------------------------------------------------------- node
def test_parameters_declared_with_dataclass_defaults():
    node = make_node()
    try:
        d = Params()
        for name, value in d.to_dict().items():
            assert node.has_parameter(name), name
            assert node.get_parameter(name).value == value, name
        assert node.params == d
        assert os.path.isdir(node.get_parameter('maps_dir').value)
    finally:
        node.destroy_node()


def test_parameter_overrides_are_coerced():
    # launch passes vehicle_id:=30618 as an integer and floats may come as ints
    node = make_node(vehicle_id=30618, acc_max=2, output_frame='utm')
    try:
        assert node.params.vehicle_id == '30618'
        assert node.params.k_for_vehicle() == Params().wheel_k_30618
        assert node.params.acc_max == 2.0 and isinstance(node.params.acc_max, float)
        assert node.pose_frame_id == Params().frame_id_utm
    finally:
        node.destroy_node()


def test_outputs_carry_exact_input_stamps_and_frames():
    node = make_node()
    try:
        stamps, _ = synthetic_run(node)
        vel, pos = node.pub_vel.msgs, node.pub_pos.msgs
        p = node.params
        assert len(vel) >= 0.5 * (len(stamps))            # at least one output per input stamp pair
        assert all(m[0] in stamps for m in vel), 'velocity stamp not equal to any input header stamp'
        assert all(m[0] in stamps for m in pos), 'position stamp not equal to any input header stamp'
        assert {m[1] for m in vel} == {p.velocity_frame_id}
        assert all(math.isfinite(m[2]) and m[2] >= 0.0 for m in vel)
        assert node.counts['errors'] == 0 and node.counts['stamp_fallback'] == 0
        # >= 10 Hz in the stamp clock
        ts = sorted(s + ns * 1e-9 for s, ns in {m[0] for m in vel})
        assert (len(ts) - 1) / (ts[-1] - ts[0]) >= 10.0
    finally:
        node.destroy_node()


def test_gnss_kept_for_aiding_by_default():
    node = make_node()
    try:
        synthetic_run(node, still_s=5.0, move_s=2.0)
        assert node.pipe.init_done and not node._gnss_done()
        assert len(node.fix_subs) == 2, 'GNSS aiding needs the fix subscriptions after the alignment'
    finally:
        node.destroy_node()


def test_gnss_released_and_pose_published():
    node = make_node(gnss_aiding=False)
    try:
        _, dist = synthetic_run(node)
        assert node.fix_subs == [], 'GNSS subscriptions not destroyed after the initial alignment'
        assert node.counts['master'] > 0
        pos = node.pub_pos.msgs
        assert pos, 'no /result/position published'
        st, frame, child, xyz, q, cov, v, var_v = pos[-1]
        p = node.params
        assert frame == node.pose_frame_id == p.frame_id_map and child == p.child_frame_id
        assert abs(math.hypot(q[2], q[3]) - 1.0) < 1e-9
        assert cov[0] > 0 and cov[7] > 0 and cov[14] > 0 and cov[35] > 0 and var_v >= 0
        assert abs(cov[1] - cov[6]) < 1e-12
        # position: close to the expected point on the map (wheels are exact here)
        x, y, z, _ = load_routes(MAPS)['T2S'].pose(S0 + dist + p.output_along_offset_m)
        err = math.hypot(xyz[0] - x, xyz[1] - y)
        assert err < 10.0, 'final position %.1f m from the expected point' % err
        assert abs(xyz[2] - (z + p.output_z_offset_m)) < 2.0
        # speed at the end ~ 8 m/s
        assert abs(node.pub_vel.msgs[-1][2] - 8.0) < 0.5
    finally:
        node.destroy_node()


def test_no_gnss_still_publishes_velocity():
    node = make_node()
    try:
        synthetic_run(node, with_gnss=False)
        assert node.pub_vel.msgs and node.counts['errors'] == 0
        assert node.fix_subs or node._gnss_done()
    finally:
        node.destroy_node()


def test_diagnostics_tick_does_not_fail():
    node = make_node()
    try:
        synthetic_run(node, still_s=1.0, move_s=1.0)
        published = []
        node.pub_diag = type('P', (), {'publish': lambda self, m: published.append(m)})()
        node._on_diag_timer()
        st = published[-1].status[0]
        keys = {kv.key for kv in st.values}
        assert {'mode', 'slip', 'init_done', 'cb_p50_ms', 'cb_max_ms', 'msgs_cmd'} <= keys
    finally:
        node.destroy_node()


def test_pipeline_exception_is_contained():
    """A failing pipeline call is counted and reported, and the node keeps serving the other inputs."""
    node = make_node()
    try:
        def broken(*_):
            raise RuntimeError('boom')
        base = 1785151800 * 10 ** 9
        m = VelocitySensor()                                # one good wheel sample starts the filter
        m.header.stamp = stamp(base - 10 ** 8)
        m.velocity = 10.0
        node._on_input('front', m)
        node.pub_vel.msgs.clear()
        node.pipe.on_wheel = broken
        for i in range(10):
            m = VelocitySensor()
            m.header.stamp = stamp(base + i * 10 ** 8)
            m.velocity = 10.0
            node._on_input('front', m)
            c = DriverControllerCommand()
            c.header.stamp = stamp(base + i * 10 ** 8 + 5 * 10 ** 7)
            c.position = 0
            node._on_input('cmd', c)
        assert node.counts['errors'] == 10
        assert len(node.pub_vel.msgs) == 10                 # cmd outputs still published
        published = []
        node.pub_diag = type('P', (), {'publish': lambda self, m: published.append(m)})()
        node._on_diag_timer()
        st = published[-1].status[0]
        assert st.level == DiagnosticStatus.ERROR and 'exception' in st.message
    finally:
        node.destroy_node()


def _cmd(node, t_ns, position=0):
    c = DriverControllerCommand()
    c.header.stamp = stamp(t_ns)
    c.position = position
    node._on_cmd(c)


def _diag_status(node):
    published = []
    node.pub_diag = type('P', (), {'publish': lambda self, m: published.append(m)})()
    node._on_diag_timer()
    assert published, 'no diagnostics published'
    return published[-1].status[0]


def test_nonfinite_outputs_are_never_published():
    """NaN speed -> nothing published; NaN pose -> velocity only; inf variance -> finite 'unknown' covariance."""
    from tram_backup_odometry.core.pipeline import Output
    node = make_node()
    try:
        nan, inf = float('nan'), float('inf')
        script = [Output(stamp=0.0, v=nan, x=1.0, y=2.0, z=3.0, yaw=0.1, has_pose=True),
                  Output(stamp=0.0, v=1.0, x=nan, y=2.0, z=3.0, yaw=0.1, has_pose=True),
                  Output(stamp=0.0, v=1.0, x=1.0, y=2.0, z=3.0, yaw=0.0, has_pose=True,
                         var_along=inf, var_v=nan)]

        def on_cmd(t, _notch):
            out = script.pop(0)
            out.stamp = t
            return out
        node.pipe.on_cmd = on_cmd
        base = T0_SEC * 10 ** 9
        for i in range(3):
            _cmd(node, base + i * 50000000)
        vel, pos = node.pub_vel.msgs, node.pub_pos.msgs
        assert [m[2] for m in vel] == [1.0, 1.0]
        assert len(pos) == 1 and pos[0][3] == (1.0, 2.0, 3.0)
        cov, var_v = pos[0][5], pos[0][7]
        assert all(math.isfinite(c) for c in cov) and math.isfinite(var_v)
        assert cov[0] >= 1e5 and var_v >= 1e5                 # 'unknown', not NaN
        assert node.counts['nonfinite'] == 2 and node.counts['errors'] == 0
        st = _diag_status(node)
        assert st.level == DiagnosticStatus.WARN and 'non-finite' in st.message
    finally:
        node.destroy_node()


def test_diagnostics_timer_never_raises():
    """An exception in a timer callback would stop rclpy.spin: odd pipeline values must not raise."""
    from tram_backup_odometry.core.pipeline import Output
    node = make_node()
    try:
        node.pipe.on_cmd = lambda t, n: Output(stamp=t, v=1.0, has_pose=True, var_along=None,
                                               mode=None, diag={'a': None, 1: 2.5, 'n': float('nan')})
        _cmd(node, T0_SEC * 10 ** 9)
        st = _diag_status(node)
        kv = {v.key: v.value for v in st.values}
        assert kv['mode'] == 'None' and kv['pipe_1'] == '2.5'
        node.last_out.diag = None                              # still no exception
        st = _diag_status(node)
        node.last_out.v = None                                 # formatting fails -> contained, counted
        n_err = node.counts['errors']
        node._on_diag_timer()
        assert node.counts['errors'] == n_err + 1
    finally:
        node.destroy_node()


def test_output_stamped_with_an_earlier_input_reuses_its_exact_stamp():
    from tram_backup_odometry.core.pipeline import Output
    node = make_node()
    try:
        first = []

        def on_cmd(t, _notch):
            first.append(t)
            return Output(stamp=first[0], v=1.0)
        node.pipe.on_cmd = on_cmd
        base = T0_SEC * 10 ** 9 + 123456789
        for i in range(5):
            _cmd(node, base + i * 50000000)
        assert {m[0] for m in node.pub_vel.msgs} == {(T0_SEC, 123456789)}
        assert node.counts['stamp_fallback'] == 0
    finally:
        node.destroy_node()


def test_dds_end_to_end():
    """Real topics: best-effort node subscriptions receive a reliable publisher; stamps survive."""
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    node = BackupOdometryNode(parameter_overrides=[Parameter('maps_dir', value=MAPS)])
    helper = Node('test_helper')
    got = []
    helper.create_subscription(VelocitySensor, '/result/velocity',
                               lambda m: got.append((m.header.stamp.sec, m.header.stamp.nanosec)), 50)
    pubs = {'cmd': helper.create_publisher(DriverControllerCommand, '/vehicle/driver_position_cmd', 50),
            'front': helper.create_publisher(VelocitySensor, '/vehicle/front_bogie_velocity', 50),
            'rear': helper.create_publisher(VelocitySensor, '/vehicle/rear_bogie_velocity', 50)}
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(helper)
    try:
        t_end = time.monotonic() + 3.0
        while (any(p.get_subscription_count() == 0 for p in pubs.values())
               and time.monotonic() < t_end):
            ex.spin_once(timeout_sec=0.05)
        sent = set()
        for i in range(40):                      # 2 s: cmd at 20 Hz, wheels at 10 Hz
            t_ns = T0_SEC * 1000000000 + 987654321 + i * 50000000
            m = DriverControllerCommand()
            m.header.stamp = stamp(t_ns)
            pubs['cmd'].publish(m)
            sent.add((m.header.stamp.sec, m.header.stamp.nanosec))
            if i % 2 == 0:
                w = VelocitySensor()
                w.header.stamp = stamp(t_ns + 3000000)
                w.velocity = 0.0
                pubs['front'].publish(w)
                pubs['rear'].publish(w)
                sent.add((w.header.stamp.sec, w.header.stamp.nanosec))
            ex.spin_once(timeout_sec=0.01)
        t_end = time.monotonic() + 3.0
        while len(got) < 20 and time.monotonic() < t_end:
            ex.spin_once(timeout_sec=0.05)
        assert len(got) >= 20, 'only %d outputs received' % len(got)
        assert set(got) <= sent
    finally:
        ex.shutdown()
        node.destroy_node()
        helper.destroy_node()
