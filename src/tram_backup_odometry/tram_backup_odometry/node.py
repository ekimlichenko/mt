"""ROS 2 node: backup odometry of a tram from bogie wheel speeds and the driver controller notch.

The node is a thin transport layer around ``core.pipeline.Pipeline`` (the very
same object the offline replay drives), so the estimate published here is
bit-for-bit what ``replay.py`` computes for the same message order.

Topics
------
in   /vehicle/front_bogie_velocity, /vehicle/rear_bogie_velocity  tram_vehicle_msgs/VelocitySensor (km/h)
     /vehicle/driver_position_cmd                                  tram_vehicle_msgs/DriverControllerCommand
     /sensing/gnss/{master,rover}/fix                              sensor_msgs/NavSatFix
         GNSS gives the initial alignment and (gnss_aiding, default on) later
         along-track position corrections; the speed never uses it.  When the
         pipeline reports ``gnss_done`` (aiding off) both subscriptions are
         destroyed, so GNSS cannot reach the main loop even by accident.
out  /result/velocity   tram_vehicle_msgs/VelocitySensor  velocity [m/s]
     /result/position   nav_msgs/Odometry                 pose + twist, only once a pose exists
     /backup_odometry/diagnostics  diagnostic_msgs/DiagnosticArray, 1 Hz (steady clock)
     /backup_odometry/slip         std_msgs/Bool, on change (transient local)

Inputs are subscribed best-effort (depth 50), which matches both best-effort and
reliable publishers (``ros2 bag play`` replays the recorded reliable QoS).

Time
----
Estimation runs entirely in the vehicle header clock.  Each output carries the
header stamp of the input message that triggered it, copied bit-exactly: the
original ``builtin_interfaces/Time`` object is kept (a float round trip of an
epoch stamp ~1.8e9 s has only ~0.24 us resolution, which would break an exact
stamp match).  The node never reads its own clock in the estimation path, so
``use_sim_time`` true/false does not change the result; the diagnostics timer
runs on the steady clock so it also ticks when no /clock is published.

Covariances (Odometry)
----------------------
The pipeline reports the along-track variance ``var_along`` (the tram is on the
rails, the heading comes from the map).  In the output frame

    C_xy = R(yaw) diag(var_along, var_cross) R(yaw)^T,

var_cross and the yaw variance come from the pipeline (``pose_sigma_cross_m`` /
``pose_sigma_yaw_rad`` on the map, growing with the distance off the map);
z and roll/pitch get constant variances (``pose_sigma_*``), twist.linear.x
gets ``var_v``, lateral/vertical velocity a small constant (rail-bound) and the
unobserved angular rates 1e6 ("unknown").  A non-finite variance from the
pipeline is published as 1e6 as well, a negative one (round-off) as 0.

Robustness
----------
A NaN/inf never reaches the result topics: an output whose speed is not finite
is dropped entirely, one whose pose is not finite is published as velocity
only (both counted as ``nonfinite`` and flagged in the diagnostics).  Pipeline
exceptions are contained per callback, and the diagnostics timer never lets an
exception escape into the executor (which would stop the node).
"""
import math
import os
import signal
import sys
import time
import traceback
from collections import OrderedDict
from dataclasses import fields

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from .core.config import Params
from .core.frames import yaw_to_quat
from .core.pipeline import Pipeline

NODE_NAME = 'backup_odometry'
TOPIC_FRONT = '/vehicle/front_bogie_velocity'
TOPIC_REAR = '/vehicle/rear_bogie_velocity'
TOPIC_CMD = '/vehicle/driver_position_cmd'
TOPIC_FIX = {'master': '/sensing/gnss/master/fix', 'rover': '/sensing/gnss/rover/fix'}
TOPIC_VEL_OUT = '/result/velocity'
TOPIC_POS_OUT = '/result/position'
TOPIC_DIAG = '/backup_odometry/diagnostics'
TOPIC_SLIP = '/backup_odometry/slip'

UNKNOWN_VAR = 1e6          # "not estimated" variance (angular rates)
RAIL_VEL_VAR = 1e-4        # lateral / vertical velocity on rails, (m/s)^2
STAMP_CACHE_SIZE = 512     # recent input stamps kept for exact output stamping
ERROR_LOG_PERIOD_S = 5.0   # at most one pipeline traceback in the log per period


def default_maps_dir():
    """<share>/tram_backup_odometry/maps when installed, else <source package>/maps."""
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('tram_backup_odometry'), 'maps')
        if os.path.isdir(cand):
            return cand
    except Exception:  # not installed through ament (source tree / tests)
        pass
    cand = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'maps'))
    return cand if os.path.isdir(cand) else ''


def _var(x):
    """Variance for a covariance matrix: negative -> 0 (round-off), non-finite / None -> UNKNOWN_VAR."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return UNKNOWN_VAR
    return max(x, 0.0) if math.isfinite(x) else UNKNOWN_VAR


def float_to_time(t):
    """Float seconds -> builtin_interfaces/Time (fallback only; not bit-exact)."""
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1000000000:
        sec, nsec = sec + 1, nsec - 1000000000
    return Time(sec=sec, nanosec=nsec)


class CallbackTimes:
    """Callback wall time: exact percentiles over the diagnostics window and a
    log-spaced histogram (40 bins per decade, 1 us .. 100 s) over the whole run."""
    BPD = 40
    NBINS = 8 * BPD

    def __init__(self):
        self.window = []
        self.hist = [0] * self.NBINS
        self.count = 0
        self.total = 0.0
        self.max = 0.0

    def add(self, dt):
        self.window.append(dt)
        self.count += 1
        self.total += dt
        if dt > self.max:
            self.max = dt
        us = dt * 1e6
        b = int(math.log10(us) * self.BPD) if us > 1.0 else 0
        self.hist[min(b, self.NBINS - 1)] += 1

    def take_window(self):
        w, self.window = self.window, []
        return w

    def percentile(self, q):
        """Upper bin edge (s) of the q-quantile over the whole run (<= 6% high)."""
        if self.count == 0:
            return float('nan')
        target = q * self.count
        acc = 0
        for b, n in enumerate(self.hist):
            acc += n
            if acc >= target:
                return 10.0 ** ((b + 1) / self.BPD) * 1e-6
        return self.max


def _quantile(sorted_vals, q):
    if not sorted_vals:
        return float('nan')
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]


class BackupOdometryNode(Node):
    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        self.params, self.maps_dir = self._declare_parameters()
        p = self.params

        t0 = time.perf_counter()
        self.pipe = Pipeline(p, self.maps_dir)
        t_load = time.perf_counter() - t0
        frame = getattr(self.pipe, 'frame', None)
        self.pose_frame_id = getattr(frame, 'frame_id', p.frame_id_map)

        self.sig_cross2 = getattr(p, 'pose_sigma_cross_m', 0.3) ** 2
        self.sig_z2 = getattr(p, 'pose_sigma_z_m', 0.5) ** 2
        self.sig_tilt2 = getattr(p, 'pose_sigma_tilt_rad', 0.02) ** 2
        self.sig_yaw2 = getattr(p, 'pose_sigma_yaw_rad', 0.03) ** 2

        # --- publishers (reliable is compatible with best-effort and reliable subscribers)
        out_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_vel = self.create_publisher(VelocitySensor, TOPIC_VEL_OUT, out_qos)
        self.pub_pos = self.create_publisher(Odometry, TOPIC_POS_OUT, out_qos)
        self.pub_diag = self.create_publisher(DiagnosticArray, TOPIC_DIAG, 10)
        self.pub_slip = self.create_publisher(Bool, TOPIC_SLIP, latched)

        # reused message objects (publish() serialises synchronously)
        self.msg_vel = VelocitySensor()
        self.msg_vel.header.frame_id = p.velocity_frame_id
        self.msg_odom = Odometry()
        self.msg_odom.header.frame_id = self.pose_frame_id
        self.msg_odom.child_frame_id = p.child_frame_id
        tw = self.msg_odom.twist.covariance
        tw[7] = tw[14] = RAIL_VEL_VAR
        tw[21] = tw[28] = tw[35] = UNKNOWN_VAR

        # --- subscriptions
        depth = int(getattr(p, 'input_qos_depth', 50))
        in_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VelocitySensor, TOPIC_FRONT, self._on_front, in_qos)
        self.create_subscription(VelocitySensor, TOPIC_REAR, self._on_rear, in_qos)
        self.create_subscription(DriverControllerCommand, TOPIC_CMD, self._on_cmd, in_qos)
        self.fix_subs = [
            self.create_subscription(NavSatFix, TOPIC_FIX['master'], self._on_fix_master, in_qos),
            self.create_subscription(NavSatFix, TOPIC_FIX['rover'], self._on_fix_rover, in_qos),
        ]

        # --- bookkeeping
        self.stamps = OrderedDict()          # float stamp -> original Time
        self.counts = {k: 0 for k in ('front', 'rear', 'cmd', 'master', 'rover',
                                      'out_vel', 'out_pos', 'errors', 'stamp_fallback', 'nonfinite')}
        self.cb = CallbackTimes()
        self.last_out = None
        self.last_stamp = None
        self.last_input_wall = None
        self.slip_state = None
        self.first_pose_logged = False
        self.window_errors = 0
        self.window_nonfinite = 0
        self.last_error_log = -math.inf
        self.t_start_wall = time.monotonic()
        self.gnss_released_after = None

        period = float(getattr(p, 'diag_period_s', 1.0))
        self.diag_timer = self.create_timer(period, self._on_diag_timer,
                                            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'ready: vehicle_id=%r k=%.4f output_frame=%s (frame_id=%s) publish_on=%s maps=%s '
            '(pipeline init %.2f s); waiting for inputs'
            % (p.vehicle_id, p.k_for_vehicle(), p.output_frame, self.pose_frame_id,
               p.publish_on, self.maps_dir, t_load))

    # ------------------------------------------------------------ parameters
    def _declare_parameters(self):
        """Every Params field becomes a read-only ROS parameter with the dataclass default.

        Dynamic typing lets e.g. ``vehicle_id:=30618`` (parsed as an int) or
        ``acc_max:=2`` through; Params.from_dict coerces to the dataclass type.
        """
        defaults = Params()
        values = {}
        for f in fields(Params):
            desc = ParameterDescriptor(name=f.name, read_only=True, dynamic_typing=True)
            param = self.declare_parameter(f.name, getattr(defaults, f.name), desc)
            if param.value is not None:
                values[f.name] = param.value
        desc = ParameterDescriptor(name='maps_dir', read_only=True,
                                   description='directory with t2s.json / s2t.json')
        maps_dir = self.declare_parameter('maps_dir', default_maps_dir(), desc).value or default_maps_dir()
        return Params.from_dict(values), maps_dir

    # ------------------------------------------------------------ callbacks
    def _on_front(self, msg):
        self._on_input('front', msg)

    def _on_rear(self, msg):
        self._on_input('rear', msg)

    def _on_cmd(self, msg):
        self._on_input('cmd', msg)

    def _on_fix_master(self, msg):
        self._on_fix('master', msg)

    def _on_fix_rover(self, msg):
        self._on_fix('rover', msg)

    def _remember_stamp(self, t, stamp):
        cache = self.stamps
        cache[t] = stamp
        if len(cache) > STAMP_CACHE_SIZE:
            cache.popitem(last=False)

    def _on_input(self, kind, msg):
        t0 = time.perf_counter()
        self.counts[kind] += 1
        self.last_input_wall = time.monotonic()
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        self._remember_stamp(t, stamp)
        try:
            if kind == 'cmd':
                out = self.pipe.on_cmd(t, int(msg.position))
            else:
                out = self.pipe.on_wheel(kind, t, float(msg.velocity))
            if out is not None:
                self._publish(out, stamp if out.stamp == t else None)
        except Exception:
            self._report_error(kind)
        if self.fix_subs and self._gnss_done():
            self._release_gnss()
        self.cb.add(time.perf_counter() - t0)

    def _on_fix(self, antenna, msg):
        self.counts[antenna] += 1
        if self._gnss_done():
            return  # subscriptions are destroyed on the next wheel/cmd callback
        t0 = time.perf_counter()
        stamp = msg.header.stamp
        try:
            self.pipe.on_fix(antenna, stamp.sec + stamp.nanosec * 1e-9, msg.latitude,
                             msg.longitude, msg.altitude, int(msg.status.status))
        except Exception:
            self._report_error(antenna)
        self.cb.add(time.perf_counter() - t0)

    def _report_error(self, kind):
        self.counts['errors'] += 1
        self.window_errors += 1
        now = time.monotonic()
        if now - self.last_error_log >= ERROR_LOG_PERIOD_S:   # format the traceback only when it is logged
            self.last_error_log = now
            self.get_logger().error('exception in the %s callback (%d so far):\n%s'
                                    % (kind, self.counts['errors'], traceback.format_exc()))

    # ------------------------------------------------------------ GNSS release
    def _gnss_done(self):
        """Pipeline no longer needs GNSS (attribute, property or method).

        ``gnss_done`` wins when present: with GNSS aiding on it stays False after
        the initial alignment.  ``init_done`` is the fallback for older pipelines."""
        for name in ('gnss_done', 'init_done'):
            if not hasattr(self.pipe, name):
                continue
            flag = getattr(self.pipe, name)
            if callable(flag):
                flag = flag()
            return bool(flag)
        return False

    def _release_gnss(self):
        for sub in self.fix_subs:
            self.destroy_subscription(sub)
        self.fix_subs = []
        self.gnss_released_after = time.monotonic() - self.t_start_wall
        self.get_logger().info(
            'initial alignment done (%d master / %d rover fixes); GNSS subscriptions destroyed, '
            'running on wheels + controller only' % (self.counts['master'], self.counts['rover']))

    # ------------------------------------------------------------ outputs
    def _publish(self, out, stamp):
        if stamp is None:
            stamp = self.stamps.get(out.stamp)
            if stamp is None:
                stamp = float_to_time(out.stamp)
                self.counts['stamp_fallback'] += 1
        v = float(out.v)
        if not math.isfinite(v):     # never publish NaN/inf: drop the whole output
            self._count_nonfinite()
            return
        mv = self.msg_vel
        mv.header.stamp = stamp
        mv.velocity = v
        self.pub_vel.publish(mv)
        self.counts['out_vel'] += 1

        if out.has_pose:
            x, y, z, yaw = float(out.x), float(out.y), float(out.z), float(out.yaw)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and math.isfinite(yaw)):
                self._count_nonfinite()
            else:
                self._publish_pose(out, stamp, v, x, y, z, yaw)

        slip = bool(out.slip)
        if slip != self.slip_state:
            self.slip_state = slip
            self.pub_slip.publish(Bool(data=slip))
        self.last_out = out
        self.last_stamp = stamp

    def _publish_pose(self, out, stamp, v, x, y, z, yaw):
        m = self.msg_odom
        m.header.stamp = stamp
        m.header.frame_id = getattr(out, 'frame_id', '') or self.pose_frame_id
        pose = m.pose.pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        q = pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quat(yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        va = _var(out.var_along)
        vc = getattr(out, 'var_cross', None)
        vc = self.sig_cross2 if vc is None or not math.isfinite(vc) else _var(vc)
        vy = getattr(out, 'var_yaw', None)
        vy = self.sig_yaw2 if vy is None or not math.isfinite(vy) else _var(vy)
        cov = m.pose.covariance
        cov[0] = va * c * c + vc * s * s
        cov[7] = va * s * s + vc * c * c
        cov[1] = cov[6] = (va - vc) * c * s
        cov[14] = self.sig_z2
        cov[21] = cov[28] = self.sig_tilt2
        cov[35] = vy
        m.twist.twist.linear.x = v
        m.twist.covariance[0] = _var(out.var_v)
        self.pub_pos.publish(m)
        self.counts['out_pos'] += 1
        if not self.first_pose_logged:
            self.first_pose_logged = True
            self.get_logger().info('first pose: %s (%.2f, %.2f, %.2f) yaw %.3f, mode %s'
                                   % (m.header.frame_id, x, y, z, yaw, out.mode))

    def _count_nonfinite(self):
        self.counts['nonfinite'] += 1
        self.window_nonfinite += 1

    # ------------------------------------------------------------ diagnostics
    def _on_diag_timer(self):
        """Timer callback: an exception here would stop rclpy.spin, so it is contained."""
        try:
            self._publish_diagnostics()
        except Exception:
            self._report_error('diagnostics')

    def _publish_diagnostics(self):
        if self.fix_subs and self._gnss_done():
            self._release_gnss()
        window = sorted(self.cb.take_window())
        out = self.last_out
        now = time.monotonic()
        idle = (now - self.last_input_wall) if self.last_input_wall is not None else float('inf')
        init_done = self._gnss_done()
        has_pose = bool(out is not None and out.has_pose)

        level, msgs = DiagnosticStatus.OK, []
        if self.window_errors:
            level = DiagnosticStatus.ERROR
            msgs.append('%d callback exceptions' % self.window_errors)
        if self.last_input_wall is None:
            level = max(level, DiagnosticStatus.WARN)
            msgs.append('no input yet')
        elif idle > 1.0:
            level = max(level, DiagnosticStatus.WARN)
            msgs.append('no input for %.1f s' % idle)
        if self.last_input_wall is not None and not has_pose:
            level = max(level, DiagnosticStatus.WARN)
            msgs.append('no pose yet (initial alignment %s)' % ('done' if init_done else 'pending'))
        if out is not None and out.slip:
            level = max(level, DiagnosticStatus.WARN)
            msgs.append('wheel slip/slide suspected')
        if self.window_nonfinite:
            level = max(level, DiagnosticStatus.WARN)
            msgs.append('%d non-finite outputs dropped' % self.window_nonfinite)
        self.window_errors = 0
        self.window_nonfinite = 0
        frame_id = (getattr(out, 'frame_id', '') if out is not None else '') or self.pose_frame_id

        ms = 1e3
        kv = [
            ('mode', str(out.mode) if out is not None else 'none'),
            ('slip', str(bool(out.slip)) if out is not None else 'n/a'),
            ('init_done', str(init_done)),
            ('gnss_subscribed', str(bool(self.fix_subs))),
            ('has_pose', str(has_pose)),
            ('velocity_mps', '%.3f' % out.v if out is not None else 'n/a'),
            ('sigma_along_m', '%.2f' % math.sqrt(_var(out.var_along)) if has_pose else 'n/a'),
            ('output_frame', '%s (%s)' % (self.params.output_frame, frame_id)),
            ('vehicle_id', self.params.vehicle_id or 'unknown'),
            ('cb_count_window', str(len(window))),
            ('cb_p50_ms', '%.3f' % (_quantile(window, 0.5) * ms)),
            ('cb_p95_ms', '%.3f' % (_quantile(window, 0.95) * ms)),
            ('cb_max_ms', '%.3f' % (window[-1] * ms if window else float('nan'))),
            ('cb_max_ms_total', '%.3f' % (self.cb.max * ms)),
            ('input_idle_s', '%.2f' % idle),
        ]
        kv += [('msgs_' + k, str(n)) for k, n in self.counts.items()]
        if out is not None and isinstance(getattr(out, 'diag', None), dict):
            for k, v in out.diag.items():
                if isinstance(v, float):
                    v = '%.6g' % v
                kv.append(('pipe_' + str(k), str(v)))

        st = DiagnosticStatus(level=level, name='%s: estimator' % NODE_NAME,
                              hardware_id='tram_%s' % (self.params.vehicle_id or 'unknown'),
                              message='; '.join(msgs) or 'ok',
                              values=[KeyValue(key=k, value=v) for k, v in kv])
        arr = DiagnosticArray()
        arr.header.stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def summary(self):
        c, ms = self.cb, 1e3
        mean = c.total / c.count * ms if c.count else float('nan')
        return ('inputs front=%d rear=%d cmd=%d fix master=%d rover=%d | outputs velocity=%d position=%d | '
                'callback mean %.3f ms p50 %.3f p95 %.3f p99 %.3f max %.3f ms | errors %d, stamp fallbacks %d, '
                'non-finite outputs dropped %d'
                % (self.counts['front'], self.counts['rear'], self.counts['cmd'], self.counts['master'],
                   self.counts['rover'], self.counts['out_vel'], self.counts['out_pos'], mean,
                   c.percentile(0.5) * ms, c.percentile(0.95) * ms, c.percentile(0.99) * ms, c.max * ms,
                   self.counts['errors'], self.counts['stamp_fallback'], self.counts['nonfinite']))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BackupOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C in a terminal reaches both `ros2 launch` and the node; a second SIGINT
        # must not interrupt the cleanup below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            text = 'shutdown: ' + node.summary()
            if rclpy.ok():
                node.get_logger().info(text)
            else:  # context already shut down by the signal handler: no /rosout any more
                print('[%s] %s' % (NODE_NAME, text), file=sys.stderr, flush=True)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
