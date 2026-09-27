#!/usr/bin/env python3
"""Input -> output latency and output rate of the backup odometry node (wall clock).

One process subscribes to the node's inputs (front/rear bogie velocity, driver
cmd) and outputs (/result/velocity, /result/position), best-effort, depth 200.
Every output carries the header stamp of the input that triggered it, so

    latency = arrival(output) - arrival(input with the same header stamp).

Front and rear bogie messages usually share a header stamp but arrive up to
~30 ms apart, so the triggering input is ambiguous.  ``latency`` uses the
latest input with that stamp that arrived before the output (the node
processes messages in order, so this is the trigger); ``latency_first`` uses
the first arrival and is a conservative upper bound.  Both arrivals are
measured by the same executor, so the probe's own delivery overhead largely
cancels; the figure covers DDS transport to the node, the node's callback and
the transport back.  Also reported:

* steady-state latency: the same without the outputs of the first --warmup
  seconds (default 2 s).  Every recording starts with a backlog burst of
  ~150 stale messages recorded within a fraction of a second; ``ros2 bag play``
  replays it as a burst, so the first second measures queueing, not latency;
* matched fraction: outputs whose stamp equals an input stamp (should be 100%);
* output rate in the wall clock and in the header-stamp clock;
* 0.1 s grid coverage: the share of points k*0.1 s (the GNSS reference grid)
  within the stamp span that have an output within +-0.05 s (the judge's
  matching tolerance).

    python3 tools/latency_probe.py [--duration S] [--idle-exit S] [--json out.json] [--csv raw.csv]

Stops after --duration, after --idle-exit seconds without messages (once
something was received) or on SIGINT, then prints the summary.
"""
import argparse
import bisect
import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

INPUTS = (('/vehicle/front_bogie_velocity', VelocitySensor),
          ('/vehicle/rear_bogie_velocity', VelocitySensor),
          ('/vehicle/driver_position_cmd', DriverControllerCommand))
OUTPUTS = (('velocity', '/result/velocity', VelocitySensor),
           ('position', '/result/position', Odometry))
KEEP_S = 30.0  # forget input stamps older than this (wall)


def pct(sorted_vals, q):
    if not sorted_vals:
        return float('nan')
    i = q * (len(sorted_vals) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (i - lo)


def grid_coverage(stamps, step=0.1, tol=0.05):
    """Share of grid points k*step in [min, max] with a stamp within tol."""
    if len(stamps) < 2:
        return float('nan')
    s = sorted(stamps)
    k0, k1 = int(math.ceil(s[0] / step)), int(math.floor(s[-1] / step))
    hit = 0
    for k in range(k0, k1 + 1):
        g = k * step
        i = bisect.bisect_left(s, g)
        d = min(abs(s[j] - g) for j in (i - 1, i) if 0 <= j < len(s))
        hit += d <= tol + 1e-9
    return hit / max(1, k1 - k0 + 1)


class LatencyProbe(Node):
    def __init__(self):
        super().__init__('latency_probe')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.arrival = {}                     # (sec, nanosec) -> [first, last] wall arrival
        self.n_in = 0
        self.lat = {k: [] for k, _, _ in OUTPUTS}
        self.lat_first = {k: [] for k, _, _ in OUTPUTS}
        self.dup = {k: 0 for k, _, _ in OUTPUTS}
        self.seen = {k: set() for k, _, _ in OUTPUTS}
        self.unmatched = {k: 0 for k, _, _ in OUTPUTS}
        self.stamps = {k: [] for k, _, _ in OUTPUTS}
        self.wall = {k: [] for k, _, _ in OUTPUTS}
        self.frame_ids = {k: set() for k, _, _ in OUTPUTS}
        self.last_msg = None
        self.raw = []                         # (wall s, output, stamp s, latency s, trigger topic)
        self.t0 = time.perf_counter()
        for topic, typ in INPUTS:
            self.create_subscription(typ, topic, lambda m, t=topic: self._on_input(m, t), qos)
        for key, topic, typ in OUTPUTS:
            self.create_subscription(typ, topic, lambda m, k=key: self._on_output(k, m), qos)

    def _on_input(self, msg, topic):
        now = time.perf_counter()
        self.last_msg = now
        self.n_in += 1
        st = msg.header.stamp
        a = self.arrival.get((st.sec, st.nanosec))
        if a is None:
            self.arrival[(st.sec, st.nanosec)] = [now, now, topic]
        else:
            a[1] = now
            a[2] = topic
        if self.n_in % 2000 == 0:
            old = now - KEEP_S
            self.arrival = {k: v for k, v in self.arrival.items() if v[1] >= old}

    def _on_output(self, key, msg):
        now = time.perf_counter()
        self.last_msg = now
        st = msg.header.stamp
        k = (st.sec, st.nanosec)
        a = self.arrival.get(k)
        if a is None:
            self.unmatched[key] += 1
        else:
            self.lat[key].append(now - a[1])
            self.lat_first[key].append(now - a[0])
            self.raw.append((now - self.t0, key, st.sec + st.nanosec * 1e-9, now - a[1], a[2]))
        if k in self.seen[key]:
            self.dup[key] += 1
        self.seen[key].add(k)
        if len(self.seen[key]) > 100000:
            self.seen[key].clear()
        self.stamps[key].append(st.sec + st.nanosec * 1e-9)
        self.wall[key].append(now)
        self.frame_ids[key].add(msg.header.frame_id)

    def summary(self, warmup_s=2.0):
        res = {'inputs': self.n_in, 'warmup_s': warmup_s}
        for key, _, _ in OUTPUTS:
            raw = [r for r in self.raw if r[1] == key]
            steady = sorted(r[3] for r in raw if r[0] >= raw[0][0] + warmup_s) if raw else []
            lat = sorted(self.lat[key])
            lat1 = sorted(self.lat_first[key])
            n = len(self.stamps[key])
            wall, st = self.wall[key], self.stamps[key]
            res[key] = {
                'n': n,
                'matched': len(lat),
                'unmatched': self.unmatched[key],
                'duplicate_stamps': self.dup[key],
                'latency_ms': {'p50': pct(lat, 0.5) * 1e3, 'p95': pct(lat, 0.95) * 1e3,
                               'p99': pct(lat, 0.99) * 1e3, 'max': (lat[-1] * 1e3 if lat else float('nan')),
                               'mean': (sum(lat) / len(lat) * 1e3 if lat else float('nan'))},
                'latency_steady_ms': {'n': len(steady), 'p50': pct(steady, 0.5) * 1e3,
                                      'p95': pct(steady, 0.95) * 1e3, 'p99': pct(steady, 0.99) * 1e3,
                                      'max': (steady[-1] * 1e3 if steady else float('nan'))},
                'latency_first_ms': {'p50': pct(lat1, 0.5) * 1e3, 'p95': pct(lat1, 0.95) * 1e3,
                                     'max': (lat1[-1] * 1e3 if lat1 else float('nan'))},
                'rate_wall_hz': (n - 1) / (wall[-1] - wall[0]) if n > 1 and wall[-1] > wall[0] else float('nan'),
                'rate_stamp_hz': (n - 1) / (max(st) - min(st)) if n > 1 and max(st) > min(st) else float('nan'),
                'grid_0p1_coverage': grid_coverage(st),
                'frame_ids': sorted(self.frame_ids[key]),
            }
        return res


def format_summary(res):
    lines = ['latency_probe: %d input messages' % res['inputs']]
    for key, _, _ in OUTPUTS:
        r = res[key]
        L, S, F = r['latency_ms'], r['latency_steady_ms'], r['latency_first_ms']
        lines.append(
            '  /result/%-8s n=%d matched=%d unmatched=%d duplicate-stamps=%d | rate wall %.2f Hz, stamp %.2f Hz '
            '| 0.1s-grid coverage %.2f%% | frame_id %s\n'
            '      latency ms, all:          p50 %.2f p95 %.2f p99 %.2f max %.2f\n'
            '      latency ms, after %.0f s:   p50 %.2f p95 %.2f p99 %.2f max %.2f (n=%d)\n'
            '      vs first input w/ stamp:  p50 %.2f p95 %.2f max %.2f'
            % (key, r['n'], r['matched'], r['unmatched'], r['duplicate_stamps'], r['rate_wall_hz'],
               r['rate_stamp_hz'], 100 * r['grid_0p1_coverage'], ','.join(r['frame_ids']),
               L['p50'], L['p95'], L['p99'], L['max'], res['warmup_s'], S['p50'], S['p95'], S['p99'], S['max'],
               S['n'], F['p50'], F['p95'], F['max']))
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=0.0, help='stop after S wall seconds (0 = no limit)')
    ap.add_argument('--idle-exit', type=float, default=0.0, help='stop after S s without messages (0 = never)')
    ap.add_argument('--json', default=None, help='write the summary as JSON')
    ap.add_argument('--warmup', type=float, default=2.0, help='seconds excluded from the steady-state figures')
    ap.add_argument('--csv', default=None, help='write matched outputs (wall, output, stamp, latency, trigger)')
    a, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = LatencyProbe()
    t0 = time.perf_counter()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.perf_counter()
            if a.duration and now - t0 > a.duration:
                break
            if a.idle_exit and node.last_msg is not None and now - node.last_msg > a.idle_exit:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    res = node.summary(a.warmup)
    print(format_summary(res), flush=True)
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(res, f, indent=1)
    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('wall_s,output,stamp_s,latency_ms,trigger\n')
            f.writelines('%.4f,%s,%.6f,%.3f,%s\n' % (w, k, st, lat * 1e3, trig) for w, k, st, lat, trig in node.raw)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
