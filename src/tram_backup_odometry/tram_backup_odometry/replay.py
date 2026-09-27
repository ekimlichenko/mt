"""Offline replay: feed a recorded bag through the same Pipeline as the ROS node.

Messages are delivered in bag record order (what `ros2 bag play` does), with the
original header stamps.  GNSS fixes are offered to the pipeline exactly like
the node does (the pipeline ignores them after initial alignment).

    python -m tram_backup_odometry.replay <bag_dir|cache.pkl> [--out out.npz] [--set key=value ...]
"""
import argparse
import copy
import os
import pickle
import sys
import time

import numpy as np

from .core.config import Params
from .core.pipeline import Pipeline

TOPIC_FRONT = '/vehicle/front_bogie_velocity'
TOPIC_REAR = '/vehicle/rear_bogie_velocity'
TOPIC_CMD = '/vehicle/driver_position_cmd'
TOPIC_FIX = {'master': '/sensing/gnss/master/fix', 'rover': '/sensing/gnss/rover/fix'}

MODE_CODES = {}


def default_maps_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, '..', 'maps'), os.path.join(here, 'maps')):
        if os.path.isdir(cand):
            return os.path.abspath(cand)
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('tram_backup_odometry'), 'maps')
    except Exception:
        raise FileNotFoundError('maps directory not found')


def load_data(bag):
    """Bag directory / .db3 -> parsed dict; .pkl -> cached parsed dict."""
    if bag.endswith('.pkl'):
        with open(bag, 'rb') as f:
            return pickle.load(f)
    from .bagio import load_bag
    return load_bag(bag)


def merged_events(D, use_gnss=True):
    """List of (t_rec, kind, payload) sorted by record time (stable)."""
    ev = []

    def add(topic, kind, fields):
        d = D.get(topic)
        if not d or len(d.get('t_rec', [])) == 0:
            return
        cols = [d['t_rec'], d['t_hdr']] + [d[f] for f in fields]
        for row in zip(*cols):
            ev.append((float(row[0]), kind, row[1:]))

    add(TOPIC_FRONT, 'front', ['v'])
    add(TOPIC_REAR, 'rear', ['v'])
    add(TOPIC_CMD, 'cmd', ['pos'])
    if use_gnss:
        for ant, topic in TOPIC_FIX.items():
            add(topic, ant, ['lat', 'lon', 'alt', 'status'])
    order = {'front': 0, 'rear': 1, 'cmd': 2, 'master': 3, 'rover': 4}
    ev.sort(key=lambda e: (e[0], order[e[1]]))
    return ev


def run_bag(bag, params=None, maps_dir=None, data=None, use_gnss=True, gnss_limit_s=None, gnss_burst=None):
    """Replay one bag; returns dict of numpy arrays (one row per published output).

    gnss_limit_s: feed GNSS only this long after the first fix; gnss_burst=(period_s, dur_s)
    additionally feeds a dur_s burst every period_s after that (the organisers' test bag
    has ~1.5 s bursts every 100-170 s)."""
    p = params or Params()
    if not p.vehicle_id:
        base = os.path.basename(os.path.normpath(bag)) if isinstance(bag, str) else ''
        if base[:5] in ('30618', '30639'):
            p = copy.copy(p)          # keeps attributes set with setattr
            p.vehicle_id = base[:5]
    D = data if data is not None else load_data(bag)
    pipe = Pipeline(p, maps_dir or default_maps_dir())
    rows = []
    t_first = None
    cpu0 = time.process_time()
    for t_rec, kind, pl in merged_events(D, use_gnss):
        if kind in ('front', 'rear'):
            out = pipe.on_wheel(kind, float(pl[0]), float(pl[1]))
        elif kind == 'cmd':
            out = pipe.on_cmd(float(pl[0]), int(pl[1]))
        else:
            if t_first is None:
                t_first = float(pl[0])
            if gnss_limit_s is not None and float(pl[0]) > t_first + gnss_limit_s:
                if not gnss_burst:
                    continue
                period, dur = gnss_burst
                if (float(pl[0]) - t_first - gnss_limit_s) % period > dur:
                    continue
            pipe.on_fix(kind, float(pl[0]), float(pl[1]), float(pl[2]), float(pl[3]), int(pl[4]))
            continue
        if out is not None:
            rows.append(out)
    cpu = time.process_time() - cpu0
    n = len(rows)
    res = {
        't': np.array([o.stamp for o in rows]),
        'v': np.array([o.v for o in rows]),
        'a': np.array([o.a for o in rows]),
        'x': np.array([o.x for o in rows]),
        'y': np.array([o.y for o in rows]),
        'z': np.array([o.z for o in rows]),
        'yaw': np.array([o.yaw for o in rows]),
        'has_pose': np.array([o.has_pose for o in rows], bool),
        'var_v': np.array([o.var_v for o in rows]),
        'var_along': np.array([o.var_along for o in rows]),
        'var_cross': np.array([o.var_cross for o in rows]),
        'var_yaw': np.array([o.var_yaw for o in rows]),
        'slip': np.array([o.slip for o in rows], bool),
        'mode': np.array([o.mode for o in rows]),
        'cpu_s': cpu,
        'n_out': n,
    }
    diag_keys = set()
    for o in rows[-1:]:
        diag_keys.update(k for k, v in o.diag.items() if isinstance(v, (int, float)))
    for k in sorted(diag_keys):
        res['diag_' + k] = np.array([float(o.diag.get(k, np.nan)) for o in rows])
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', default=None)
    ap.add_argument('--params', default=None, help='params.yaml')
    ap.add_argument('--maps', default=None)
    ap.add_argument('--set', nargs='*', default=[], help='key=value overrides')
    a = ap.parse_args(argv)
    p = Params.from_yaml(a.params) if a.params else Params()
    over = {}
    for kv in a.set:
        k, v = kv.split('=', 1)
        over[k] = v
    if over:
        d = p.to_dict(); d.update(over); p = Params.from_dict(d)
    res = run_bag(a.bag, p, a.maps)
    print('outputs %d, cpu %.2f s, pose %.1f%%' % (res['n_out'], res['cpu_s'], 100 * res['has_pose'].mean() if res['n_out'] else 0))
    if a.out:
        np.savez_compressed(a.out, **{k: v for k, v in res.items()})


if __name__ == '__main__':
    sys.exit(main())
