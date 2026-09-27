#!/usr/bin/env python3
"""Offline copy of the organisers' checker (hackathon_solution_checker) on a test bag.

The organisers' checker (check-code/src/checker_ros/hackathon_solution_checker/metrics.py)
pairs /result/velocity and /result/position with the reference /localization/kinematic_state
(nav_msgs/Odometry, 50 Hz, map frame, base_link, on the pathgraph incl. z) through
message_filters.ApproximateTimeSynchronizer (python, queue 100, slop 0.05 s) and scores
speed RMSE (twist.linear.x) and the 3D position error (x, y, z).

This script replays the bag through the same Pipeline as the ROS node (replay order =
record time, output arrival = record time + 1 ms), emulates the synchroniser
(on each arrival the closest stamp of the other queue within the slop is taken, both
messages are consumed) and prints/writes the metrics, also per minute and for a few
variants (GNSS aiding off, loops off, old output point, ...).

usage:
  python tools/checker_sim.py [--bag ../check_code/check-code/bags/30618_88aea4d9]
                              [--variants all|final] [--set k=v ...] [--out results/checker_30618_88aea4d9]
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'tram_backup_odometry'))

from tram_backup_odometry import bagio                          # noqa: E402
from tram_backup_odometry.bagio import _Cdr                     # noqa: E402
from tram_backup_odometry.core.config import Params             # noqa: E402
from tram_backup_odometry.core.pipeline import Pipeline         # noqa: E402
from tram_backup_odometry.replay import default_maps_dir, merged_events  # noqa: E402

DEFAULT_BAG = os.path.join(ROOT, '..', 'check_code', 'check-code', 'bags', '30618_88aea4d9')
REF = '/localization/kinematic_state'

_orig_parse = bagio._parse


def _parse(typ, b):
    """bagio + nav_msgs/Odometry (the reference topic of the test bag)."""
    if typ.endswith('Odometry'):
        r = _Cdr(b)
        th, fid = r.header()
        child = r.string()
        p = [r.f64() for _ in range(7)]
        [r.f64() for _ in range(36)]
        tw = [r.f64() for _ in range(6)]
        return th, fid, {'x': p[0], 'y': p[1], 'z': p[2], 'qx': p[3], 'qy': p[4], 'qz': p[5], 'qw': p[6],
                         'vx': tw[0], 'vy': tw[1], 'vz': tw[2], 'wz': tw[5], 'child': child}
    return _orig_parse(typ, b)


bagio._parse = _parse

VARIANTS = {
    'final': {},
    'no_gnss_aiding': {'gnss_aiding': False},
    'no_loops': {'loops_enable': False},
    'no_velocity_delay': {'output_velocity_delay_s': 0.0},
    'master_antenna_point': {'output_along_offset_m': 0.0, 'output_z_offset_m': 3.10},
    'pre_checker_version': {'gnss_aiding': False, 'loops_enable': False, 'output_velocity_delay_s': 0.0,
                            'output_along_offset_m': 0.0, 'output_z_offset_m': 3.10},
}


def sync_pairs(ref_arr, ref_st, res_arr, res_st, slop=0.05, qsize=100):
    """ApproximateTimeSynchronizer for two topics as the python message_filters does it."""
    ev = [(a, 0, i) for i, a in enumerate(ref_arr)] + [(a, 1, i) for i, a in enumerate(res_arr)]
    ev.sort()
    stamps = (ref_st, res_st)
    queues = ({}, {})
    pairs = []
    for _, q, i in ev:
        st = int(round(stamps[q][i] * 1e9))
        my, other = queues[q], queues[1 - q]
        my[st] = i
        while len(my) > qsize:
            del my[min(my)]
        best = None
        for s in other:
            dd = abs(s - st)
            if dd <= slop * 1e9 and (best is None or dd < best[0]):
                best = (dd, s)
        if best is None or best[0] >= slop * 1e9:
            continue
        j = other[best[1]]
        pairs.append((i, j) if q == 0 else (j, i))
        del other[best[1]]
        del my[st]
    return pairs


def run(p, D, gnss_until_s=None):
    pipe = Pipeline(p, default_maps_dir())
    rows, arr = [], []
    t_first = None
    c0 = time.process_time()
    for t_rec, kind, pl in merged_events(D, True):
        if kind in ('front', 'rear'):
            out = pipe.on_wheel(kind, float(pl[0]), float(pl[1]))
        elif kind == 'cmd':
            out = pipe.on_cmd(float(pl[0]), int(pl[1]))
        else:
            t_first = float(pl[0]) if t_first is None else t_first
            if gnss_until_s is None or float(pl[0]) <= t_first + gnss_until_s:
                pipe.on_fix(kind, float(pl[0]), float(pl[1]), float(pl[2]), float(pl[3]), int(pl[4]))
            continue
        if out is not None:
            rows.append(out)
            arr.append(t_rec + 0.001)
    return pipe, rows, np.array(arr), time.process_time() - c0


def score(D, rows, arr):
    o = D[REF]
    st = np.array([r.stamp for r in rows])
    pv = sync_pairs(o['t_rec'], o['t_hdr'], arr, st)
    ev = np.array([rows[j].v - o['vx'][i] for i, j in pv])
    idx = np.where(np.array([r.has_pose for r in rows]))[0]
    pp = sync_pairs(o['t_rec'], o['t_hdr'], arr[idx], st[idx])
    E = np.array([(rows[idx[j]].x - o['x'][i], rows[idx[j]].y - o['y'][i], rows[idx[j]].z - o['z'][i]) for i, j in pp])
    d = np.linalg.norm(E, axis=1)
    d2 = np.linalg.norm(E[:, :2], axis=1)
    tref = np.array([o['t_rec'][i] for i, _ in pp]) - o['t_rec'][0]
    res = dict(v_rmse=float(np.sqrt(np.mean(ev ** 2))), v_mae=float(np.mean(np.abs(ev))), v_max=float(np.abs(ev).max()),
               n_vel_pairs=len(ev), n_ref=len(o['t_hdr']),
               x_rmse=float(np.sqrt(np.mean(E[:, 0] ** 2))), y_rmse=float(np.sqrt(np.mean(E[:, 1] ** 2))),
               z_rmse=float(np.sqrt(np.mean(E[:, 2] ** 2))), d3_rmse=float(np.sqrt(np.mean(d ** 2))),
               d3_mean=float(d.mean()), d3_median=float(np.median(d)), d3_p95=float(np.percentile(d, 95)),
               d3_max=float(d.max()), d2_rmse=float(np.sqrt(np.mean(d2 ** 2))), n_pos_pairs=len(d), n_out=len(rows))
    per_min = []
    for a in np.arange(0, tref.max() + 60, 60):
        m = (tref >= a) & (tref < a + 60)
        if m.any():
            per_min.append((int(a), float(np.sqrt(np.mean(d[m] ** 2)))))
    res['d3_rmse_per_minute'] = per_min
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', default=DEFAULT_BAG)
    ap.add_argument('--vehicle-id', default='30618', help="the hidden check is tram 30618 ('' = unknown)")
    ap.add_argument('--variants', default='all', help="'all', 'final' or a comma list of %s" % ', '.join(VARIANTS))
    ap.add_argument('--gnss-start-only', type=float, default=None,
                    help='also run the final config with GNSS only in the first N s after the first fix')
    ap.add_argument('--set', nargs='*', default=[], help='key=value overrides for every variant')
    ap.add_argument('--out', default=None, help='directory for summary.json / summary.md')
    a = ap.parse_args(argv)
    D = bagio.load_bag(a.bag)
    names = list(VARIANTS) if a.variants == 'all' else a.variants.split(',')
    runs = [(n, VARIANTS[n], None) for n in names]
    if a.gnss_start_only is not None:
        runs.append(('final_gnss_first_%gs_only' % a.gnss_start_only, {}, a.gnss_start_only))
    results = {}
    for name, over, gl in runs:
        p = Params()
        p.vehicle_id = a.vehicle_id
        for k, v in list(over.items()) + [kv.split('=', 1) for kv in a.set]:
            t = type(getattr(p, k))
            setattr(p, k, (str(v).lower() in ('1', 'true', 'yes')) if t is bool and isinstance(v, str) else t(v))
        pipe, rows, arr, cpu = run(p, D, gl)
        r = score(D, rows, arr)
        r.update(cpu_s=cpu, overrides=over, gnss_until_s=gl, gnss_bursts=pipe.n_bursts,
                 gnss_bursts_rejected=pipe.n_bursts_rejected, init_kind=getattr(pipe.init_res, 'kind', None))
        results[name] = r
        print('%-28s v_rmse %.4f  d3_rmse %.2f  d3_med %.2f  d3_p95 %.2f  d3_max %.1f  z_rmse %.2f  bursts %d/%d' % (
            name, r['v_rmse'], r['d3_rmse'], r['d3_median'], r['d3_p95'], r['d3_max'], r['z_rmse'],
            r['gnss_bursts'], r['gnss_bursts_rejected']))
        print('   per minute:', ' '.join('%d:%.1f' % q for q in r['d3_rmse_per_minute']))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, 'summary.json'), 'w') as f:
            json.dump(dict(bag=os.path.basename(os.path.normpath(a.bag)), vehicle_id=a.vehicle_id, results=results), f, indent=1)
        L = ['# Offline checker on %s' % os.path.basename(os.path.normpath(a.bag)), '',
             'Emulation of the organisers\' hackathon_solution_checker (ApproximateTimeSynchronizer, slop 0.05 s, '
             'queue 100; reference `%s`, base_link on the pathgraph). Command: `python tools/checker_sim.py --out %s`.' % (
                 REF, os.path.relpath(a.out, ROOT)), '',
             '| variant | v_rmse, m/s | 3D RMSE, m | 3D median, m | 3D p95, m | 3D max, m | z RMSE, m | GNSS bursts used/rejected |',
             '|---|---|---|---|---|---|---|---|']
        for n, r in results.items():
            L.append('| %s | %.4f | %.2f | %.2f | %.2f | %.1f | %.2f | %d/%d |' % (
                n, r['v_rmse'], r['d3_rmse'], r['d3_median'], r['d3_p95'], r['d3_max'], r['z_rmse'],
                r['gnss_bursts'], r['gnss_bursts_rejected']))
        L += ['', '3D RMSE per minute (s from the bag start: m):', '']
        for n, r in results.items():
            L.append('- %s: %s' % (n, ' '.join('%d:%.1f' % q for q in r['d3_rmse_per_minute'])))
        with open(os.path.join(a.out, 'summary.md'), 'w') as f:
            f.write('\n'.join(L) + '\n')
        print('->', a.out)


if __name__ == '__main__':
    main()
