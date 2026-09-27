#!/usr/bin/env python3
"""Index of all recorded bags -> .work/bag_index.json (grouping, folds, reference quality).

Per bag (from .work/cache/<bag>.pkl and dataset/data/<bag>/metadata.yaml):
  vehicle      numeric prefix of the bag id ('30618' / '30639')
  day          Europe/Moscow date of the recording start (metadata starting_time)
  group        '<vehicle>_<day>': one recording session; leave-one-group-out unit
  fold         index of the group in the sorted group list (LOGO fold id)
  duration_s   metadata duration
  has_gnss     master fix topic has messages
  rtk_frac     fraction of master fixes with status == 2
  rtk_s        seconds covered by status-2 master fixes (count x median fix period)
  vel_s        seconds of master velocity usable as speed reference (status-2 fix within 0.05 s)
  direction    'T2S' / 'S2T' by the evaluator's majority rule on the status-2 fixes
               (falls back to all fixes when there is no RTK; None when undecidable)
  onmap_frac   fraction of status-2 master fixes within 5 m of the direction's map and inside it
  md5_front    md5 of the front-bogie speed array; identical arrays = duplicate recordings
  dup_of       lexicographically first bag with the same md5 (None for the original)
  eval_ok      unique, >= 60 s of both rtk_s and vel_s, direction from RTK fixes: scored bags

Duplicates always share a group (same start time), so leave-one-group-out folds
never leak a duplicate into training.

    python tools/bag_index.py [--out .work/bag_index.json]
"""
import argparse
import datetime
import hashlib
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate as ev  # noqa: E402

INDEX_PATH = os.path.join(ev.ROOT, '.work', 'bag_index.json')
MSK = datetime.timezone(datetime.timedelta(hours=3))  # Europe/Moscow, no DST since 2014
MIN_EVAL_S = 60.0


def list_cached_bags():
    return sorted(f[:-4] for f in os.listdir(ev.CACHE_DIR) if f.endswith('.pkl'))


def read_metadata(bag):
    path = os.path.join(ev.DATA_DIR, bag, 'metadata.yaml')
    if not os.path.exists(path):
        return None, None
    with open(path) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    t0 = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
    return t0, info['duration']['nanoseconds'] * 1e-9


def _period(t):
    dt = np.diff(np.sort(t))
    dt = dt[dt > 0]
    return float(np.median(dt)) if len(dt) else 0.1


def index_bag(bag):
    D = ev.load_cache(bag)
    t0, duration = read_metadata(bag)
    if t0 is None:  # no metadata: fall back to the record time of the first message
        recs = [d['t_rec'] for d in D.values() if len(d.get('t_rec', []))]
        t0 = min(float(r[0]) for r in recs) if recs else 0.0
        duration = max(float(r[-1]) for r in recs) - t0 if recs else 0.0
    day = datetime.datetime.fromtimestamp(t0, MSK).strftime('%Y-%m-%d')
    vehicle = bag.split('_')[0]
    front = D.get('/vehicle/front_bogie_velocity') or {}
    fv = np.ascontiguousarray(np.asarray(front.get('v', []), float))
    rec = dict(bag=bag, vehicle=vehicle, day=day, group='%s_%s' % (vehicle, day),
               start_unix=round(t0, 3), duration_s=round(duration, 2),
               n_front=int(len(fv)), n_rear=int(len((D.get('/vehicle/rear_bogie_velocity') or {}).get('v', []))),
               n_cmd=int(len((D.get('/vehicle/driver_position_cmd') or {}).get('pos', []))),
               md5_front=hashlib.md5(fv.tobytes()).hexdigest() if len(fv) else None)
    if len(fv) > 1:
        ft = np.asarray(front['t_hdr'], float)
        rec['wheel_dist_m'] = round(float(np.sum(np.clip(fv[1:], 0, None) / 3.6 * np.clip(np.diff(ft), 0, 1.0))), 1)
    m = ev.fixes(D, 'master')
    r = ev.fixes(D, 'rover')
    rec.update(has_gnss=m is not None, has_rover=r is not None, n_master=0 if m is None else int(len(m['t'])))
    rec.update(rtk_frac=0.0, rtk_s=0.0, vel_s=0.0, direction=None, direction_src=None, onmap_frac=None,
               rtk_at_start=False)
    if m is None:
        return rec
    rtk = m['st'] == 2
    rec['rtk_frac'] = round(float(rtk.mean()), 4)
    rec['rtk_s'] = round(float(rtk.sum() * _period(m['t'])), 1)
    rec['rtk_at_start'] = bool(rtk[m['t'] <= m['t'][0] + 3.0].any())
    rec['gnss_start_status'] = int(np.median(m['st'][m['t'] <= m['t'][0] + 3.0]))
    gv = D.get(ev.TOPIC_MVEL) or {}
    if len(gv.get('t_hdr', [])) and rtk.any():
        tv = np.sort(np.asarray(gv['t_hdr'], float))
        j, dt = ev.nearest(tv, m['t'])
        okv = (dt < 0.05) & rtk[j]
        rec['vel_s'] = round(float(okv.sum() * _period(tv)), 1)
    if rtk.sum() >= 2:
        rec['direction'] = ev.detect_direction(m['xyz'][rtk][:, :2])
        rec['direction_src'] = 'rtk'
    if rec['direction'] is None:
        rec['direction'] = ev.detect_direction(m['xyz'][:, :2])
        rec['direction_src'] = 'any_status' if rec['direction'] else None
    if rec['direction'] is not None:
        use = rtk if rtk.any() else np.ones(len(rtk), bool)
        _, d, _, beyond = ev.routes()[rec['direction']].project(m['xyz'][use][:, :2])
        rec['onmap_frac'] = round(float(((d < 5.0) & ~beyond).mean()), 4)
    return rec


def build_index(bags, jobs=8):
    with ProcessPoolExecutor(jobs) as ex:
        recs = {r['bag']: r for r in ex.map(index_bag, bags)}
    first = {}
    for b in sorted(recs):
        h = recs[b]['md5_front']
        recs[b]['dup_of'] = first.get(h) if h is not None else None
        if h is not None and h not in first:
            first[h] = b
    for b, r in recs.items():
        if r['dup_of'] is not None and recs[r['dup_of']]['group'] != r['group']:
            r['group'] = recs[r['dup_of']]['group']  # keep duplicates in one fold
        # direction from the RTK fixes, as evaluate.load_ref decides it (otherwise no along-track metrics)
        r['eval_ok'] = bool(r['dup_of'] is None and r['rtk_s'] >= MIN_EVAL_S and r['vel_s'] >= MIN_EVAL_S
                            and r['direction_src'] == 'rtk')
    groups = sorted({r['group'] for r in recs.values()})
    for r in recs.values():
        r['fold'] = groups.index(r['group'])
    summary = {g: dict(fold=i, bags=sorted(b for b, r in recs.items() if r['group'] == g),
                       eval_ok=sorted(b for b, r in recs.items() if r['group'] == g and r['eval_ok']))
               for i, g in enumerate(groups)}
    return dict(n_bags=len(recs), n_unique=sum(r['dup_of'] is None for r in recs.values()),
                n_eval_ok=sum(r['eval_ok'] for r in recs.values()), min_eval_s=MIN_EVAL_S,
                groups=summary, bags=dict(sorted(recs.items())))


def load_index(path=INDEX_PATH):
    with open(path) as f:
        return json.load(f)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=INDEX_PATH)
    ap.add_argument('--jobs', type=int, default=8)
    a = ap.parse_args(argv)
    idx = build_index(list_cached_bags(), a.jobs)
    with open(a.out, 'w') as f:
        json.dump(idx, f, indent=1)
    print('bags %d, unique %d, eval_ok %d -> %s' % (idx['n_bags'], idx['n_unique'], idx['n_eval_ok'], a.out))
    print('%-22s %4s %5s %7s  %s' % ('group', 'fold', 'bags', 'eval_ok', 'directions (eval_ok)'))
    for g, s in idx['groups'].items():
        dirs = [idx['bags'][b]['direction'] for b in s['eval_ok']]
        print('%-22s %4d %5d %7d  T2S %d / S2T %d' % (g, s['fold'], len(s['bags']), len(s['eval_ok']),
                                                     dirs.count('T2S'), dirs.count('S2T')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
