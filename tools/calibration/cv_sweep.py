"""One-parameter sweep through the full replay + judge-like evaluation of tools/run_cv.py: the CV
tuning of the Params that are not measured directly from the data, and the flatness of the plateau
around the chosen value.

For every value of --values the selected bags (default: the 60 eval_ok bags) are replayed with
Params = defaults <- --params yaml <- --set <- {--param: value} (exactly tools/run_cv.py, vehicle_id
from the bag name, GNSS only for the first 10 s) and scored against the RTK master reference with
tools/evaluate.py.  Printed per value: mean / median over bags of score_loss, al_rmse (along-track
RMSE, m), v_rmse (speed RMSE, m/s), drift_pct, p_in95, p_nees, and the change of the mean score_loss
relative to the best value (the plateau).  --cv adds a leave-one-group-out selection (group = vehicle
day, the fold of .work/bag_index.json): for each group the value is chosen on the other groups and
scored on the held-out one; the spread of the chosen values shows how stable the choice is.

Used for (see tools/calibration/README.md; results of the final code):
    wheel_k_30639        --bags 'eval&vehicle:30639' --values 3.585,3.59,3.595,3.60,3.605   (-> 3.59)
    trn_sigma            --values 0.065,0.08,0.1,0.13,0.16                                   (-> 0.1)
    trn_lookahead_t2s_m  --values 6,9,12,15,18                                               (-> 12)
    trn_lookahead_s2t_m  --values 6,8,10,12,14                                               (-> 10)
    bridge_tangent_k_t2s / bridge_tangent_k_s2t, trn_apply_sigma_m, ... in the same way.
The covariance scales (pose_trn_sigma_scale, pose_sigma_cross_m) do not change the estimate: they are
calibrated without re-running by covariance_consistency.py.

    python tools/calibration/cv_sweep.py --param wheel_k_30639 --values 3.585,3.59,3.595,3.60,3.605 \
           --bags 'eval&vehicle:30639' [--metric score_loss] [--cv] [--jobs 4] [--set k=v ...]
~20 s per value on the 60 eval bags with 4 processes.  Output: $CALIB_WORK/cv_sweep/<param>.json.
"""
import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C
import run_cv

KEYS = ('score_loss', 'al_rmse', 'v_rmse', 'drift_pct', 'p_in95', 'p_nees')


def _cfg(a, value):
    ov = run_cv.parse_sets(a.set)
    ov[a.param] = value
    return dict(tag='cv_sweep', out=C.out_dir('cv_sweep'), params=a.params, fold_params=None, overrides=ov,
                estimator=run_cv.DEFAULT_ESTIMATOR, antennas=['master'], gnss_limit=a.gnss_limit, tol=0.05,
                gate=5.0, clean_ref=False, plot_all=False, plot_bags=set(), save_est=False)


def one(args):
    bag, info, cfg = args
    r = run_cv.process_bag(bag, info, cfg)
    m = (r.get('metrics') or {}).get('master', {})
    return dict(bag=bag, group=info.get('group'), vehicle=info.get('vehicle'), status=r['status'],
                error=(r['error'] or '').strip().splitlines()[-1:] if r['status'] != 'ok' else None,
                **{k: m.get(k, np.nan) for k in KEYS})


def stat(rows, key, fn):
    v = np.array([r[key] for r in rows], float)
    v = np.abs(v[np.isfinite(v)]) if key not in ('p_in95', 'p_nees') else v[np.isfinite(v)]
    return float(fn(v)) if len(v) else np.nan


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--param', required=True, help='Params field to sweep')
    ap.add_argument('--values', required=True, help='comma list of values')
    ap.add_argument('--bags', default='eval', help="tools/run_cv.py selector (eval, 'eval&vehicle:30639', 'eval&dir:S2T', ...)")
    ap.add_argument('--metric', default='score_loss', choices=KEYS[:4], help='selection metric (mean over bags)')
    ap.add_argument('--cv', action='store_true', help='leave-one-group-out selection of the value')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--params', default=None, help='params yaml (as run_cv.py --params)')
    ap.add_argument('--set', nargs='*', default=[], help='fixed key=value overrides')
    ap.add_argument('--gnss-limit', type=float, default=10.0)
    a = ap.parse_args()
    if a.param not in {f.name for f in __import__('dataclasses').fields(C.Params)}:
        raise SystemExit('unknown Params field %r' % a.param)
    index = C.load_index()
    bags = [b for b in run_cv.select_bags(a.bags, index) if index['bags'][b]['eval_ok']]
    values = [v.strip() for v in a.values.split(',') if v.strip()]
    cur = getattr(C.Params(), a.param)
    print('%s: %d eval bags, values %s (Params default %s), metric %s' % (a.param, len(bags), values, cur, a.metric))
    out = {'param': a.param, 'bags': bags, 'default': cur, 'set': a.set, 'values': {}}
    t0 = time.time()
    with ProcessPoolExecutor(max(a.jobs, 1)) as ex:
        for val in values:
            cfg = _cfg(a, val)
            rows = list(ex.map(one, [(b, index['bags'][b], cfg) for b in bags]))
            bad = [r for r in rows if r['status'] != 'ok']
            if bad:
                print('   %s: %d bags failed, e.g. %s %s' % (val, len(bad), bad[0]['bag'], bad[0]['error']))
            out['values'][val] = rows
            print('   %-8s done (%.0f s)' % (val, time.time() - t0), flush=True)
    print('\n%-8s %5s | %-15s | %-15s | %-15s | %-8s | %-6s %-6s | d%s vs best' % (
        a.param[:8], 'n', 'score_loss mean/med', 'al_rmse mean/med', 'v_rmse mean/med', 'drift%', 'in95', 'nees', a.metric))
    means = {v: stat(out['values'][v], a.metric, np.mean) for v in values}
    best = min(means, key=lambda v: means[v] if np.isfinite(means[v]) else np.inf)
    summ = {}
    for v in values:
        R = [r for r in out['values'][v] if r['status'] == 'ok']
        s = {k: dict(mean=stat(R, k, np.mean), median=stat(R, k, np.median)) for k in KEYS}
        summ[v] = s
        mark = '*' if str(v) == str(cur) or (_num(v) is not None and _num(v) == _num(cur)) else ' '
        print('%s%-7s %5d | %6.3f / %6.3f | %6.3f / %6.3f | %6.4f / %6.4f | %8.3f | %.3f  %5.2f | %+6.2f%%' % (
            mark, v, len(R), s['score_loss']['mean'], s['score_loss']['median'], s['al_rmse']['mean'],
            s['al_rmse']['median'], s['v_rmse']['mean'], s['v_rmse']['median'], s['drift_pct']['mean'],
            s['p_in95']['mean'], s['p_nees']['mean'], 100 * (means[v] / means[best] - 1)))
    print('best by mean %s: %s   (* = Params default)' % (a.metric, best))
    out['summary'] = summ
    out['best'] = best
    if a.cv:
        groups = sorted({r['group'] for r in out['values'][values[0]]}, key=str)
        chosen, held = [], []
        print('per group mean %s:' % a.metric)
        for g in groups:
            print('   %-12s n=%2d  %s' % (g, sum(r['group'] == g for r in out['values'][values[0]]), '  '.join(
                '%s: %.3f' % (v, np.mean([r[a.metric] for r in out['values'][v] if r['group'] == g and np.isfinite(r[a.metric])]))
                for v in values)))
        for g in groups:
            tr = {v: np.mean([r[a.metric] for r in out['values'][v] if r['group'] != g and np.isfinite(r[a.metric])])
                  for v in values}
            vb = min(tr, key=tr.get)
            te = [r[a.metric] for r in out['values'][vb] if r['group'] == g and np.isfinite(r[a.metric])]
            chosen.append(vb)
            held += te
        cnt = {v: chosen.count(v) for v in values}
        print('leave-one-group-out (%d groups): chosen value counts %s; held-out mean %s %.3f (all-bag best %.3f)' % (
            len(groups), cnt, a.metric, float(np.mean(held)), means[best]))
        out['cv'] = dict(groups=[str(g) for g in groups], chosen=chosen, heldout_mean=float(np.mean(held)))
    fn = os.path.join(C.out_dir('cv_sweep'), '%s.json' % a.param)
    C.save_json(fn, out)
    print('written', fn)


def _num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


if __name__ == '__main__':
    main()
