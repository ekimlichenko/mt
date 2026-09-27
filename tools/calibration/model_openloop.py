"""Open-loop check of the notch-driven model (core TractionModel + SpeedEstimator without wheel
updates): error budget of model-only dead reckoning (analysis_notes D10) and the effect of the
model terms.  Source: .work/scratch_model/openloop.py.

Windows start every --every s (>= 5 s after the start of data) at the true (gated GNSS) speed; the
lag states / latch timer are warmed up over the previous 3 s with the true speed; then only cmd
messages drive the model for 60 s.  Grade at the true route s (RTK projection) unless mode
'coupled' (route_offset fixed at the window start, the grade follows the model's own odometer).
Errors at H = 2/5/10/30/60 s: speed MAE / RMSE / bias, distance MAE / p90 / relative to distance.

Configurations (--cfg, default: all):
  ref_la0      current Params, grade at the antenna (lookahead 0)
  ref_la_trn   grade at trn_lookahead_* ahead of the antenna (model_lookahead)
  nograde      without grade / curve terms                      (D4: grade halves the 30 s error)
  coupled_la0  grade from the model's own odometer
  hold8_c0     grade_c_hold8 = 0 instead of the brake value      (-> grade_c_hold8 -0.812)
  no_latch     standstill_v = 0 (no standstill latch)            (-> start_delay_s / standstill_v, D8)

Result (49 bags, final code), distance MAE at H = 30 / 60 s: ref_la0 10.8 / 26.4 m, ref_la_trn
10.6 / 25.9, hold8_c0 10.7 / 26.1, coupled_la0 11.0 / 28.1, no_latch 13.9 / 36.2 (speed bias at 30 s
+0.34 vs +0.10 m/s), nograde 25.2 / 73.0 m (the grade term halves the error, D4).

~40-60 s per configuration on 49 bags with 4 processes (all six ~5 min).  Smoke: --bags 2 --cfg ref_la0.

    python tools/calibration/model_openloop.py [--cfg ref_la0 nograde ...] [--bags N|b1,b2] [--every 5] [--jobs 4] [--set k=v ...]
"""
import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C
import model_common as M
from tram_backup_odometry.core.estimator import SpeedEstimator
from tram_backup_odometry.core.traction import TractionModel

H = [2.0, 5.0, 10.0, 30.0, 60.0]
STEP = 0.05

CFGS = {
    'ref_la0': dict(lookahead=0.0),
    'ref_la_trn': dict(lookahead='trn'),
    'nograde': dict(grade=False),
    'coupled_la0': dict(lookahead=0.0, mode='coupled'),
    'hold8_c0': dict(lookahead=0.0, params=dict(grade_c_hold8=0.0)),
    'no_latch': dict(lookahead=0.0, params=dict(standstill_v=0.0)),
}


def run_bag(args):
    b, cfg, sets, every = args
    d = M.load(b)
    p = M.params(**cfg.get('params', {}))
    C.set_params(p, sets)
    r = M.routes()[d['direction']]
    la = cfg.get('lookahead', 0.0)
    if la == 'trn':
        la = p.trn_lookahead_t2s_m if d['direction'] == 'T2S' else p.trn_lookahead_s2t_m
    vt = M.true_speed_fn(d)
    st = M.true_s_fn(d)
    ct, cn = d['C']['t'], d['C']['n']
    o = np.argsort(ct, kind='stable')
    ct, cn = ct[o], cn[o]
    t_lo = max(d['F']['t'][0], d['R']['t'][0], ct[0]) + 5.0
    t_hi = min(d['F']['t'][-1], d['R']['t'][-1], ct[-1]) - H[-1] - 1
    mode = cfg.get('mode', 'oracle')
    rows = []
    for t0 in np.arange(t_lo, t_hi, every):
        grid = t0 + STEP * np.arange(int(round(H[-1] / STEP)) + 1)
        vg = vt(grid)
        if np.isnan(vg[0]):
            continue
        sg = st(grid)
        m = TractionModel(p)
        est = SpeedEstimator(p, m, r.grade_at if cfg.get('grade', True) else None,
                             r.curv_at if cfg.get('grade', True) else None, la)
        ic = np.searchsorted(ct, t0 - 4.0)
        tw = np.arange(t0 - 3.0, t0 + 1e-9, STEP)
        vw = vt(tw)
        for k, tk in enumerate(tw):              # warm-up of the lag states / latch timer
            while ic < len(ct) and ct[ic] <= tk:
                m.add_cmd(ct[ic], cn[ic])
                ic += 1
            m.step(tk, STEP, vw[k] if not np.isnan(vw[k]) else vg[0])
        est.reset(grid[0], vg[0])
        if mode == 'coupled':
            est.route_offset = sg[0] if not np.isnan(sg[0]) else -1e9
        vs = np.empty(len(grid))
        ss = np.empty(len(grid))
        vs[0] = est.v
        ss[0] = 0.0
        h8 = False
        modes = []
        for k in range(1, len(grid)):
            tk = grid[k]
            while ic < len(ct) and ct[ic] <= tk:
                m.add_cmd(ct[ic], cn[ic])
                ic += 1
            if mode == 'oracle':
                est.route_offset = (sg[k - 1] - est.s) if not np.isnan(sg[k - 1]) else -1e9
            out = est.predict_to(tk)
            vs[k] = est.v
            ss[k] = est.s
            if out.mode == 'hold8' and est.v > 2.5:
                h8 = True
            modes.append(h8)
        ok_frac = np.r_[1.0, np.cumsum(~np.isnan(vg[1:])) / np.arange(1, len(vg))]
        vg_f = np.where(np.isnan(vg), np.interp(grid, grid[~np.isnan(vg)], vg[~np.isnan(vg)]), vg)
        dtrue = np.r_[0.0, np.cumsum(0.5 * (vg_f[1:] + vg_f[:-1]) * STEP)]
        on_frac = np.r_[1.0, np.cumsum(~np.isnan(sg[1:])) / np.arange(1, len(sg))]
        for h in H:
            k = int(round(h / STEP))
            if ok_frac[k] < 0.9 or np.isnan(vg[k]):
                continue
            rows.append((t0, h, vs[k] - vg[k], ss[k] - dtrue[k], dtrue[k], vg[0], float(modes[k - 1]), on_frac[k]))
    return rows


def summarize(R, label):
    lines = []
    for h in H:
        m = R[:, 1] == h
        if not m.any():
            continue
        e_v, e_s, d, h8 = R[m, 2], R[m, 3], R[m, 4], R[m, 6] > 0
        mov = d / h > 0.5

        def st(mask):
            return (np.mean(np.abs(e_v[mask])), np.sqrt(np.mean(e_v[mask] ** 2)), np.mean(e_v[mask]),
                    np.mean(np.abs(e_s[mask])), np.percentile(np.abs(e_s[mask]), 90),
                    100 * np.sum(np.abs(e_s[mask])) / max(np.sum(d[mask]), 1e-9), mask.sum())
        a, b_, c = st(np.ones(len(d), bool)), st(mov), st(~h8)
        lines.append('%-12s H=%4.0fs N=%5d | v MAE %.3f RMSE %.3f bias %+.3f | d MAE %6.2f m p90 %6.2f  rel %5.2f%% | moving rel %5.2f%% | no-hold8 rel %5.2f%% (MAE v %.3f)'
                     % (label, h, a[6], a[0], a[1], a[2], a[3], a[4], a[5], b_[5], c[5], c[0]))
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', nargs='*', default=list(CFGS), choices=list(CFGS))
    ap.add_argument('--bags', default='')
    ap.add_argument('--every', type=float, default=5.0, help='window spacing, s')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--set', nargs='*', default=[], help='Params overrides for every configuration')
    a = ap.parse_args()
    B = M.bag_list(a.bags)
    od = C.out_dir('model_openloop')
    for name in a.cfg:
        t0 = time.time()
        jobs = [(b, CFGS[name], a.set, a.every) for b in B]
        with ProcessPoolExecutor(max(a.jobs, 1)) as ex:
            rows = [r for rr in ex.map(run_bag, jobs) for r in rr]
        R = np.array(rows, float).reshape(-1, 8)
        np.save(os.path.join(od, 'ol_%s.npy' % name), R)
        print('\n'.join(summarize(R, name)), '  (%d bags, %.0f s)' % (len(B), time.time() - t0), flush=True)


if __name__ == '__main__':
    main()
