"""Closed-loop speed accuracy of core.estimator.SpeedEstimator (model + wheel updates) and its
robustness to wheel dropouts; Q / R consistency checks for the Params bias_enable, bias_tau_s,
bias_sigma, accel_corr_s, gain_adapt, s_correction, meas_sigma.
Source: .work/scratch_model/{closedloop,calib_q}.py (paths / Params fixed, current Measurement type).

Bags: the clean RTK bags of model_common.py (49).  Event stream: F / R wheel samples and cmd messages
sorted by header stamp (cmd first at equal stamps):
  cmd    -> model.add_cmd, predict_to(t)
  F / R  -> predict_to(t), update(fused wheel speed, sigma meas_sigma, state 'both' / 'single_*')
The fused wheel speed is the mean of the fresh bogies (<= 0.5 s): 'lin' = per-bogie linear
extrapolation to t (what core/wheel_fusion does, default), 'zoh' = last values.  Baselines printed
next to the EKF: zoh and lin themselves.  Truth: gated GNSS master speed (status-2 fix within 0.05 s),
nearest sample within 0.05 s.  Grade: route_offset aligned to the RTK route s every 10 s (stands in
for TRN), frozen in the dropout branches.
Dropouts: every 30 s from 120 s after the first alignment (true v > 3 m/s) the estimator is
deep-copied; the branch ignores every wheel sample for T = 10 / 30 / 60 s (cmds still arrive), then
re-acquires for 10 s.  Distance error at the end of the gap vs the true distance, its z-score with
the filter's P_ss, and the constant-velocity baseline (last wheel speed * T).

Printed per configuration: speed RMSE / MAE per regime (moving without glitches, traction, brake,
hold8, ...), NIS mean / P(NIS > 9), share of clipped updates, innovation lag-1 autocorrelation,
dropout distance errors.  Result on the 49 bags (final code, re-run; final Params = 'ref'):
moving-noglitch RMSE ekf 0.037 vs zoh 0.048 / lin 0.039 m/s (ref_zoh: ekf 0.042); NIS mean 0.21
(R conservative: measurement sigma 0.03 also covers the quantisation); dropout d err MAE at the end
of the gap 1.6 / 9.0 / 22.4 m for 10 / 30 / 60 s (constant-velocity baseline 17 / 94 / 202 m),
rms z 0.79 / 0.83 / 0.74 (P_ss consistent-to-conservative), 10 s after re-acquisition
0.87 / 5.4 / 16.0 m.  Per option (end-of-gap MAE 10 / 30 / 60 s, rms z; after re-acquisition):
  no_scorr   the same at the end, after re-acquisition 1.6 / 9.0 / 22.4 m -> s_correction = True
  gain_off   1.84 / 9.91 / 22.0 m (+15 / +10 / -2 %)                     -> gain_adapt = True
  bias_off   1.62 / 8.97 / 22.8 m, rms z 1.06-1.19 (P_ss over-confident) -> bias_enable = True
  b3_s0.1    1.58 / 8.96 / 22.5 m, rms z 0.61-0.70; b30_s0.05 1.65 / 9.33 / 23.5 m; b10_s0.1 rms z 0.53-0.63
             -> bias_tau_s 10 / bias_sigma 0.05: equal errors, the rms z closest to 1
  ac0.5 / ac2  equal errors, rms z 0.88-1.02 / 0.61-0.67                -> accel_corr_s 1.0

Configurations (--cfg, default: ref ref_zoh bias_off gain_off no_scorr):
  ref            current Params, lin measurement
  ref_zoh        zoh measurement
  bias_off       bias_enable = False              gain_off       gain_adapt = False
  gain_off_bias_off                               no_scorr       s_correction = False
  b30_s0.05      bias_tau_s 30, bias_sigma 0.05   b10_s0.1       bias_tau_s 10, bias_sigma 0.1
  b3_s0.1        bias_tau_s 3, bias_sigma 0.1     ac0.5 / ac2    accel_corr_s 0.5 / 2.0
~10 s per configuration on 49 bags with 4 processes once model_prep.py has built the arrays
(building them on demand adds ~1 min).  Smoke: --bags 3 --cfg ref (2 s).

    python tools/calibration/model_closedloop.py [--cfg ref bias_off ...] [--bags N|b1,b2] [--jobs 4] [--no-dropout] [--set k=v ...]
"""
import argparse
import copy
import math
import os
import pickle
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C
import model_common as M
from tram_backup_odometry.core.estimator import SpeedEstimator
from tram_backup_odometry.core.traction import TractionModel
from tram_backup_odometry.core.wheel_fusion import Measurement

REALIGN_S = 10.0
FRESH = 0.5
EXTRAP_CAP = 0.2
T_DROP = [10.0, 30.0, 60.0]
REACQ_S = 10.0
DROP_EVERY = 30.0
WARM_S = 120.0
MODE = {'traction': 1, 'brake': 2, 'coast': 3, 'hold8': 4, 'standstill': 5}

CFGS = {
    'ref': dict(),
    'ref_zoh': dict(meas='zoh'),
    'bias_off': dict(params=dict(bias_enable=False)),
    'gain_off': dict(params=dict(gain_adapt=False)),
    'gain_off_bias_off': dict(params=dict(gain_adapt=False, bias_enable=False)),
    'no_scorr': dict(params=dict(s_correction=False)),
    'b30_s0.05': dict(params=dict(bias_tau_s=30.0, bias_sigma=0.05)),
    'b10_s0.1': dict(params=dict(bias_tau_s=10.0, bias_sigma=0.1)),
    'b3_s0.1': dict(params=dict(bias_tau_s=3.0, bias_sigma=0.1)),
    'ac0.5': dict(params=dict(accel_corr_s=0.5)),
    'ac2': dict(params=dict(accel_corr_s=2.0)),
}


def bogie_meas(last, t):
    """Mean of the fresh bogies (last values): (v, state) or None."""
    vals, names = [], []
    for k in (0, 1):
        L = last[k]
        if L is not None and 0.0 <= t - L[0] <= FRESH:
            vals.append(L[1])
            names.append(k)
    if not vals:
        return None
    st = 'both' if len(vals) == 2 else ('single_front' if names[0] == 0 else 'single_rear')
    return sum(vals) / len(vals), st


def bogie_lin(last, t):
    """Mean of the per-bogie linear extrapolations to t (capped at EXTRAP_CAP s)."""
    vals = []
    for k in (0, 1):
        L = last[k]
        if L is not None and 0.0 <= t - L[0] <= FRESH:
            v = L[1]
            if L[2] is not None and 0.0 < L[0] - L[2] <= 0.3:
                v += (L[1] - L[3]) / (L[0] - L[2]) * min(t - L[0], EXTRAP_CAP)
            vals.append(max(v, 0.0))
    return sum(vals) / len(vals) if vals else float('nan')


def filter_meas(last, t, mode):
    zm = bogie_meas(last, t)
    if zm is None or mode == 'zoh':
        return zm
    return bogie_lin(last, t), zm[1]


def meas(v, sigma, state):
    return Measurement(v=v, sigma=sigma, state=state, slip=False, front_ok=state != 'single_rear',
                       rear_ok=state != 'single_front', v_front=float('nan'), v_rear=float('nan'))


def events(d):
    tF, vF, tR, vR, tC, nC = d['F']['t'], d['F']['v'], d['R']['t'], d['R']['v'], d['C']['t'], d['C']['n']
    t = np.r_[tF, tR, tC]
    kind = np.r_[np.zeros(len(tF), int), np.ones(len(tR), int), np.full(len(tC), 2)]
    val = np.r_[vF, vR, nC.astype(float)]
    order = np.lexsort((np.where(kind == 2, 0, 1), t))       # at equal stamps: cmd first
    return t[order], kind[order], val[order]


class Truth:
    def __init__(self, d):
        m = d['vel']['gate']
        self.tg, self.vg = d['vel']['t'][m], d['vel']['v'][m]
        o = np.argsort(self.tg)
        self.tg, self.vg = self.tg[o], self.vg[o]
        self.v_interp = M.true_speed_fn(d)
        g = np.arange(self.tg[0], self.tg[-1], 0.05)          # cumulative true distance, gaps bridged
        vv = self.v_interp(g)
        ok = ~np.isnan(vv)
        vf = np.interp(g, g[ok], vv[ok])
        self.g = g
        self.cum = np.r_[0.0, np.cumsum(0.5 * (vf[1:] + vf[:-1]) * 0.05)]
        self.cok = np.r_[0.0, np.cumsum(ok[1:].astype(float))]

    def dist(self, ta, tb):
        return float(np.interp(tb, self.g, self.cum) - np.interp(ta, self.g, self.cum))

    def coverage(self, ta, tb):
        if ta < self.g[0] or tb > self.g[-1]:
            return 0.0
        return float((np.interp(tb, self.g, self.cok) - np.interp(ta, self.g, self.cok)) / max((tb - ta) / 0.05, 1))

    def at(self, tq):
        j, dt = C.nearest(np.asarray(tq, float), self.tg)
        return np.where(dt <= 0.05, self.vg[j], np.nan)


def run_branch(est, last, tl, kl, vl, i0, t_d, T, truth, sigma, mmode):
    """Dropout branch from the snapshot (est, last) taken right after event i0-1 at t_d."""
    last = [None if L is None else tuple(L) for L in last]
    s0, pss0 = est.s, est.P[4][4]
    t_end, t_re = t_d + T, t_d + T + REACQ_S
    res = {}
    ts, vs = [], []
    done_end = False
    zoh0 = bogie_meas(last, t_d)
    i = i0
    while i < len(tl):
        t = tl[i]
        if not done_end and t >= t_end:
            est.predict_to(t_end)
            D = truth.dist(t_d, t_end)
            res['end'] = dict(err=est.s - s0 - D, D=D, P=est.P[4][4] - pss0,
                              v_err=est.v - truth.v_interp(np.array([t_end]))[0], Pvv=est.P[0][0],
                              cv_err=(zoh0[0] if zoh0 else 0.0) * T - D, g_tr=est.g_tr, g_br=est.g_br)
            done_end = True
        if t >= t_re:
            est.predict_to(t_re)
            res['reacq'] = dict(err=est.s - s0 - truth.dist(t_d, t_re))
            break
        k = kl[i]
        if k == 2:
            est.model.add_cmd(t, int(vl[i]))
            est.predict_to(t)
        elif t < t_end:
            est.predict_to(t)                   # wheel sample blanked
        else:
            L = last[k]
            last[k] = (t, vl[i], L[0] if L else None, L[1] if L else None)
            est.predict_to(t)
            zm = filter_meas(last, t, mmode)
            if zm is not None:
                est.update(meas(zm[0], sigma, zm[1]))
        if t < t_end:
            ts.append(t)
            vs.append(est.v)
        i += 1
    if 'reacq' not in res:
        return None
    vt = truth.at(np.array(ts))
    ok = ~np.isnan(vt)
    e = np.array(vs)[ok] - vt[ok]
    res['win_sse'] = float(np.sum(e * e))
    res['win_n'] = int(ok.sum())
    return res


def run_bag(args):
    b, cfg, sets, dropout = args
    d = M.load(b)
    p = M.params(**cfg.get('params', {}))
    C.set_params(p, sets)
    route = M.routes()[d['direction']]
    sigma = cfg.get('sigma', p.meas_sigma)
    mmode = cfg.get('meas', 'lin')
    truth = Truth(d)
    t_ev, k_ev, v_ev = events(d)
    s_ev = M.true_s_fn(d)(t_ev)
    tl, kl, vl, sl = t_ev.tolist(), k_ev.tolist(), v_ev.tolist(), s_ev.tolist()
    est = SpeedEstimator(p, TractionModel(p), route.grade_at, route.curv_at, 0.0)
    est.route_offset = -1e9
    last = [None, None]
    n = len(tl)
    out_t = np.empty(n)
    out_ekf = np.full(n, np.nan)
    out_zoh = np.full(n, np.nan)
    out_lin = np.full(n, np.nan)
    out_kind = np.array(kl)
    out_mode = np.zeros(n, np.int8)
    nis, innov = [], []
    n_clip = n_upd = n_resync = 0
    t_align, t_last_align, started, t_start = None, -1e18, False, None
    drop_plan, drops = [], []
    for i in range(n):
        t, k = tl[i], kl[i]
        out_t[i] = t
        if k == 2:
            est.model.add_cmd(t, int(vl[i]))
        else:
            L = last[k]
            last[k] = (t, vl[i], L[0] if L else None, L[1] if L else None)
        zm = bogie_meas(last, t)
        if not started:
            if zm is None:
                continue
            est.reset(t, zm[0])
            started, t_start = True, t
        if not math.isnan(sl[i]) and t - t_last_align >= REALIGN_S:
            est.route_offset = sl[i] - est.s
            t_last_align = t
            if t_align is None:
                t_align = t
                if dropout:
                    tt = t + WARM_S
                    while tt + max(T_DROP) + REACQ_S < tl[-1] - 1.0:
                        vt0 = truth.v_interp(np.array([tt]))[0]
                        if not np.isnan(vt0) and vt0 > 3.0:
                            drop_plan.append(tt)
                        tt += DROP_EVERY
        out = est.predict_to(t)
        if k != 2 and zm is not None:
            zf = filter_meas(last, t, mmode)
            info = est.update(meas(zf[0], sigma, zf[1]))
            if info.get('used'):
                n_upd += 1
                n_clip += bool(info.get('clipped'))
                n_resync += bool(info.get('resync'))
                if zm[1] == 'both' and est.v > 1.0 and t - t_start > 10.0:
                    nis.append(info['nis'])
                    innov.append(info['innov'])
        out_ekf[i] = est.v
        out_zoh[i] = zm[0] if zm is not None else np.nan
        out_lin[i] = bogie_lin(last, t)
        out_mode[i] = MODE.get(out.mode, 0) if out is not None else 0
        while drop_plan and t >= drop_plan[0]:          # dropout snapshots (the main run continues)
            drop_plan.pop(0)
            for T in T_DROP:
                if truth.coverage(t, t + T + REACQ_S) < 0.9:
                    continue
                r = run_branch(copy.deepcopy(est), last, tl, kl, vl, i + 1, t, T, truth, sigma, mmode)
                if r is not None:
                    r['T'] = T
                    drops.append(r)
    vt = truth.at(out_t)
    valid = (~np.isnan(vt)) & (out_t > (t_start if t_start is not None else np.inf) + 10.0) & ~np.isnan(out_zoh)
    return dict(bag=b, vt=vt[valid].astype(np.float32), ekf=out_ekf[valid].astype(np.float32),
                zoh=out_zoh[valid].astype(np.float32), lin=out_lin[valid].astype(np.float32),
                kind=out_kind[valid].astype(np.int8), mode=out_mode[valid], nis=np.array(nis, np.float32),
                innov=np.array(innov, np.float32), n_upd=n_upd, n_clip=n_clip, n_resync=n_resync, drops=drops)


def rmse(x):
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    return float(np.sqrt(np.mean(x * x))) if len(x) else float('nan')


def summarize(res, name):
    cat = lambda k: np.concatenate([r[k] for r in res])        # noqa: E731
    vt, ekf, zoh, lin, kind, mode = cat('vt'), cat('ekf'), cat('zoh'), cat('lin'), cat('kind'), cat('mode')
    lines = []
    glitch = np.abs(zoh - vt) > 0.5          # GNSS / wheel glitches (common exclusion)
    for lab, m in [('all', np.ones(len(vt), bool)), ('all-noglitch', ~glitch), ('moving-noglitch', (vt > 0.5) & ~glitch),
                   ('moving', vt > 0.5), ('traction', mode == 1), ('brake', mode == 2), ('hold8', mode == 4)]:
        if not m.any():
            continue
        lines.append('%-18s %-15s N=%8d  RMSE ekf %.4f  zoh %.4f  lin %.4f | MAE ekf %.4f zoh %.4f lin %.4f' % (
            name, lab, m.sum(), rmse(ekf[m] - vt[m]), rmse(zoh[m] - vt[m]), rmse(lin[m] - vt[m]),
            np.mean(np.abs(ekf[m] - vt[m])), np.mean(np.abs(zoh[m] - vt[m])), np.mean(np.abs(lin[m] - vt[m]))))
    nis = cat('nis')
    nupd = sum(r['n_upd'] for r in res)
    ac = [np.corrcoef(r['innov'][1:], r['innov'][:-1])[0, 1] for r in res if len(r['innov']) > 3]
    lines.append('%-18s NIS mean %.2f median %.2f P(NIS>9) %.4f | clipped %.5f resync %d / %d updates | innov lag-1 autocorr %.2f' % (
        name, np.mean(nis), np.median(nis), np.mean(nis > 9), sum(r['n_clip'] for r in res) / max(nupd, 1),
        sum(r['n_resync'] for r in res), nupd, np.nanmean(ac) if ac else np.nan))
    D = [x for r in res for x in r['drops']]
    for T in T_DROP:
        R = [x for x in D if x['T'] == T]
        if not R:
            continue
        err = np.array([x['end']['err'] for x in R])
        Dd = np.array([x['end']['D'] for x in R])
        sig = np.sqrt(np.maximum(np.array([x['end']['P'] for x in R]), 0) + (0.003 * Dd) ** 2)
        cv = np.array([x['end']['cv_err'] for x in R])
        er = np.array([x['reacq']['err'] for x in R])
        ve = np.array([x['end']['v_err'] for x in R])
        wr = math.sqrt(sum(x['win_sse'] for x in R) / max(sum(x['win_n'] for x in R), 1))
        z = err / sig
        lines.append('%-18s drop T=%2.0fs N=%4d | d err MAE %6.2f m p90 %6.2f rel %5.2f%% (bias %+.2f) | CV-baseline MAE %6.2f | '
                     'v RMSE win %.3f end %.3f | |z|<1 %.2f <2 %.2f rms z %.2f | after reacq MAE %6.2f' % (
                         name, T, len(R), np.mean(np.abs(err)), np.percentile(np.abs(err), 90),
                         100 * np.sum(np.abs(err)) / np.sum(Dd), np.mean(err), np.mean(np.abs(cv)), wr, rmse(ve),
                         np.mean(np.abs(z) < 1), np.mean(np.abs(z) < 2), rmse(z), np.mean(np.abs(er))))
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', nargs='*', default=['ref', 'ref_zoh', 'bias_off', 'gain_off', 'no_scorr'], choices=list(CFGS))
    ap.add_argument('--bags', default='')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--no-dropout', action='store_true', help='skip the dropout branches (faster)')
    ap.add_argument('--set', nargs='*', default=[], help='Params overrides for every configuration')
    a = ap.parse_args()
    B = M.bag_list(a.bags)
    od = C.out_dir('model_closedloop')
    for name in a.cfg:
        t0 = time.time()
        with ProcessPoolExecutor(max(a.jobs, 1)) as ex:
            res = list(ex.map(run_bag, [(b, CFGS[name], a.set, not a.no_dropout) for b in B]))
        with open(os.path.join(od, 'cl_%s.pkl' % name), 'wb') as f:
            pickle.dump(res, f)
        print('\n'.join(summarize(res, name)), '  (%d bags, %.0f s)' % (len(B), time.time() - t0), flush=True)


if __name__ == '__main__':
    main()
