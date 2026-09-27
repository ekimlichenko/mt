"""Real-data validation of TimeGuard + WheelFusion against the RTK GNSS speed (threshold variants).

For every selected unique bag the front/rear/cmd messages are replayed in RECORD order through
core.preprocess.TimeGuard and core.wheel_fusion.WheelFusion (v_pred = previous fused value).
At each wheel event the output (fused speed; previous output for late / no-bogie events) is
compared with |(vx, vy)| of the master GNSS velocity at the nearest stamp within 0.05 s (only
samples with a status-2 master fix within 0.05 s).  The naive baseline B6 (mean of the fresh
bogies, 0.5 s stale, 0.2 s extrapolation, header-sorted, non-causal) is scored at the same stamps.

Variants = Params overrides (VARIANTS below).  W is the shipped config; W_pre restores the two
values used before this validation (stuck_zero_other_mps 0.5, slip_dec_flag = slip_acc_flag 2.5);
the calibrated values stuck_zero_other_mps 0.2 / slip_dec_flag 6.0 were adopted from it
(variant W_rec of the original run: median RMSE 0.0310 vs 0.0313, dropout-bag max error 0.47 -> 0.22 m/s).
W_late005 checks time_late_tol_s (0.05 s drops real samples: median RMSE 0.049 vs 0.031).

    python tools/calibration/wheels_validate.py [--variants W,B6,W_pre] [--bags N|b1,b2] [--jobs 4]
    python tools/calibration/wheels_report.py            # summary tables of the pickle
Heavy: all 97 unique bags x 12 variants ~ 2M wheel events per variant (tens of CPU minutes).
"""
import argparse
import os
import pickle
import time
import warnings
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C
from tram_backup_odometry.core.preprocess import TimeGuard
from tram_backup_odometry.core.wheel_fusion import WheelFusion

# robustness bag lists (analysis_notes PARAMS robustness_test_bags)
DROPOUT = ['30639_3b3d9eb8', '30639_4285f2bc', '30639_927002c2', '30639_d927f360', '30639_c31df386',
           '30639_44226bde', '30639_584b6e32', '30639_9f0b519f']
SLIP = ['30618_2050d396', '30618_33bec73f', '30639_50956d6e', '30618_68d1748a']
STAMP = ['30618_2255aade', '30618_40ffd323', '30639_9c362687', '30618_28538acf', '30618_af7496f0']

VARIANTS = {
    'W': {},                                                   # shipped config defaults
    'W_sz050': {'stuck_zero_other_mps': 0.5},                  # pre-calibration stuck-zero threshold
    'W_dec25': {'slip_dec_flag': 2.5},                         # pre-calibration slide threshold
    'W_pre': {'stuck_zero_other_mps': 0.5, 'slip_dec_flag': 2.5},
    'W_nolock': {'lock_zero_dec': 0.0},
    'W_noalign': {'wheel_align_other': False},
    'W_noanchor': {'slip_anchor_s': -1.0},
    'W_nospike': {'spike_kmh': 1e9},
    'W_late005': {'time_late_tol_s': 0.05},
    'W_late030': {'time_late_tol_s': 0.30},
    'W_glitch05': {'time_glitch_s': 0.5},
    # causal B6: TimeGuard + record order, mean of fresh bogies with own extrapolation, no rules
    'W_naive': {'agree_abs_mps': 1e9, 'stuck_zero_other_mps': 1e9, 'frozen_s': 1e9,
                'spike_kmh': 1e9, 'wheel_align_other': False, 'lock_zero_dec': 0.0},
}


def events(D):
    """All vehicle messages in record order: list of (t_rec, t_hdr, stream, value)."""
    ev = []
    for nm, k, key in (('front', C.FT, 'v'), ('rear', C.RT, 'v'), ('cmd', C.CT, 'pos')):
        x = D.get(k, {})
        tr = np.asarray(x.get('t_rec', []), float)
        th = np.asarray(x.get('t_hdr', []), float)
        vv = np.asarray(x.get(key, []), float)
        for i in range(len(tr)):
            ev.append((float(tr[i]), float(th[i]), nm, float(vv[i])))
    ev.sort(key=lambda e: e[0])
    return ev


def b6_inputs(D, K, extrap=True, stale=0.5):
    """B6 baseline inputs() from analysis_notes (header-sorted union, mean of fresh bogies)."""
    Fd = D[C.FT]
    Rd = D.get(C.RT, {'t_hdr': np.array([]), 'v': np.array([])})
    Cd = D.get(C.CT, {'t_hdr': np.array([])})
    t = np.sort(np.r_[Fd['t_hdr'], Rd['t_hdr'], Cd['t_hdr']], kind='stable')
    vs = []
    for S in (Fd, Rd):
        T = np.asarray(S['t_hdr'], float)
        V = np.asarray(S['v'], float)
        if len(T) == 0:
            vs.append(np.full(len(t), np.nan))
            continue
        o = np.argsort(T, kind='stable')
        T, V = T[o], V[o]
        i = np.searchsorted(T, t, side='right') - 1
        ic = np.clip(i, 0, None)
        v = V[ic].astype(float)
        if extrap:
            ip = np.clip(ic - 1, 0, None)
            dtp = T[ic] - T[ip]
            sl = np.where(dtp > 0.02, (V[ic] - V[ip]) / np.maximum(dtp, 0.02), 0)
            v = np.maximum(v + np.clip(t - T[ic], 0, 0.2) * sl, 0)
        v[(i < 0) | ((t - T[ic]) > stale)] = np.nan
        vs.append(v)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)      # all-NaN rows: no fresh bogie
        vw = np.nanmean(np.c_[vs[0], vs[1]], axis=1)
    bad = np.isnan(vw)
    if bad.any():
        idx = np.where(~bad, np.arange(len(vw)), 0)
        np.maximum.accumulate(idx, out=idx)
        vw = np.nan_to_num(vw[idx])
    return t, vw / K


def replay(D, p):
    """Replay vehicle messages in record order; returns per-wheel-event arrays and diagnostics."""
    K = p.k_for_vehicle()
    tg = TimeGuard(p)
    wf = WheelFusion(p, K)
    out_t, out_v, out_flag, out_state, out_slip = [], [], [], [], []
    prev = None
    n_frozen = n_stuck = 0
    fb, rb = wf._b['front'], wf._b['rear']
    for tr, th, s, val in events(D):
        t_eff, flag = tg.correct(s, th)
        if s == 'cmd' or flag == 3:
            continue
        state, slip = 'late', False
        if flag in (0, 1):
            wf.add_sample(s, t_eff, val)
            m = wf.measure(t_eff, prev if prev is not None else float('nan'))
            n_frozen += fb.frozen or rb.frozen
            n_stuck += fb.stuck_zero or rb.stuck_zero
            if m is not None:
                prev = m.v
                state, slip = m.state, m.slip
            else:
                state = 'none'
        out_t.append(th)
        out_v.append(prev if prev is not None else 0.0)
        out_flag.append(flag)
        out_state.append(state)
        out_slip.append(slip)
    h = wf.health()
    diag = {
        'tg_counts': list(tg.counts), 'tg_resyncs': tg.resyncs,
        'spikes': h['front']['spikes'] + h['rear']['spikes'],
        'rejected': h['front']['rejected'] + h['rear']['rejected'],
        'n_disagree': h['n_disagree'], 'n_uncertain': h['n_uncertain'], 'n_accel_flag': h['n_accel_flag'],
        'n_lock': h['n_lock'], 'n_frozen': n_frozen, 'n_stuck_zero': n_stuck,
    }
    return dict(t=np.array(out_t), v=np.array(out_v), flag=np.array(out_flag),
                state=np.array(out_state), slip=np.array(out_slip, bool), diag=diag)


def score(t, v, ref, rover=None):
    tv, vv = ref
    j, dt = C.nearest(t, tv)
    ok = dt <= 0.05
    e = v[ok] - vv[j[ok]]
    res = {'n': int(ok.sum())}
    if ok.sum() == 0:
        return res
    res['rmse'] = float(np.sqrt(np.mean(e ** 2)))
    res['max'] = float(np.max(np.abs(e)))
    res['bias'] = float(np.mean(e))
    if rover is not None:
        tr_, vr_ = rover
        jr, dtr = C.nearest(tv[j[ok]], tr_)
        # master/rover-consistent samples; samples without a rover counterpart are kept
        clean = ((dtr <= 0.05) & (np.abs(vr_[jr] - vv[j[ok]]) <= 0.3)) | (dtr > 0.05)
        ec = e[clean]
        res['n_clean'] = int(clean.sum())
        res['rmse_clean'] = float(np.sqrt(np.mean(ec ** 2))) if len(ec) else np.nan
        res['max_clean'] = float(np.max(np.abs(ec))) if len(ec) else np.nan
    return res


def rover_vel(D):
    rv = D.get(C.RV, {})
    if len(rv.get('t_hdr', [])) == 0:
        return None
    t = np.asarray(rv['t_hdr'], float)
    v = np.hypot(rv['vx'], rv['vy'])
    o = np.argsort(t, kind='stable')
    return t[o], v[o]


def run_one(args):
    bag, variants, sets = args
    D = C.load_cache(bag)
    g = C.gnss_speed_ref(D)
    ref = (g[0], g[1]) if g is not None else None
    out = {'bag': bag, 'has_ref': ref is not None and len(ref[0]) > 0}
    rover = rover_vel(D)
    t0 = time.process_time()
    for name in variants:
        if name == 'B6':
            continue                               # evaluated at the stamps of variant W
        p = C.params_for(bag, sets, **VARIANTS[name])
        r = replay(D, p)
        out[name] = {'diag': r['diag'], 'n_events': len(r['t']),
                     'n_late': int((r['flag'] == 2).sum()), 'n_fwd': int((r['flag'] == 1).sum()),
                     'states': {s: int((r['state'] == s).sum()) for s in np.unique(r['state'])},
                     'n_slip': int(r['slip'].sum())}
        valid = r['flag'] != 2                     # late samples are not published
        if out['has_ref']:
            out[name].update(score(r['t'], r['v'], ref, rover))
            out[name].update({k + '_valid': v for k, v in score(r['t'][valid], r['v'][valid], ref).items()})
        if name == 'W' and 'B6' in variants:
            tb, vb = b6_inputs(D, p.k_for_vehicle())
            i = np.searchsorted(tb, r['t'], side='right') - 1
            vb_at = vb[np.clip(i, 0, len(vb) - 1)]
            out['B6'] = {'n_events': len(r['t'])}
            if out['has_ref']:
                out['B6'].update(score(r['t'], vb_at, ref, rover))
                out['B6'].update({k + '_valid': v for k, v in score(r['t'][valid], vb_at[valid], ref).items()})
                ns = valid & ~r['slip']                    # W not flagging slip
                for nm, vv in (('W', r['v']), ('B6', vb_at)):
                    out[nm].update({k + '_noslip': v for k, v in score(r['t'][ns], vv[ns], ref).items()})
                    sl = valid & r['slip']
                    out[nm].update({k + '_slip': v for k, v in score(r['t'][sl], vv[sl], ref).items()})
    out['cpu_s'] = time.process_time() - t0
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--variants', default=','.join(['W', 'B6'] + [k for k in VARIANTS if k != 'W']))
    ap.add_argument('--bags', default='', help="'' = all unique bags, N = first N, or a comma list")
    ap.add_argument('--set', nargs='*', default=[], help='key=value applied to every variant')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--out', default=None, help='default $CALIB_WORK/wheels/validate.pkl')
    a = ap.parse_args()
    variants = a.variants.split(',')
    unknown = [v for v in variants if v != 'B6' and v not in VARIANTS]
    if unknown:
        raise SystemExit('unknown variants %s; known: %s' % (unknown, ', '.join(VARIANTS)))
    bags = C.select_bags(a.bags, C.unique_bags())
    jobs = [(b, variants, a.set) for b in bags]
    if a.jobs > 1:
        with ProcessPoolExecutor(a.jobs) as ex:
            res = list(ex.map(run_one, jobs))
    else:
        res = [run_one(j) for j in jobs]
    fn = a.out or os.path.join(C.out_dir('wheels'), 'validate.pkl')
    with open(fn, 'wb') as f:
        pickle.dump(res, f)
    print('done %d bags, %.0f CPU s -> %s' % (len(res), sum(r['cpu_s'] for r in res), fn))


if __name__ == '__main__':
    main()
