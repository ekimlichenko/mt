"""Wheel scale K per vehicle, wheel noise and the thresholds of WheelFusion (Params: wheels section).

Reference: GNSS master speed |(vx, vy)| at status-2 fixes (tools/evaluate.load_ref, clean=True:
stamp glitches and frozen speeds removed).  Wheels: front/rear header-sorted, repeated stamps
(dt <= 0.02 s) dropped, interpolated to the reference stamps (nearest sample within 0.25 s).

K (analysis_notes F09): per bag the least-squares ratio  K = sum(w*g) / sum(g*g)  (w = mean
bogie km/h, g = GNSS m/s) on steady running: g > 3 m/s, |a_gnss(1 s)| < 0.1 m/s^2, |F-R| < 0.15
m/s.  Per vehicle: median over bags (bags > OUTLIER_K away from it are reported and left out);
unknown vehicle: median over all bags.  CV: parity split and leave-one-vehicle-day-out (bag_index
folds); the fold values are written to fold_params/fold_<k>.yaml for
tools/run_cv.py --fold-params.
Result (eval bags): 30618 3.597 (49 bags, -> Params wheel_k_30618), all 3.596 (-> wheel_k 3.595,
unknown vehicle), 30639 3.586 (8 bags of 2026-08-26 at 3.584-3.590; the 2 bags of 2026-05-05 at
3.622 / 3.633 are left out as outliers - the K of this vehicle changed between the two days).
wheel_k_30639 = 3.59 is the end-to-end choice of cv_sweep.py --param wheel_k_30639 (see
wheel_k_sweep.md).

Noise and thresholds (with the per-vehicle K of the current Params):
  residual w/K - g (both bogies / one bogie; all, robust, cruise, stop)  -> meas_sigma(_single)
  |F - R| quantiles (absolute, relative at v > 10 km/h)                   -> agree_abs_mps, agree_rel
  wheel acceleration envelope over 1 s / 0.2 s on GNSS-consistent data    -> acc_max, dec_max_*, slip_*_flag
  genuine hard braking (wheels, both bogies and GNSS agree)               -> dec_max_consistent, slip_dec_flag
  header dt of a bogie stream                                             -> wheel_stale_s
  smallest positive raw value                                             -> standstill_v

    python tools/calibration/wheel_scale_noise.py [--bags eval|N|b1,b2] [--jobs 4]
"""
import argparse
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C
import evaluate as ev

OUTLIER_K = 0.015        # |K_bag - K_vehicle| beyond this: reported, left out of the vehicle median
V_MIN = 3.0
A_MAX = 0.1


def _interp_near(t, ts, vs, max_dt=0.25):
    j, dt = C.nearest(t, ts)
    return np.where(dt <= max_dt, np.interp(t, ts, vs), np.nan)


def one(args):
    bag, K = args
    D = C.load_cache(bag)
    out = {'bag': bag, 'vehicle': C.vehicle_of(bag)}
    # header dt and the smallest positive raw value do not need a reference
    dts, minpos = [], np.inf
    for top in (C.FT, C.RT):
        x = D.get(top) or {}
        if len(x.get('t_hdr', [])) < 10:
            continue
        t, v = C.dedup_stream(x)
        dts.append(np.diff(t))
        pos = np.asarray(x['v'], float)
        pos = pos[pos > 0]
        if len(pos):
            minpos = min(minpos, float(pos.min()))
    out['dt'] = np.concatenate(dts) if dts else np.zeros(0)
    out['minpos_kmh'] = minpos
    ref = ev.load_ref(bag, clean=True, D=D)
    if ref is None or len(ref['tv']) < 600 or C.FT not in D or C.RT not in D:
        return out
    tF, vF = C.dedup_stream(D[C.FT])
    tR, vR = C.dedup_stream(D[C.RT])
    t, g = ref['tv'], ref['v']
    wF = _interp_near(t, tF, vF)
    wR = _interp_near(t, tR, vR)
    both = np.isfinite(wF) & np.isfinite(wR)
    w = 0.5 * (wF + wR)
    # 1 s centred accelerations (GNSS and wheel mean), valid only without reference gaps
    gp = np.interp(t + 0.5, t, g); gm = np.interp(t - 0.5, t, g)
    jl, dl = C.nearest(t - 0.5, t); jr, dr = C.nearest(t + 0.5, t)
    agood = (dl < 0.15) & (dr < 0.15)
    a_g = (gp - gm) / 1.0
    fr = np.abs(wF - wR) / K
    steady = both & agood & (g > V_MIN) & (np.abs(a_g) < A_MAX) & (fr < 0.15) & (np.abs(w / np.maximum(g, 1e-3) - 3.6) < 0.18)
    out['n_steady'] = int(steady.sum())
    if steady.sum() > 200:
        ws, gs = w[steady], g[steady]
        out['K'] = float(np.sum(ws * gs) / np.sum(gs * gs))
        out['K_front'] = float(np.sum(wF[steady] * gs) / np.sum(gs * gs))
        out['K_rear'] = float(np.sum(wR[steady] * gs) / np.sum(gs * gs))
    # residuals with the configured vehicle K
    e = w / K - g
    eF = wF / K - g
    ok = both & (np.abs(e) < 1.0)
    out['n_ref'] = int(both.sum())
    out['n_gross'] = int((both & (np.abs(e) >= 1.0)).sum())
    out['e'] = e[ok].astype(np.float32)
    out['e_single'] = eF[np.isfinite(eF) & (np.abs(eF) < 1.0)].astype(np.float32)
    out['e_cruise'] = e[ok & agood & (g > V_MIN) & (np.abs(a_g) < A_MAX)].astype(np.float32)
    out['e_stop'] = e[ok & (g < 0.05)].astype(np.float32)
    # front/rear disagreement on GNSS-consistent samples
    cons = both & (np.abs(e) < 0.3) & (g > 0.5)
    out['fr_abs'] = fr[cons].astype(np.float32)
    fast = cons & (np.maximum(wF, wR) > 10.0)
    with np.errstate(invalid='ignore', divide='ignore'):
        out['fr_rel'] = (np.abs(wF - wR) / np.maximum(wF, wR))[fast].astype(np.float32)
    # acceleration envelope of the wheel mean (1 s and 0.2 s), GNSS-consistent windows
    for win, key in ((1.0, 'acc1'), (0.2, 'acc02')):
        tp = t + win / 2; tm = t - win / 2
        wp = 0.5 * (_interp_near(tp, tF, vF) + _interp_near(tp, tR, vR))
        wm = 0.5 * (_interp_near(tm, tF, vF) + _interp_near(tm, tR, vR))
        a_w = (wp - wm) / K / win
        gp2 = np.interp(tp, t, g); gm2 = np.interp(tm, t, g)
        j1, d1 = C.nearest(tp, t); j2, d2 = C.nearest(tm, t)
        good = np.isfinite(a_w) & (d1 < 0.15) & (d2 < 0.15) & (np.abs(wp / K - gp2) < 0.3) & (np.abs(wm / K - gm2) < 0.3)
        out[key] = a_w[good].astype(np.float32)
        if key == 'acc1':
            a_g1 = (gp2 - gm2) / win
            genuine = good & (np.abs(a_w - a_g1) < 0.3) & (fr < 0.15)
            out['acc1_genuine_min'] = float(a_w[genuine].min()) if genuine.any() else np.nan
    return out


def q(x, ps):
    return ' '.join('p%g=%.4f' % (p, np.percentile(x, p)) for p in ps)


def robust(x):
    return 1.4826 * np.median(np.abs(x - np.median(x)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='eval', help="'' = unique bags, 'eval' = eval_ok bags (default), N, or a comma list")
    ap.add_argument('--jobs', type=int, default=4)
    a = ap.parse_args()
    bags = C.select_bags(a.bags, C.unique_bags())
    out = C.out_dir('wheel_scale_noise')
    Kcfg = {b: C.params_for(b).k_for_vehicle() for b in bags}
    jobs = [(b, Kcfg[b]) for b in bags]
    if a.jobs > 1:
        with ProcessPoolExecutor(a.jobs) as ex:
            res = list(ex.map(one, jobs))
    else:
        res = [one(j) for j in jobs]
    idx = C.load_index()['bags'] if os.path.exists(C.INDEX) else {}

    # ---------------------------------------------------------------- K
    rows = [r for r in res if 'K' in r]
    print('== wheel scale K (km/h per m/s), %d bags with >200 steady samples of %d' % (len(rows), len(res)))
    summary = {'K_bags': {r['bag']: round(r['K'], 5) for r in rows}}
    Kveh = {}
    for veh in sorted(set(r['vehicle'] for r in rows)):
        Ks = np.array([r['K'] for r in rows if r['vehicle'] == veh])
        names = [r['bag'] for r in rows if r['vehicle'] == veh]
        med = np.median(Ks)
        inl = np.abs(Ks - med) <= OUTLIER_K
        Kveh[veh] = float(np.median(Ks[inl]))
        print('  %s: n=%d median %.4f sd %.4f | without %d outliers: median %.4f sd %.4f  -> wheel_k_%s' % (
            veh, len(Ks), med, Ks.std(), (~inl).sum(), Kveh[veh], Ks[inl].std(), veh))
        for b, k in zip(names, Ks):
            if abs(k - med) > OUTLIER_K:
                print('      outlier %s K=%.4f' % (b, k))
        # parity CV
        o = np.argsort(names)
        Ks_s, inl_s = Ks[o], inl[o]
        halves = [np.median(Ks_s[i::2][inl_s[i::2]]) for i in (0, 1) if inl_s[i::2].any()]
        print('      parity CV halves: %s' % ' / '.join('%.4f' % h for h in halves))
        fr_ratio = np.array([r['K_front'] / r['K_rear'] for r in rows if r['vehicle'] == veh])
        print('      front/rear K ratio median %.4f' % np.median(fr_ratio))
    allK = np.array([r['K'] for r in rows])
    Kall = float(np.median(allK[np.abs(allK - np.median(allK)) <= OUTLIER_K]))
    nveh = {v: sum(1 for r in rows if r['vehicle'] == v) for v in Kveh}
    Kw = sum(Kveh[v] * nveh[v] for v in Kveh) / max(sum(nveh.values()), 1)
    print('  all bags: median %.4f; bag-weighted mean of the vehicle values %.4f  -> wheel_k (unknown vehicle)' % (Kall, Kw))
    summary.update(K_vehicle=Kveh, K_all=Kall, K_weighted=Kw)
    # leave-one-vehicle-day-out folds -> fold_params/fold_<k>.yaml
    folds = sorted(set(idx[r['bag']]['fold'] for r in rows if r['bag'] in idx))
    if folds:
        fd = os.path.join(out, 'fold_params')
        os.makedirs(fd, exist_ok=True)
        print('  leave-one-vehicle-day-out (out-of-fold K per vehicle):')
        for k in folds:
            tr = [r for r in rows if r['bag'] in idx and idx[r['bag']]['fold'] != k]
            vals = {}
            for veh in ('30618', '30639'):
                Ks = np.array([r['K'] for r in tr if r['vehicle'] == veh])
                if len(Ks):
                    vals['wheel_k_' + veh] = float(np.median(Ks[np.abs(Ks - np.median(Ks)) <= OUTLIER_K]))
            Ka = np.array([r['K'] for r in tr])
            vals['wheel_k'] = float(np.median(Ka[np.abs(Ka - np.median(Ka)) <= OUTLIER_K]))
            with open(os.path.join(fd, 'fold_%d.yaml' % k), 'w') as f:
                for kk, vv in vals.items():
                    f.write('%s: %.4f\n' % (kk, vv))
            print('      fold %d: %s' % (k, '  '.join('%s=%.4f' % kv for kv in vals.items())))
        print('      written %s/fold_<k>.yaml (tools/run_cv.py --fold-params %s)' % (fd, fd))

    # ---------------------------------------------------------------- noise
    cat = lambda key: np.concatenate([r[key] for r in res if key in r and len(r[key])]) if any(key in r for r in res) else np.zeros(0)
    e, es, ec, e0 = cat('e'), cat('e_single'), cat('e_cruise'), cat('e_stop')
    ng = sum(r.get('n_gross', 0) for r in res); nref = sum(r.get('n_ref', 0) for r in res)
    print('\n== residual wheel - GNSS speed [m/s] (K of the current Params), %d samples, |e|>=1 dropped: %d (%.3f%%)' % (
        len(e), ng, 100.0 * ng / max(nref, 1)))
    if len(e):
        print('  both bogies: std %.4f robust %.4f bias %+.4f   -> meas_sigma' % (e.std(), robust(e), e.mean()))
        print('  one bogie  : std %.4f robust %.4f            -> meas_sigma_single' % (es.std(), robust(es)))
        print('  cruise (v>3, |a|<0.1): std %.4f   stop (v<0.05): std %.4f' % (ec.std(), e0.std() if len(e0) else np.nan))
    fa, fr = cat('fr_abs'), cat('fr_rel')
    if len(fa):
        print('\n== |F - R| on GNSS-consistent samples: %s m/s  -> agree_abs_mps' % q(fa, (99, 99.9, 99.99)))
        print('   relative at v > 10 km/h: %s  -> agree_rel' % q(fr, (99, 99.9)))
    a1, a02 = cat('acc1'), cat('acc02')
    if len(a1):
        print('\n== wheel acceleration on GNSS-consistent windows [m/s^2]')
        print('  1 s  : min %.2f max %.2f  %s' % (a1.min(), a1.max(), q(a1, (0.01, 99.99))))
        print('  0.2 s: min %.2f max %.2f  %s' % (a02.min(), a02.max(), q(a02, (0.01, 99.99))))
        gen = np.array([r['acc1_genuine_min'] for r in res if np.isfinite(r.get('acc1_genuine_min', np.nan))])
        print('  genuine hard braking (wheels = GNSS, bogies agree), most negative 1 s accel: %.2f (bag min p10 %.2f)' % (
            gen.min(), np.percentile(gen, 10)))
        print('  -> acc_max above the traction max, dec_max_slip / slip_acc_flag above the clean envelope,')
        print('     dec_max_consistent / slip_dec_flag below the genuine braking minimum')
    dt = cat('dt')
    if len(dt):
        print('\n== bogie header dt [s]: %s max %.3f; gaps >0.35 s: %d, >0.5 s: %d  -> wheel_stale_s' % (
            q(dt, (50, 95, 99.9)), dt.max(), (dt > 0.35).sum(), (dt > 0.5).sum()))
    mp = min(r['minpos_kmh'] for r in res)
    print('== smallest positive raw wheel value %.3f km/h = %.3f m/s  -> standstill_v' % (mp, mp / 3.6))
    summary.update(
        resid_std=float(e.std()) if len(e) else None, resid_robust=float(robust(e)) if len(e) else None,
        resid_single_std=float(es.std()) if len(es) else None, resid_cruise_std=float(ec.std()) if len(ec) else None,
        fr_abs_p999=float(np.percentile(fa, 99.9)) if len(fa) else None,
        fr_rel_p999=float(np.percentile(fr, 99.9)) if len(fr) else None,
        acc1_min=float(a1.min()) if len(a1) else None, acc1_max=float(a1.max()) if len(a1) else None,
        acc02_p001=float(np.percentile(a02, 0.01)) if len(a02) else None,
        acc02_p9999=float(np.percentile(a02, 99.99)) if len(a02) else None,
        dt_max=float(dt.max()) if len(dt) else None, minpos_kmh=mp, bags=bags)
    C.save_json(os.path.join(out, 'summary.json'), summary)
    print('\nwritten', os.path.join(out, 'summary.json'))


if __name__ == '__main__':
    main()
