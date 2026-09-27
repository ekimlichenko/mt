"""Consistency of the Odometry pose covariance with the real position error; what-if calibration of
pose_trn_sigma_scale and pose_sigma_cross_m (Params, output section).
Source: .work/scratch_main/cov_anatomy2.py (cleaned, what-if relative to the current Params).

Every selected bag (default: the eval_ok bags) is replayed ONCE through the full pipeline
(tram_backup_odometry.replay.run_bag, GNSS only for the first --gnss-limit s, Params built exactly
as tools/run_cv.py builds them: defaults <- --params yaml <- --set, vehicle_id from the bag name)
and matched to the status-2 master reference like tools/evaluate.py (nearest output stamp within
0.05 s, has_pose outputs only).  The horizontal error is split into the reported along / cross axes
(reported yaw) and normalised by the reported variances:
    m2 = e_along^2 / var_along + e_cross^2 / var_cross      (chi2, 2 dof when consistent)
    in95 = share of m2 <= 5.991 (0.95 when consistent), NEES = mean(min(m2, 1e4)) / 2 (1 when consistent)
per segment of the run: bridge (s_route < 0, before the pathgraph), map (0..L), tail (s_route > L).

The covariance does not feed back into the estimate, so the what-if needs no re-run.  The reported
variances are decomposed with the formulas of core/pipeline.py (Pipeline._output):
    TRN active (mode nav_trn*):  var_along = trn * scale^2 + offmap_along
    otherwise:                   var_along = sigma0^2 + var_s + offmap_along   (not scaled)
    var_cross = pose_sigma_cross_m^2 + offmap_cross
    offmap: tail  along (tail_sigma_frac^2 + tail_cross_frac^2) off^2, cross tail_cross_frac^2 off^2
            bridge along 0, cross bridge_cross_frac^2 off^2
and recombined for every (scale, cross) of --scales x --cross (the reconstruction of the current
values is checked).  Also printed: the anatomy of the on-map error (|cross| quantiles, TRN along
error vs the unscaled TRN std) and the bags with p_in95 < 0.7.

Result on the 60 eval bags (final code, 651731 matched samples, 17 s with 4 processes): the unscaled
TRN posterior (scale 1) is ~2x wider than the along-track error (rms e_along / std 0.45, robust 0.31);
on-map |cross| is 90 % within 0.35 m, but S2T runs sometimes follow the parallel track ~4 m aside
-> p95 1.04 m, p99 4.2 m, which pose_sigma_cross_m 0.3 cannot cover (on-map zc2 7.1).
    scale / cross   on-map in95  NEES   za2    per-bag p_in95 median / mean  n<0.7  NEES mean
    1.0 / 0.3       0.928        3.65   0.21   0.986 / 0.926                 5      5.30   (before)
    1.0 / 0.8       0.963        0.60   0.21   0.998 / 0.964                 1      0.90
    0.6 / 0.8       0.954        0.78   0.56   0.998 / 0.956                 2      1.05   (chosen)
    0.4 / 0.8       0.922        1.13   1.25   0.996 / 0.930                 4      1.34
Chosen: pose_trn_sigma_scale 0.6 (in95 closest to 0.95, NEES closest to 1; 1.0 is over-conservative
along track), pose_sigma_cross_m 0.8 (on-map zc2 1.00).

    python tools/calibration/covariance_consistency.py [--bags eval] [--jobs 4] [--set k=v ...]
           [--scales 1,0.8,0.7,0.6,0.5,0.4] [--cross 0.3,0.5,0.8,1.0] [--reuse]
Replay of the 60 eval bags ~1-2 min with 4 processes; --reuse re-analyses the saved samples
($CALIB_WORK/covariance/samples.npz) without replaying.
"""
import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C
import evaluate as ev
import run_cv

CHI2_95 = 5.991


def _cfg(a):
    return dict(params=a.params, fold_params=None, overrides=run_cv.parse_sets(a.set))


def one(args):
    """Replay one bag; per matched reference sample: errors, reported variances and the parts."""
    bag, cfg, gnss_limit, tol = args
    from tram_backup_odometry.replay import run_bag
    D = ev.load_cache(bag)
    p, _ = run_cv.build_params(bag, 0, cfg)
    est = run_bag(ev.cache_path(bag), p, data=D, gnss_limit_s=gnss_limit)
    ref = ev.load_ref(bag, 'master', False, D=D)
    if ref is None:
        return None
    # the same filtering / ordering / matching as evaluate.as_estimate + evaluate.evaluate
    t = np.asarray(est['t'], float)
    fin = np.isfinite(t)
    n = len(t)
    cols = {k: np.asarray(est.get(k, np.full(n, np.nan)))[fin]
            for k in ('x', 'y', 'z', 'yaw', 'var_along', 'var_cross', 'has_pose', 'mode', 'diag_s_route', 'diag_off_map_m')}
    t = t[fin]
    o = np.argsort(t, kind='stable')
    t = t[o]
    last = np.r_[t[1:] != t[:-1], True] if len(t) else np.zeros(0, bool)   # several outputs per stamp: keep the last
    t = t[last]
    cols = {k: v[o][last] for k, v in cols.items()}
    xyz = np.column_stack([cols[k].astype(float) for k in ('x', 'y', 'z')])
    pm = cols['has_pose'].astype(bool) & np.isfinite(xyz).all(axis=1)
    idx = np.flatnonzero(pm)
    if len(idx) == 0:
        return None
    j, dt = ev.nearest(ref['tp'], t[idx])
    ok = dt <= tol
    k = idx[j[ok]]
    d = xyz[k, :2] - ref['xyz'][ok, :2]
    yaw = cols['yaw'][k].astype(float)
    ea = d[:, 0] * np.cos(yaw) + d[:, 1] * np.sin(yaw)
    ec = -d[:, 0] * np.sin(yaw) + d[:, 1] * np.cos(yaw)
    sr = cols['diag_s_route'][k].astype(float)
    off = np.nan_to_num(cols['diag_off_map_m'][k].astype(float))
    # 0 bridge (s < 0), 1 map, 2 tail (s > L), 3 no pathgraph pose (fallback odometry, var_cross unset)
    seg = np.where(~np.isfinite(sr), 3, np.where(off > 0, np.where(sr < 0, 0, 2), 1)).astype(np.int8)
    trn = np.array([str(m).startswith('nav_trn') for m in cols['mode'][k]])
    va = cols['var_along'][k].astype(float)
    vc = cols['var_cross'][k].astype(float)
    vc = np.where(np.isfinite(vc), vc, 0.09)
    # off-map parts of the reported variances (pipeline._output)
    tail = seg == 2
    oa = np.where(tail, (p.tail_sigma_frac ** 2 + p.tail_cross_frac ** 2) * off ** 2, 0.0)
    oc = np.where(tail, p.tail_cross_frac ** 2 * off ** 2, np.where(seg == 0, p.bridge_cross_frac ** 2 * off ** 2, 0.0))
    base = va - oa                                       # TRN part * scale^2 (TRN) or sigma0^2 + var_s
    trn_part = np.where(trn, base / max(p.pose_trn_sigma_scale, 1e-9) ** 2, np.nan)
    return dict(bag=bag, dir=ref['dir'], ea=ea, ec=ec, va=va, vc=vc, seg=seg, trn=trn, off=off,
                base=base, trn_part=trn_part, oa=oa, oc=oc,
                scale0=p.pose_trn_sigma_scale, cross0=p.pose_sigma_cross_m)


def variances(r, scale, cross):
    va = np.where(r['trn'], r['trn_part'] * scale ** 2, r['base']) + r['oa']
    vc = np.where(r['seg'] == 3, 0.09, cross ** 2 + r['oc'])     # evaluate.py: unset var_cross -> 0.09
    return np.maximum(va, 1e-6), np.maximum(vc, 1e-6)


def m2_of(r, scale, cross):
    va, vc = variances(r, scale, cross)
    return r['ea'] ** 2 / va + r['ec'] ** 2 / vc, va, vc


def pooled(res, key):
    return np.concatenate([np.asarray(r[key]) for r in res])


def save(res, fn):
    keys = ('ea', 'ec', 'va', 'vc', 'seg', 'trn', 'off', 'base', 'trn_part', 'oa', 'oc')
    np.savez_compressed(fn, bags=np.array([r['bag'] for r in res]), dirs=np.array([str(r['dir']) for r in res]),
                        n=np.array([len(r['ea']) for r in res]), scale0=res[0]['scale0'], cross0=res[0]['cross0'],
                        **{k: pooled(res, k) for k in keys})


def load(fn):
    z = np.load(fn, allow_pickle=False)
    cut = np.r_[0, np.cumsum(z['n'])]
    keys = ('ea', 'ec', 'va', 'vc', 'seg', 'trn', 'off', 'base', 'trn_part', 'oa', 'oc')
    res = []
    for i, b in enumerate(z['bags']):
        r = {k: z[k][cut[i]:cut[i + 1]] for k in keys}
        r.update(bag=str(b), dir=str(z['dirs'][i]), scale0=float(z['scale0']), cross0=float(z['cross0']))
        res.append(r)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='eval', help="tools/run_cv.py selector (default eval = eval_ok bags) or N = first N eval bags")
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--params', default=None, help='params yaml (as run_cv.py --params)')
    ap.add_argument('--set', nargs='*', default=[], help='key=value Params overrides (as run_cv.py --set)')
    ap.add_argument('--gnss-limit', type=float, default=10.0)
    ap.add_argument('--scales', default='1.0,0.8,0.7,0.6,0.5,0.4', help='pose_trn_sigma_scale values (absolute)')
    ap.add_argument('--cross', default='0.3,0.5,0.8,1.0', help='pose_sigma_cross_m values (absolute)')
    ap.add_argument('--reuse', action='store_true', help='re-analyse the saved samples, no replay')
    a = ap.parse_args()
    od = C.out_dir('covariance')
    fn = os.path.join(od, 'samples.npz')
    t0 = time.time()
    if a.reuse:
        res = load(fn)
    else:
        index = C.load_index()
        if a.bags.isdigit():                                  # smoke: first N eval bags
            bags = run_cv.select_bags('eval', index)[:int(a.bags)]
        else:
            bags = run_cv.select_bags(a.bags, index)
        cfg = _cfg(a)
        jobs = [(b, cfg, a.gnss_limit, 0.05) for b in bags]
        if a.jobs > 1:
            with ProcessPoolExecutor(a.jobs) as ex:
                res = [r for r in ex.map(one, jobs) if r is not None]
        else:
            res = [r for r in map(one, jobs) if r is not None]
        save(res, fn)
    s0, c0 = res[0]['scale0'], res[0]['cross0']
    print('%d bags, %d matched samples (%.0f s); current pose_trn_sigma_scale %.2f pose_sigma_cross_m %.2f' % (
        len(res), sum(len(r['ea']) for r in res), time.time() - t0, s0, c0))
    # reconstruction check: the what-if at the current values must give the reported variances
    err = max(float(np.nanmax(np.abs(variances(r, s0, c0)[0] - np.maximum(r['va'], 1e-6)) / np.maximum(r['va'], 1e-6)))
              for r in res if len(r['va']))
    errc = max(float(np.nanmax(np.abs(variances(r, s0, c0)[1] - np.maximum(r['vc'], 1e-6)) / np.maximum(r['vc'], 1e-6)))
               for r in res if len(r['vc']))
    print('variance reconstruction at the current values: max rel. error along %.1e, cross %.1e' % (err, errc))

    ea, ec, seg, trn = pooled(res, 'ea'), pooled(res, 'ec'), pooled(res, 'seg'), pooled(res, 'trn').astype(bool)
    on = seg == 1
    print('\n-- anatomy of the on-map error (%d samples, %.1f%% with TRN active)' % (on.sum(), 100 * np.mean(trn[on])))
    aco = np.abs(ec[on])
    print('   |cross| p50 %.2f p90 %.2f p95 %.2f p99 %.2f max %.1f m; share within 0.35 m %.3f' % (
        *np.percentile(aco, (50, 90, 95, 99)), aco.max(), np.mean(aco <= 0.35)))
    for d in ('T2S', 'S2T'):
        m = np.concatenate([np.full(len(r['ea']), r['dir'] == d) for r in res]) & on
        if m.any():
            print('   %s |cross| p90 %.2f p95 %.2f p99 %.2f m (n=%d)' % (d, *np.percentile(np.abs(ec[m]), (90, 95, 99)), m.sum()))
    tp = pooled(res, 'trn_part')
    mt = on & trn & np.isfinite(tp) & (tp > 0)
    if mt.any():
        z = ea[mt] / np.sqrt(tp[mt])
        print('   TRN on-map: rms(e_along) %.2f m, median unscaled TRN std %.2f m, rms(e_along / std) %.3f,'
              ' robust %.3f  -> scale ~ rms z' % (np.sqrt(np.mean(ea[mt] ** 2)), np.median(np.sqrt(tp[mt])),
                                                   np.sqrt(np.mean(np.minimum(z ** 2, 1e4))),
                                                   1.4826 * np.median(np.abs(z))))

    scales = [float(x) for x in a.scales.split(',')]
    cross = [float(x) for x in a.cross.split(',')]
    if s0 not in scales:
        scales.append(s0)
    if c0 not in cross:
        cross.append(c0)
    print('\n-- what-if (in95 target 0.95, NEES target 1); pooled per segment | per bag (all segments)')
    table = []
    for sc in scales:
        for cr in cross:
            W = [m2_of(r, sc, cr) for r in res]
            M = [w[0] for w in W]
            m2 = np.concatenate(M)
            va, vc = np.concatenate([w[1] for w in W]), np.concatenate([w[2] for w in W])
            line = '%s scale %.2f cross %.2f:' % ('*' if (sc == s0 and cr == c0) else ' ', sc, cr)
            row = dict(scale=sc, cross=cr)
            for nm, s in (('bridge', 0), ('map', 1), ('tail', 2), ('nopath', 3)):
                m = seg == s
                if not m.any():
                    continue
                in95 = float(np.mean(m2[m] <= CHI2_95))
                nees = float(np.mean(np.minimum(m2[m], 1e4)) / 2)
                za2 = float(np.mean(np.minimum(ea[m] ** 2 / va[m], 1e4)))
                zc2 = float(np.mean(np.minimum(ec[m] ** 2 / vc[m], 1e4)))
                line += ' %s in95 %.3f nees %5.2f za2 %5.2f zc2 %5.2f |' % (nm, in95, nees, za2, zc2)
                row[nm] = dict(n=int(m.sum()), in95=in95, nees=nees, za2=za2, zc2=zc2)
            pb = np.array([(np.mean(x <= CHI2_95), np.mean(np.minimum(x, 1e4)) / 2) for x in M if len(x)])
            line += ' bag p_in95 med %.3f mean %.3f n<0.7 %d | NEES med %.2f mean %.2f' % (
                np.median(pb[:, 0]), pb[:, 0].mean(), int(np.sum(pb[:, 0] < 0.7)), np.median(pb[:, 1]), pb[:, 1].mean())
            row.update(bag_in95_median=float(np.median(pb[:, 0])), bag_in95_mean=float(pb[:, 0].mean()),
                       bag_in95_lt07=int(np.sum(pb[:, 0] < 0.7)), bag_nees_median=float(np.median(pb[:, 1])),
                       bag_nees_mean=float(pb[:, 1].mean()))
            table.append(row)
            print(line)
    print('\n-- bags with p_in95 < 0.7 at the current values')
    for r in res:
        x = m2_of(r, s0, c0)[0]
        if len(x) and np.mean(x <= CHI2_95) < 0.7:
            onr = r['seg'] == 1
            print('   %-16s %s p_in95 %.3f NEES %.1f  on-map |cross| median %.2f m' % (
                r['bag'], r['dir'], np.mean(x <= CHI2_95), np.mean(np.minimum(x, 1e4)) / 2,
                np.median(np.abs(r['ec'][onr])) if onr.any() else np.nan))
    C.save_json(os.path.join(od, 'whatif.json'), dict(scale0=s0, cross0=c0, bags=[r['bag'] for r in res], table=table))
    print('\nwritten %s, %s' % (fn, os.path.join(od, 'whatif.json')))


if __name__ == '__main__':
    main()
