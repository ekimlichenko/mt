"""Hammerstein fit of the notch -> acceleration table, grade coefficients c_mode, curve term,
and the per-mode process noise (analysis_notes D1-D5, D13; source: notes 'fit_dynamics.py').

Data: unique bags with >= 3000 front wheel samples and master GNSS fixes (68 bags), 20 Hz grid.
  v   = mean of front/rear wheel speed / KSCALE,  a = d/dt of it (3-sample smoothing)
  n   = driver notch (zero-order hold on header stamps)
  grade(s) = dz/ds of the pathgraph smoothed over --grade-window m (31), signed by the travel
      direction (GNSS velocity . map tangent); curv = |curv| of the map smoothed over 5 points;
      samples > 6 m from the map are dropped (grade NaN).
  clean = |vF - vR| < 0.15 & |v - vGNSS| < 0.3 & |aF - aR| < 0.3
Linear least squares (ridge 1e-2) over every other sample of
  clean & v > 0.3 & on-map & not (traction, running < 1.3 s, v < 0.5)   (start-from-rest delay)
  a = R(v) + sum_k z_k(t) * T_k(v) + g*grade*(c_tr z_tr + c_br z_br + c_coast (1 - z_tr - z_br)) + k_curv*|curv|
with R, T_k piecewise-linear on the speed knots KN, z_k = one-hot notch k through dead time + first
order lag (traction --pt 0.05,0.25; brake --pb 0.15,0.30).

Outputs: notch_accel_table.json/.csv (table cells with support >= 20 s; input of
fit_traction_params.py), grade_coef (-> grade_c_traction / grade_c_brake / grade_c_coast),
curv_coef (-> curve_k ~ 2.2, rounded up to 3 with the coast-only Davis fit 4.4, D5) and the
residual std per mode of the fitted model vs the 1 s smoothed GNSS acceleration
(-> sigma_a_traction / _coast / _brake / _hold8 / _brake_high, D13).

--lag-grid: instead of the table, the in-sample RMSE of the same least-squares fit over a grid of
dead time d x time constant tau, traction (brake fixed at --pb) and brake (traction fixed at --pt)
-> lag_tr_dead_s / lag_tr_tau_s / lag_br_dead_s / lag_br_tau_s (~2.5 min for the default 36 points
on 68 bags).  Result (final code): the RMSE surface is a flat valley along d + tau ~ const (traction
0.30-0.35 s, brake 0.45-0.50 s), so only the total delay is identified; grid minima traction
d 0.10 / tau 0.25, brake d 0.10 / tau 0.40 (RMSE 0.1632); the config points (0.05, 0.25) and
(0.15, 0.30) lie in the valley, 0.00004 / 0.00002 m/s^2 above the minimum.

    python tools/calibration/fit_dynamics.py [--bags N|b1,b2] [--kscale 3.597|vehicle] [--jobs 4] [--lag-grid]
"""
import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.signal import lfilter
from scipy.spatial import cKDTree

import _common as C

DT = 0.05
KN = np.array([0, 1, 2, 3, 4, 6, 8, 10, 12, 14, 16.])
NOT = [k for k in range(-15, 16) if k != 0]
NK = len(KN)
G = 9.81
_MAP = {}


def map_arrays(grade_window):
    """Pathgraph points of both routes: xy, grade (dz/ds over grade_window points ~ m), |curv|, tangent."""
    key = grade_window
    if key not in _MAP:
        P, GR, TG, CV = [], [], [], []
        for fn in ('t2s.json', 's2t.json'):
            j = json.load(open(os.path.join(C.MAPS, fn)))
            p = np.array([[q['x'], q['y'], q['z'], q['curv']] for q in j['points']])
            P.append(p[:, :2])
            GR.append(uniform_filter1d(np.gradient(p[:, 2]), int(grade_window)))
            CV.append(uniform_filter1d(p[:, 3], 5))
            tg = np.gradient(p[:, :2], axis=0)
            TG.append(tg / np.linalg.norm(tg, axis=1, keepdims=True))
        xy = np.vstack(P)
        _MAP[key] = (cKDTree(xy), np.r_[GR[0], GR[1]], np.vstack(TG), np.r_[CV[0], CV[1]])
    return _MAP[key]


def build(args):
    b, K, grade_window = args
    tree, allgr, alltg, allcv = map_arrays(grade_window)
    d = C.load_cache(b)
    fr = d[C.FT]; rr_ = d[C.RT]; c = d[C.CT]; mv = d[C.MV]; mf = d[C.MF]
    T0 = max(fr['t_hdr'][0], rr_['t_hdr'][0], c['t_hdr'][0], mv['t_hdr'][0]) + 1
    T1 = min(fr['t_hdr'][-1], rr_['t_hdr'][-1], c['t_hdr'][-1], mv['t_hdr'][-1]) - 1
    t = np.arange(T0, T1, DT)
    vG = np.interp(t, mv['t_hdr'], np.hypot(mv['vx'], mv['vy']))
    ff = np.r_[True, np.diff(fr['t_hdr']) > 0.02]
    rr = np.r_[True, np.diff(rr_['t_hdr']) > 0.02]
    vF = np.interp(t, fr['t_hdr'][ff], fr['v'][ff]) / K
    vR = np.interp(t, rr_['t_hdr'][rr], rr_['v'][rr]) / K
    aF = np.interp(t, fr['t_hdr'][ff], np.gradient(fr['v'][ff] / K, fr['t_hdr'][ff]))
    aR = np.interp(t, rr_['t_hdr'][rr], np.gradient(rr_['v'][rr] / K, rr_['t_hdr'][rr]))
    a = uniform_filter1d(0.5 * (aF + aR), 3)
    n = np.asarray(c['pos'])[np.clip(np.searchsorted(c['t_hdr'], t, 'right') - 1, 0, None)].astype(int)
    xf, yf = C.utm_vec(mf['lat'], mf['lon'])
    x = np.interp(t, mf['t_hdr'], xf)
    y = np.interp(t, mf['t_hdr'], yf)
    vx = np.interp(t, mv['t_hdr'], mv['vx'])
    vy = np.interp(t, mv['t_hdr'], mv['vy'])
    dist, k = tree.query(np.c_[x, y])
    dot = alltg[k, 0] * vx + alltg[k, 1] * vy
    sgn = np.where(np.abs(dot) > 0.5, np.sign(dot), np.nan)
    idx = np.where(~np.isnan(sgn), np.arange(len(sgn)), 0)
    np.maximum.accumulate(idx, out=idx)
    sgn = sgn[idx]
    sgn[np.isnan(sgn)] = 1
    on = dist < 6
    grade = np.where(on, allgr[k] * sgn, np.nan)
    curv = np.where(on, np.abs(allcv[k]), np.nan)
    clean = (np.abs(vF - vR) < 0.15) & (np.abs(0.5 * (vF + vR) - vG) < 0.3) & (np.abs(aF - aR) < 0.3)
    pos_ = n > 0
    last0 = np.maximum.accumulate(np.where(~pos_, np.arange(len(n)), -1))
    trun = (np.arange(len(n)) - last0) * DT * pos_
    aG1 = (np.interp(t + 0.5, t, vG) - np.interp(t - 0.5, t, vG)) / 1.0
    return b, dict(v=0.5 * (vF + vR), a=a, n=n, grade=grade, curv=curv, clean=clean, trun=trun, aG1=aG1)


def hats(v):
    v = np.clip(v, KN[0], KN[-1])
    j = np.clip(np.searchsorted(KN, v, 'right') - 1, 0, NK - 2)
    w = (v - KN[j]) / (KN[j + 1] - KN[j])
    H = np.zeros((len(v), NK))
    H[np.arange(len(v)), j] = 1 - w
    H[np.arange(len(v)), j + 1] = w
    return H


def fo(u, d, tau):
    """Dead time d + first-order lag tau on the 20 Hz grid."""
    D_ = int(round(d / DT))
    u = np.r_[np.full(D_, u[0]), u[:len(u) - D_]] if D_ > 0 else u
    al = 1 - np.exp(-DT / tau)
    return lfilter([al], [1, -(1 - al)], u, zi=[u[0] * (1 - al)])[0]


def design(x, PT, PB):
    H = hats(x['v'])
    Z = np.stack([fo((x['n'] == k).astype(float), *(PT if k > 0 else PB)) for k in NOT], 1)
    ztr = Z[:, np.array(NOT) > 0].sum(1)
    zbr = Z[:, np.array(NOT) < 0].sum(1)
    gg = G * np.nan_to_num(x['grade'])
    return np.hstack([H] + [Z[:, i:i + 1] * H for i in range(len(NOT))]
                     + [(gg * ztr)[:, None], (gg * zbr)[:, None], (gg * (1 - ztr - zbr))[:, None],
                        np.nan_to_num(x['curv'])[:, None]])


def fit_mask(x):
    m = x['clean'] & (x['v'] > 0.3) & ~np.isnan(x['grade']) & ~((x['n'] > 0) & (x['trun'] < 1.3) & (x['v'] < 0.5))
    m = m.copy()
    m[1::2] = False
    return m


def normal_eq(D, sel, PT, PB):
    P = NK * (1 + len(NOT)) + 4
    A = np.zeros((P, P)); B = np.zeros(P); W = np.zeros(P); aa = 0.0; nfit = 0
    for b in sel:
        x = D[b]
        X = design(x, PT, PB)
        m = fit_mask(x)
        A += X[m].T @ X[m]; B += X[m].T @ x['a'][m]; W += np.abs(X[m]).sum(0) * 2 * DT
        aa += float(x['a'][m] @ x['a'][m]); nfit += int(m.sum())
    cf = np.linalg.solve(A + 1e-2 * np.eye(P), B)
    rmse = np.sqrt(max(aa - 2 * cf @ B + cf @ A @ cf, 0.0) / max(nfit, 1))
    return cf, B, W, rmse, nfit


def lag_grid(D, sel, PT, PB):
    """In-sample RMSE over dead time x tau, one mode at a time (the other at its given value)."""
    out = {}
    for mode, dgrid, tgrid in (('traction', (0.0, 0.05, 0.10, 0.15), (0.15, 0.25, 0.35, 0.50)),
                               ('brake', (0.05, 0.10, 0.15, 0.20, 0.25), (0.20, 0.30, 0.40, 0.50))):
        print('%s (other mode fixed at %s): in-sample RMSE [m/s^2]' % (mode, PB if mode == 'traction' else PT))
        print('   dead \\ tau ' + ''.join('%8.2f' % t for t in tgrid))
        rows = {}
        for d in dgrid:
            r = [float(normal_eq(D, sel, (d, t) if mode == 'traction' else PT, PB if mode == 'traction' else (d, t))[3])
                 for t in tgrid]
            rows[d] = r
            print('   %8.2f    ' % d + ''.join('%8.4f' % x for x in r), flush=True)
        best = min(((d, t, rows[d][j]) for d in dgrid for j, t in enumerate(tgrid)), key=lambda z: z[2])
        print('   minimum: dead %.2f s, tau %.2f s, RMSE %.4f' % best)
        out[mode] = dict(dead=list(dgrid), tau=list(tgrid), rmse=[rows[d] for d in dgrid], best=best)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='', help="'' = unique bags with wheels + master GNSS, N, or a comma list")
    ap.add_argument('--kscale', default='3.597', help="wheel scale: a number (original fit: 3.597 for both vehicles) or 'vehicle'")
    ap.add_argument('--grade-window', type=float, default=31.0, help='map grade smoothing, points (~m)')
    ap.add_argument('--pt', default='0.05,0.25', help='traction dead time, tau [s]')
    ap.add_argument('--pb', default='0.15,0.30', help='brake dead time, tau [s]')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--lag-grid', action='store_true', help='RMSE over a dead time x tau grid instead of the table')
    a = ap.parse_args()
    PT = tuple(float(v) for v in a.pt.split(','))
    PB = tuple(float(v) for v in a.pb.split(','))
    sel = []
    for b in C.select_bags(a.bags, C.unique_bags()):
        d = C.load_cache(b)
        fv = d.get(C.FT, {}).get('v', np.array([]))
        if len(fv) >= 3000 and len(d.get(C.MF, {}).get('status', [])) > 0 and C.RT in d and C.CT in d and C.MV in d:
            sel.append(b)
    jobs = [(b, C.params_for(b).k_for_vehicle() if a.kscale == 'vehicle' else float(a.kscale), a.grade_window) for b in sel]
    if a.jobs > 1:
        with ProcessPoolExecutor(a.jobs) as ex:
            D = dict(ex.map(build, jobs))
    else:
        D = dict(build(j) for j in jobs)
    if a.lag_grid:
        g = lag_grid(D, sel, PT, PB)
        fn = os.path.join(C.out_dir('dynamics'), 'lag_grid.json')
        C.save_json(fn, g)
        print('written', fn)
        return
    cf, B, W, rmse, nfit = normal_eq(D, sel, PT, PB)
    R = cf[:NK]
    TAB = {k: (R + cf[NK * (1 + i):NK * (2 + i)]) for i, k in enumerate(NOT)}
    SUP = {k: W[NK * (1 + i):NK * (2 + i)] for i, k in enumerate(NOT)}
    # per-mode residual of the fitted model vs the 1 s smoothed GNSS acceleration (D13)
    res = {'traction': [], 'coast': [], 'brake -1..-7': [], 'hold8 (-8)': [], 'brake <= -9': []}
    for b in sel:
        x = D[b]
        pred = uniform_filter1d(design(x, PT, PB) @ cf, 21)
        m = x['clean'] & (x['v'] > 0.3) & ~np.isnan(x['grade'])
        r = x['aG1'] - pred
        n = x['n']
        for key, mm in (('traction', n > 0), ('coast', n == 0), ('brake -1..-7', (n < 0) & (n >= -7)),
                        ('hold8 (-8)', n == -8), ('brake <= -9', n <= -9)):
            res[key].append(r[m & mm])
    out = {'units': 'm/s^2 level-track accel a(notch,v) incl. running resistance; v in m/s',
           'knots_v': KN.tolist(),
           'lag': {'traction': {'dead_s': PT[0], 'tau_s': PT[1]}, 'brake': {'dead_s': PB[0], 'tau_s': PB[1]}},
           'kscale': a.kscale, 'grade_window': a.grade_window,
           'coast_R': R.tolist(),
           'table': {str(k): [None if SUP[k][j] < 20 else round(float(TAB[k][j]), 3) for j in range(NK)] for k in NOT},
           'support_s': {str(k): np.round(SUP[k], 1).tolist() for k in NOT},
           'grade_coef': {'traction': float(cf[-4]), 'brake': float(cf[-3]), 'coast': float(cf[-2])},
           'curv_coef': float(cf[-1]), 'rmse_in_sample': float(rmse), 'n_fit': nfit, 'bags': sel,
           'resid_std_by_mode': {}}
    print('fit: %d bags, %d samples, in-sample RMSE %.4f m/s^2 (lags tr %s br %s, grade window %g, K %s)' % (
        len(sel), nfit, rmse, PT, PB, a.grade_window, a.kscale))
    print('grade coef (a += c*g*grade): traction %.3f  brake %.3f  coast %.3f   -> grade_c_traction / _brake / _coast' % (
        cf[-4], cf[-3], cf[-2]))
    print('curve coef (a += k*|curv|): %.2f m^2/s^2   -> curve_k = -k (rounded, see D5)' % cf[-1])
    print('coast R(v) at knots %s: %s' % (KN.tolist(), np.round(R, 3).tolist()))
    print('residual vs 1 s GNSS accel by mode [m/s^2] (std / robust)   -> sigma_a_*')
    for key, v in res.items():
        v = np.concatenate(v) if v else np.zeros(0)
        if len(v):
            s, rb = float(v.std()), float(1.4826 * np.median(np.abs(v - np.median(v))))
            out['resid_std_by_mode'][key] = {'std': s, 'robust': rb, 'n': int(len(v))}
            print('   %-13s n=%7d  std %.3f  robust %.3f' % (key, len(v), s, rb))
    od = C.out_dir('dynamics')
    C.save_json(os.path.join(od, 'notch_accel_table.json'), out)
    with open(os.path.join(od, 'notch_accel_table.csv'), 'w') as fh:
        fh.write('notch,' + ','.join('v%g' % v for v in KN) + '\n')
        for k in NOT:
            fh.write('%d,' % k + ','.join('' if SUP[k][j] < 20 else '%.3f' % TAB[k][j] for j in range(NK)) + '\n')
    print('written', os.path.join(od, 'notch_accel_table.json'), '(+ .csv)')


if __name__ == '__main__':
    main()
