"""Parametric traction / brake curves fitted to the Hammerstein table of fit_dynamics.py
(analysis_notes D1, D2; Params tr_* / br_*).

    a_tr(n,v) = min(b0 + a0*n/(1+max(v-v1,0)/va), A0*min(1,vb/v)^pw) - r0          n = 1..15
    a_br(n,v) = max(-(d0 + d1|n|)*(1 - f*exp(-v/vf)), cap) - r0                        n = -1..-7, -9..-15
r0 = res_r0 = 0.04 (running resistance, fixed), cap = br_cap = -1.7 (fixed; below every fitted cell).
Weighted least squares over the table cells with support >= 20 s at v = 1..14 m/s (the v = 0 / 16
columns are extrapolation of the hat basis), weight = support [s].  Notch -8 is excluded: it is the
closed-loop hold mode (standstill_hold8.py).  Also reports the fit on -1..-7 only (D2: the -9..-15
cells come from stop transients) and the per-vehicle difference of the table is left to
fit_dynamics.py --kscale vehicle --bags <vehicle bags>.

The fit starts from the config values (--start config, default) or from a neutral point
(--start neutral).  The traction objective is non-smooth (min of two branches) and vb / pw are weakly
identified: from the neutral start it lands in a second minimum (vb ~7.1, pw ~0.84) with the same
wRMSE (0.044 vs 0.045); the config keeps the first one (analysis_notes D1).  Brake converges to the
same point from both starts.

    python tools/calibration/fit_traction_params.py [--table path/to/notch_accel_table.json] [--start config|neutral]
"""
import argparse
import json
import os

import numpy as np
from scipy.optimize import least_squares

import _common as C

R0 = 0.04
CAP = -1.7
VMIN, VMAX = 1.0, 14.0


def a_tr(q, n, v):
    b0, a0, v1, va, A0, vb, pw = q
    lin = b0 + a0 * n / (1 + np.maximum(v - v1, 0) / va)
    cap = A0 * np.minimum(1.0, vb / np.maximum(v, 1e-3)) ** pw
    return np.minimum(lin, cap) - R0


def a_br(q, n, v):
    d0, d1, f, vf = q
    return np.maximum(-(d0 + d1 * np.abs(n)) * (1 - f * np.exp(-v / vf)), CAP) - R0


def cells(tab, notches):
    kn = np.array(tab['knots_v'])
    N, V, A, W = [], [], [], []
    for k in notches:
        row = tab['table'].get(str(k))
        sup = tab['support_s'].get(str(k))
        if row is None:
            continue
        for j, v in enumerate(kn):
            if row[j] is None or not (VMIN <= v <= VMAX):
                continue
            N.append(k); V.append(v); A.append(row[j]); W.append(sup[j])
    return np.array(N, float), np.array(V), np.array(A), np.array(W)


def fit(fun, q0, lb, ub, cl):
    n, v, a, w = cl
    sw = np.sqrt(w)
    r = least_squares(lambda q: sw * (fun(q, n, v) - a), q0, bounds=(lb, ub), x_scale='jac')
    e = fun(r.x, n, v) - a
    return r.x, float(np.sqrt((w * e ** 2).sum() / w.sum())), float(np.abs(e).max()), len(a)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--table', default=os.path.join(C.WORK, 'dynamics', 'notch_accel_table.json'))
    ap.add_argument('--start', choices=('config', 'neutral'), default='config')
    a = ap.parse_args()
    tab = json.load(open(a.table))
    p = C.Params()
    out = {'start': a.start}
    if a.start == 'config':
        q0 = [p.tr_b0, p.tr_a0, p.tr_v1, p.tr_va, p.tr_A0, p.tr_vb, p.tr_pw]
        qb = [p.br_d0, p.br_d1, p.br_f, p.br_vf]
    else:
        q0 = [0.1, 0.1, 3.0, 5.0, 1.0, 6.0, 0.5]
        qb = [0.2, 0.1, 0.3, 3.0]
    cfg_tr = [p.tr_b0, p.tr_a0, p.tr_v1, p.tr_va, p.tr_A0, p.tr_vb, p.tr_pw]
    cfg_br = [p.br_d0, p.br_d1, p.br_f, p.br_vf]
    q, rm, mx, nc = fit(a_tr, q0, [-0.5, 0, 0.5, 0.5, 0.3, 1, 0.1], [0.5, 0.5, 10, 50, 2, 15, 2],
                        cells(tab, range(1, 16)))
    names = ['tr_b0', 'tr_a0', 'tr_v1', 'tr_va', 'tr_A0', 'tr_vb', 'tr_pw']
    print('traction n=1..15: %d cells, weighted RMSE %.3f, max %.2f m/s^2' % (nc, rm, mx))
    for k, x, c in zip(names, q, cfg_tr):
        print('   %-6s %8.4f   (config %.4f)' % (k, x, c))
    out['traction'] = dict(zip(names, q.tolist()), wrmse=rm, max=mx, n_cells=nc)
    names = ['br_d0', 'br_d1', 'br_f', 'br_vf']
    for title, nn in (('brake n=-1..-7,-9..-15', [k for k in range(-15, 0) if k != -8]),
                      ('brake n=-1..-7 only', list(range(-7, 0)))):
        q, rm, mx, nc = fit(a_br, qb, [0, 0, 0, 0.3], [1, 0.5, 0.9, 20], cells(tab, nn))
        print('%s: %d cells, weighted RMSE %.3f, max %.2f m/s^2' % (title, nc, rm, mx))
        for k, x, c in zip(names, q, cfg_br):
            print('   %-6s %8.4f   (config %.4f)' % (k, x, c))
        out[title] = dict(zip(names, q.tolist()), wrmse=rm, max=mx, n_cells=nc)
    o = os.path.join(C.out_dir('dynamics'), 'parametric_fit.json')
    C.save_json(o, out)
    print('written', o)


if __name__ == '__main__':
    main()
