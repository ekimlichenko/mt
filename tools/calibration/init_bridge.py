"""Start of a run: accuracy of the Initializer on the recorded bag starts, calibration of the G1 Hermite
bridge tangent factor (Params bridge_tangent_k_t2s / bridge_tangent_k_s2t) and of the initial along-track
std (sigma0_rtk_onmap_m, sigma0_bridge_t2s_m, sigma0_bridge_s2t_m, init_bridge_sigma_frac, sigma0_nortk_m).
Source: .work/scratch_init/{common,validate_start,analyze,k_fine,sigma_calib}.py (merged, paths fixed).

1. Every unique cached bag with a GNSS fix topic is fed to core.initializer.Initializer like the
   pipeline does: vehicle / fix messages in record order, add_fix(antenna, header t, lat, lon, alt,
   status, odometer at t), maybe_finalize(header t) on every message, until it finalises (first fix +
   init_window_s 3 s).  Odometer = integral of the mean fresh bogie speed / K(vehicle).
2. Truth (RTK, the whole bag): direction = route on which on-map (d < 3 m) status-2 master fixes move
   forward; s_offset = median(s_rtk - odometer) over the first 300 m of on-map RTK travel (rover RTK
   minus rover_ahead_m 12.44 m, or any-status master, when the master RTK is too sparse).  A bridge
   start is scored only when the first on-map truth fix is within 150 m of the map entry (beyond it
   the odometer scale error, ~0.1 %, and the unknown approach length mix).
   error e = s_offset(Initializer) - s_offset(truth) [m] (> 0: the reported position is ahead).
3. K sweep: for the bridge starts with a heading the bridge is rebuilt with
   track_map.hermite_bridge(p0, yaw0, route start, route yaw(0), k) for k = 1.00..2.00; e(k) =
   -L(k) - odo_ref - back - truth.  Printed per route: median / mean / |e| quantiles, the k of the
   smallest median |e| and of zero median e, and 2-fold CV (bag parity; leave-one-vehicle-out).
   Result (final code, 78 unique GNSS bags, 70 with truth, direction 70/70 correct):
     T2S n=29: zero-median k 1.55 (median e +0.1, |e| median 1.2 / p90 8.7 m); best median |e| 1.50
     S2T n=31: zero-median = best k 1.25 (median e -0.4, |e| median 2.2 / p90 15.5 m)
     both stable under parity and leave-one-vehicle-out CV (T2S 1.50-1.55, S2T 1.25 in every fold);
     k 1.5 (the old common value) biases S2T by -35 m (the S2T approach is a ~600 m loop, T2S 130-215 m).
   sigma0 (RTK, heading): T2S bridge |e| p68 1.7 / p95 9.0 m vs sigma0_bridge_t2s_m 8; S2T bridge
   p68 3.3 / p95 13.6 m vs max(sigma0_bridge_s2t_m 12, init_bridge_sigma_frac 0.03 * L ~600 m) = 18:
   sigma0 is set near the p95 of |e| (heavy tails), so the TRN prior always covers the true s.
4. sigma0: per start kind (route, onmap / bridge, rtk, heading), rms / 1.48 MAD / p68 / p95 of e vs
   the reported sigma0 (|e| / sigma0 p68 should be ~1, p95 ~2).

    python tools/calibration/init_bridge.py [--bags N|b1,b2] [--jobs 4] [--set k=v ...] [--kgrid 1.0,2.0,0.05]
~1 min on the 97 unique bags with 4 processes.  Output: $CALIB_WORK/init_bridge/{starts.pkl, summary.json}.
"""
import argparse
import math
import os
import pickle
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.spatial import cKDTree

import _common as C
from tram_backup_odometry.core.initializer import Initializer
from tram_backup_odometry.core.track_map import hermite_bridge, load_routes

S_FIRST_MAX = 150.0      # bridge truth valid only when the first on-map truth fix is near the map entry
S_LEN = 300.0            # truth s_offset window (m of on-map RTK travel)
_ROUTES = None
_TREES = {}


def routes():
    global _ROUTES
    if _ROUTES is None:
        _ROUTES = load_routes(C.MAPS, C.Params().grade_window_m)
    return _ROUTES


def proj_batch(name, X, Y):
    """Vectorised projection on a route: s, dist, on (not beyond either end)."""
    r = routes()[name]
    if name not in _TREES:
        _TREES[name] = cKDTree(np.c_[r.x, r.y])
    X = np.asarray(X, float)
    Y = np.asarray(Y, float)
    _, i = _TREES[name].query(np.c_[X, Y])
    n = len(r.x)
    best_d = np.full(len(X), np.inf)
    best_s = np.zeros(len(X))
    beyond = np.zeros(len(X), bool)
    for j in (i - 1, i):
        j = np.clip(j, 0, n - 2)
        ax, ay = r.x[j], r.y[j]
        dx, dy = r.x[j + 1] - ax, r.y[j + 1] - ay
        L = np.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        t = (X - ax) * ux + (Y - ay) * uy
        tc = np.clip(t, 0, L)
        d = np.hypot(X - ax - ux * tc, Y - ay - uy * tc)
        s = r.sg[j] + tc
        b = ((j == 0) & (t < 0)) | ((j == n - 2) & (t > L))
        better = d < best_d
        best_d = np.where(better, d, best_d)
        best_s = np.where(better, s, best_s)
        beyond = np.where(better, b, beyond)
    return best_s, best_d, ~beyond


def odometer(D, K, stale=0.5, max_dt=1.0):
    """(t, odo): integral of the mean fresh bogie speed / K over header time, from the first message."""
    ev = []
    for b, top in ((0, C.FT), (1, C.RT)):
        d = D.get(top) or {}
        if len(d.get('t_hdr', [])):
            ev.append(np.c_[d['t_hdr'], d['v'], np.full(len(d['v']), b)])
    t_first = min(float(v['t_hdr'][0]) for v in D.values() if isinstance(v, dict) and len(v.get('t_hdr', [])))
    if not ev:
        return np.array([t_first, t_first + 1e6]), np.zeros(2)
    E = np.concatenate(ev)
    E = E[np.argsort(E[:, 0], kind='stable')]
    last_t = [-1e18, -1e18]
    last_v = [0.0, 0.0]
    ts, od = [t_first], [0.0]
    v_prev, t_prev = 0.0, t_first
    for t, v, b in E:
        b = int(b)
        if not math.isfinite(v):
            continue
        dt = t - t_prev
        if dt > 0:
            od.append(od[-1] + v_prev * min(dt, max_dt))
            ts.append(t)
            t_prev = t
        last_t[b] = t
        last_v[b] = max(v, 0.0) / K
        vals = [last_v[k] for k in (0, 1) if t - last_t[k] <= stale]
        v_prev = sum(vals) / len(vals) if vals else 0.0
    return np.array(ts), np.array(od)


def gnss(D, top):
    d = D.get(top) or {}
    if not len(d.get('t_hdr', [])):
        return None
    x, y = C.utm_vec(np.asarray(d['lat'], float), np.asarray(d['lon'], float))
    return dict(t=np.asarray(d['t_hdr'], float), x=x, y=y, st=np.asarray(d['status'], int))


def truth(D, odo_t, odo_v, ahead):
    """Truth direction and s_offset (see the module docstring); None when not determinable."""
    out = {}
    m, rv = gnss(D, C.MF), gnss(D, C.RF)
    srcs = []
    if m is not None:
        srcs.append(('master_rtk', m, m['st'] == 2, 0.0))
    if rv is not None:
        srcs.append(('rover_rtk', rv, rv['st'] == 2, ahead))
    if m is not None:
        srcs.append(('master_any', m, m['st'] >= 0, 0.0))
    votes = {}
    for src, g, sel, back in srcs:
        for name in routes():
            s, d, on = proj_batch(name, g['x'][sel], g['y'][sel])
            ok = on & (d < 3.0)
            t, ss = g['t'][sel][ok], s[ok]
            if len(t) < 20:
                continue
            o = np.argsort(t)
            t, ss = t[o], ss[o]
            ds, dt = np.diff(ss), np.diff(t)
            good = (dt > 0) & (dt < 1.0) & (np.abs(ds) > 0.05) & (np.abs(ds) < 30 * dt)
            votes[name] = (int((ds[good] > 0).sum()), int((ds[good] < 0).sum()))
        if votes and max(v[0] for v in votes.values()) > 50:
            out['dir_src'] = src
            break
        votes = {}
    if not votes:
        return None
    direction = max(votes, key=lambda n: votes[n][0] - votes[n][1])
    out['direction'] = direction
    for src, g, sel, back in srcs:
        if sel.sum() < 20:
            continue
        s, d, on = proj_batch(direction, g['x'][sel], g['y'][sel])
        ok = on & (d < 3.0)
        if ok.sum() < 20:
            continue
        t, ss = g['t'][sel][ok], s[ok] - back
        oo = np.interp(t, odo_t, odo_v)
        o = np.argsort(t)
        t, ss, oo = t[o], ss[o], oo[o]
        win = oo <= oo[0] + S_LEN
        if win.sum() < 10:
            continue
        out.update(s_offset=float(np.median(ss[win] - oo[win])), s_first=float(ss[0]), s_src=src,
                   spread=float(np.subtract(*np.percentile(ss[win] - oo[win], (90, 10)))))
        break
    return out


def events(D, t_to):
    """Vehicle and fix messages in record order up to header time t_to: (t_rec, kind, t_hdr, payload)."""
    ev = []
    for kind, top in (('front', C.FT), ('rear', C.RT), ('cmd', C.CT)):
        d = D.get(top) or {}
        for tr, th in zip(d.get('t_rec', []), d.get('t_hdr', [])):
            if th <= t_to:
                ev.append((float(tr), kind, float(th), None))
    for kind, top in (('master', C.MF), ('rover', C.RF)):
        d = D.get(top) or {}
        for i, (tr, th) in enumerate(zip(d.get('t_rec', []), d.get('t_hdr', []))):
            if th <= t_to:
                ev.append((float(tr), kind, float(th),
                           (float(d['lat'][i]), float(d['lon'][i]), float(d['alt'][i]), int(d['status'][i]))))
    ev.sort(key=lambda e: e[0])
    return ev


def one(args):
    bag, sets = args
    D = C.load_cache(bag)
    fix_t = [float(np.min((D.get(top) or {}).get('t_hdr'))) for top in (C.MF, C.RF)
             if len((D.get(top) or {}).get('t_hdr', []))]
    if not fix_t:
        return None
    p = C.params_for(bag, sets)
    odo_t, odo_v = odometer(D, p.k_for_vehicle())
    ini = Initializer(p, routes())
    t_fin = None
    for tr, kind, th, pl in events(D, min(fix_t) + p.init_window_s + p.init_timeout_s + 60.0):
        if kind in ('master', 'rover'):
            ini.add_fix(kind, th, pl[0], pl[1], pl[2], pl[3], float(np.interp(th, odo_t, odo_v)))
        if ini.maybe_finalize(th):
            t_fin = th
            break
    res = ini.result()
    tru = truth(D, odo_t, odo_v, p.rover_ahead_m)
    row = dict(bag=bag, vehicle=C.vehicle_of(bag), t_fin=t_fin, truth=tru, kind=None)
    if res is None:
        return row
    row.update(kind=res.kind, direction=res.direction, s_offset=res.s_offset, sigma0=res.sigma0, rtk=bool(res.rtk),
               heading_src=res.diag.get('heading_src'), rover_only=bool(res.diag.get('rover_only')),
               odo_ref=res.diag.get('odo_ref'), yaw0=res.yaw0, rule=res.diag.get('rule'))
    if res.kind == 'bridge' and res.head is not None:
        row.update(p0=(float(res.head['x'][0]), float(res.head['y'][0])), L=float(res.head['L']),
                   back=p.rover_ahead_m if res.diag.get('rover_only') else 0.0)
    return row


def scored(rows):
    """(row, error) for starts with a determinable truth on the same route."""
    out = []
    for r in rows:
        tr = r['truth']
        if r['kind'] in (None, 'none') or not tr or 's_offset' not in tr:
            continue
        if r['direction'] != tr['direction']:
            continue
        if r['kind'] == 'bridge' and tr.get('s_first', 0) > S_FIRST_MAX:
            continue
        out.append((r, r['s_offset'] - tr['s_offset']))
    return out


def stats(e):
    e = np.asarray(e, float)
    if len(e) == 0:
        return 'n=  0'
    a = np.abs(e)
    return 'n=%3d  median %+7.2f mean %+7.2f | |e| median %6.2f p90 %6.2f max %6.1f m' % (
        len(e), np.median(e), e.mean(), np.median(a), np.percentile(a, 90), a.max())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='', help="'' = all unique bags, N = first N, or a comma list")
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--set', nargs='*', default=[], help='Params overrides, key=value')
    ap.add_argument('--kgrid', default='1.0,2.0,0.05', help='k sweep: start,stop,step')
    a = ap.parse_args()
    bags = C.select_bags(a.bags, C.unique_bags())
    sets = list(a.set)
    with ProcessPoolExecutor(max(a.jobs, 1)) as ex:
        rows = [r for r in ex.map(one, [(b, sets) for b in bags]) if r is not None]
    od = C.out_dir('init_bridge')
    with open(os.path.join(od, 'starts.pkl'), 'wb') as f:
        pickle.dump(rows, f)
    p = C.Params()
    C.set_params(p, sets)
    summ = {'n_gnss_bags': len(rows)}
    n_tr = [r for r in rows if r['truth'] and r['kind'] not in (None, 'none')]
    ok_dir = sum(r['direction'] == r['truth']['direction'] for r in n_tr)
    kinds = {str(k): sum(r['kind'] == k for r in rows) for k in {r['kind'] for r in rows}}
    print('%d unique bags with GNSS; with truth %d; direction correct %d/%d; kinds %s' % (
        len(rows), len(n_tr), ok_dir, len(n_tr), kinds))
    summ['direction_correct'] = [ok_dir, len(n_tr)]
    er = scored(rows)
    print('\n-- start error e = s_offset - truth (current Params: k T2S %.2f S2T %.2f)' % (
        p.bridge_tangent_k_t2s, p.bridge_tangent_k_s2t))
    for d in ('T2S', 'S2T'):
        for kind in ('onmap', 'bridge'):
            e = [x for r, x in er if r['direction'] == d and r['kind'] == kind]
            print('   %s %-6s %s' % (d, kind, stats(e)))
            summ['err_%s_%s' % (d, kind)] = dict(n=len(e), median=float(np.median(e)) if e else None,
                                                abs_median=float(np.median(np.abs(e))) if e else None)
    big = sorted(er, key=lambda x: -abs(x[1]))[:8]
    print('   largest |e|:', ', '.join('%s %s %s %+.1f' % (r['bag'], r['direction'], r['kind'], x) for r, x in big))

    # ---- k sweep on the bridge starts with a heading
    k0, k1, dk = [float(x) for x in a.kgrid.split(',')]
    K = np.round(np.arange(k0, k1 + 1e-9, dk), 3)
    items = [(r, x) for r, x in er if r['kind'] == 'bridge' and r.get('p0') and r['heading_src'] != 'none']
    print('\n-- bridge tangent factor k (bridge starts with a heading; e(k) = -L(k) - odo_ref - back - truth)')
    summ['k'] = {}
    for d in ('T2S', 'S2T'):
        its = [r for r, _ in items if r['direction'] == d]
        if not its:
            continue
        rt = routes()[d]
        M = np.array([[-hermite_bridge(r['p0'], r['yaw0'], (rt.x[0], rt.y[0]), rt.yaw[0], k)['L'] - r['odo_ref'] - r['back']
                       - r['truth']['s_offset'] for k in K] for r in its])
        print('   %s n=%d' % (d, len(its)))
        for j, k in enumerate(K):
            if abs((k - K[0]) / dk) % 2 < 1e-6 or abs(k - getattr(p, 'bridge_tangent_k_' + d.lower())) < 1e-6:
                e = M[:, j]
                print('     k=%.2f  %s' % (k, stats(e)))
        kb = K[np.argmin(np.median(np.abs(M), axis=0))]
        kz = K[np.argmin(np.abs(np.median(M, axis=0)))]
        print('     best k by median |e| %.2f; zero-median k %.2f   (Params bridge_tangent_k_%s = %.2f)' % (
            kb, kz, d.lower(), getattr(p, 'bridge_tangent_k_' + d.lower())))
        summ['k'][d] = dict(n=len(its), best_abs=float(kb), zero_median=float(kz))
        for name, split in (('parity', np.arange(len(its)) % 2),
                            ('vehicle', np.array([0 if r['vehicle'] == '30618' else 1 for r in its]))):
            for f in (0, 1):
                tr_, te = M[split != f], M[split == f]
                if len(tr_) == 0 or len(te) == 0:
                    continue
                j = np.argmin(np.median(np.abs(tr_), axis=0))
                print('     CV %-7s fold %d: train n=%2d -> k=%.2f; test n=%2d |e| median %.2f p90 %.2f, median e %+.2f' % (
                    name, f, len(tr_), K[j], len(te), np.median(np.abs(te[:, j])), np.percentile(np.abs(te[:, j]), 90),
                    np.median(te[:, j])))

    # ---- sigma0 vs the actual error
    print('\n-- sigma0 per start kind: actual spread vs the reported sigma0 (|e|/sigma0 p68 ~1, p95 ~2 if calibrated)')
    g = defaultdict(lambda: ([], []))
    for r, x in er:
        key = '%s %-6s rtk=%d head=%d' % (r['direction'], r['kind'], r['rtk'], r['heading_src'] != 'none')
        g[key][0].append(x)
        g[key][1].append(r['sigma0'])
    summ['sigma0'] = {}
    for key in sorted(g):
        E, S = np.asarray(g[key][0]), np.asarray(g[key][1])
        aa = np.abs(E)
        z = aa / S
        print('   %-24s n=%3d rms %6.2f 1.48MAD %6.2f p68 %6.2f p95 %6.2f | sigma0 median %5.1f | |e|/sigma0 p68 %.2f p95 %.2f >3sig %.3f' % (
            key, len(E), np.sqrt(np.mean(E ** 2)), 1.4826 * np.median(np.abs(E - np.median(E))),
            np.percentile(aa, 68), np.percentile(aa, 95), np.median(S), np.percentile(z, 68), np.percentile(z, 95), np.mean(z > 3)))
        summ['sigma0'][key] = dict(n=len(E), p68=float(np.percentile(aa, 68)), p95=float(np.percentile(aa, 95)),
                                   sigma0=float(np.median(S)))
    fn = os.path.join(od, 'summary.json')
    C.save_json(fn, summ)
    print('\nwritten', fn, os.path.join(od, 'starts.pkl'))


if __name__ == '__main__':
    main()
