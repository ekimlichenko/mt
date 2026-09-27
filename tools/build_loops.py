#!/usr/bin/env python3
"""Terminus-loop centrelines from the RTK tracks of the training runs -> maps/loops.json.

Every run starts at a terminus stop before a turning loop (head: off the map
until route s = 0) and ends after s = L inside the loop of the other terminus
(tail).  This tool restores the loop geometry offline from the status-2 master
fixes of all unique training bags (both vehicles; the master antenna lies on
the track centreline, docs/analysis_notes.md F3/F8):

1. Fixes: master fixes with status == 2 in the map frame; header-stamp glitches
   and position jumps (1 s displacement vs integrated GNSS Doppler speed,
   ``evaluate.position_glitch_mask``) are dropped.  Duplicated recordings (same
   front-bogie speed array) are used once.  Run quality q = fraction of on-map
   fixes within 0.15 m of the run's lateral bias and 0.3 m of map z + 3.10 m;
   only runs with q >= 0.45 build the geometry (all runs are validated).
2. Segments: head = fixes before the first on-map fix (only runs that enter
   the map within 20 m of s = 0, i.e. not mid-route starts), tail = fixes after
   the last on-map fix (within 5 m of s = L).  Each is parametrised by the
   arc length u of its own track, anchored at the map joint (tail u >= 0 from
   s = L, head u <= 0 up to s = 0).
3. Clusters per (terminus, head/tail): greedy, reference-grade runs (q >= 0.85,
   own arc within 2 % of the wheel distance) first, longest first; a run joins
   the first cluster whose first member it matches (p90 of the point-to-track
   distance over the common range < 2 m), else it seeds a new cluster.  Runs
   that only cover a common part (short tails) join the first (main) cluster.
4. Template per cluster: the seed track binned per 1 m of WHEEL distance (per-bin
   median, so stationary jitter at stops collapses into one point), then 3
   iterations of "project every member fix, per-run median lateral per 1 m bin,
   median over runs, 7 m moving average, shift the template along its normal";
   z = median (GNSS alt - 3.10), 15 m moving average.  Finally the antenna path
   is moved to the track centreline: the master antenna sits 0.03 + 8.7 m^2 *
   curvature to the outside of curves (F4), which also makes the loop arc length
   agree with the wheel odometry (``--antenna-path`` keeps the antenna path).
5. Branches (complete paths from the end of route ``from`` to the start of route
   ``to``, forward in the direction of travel):
     S (Shchukinskaya): T2S tail + S2T head, joined where they overlap at the stop.
     T (Tallinskaya):   A = S2T tail A + T2S head A (joined at the platform);
                        B = S2T tail B + [unobserved turnaround] + T2S head B.
                            B is a double-track branch: tails go out on one
                            track, heads come back on the parallel one 3.5 m
                            away; the turnaround beyond the recordings was never
                            observed and is a placeholder balloon loop (R 18 m);
                        C = A with the observed lateral offset of the 5 m
                            variant heads (a parallel platform track), ramped
                            in over 25 m before its first observed point.
   Assembled branches: points without forward progress are dropped (monotone),
   5 m moving average, 1 m resampling (check: arc - chord over 10 m < 0.3 m).
   Ends: re-anchored at the projections of the map joints, then blended over
   15 m (xy) / 20 m (z) into the map end/start pose continued along the map
   curvature (tangent-continuous), resampled every ~1 m of arc.
   Extra keys per branch: ``n_tail``, ``n_head`` (runs used), ``unobserved``
   (arc intervals [a0, a1] not covered by any fix).
6. Validation (printed): for every run (and the organisers' test bag if present)
   off-map fixes are projected onto each branch inside a +-40 m window around the
   wheel-odometry arc position (front bogie km/h / 3.597 for 30618, / 3.59 for
   30639); cross-track p50/p95/max to the best branch, and the relative
   difference between loop arc distance to the joint and wheel distance.

    OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry \\
        /opt/miniconda3/envs/ml/bin/python tools/build_loops.py [--no-test-bag] [--no-plot]

Runtime ~5 s (122 cached bags + the test bag).  Result:
src/tram_backup_odometry/maps/loops.json and docs/plots/loops.png.
"""
import argparse
import hashlib
import json
import math
import os
import pickle
import sys
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate as ev  # noqa: E402

ROOT = ev.ROOT
sys.path.insert(0, ev.PKG_DIR)
from tram_backup_odometry.core.track_map import load_routes  # noqa: E402

OUT_JSON = os.path.join(ev.MAPS_DIR, 'loops.json')
OUT_PNG = os.path.join(ROOT, 'docs', 'plots', 'loops.png')
TEST_BAG = os.path.join(os.path.dirname(ROOT), 'check_code', 'check-code', 'bags', '30618_88aea4d9')
TEST_LOADER = os.path.join(ROOT, '.work', 'check')

ALT_OFF = 3.10               # GNSS master alt - pathgraph z
WHEEL_DIV = {'30618': 3.597, '30639': 3.59}
Q_MIN = 0.45                 # run quality needed to build geometry
CLUSTER_THR = 2.0            # m, p90 point-to-track distance
LAT_SMOOTH = 7               # m
Z_SMOOTH = 15                # m
BLEND_XY = 15.0              # m
BLEND_Z = 20.0               # m
C_RAMP = 25.0                # m, lateral ramp-in of the 5 m variant before its first fix
CURVE_K2 = 8.7               # m^2: antenna lateral = -ANT_LAT0 - CURVE_K2 * curvature (F4); 0 = keep the antenna path
ANT_LAT0 = 0.03              # m, master antenna right of the centreline on straight track
WIN = 40.0                   # m, validation projection window around the wheel arc
REF_AHEAD = 9.873            # m, test-bag reference (base_link) ahead of the master antenna
LOOPS = {'S': ('T2S', 'S2T'), 'T': ('S2T', 'T2S')}   # loop: (from route, to route)


# ------------------------------------------------------------------ helpers
def movavg(x, n):
    n = max(1, int(n))
    if n <= 1 or len(x) < 2:
        return np.asarray(x, float).copy()
    pad = n // 2
    xp = np.r_[np.full(pad, x[0]), x, np.full(n - 1 - pad, x[-1])]
    c = np.cumsum(np.r_[0.0, xp])
    return (c[n:] - c[:-n]) / n


def track_arc(P, step=0.5):
    """Cumulative arc length of a noisy track (decimated at ``step`` m, so jitter at stops adds nothing)."""
    k = [0]
    for i in range(1, len(P)):
        if math.hypot(P[i, 0] - P[k[-1], 0], P[i, 1] - P[k[-1], 1]) >= step:
            k.append(i)
    k = np.array(k)
    a = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P[k, :2], axis=0).T))]
    return np.interp(np.arange(len(P)), k, a)


def group_median(b, v, nb):
    """Median of v per integer bin b in [0, nb) (NaN for empty bins)."""
    out = np.full(nb, np.nan)
    if len(b) == 0:
        return out
    o = np.lexsort((v, b))
    b, v = b[o], v[o]
    starts = np.r_[0, np.flatnonzero(np.diff(b)) + 1]
    ends = np.r_[starts[1:], len(b)]
    lo = v[starts + (ends - starts - 1) // 2]
    hi = v[starts + (ends - starts) // 2]
    out[b[starts]] = 0.5 * (lo + hi)
    return out


def arclen(P):
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(P[:, :2], axis=0).T))]


def resample(P, z=None, step=1.0, n=None):
    """Polyline (and z) resampled uniformly in arc length (n points or ~step spacing)."""
    a = arclen(P)
    keep = np.r_[True, np.diff(a) > 1e-9]
    P, a = P[keep], a[keep]
    if n is None:
        n = max(2, int(round(a[-1] / step)) + 1)
    g = np.linspace(0.0, a[-1], n)
    Q = np.c_[np.interp(g, a, P[:, 0]), np.interp(g, a, P[:, 1])]
    if z is None:
        return Q
    return Q, np.interp(g, a, np.asarray(z, float)[keep])


def normals(P):
    t = np.gradient(P, axis=0)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    return np.c_[-t[:, 1], t[:, 0]]           # left normal


def curvature(P, win=5):
    """Signed curvature (left +) of a ~1 m polyline, smoothed over ``win`` m."""
    hd = np.unwrap(np.arctan2(*np.gradient(P, axis=0).T[::-1]))
    ds = np.maximum(np.hypot(*np.gradient(P, axis=0).T), 1e-6)
    return movavg(np.gradient(movavg(hd, win)) / ds, win)


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def project_window(P, a, Q, a_pred, half=WIN):
    """Project points Q onto polyline P (arc a) using only segments with |arc - a_pred| <= half.

    Returns arc, distance, signed lateral (left +).  Vectorised: K segments per point."""
    nseg = len(P) - 1
    ds = a[-1] / nseg
    K = int(2 * half / ds) + 3
    i0 = np.clip(np.floor((a_pred - half) / ds).astype(int), 0, max(0, nseg - 1))
    I = np.clip(i0[:, None] + np.arange(K)[None, :], 0, nseg - 1)
    A = P[I]; B = P[I + 1]
    d = B - A
    l2 = np.maximum((d ** 2).sum(-1), 1e-12)
    r = Q[:, None, :] - A
    t = np.clip((r * d).sum(-1) / l2, 0.0, 1.0)
    c = A + t[..., None] * d
    dist = np.hypot(*(Q[:, None, :] - c).transpose(2, 0, 1))
    k = np.argmin(dist, axis=1)
    j = np.arange(len(Q))
    ii = I[j, k]
    lat = (d[j, k, 0] * r[j, k, 1] - d[j, k, 1] * r[j, k, 0]) / np.sqrt(l2[j, k])
    return a[ii] + t[j, k] * (a[ii + 1] - a[ii]), dist[j, k], lat


def wheel_odometer(tw, vw, veh):
    div = WHEEL_DIV.get(veh, 3.6)
    v = np.asarray(vw, float) / div
    dt = np.clip(np.diff(tw), 0.0, 1.0)
    return np.r_[0.0, np.cumsum(dt * (v[1:] + v[:-1]) / 2)]


def time_at_odo(tw, W, w):
    """Inverse of the (non-decreasing) odometer: time when odometer reaches w."""
    Wu, iu = np.unique(W, return_index=True)
    return float(np.interp(w, Wu, np.asarray(tw)[iu]))


# ------------------------------------------------------------------ data
def load_runs(verbose=True):
    """Unique training runs with RTK master fixes: dict bag -> run dict."""
    R = ev.routes()
    names = sorted(f[:-4] for f in os.listdir(ev.CACHE_DIR) if f.endswith('.pkl'))
    seen, runs = {}, {}
    for bag in names:
        D = ev.load_cache(bag)
        fb = D.get('/vehicle/front_bogie_velocity') or {}
        if len(fb.get('v', [])) == 0:
            continue
        md5 = hashlib.md5(np.asarray(fb['v']).tobytes() + np.asarray(fb['t_hdr']).tobytes()).hexdigest()
        if md5 in seen:
            continue
        seen[md5] = bag
        run = make_run(bag, D, R)
        if run is not None:
            runs[bag] = run
    if verbose:
        print('unique bags %d, with RTK master track %d' % (len(seen), len(runs)))
    return runs


def make_run(bag, D, R, xyz_override=None):
    fb = D['/vehicle/front_bogie_velocity']
    o = np.argsort(fb['t_hdr'], kind='stable')
    tw = np.asarray(fb['t_hdr'], float)[o]
    W = wheel_odometer(tw, np.asarray(fb['v'], float)[o], bag[:5])
    if xyz_override is not None:
        t, xyz = xyz_override
    else:
        m = ev.fixes(D, 'master')
        if m is None:
            return None
        ok = m['st'] == 2
        if ok.sum() < 100:
            return None
        t, xyz, trec = m['t'][ok], m['xyz'][ok], m['t_rec'][ok]
        bad = ev.stamp_glitch_mask(t, trec)
        gv = D.get(ev.TOPIC_MVEL) or {}
        if len(gv.get('t_hdr', [])):
            ov = np.argsort(gv['t_hdr'], kind='stable')
            bad |= ev.position_glitch_mask(t, xyz, np.asarray(gv['t_hdr'], float)[ov], np.hypot(gv['vx'], gv['vy'])[ov])
        t, xyz = t[~bad], xyz[~bad]
    d = ev.detect_direction(xyz[:, :2])
    if d is None:
        return None
    rt = R[d]
    s, dist, lat, _ = rt.project(xyz[:, :2])
    on = (dist < 2.0) & (s >= 0) & (s <= rt.L)
    if on.sum() < 50:
        return None
    zmap = np.interp(np.clip(s, 0, rt.L), rt.s, rt.P[:, 2])
    q = float(np.mean((np.abs(lat[on] - np.median(lat[on])) < 0.15) & (np.abs(xyz[on, 2] - zmap[on] - ALT_OFF) < 0.3)))
    run = dict(bag=bag, dir=d, q=q, t=t, xyz=xyz, s=s, on=on, tw=tw, W=W, veh=bag[:5], segs={})
    ion = np.flatnonzero(on)
    i0, i1 = ion[0], ion[-1]
    Wf = np.interp(t, tw, W)
    if i0 >= 20 and s[i0] < 20.0:
        P = xyz[:i0 + 1]
        a = track_arc(P)
        u = s[i0] - (a[-1] - a)
        wj = Wf[i0] - s[i0]                       # odometer at the joint (s = 0)
        run['segs']['head'] = dict(t=t[:i0], xyz=xyz[:i0], u=u[:-1], w=Wf[:i0] - wj, tj=time_at_odo(tw, W, wj))
    if i1 <= len(t) - 20 and s[i1] > rt.L - 5.0:
        P = xyz[i1:]
        a = track_arc(P)
        u = (s[i1] - rt.L) + a
        wj = Wf[i1] + (rt.L - s[i1])              # odometer at the joint (s = L)
        run['segs']['tail'] = dict(t=t[i1 + 1:], xyz=xyz[i1 + 1:], u=u[1:], w=Wf[i1 + 1:] - wj, tj=time_at_odo(tw, W, wj))
    return run


def terminus_segments(runs, loop, kind):
    """Segments of one (loop, kind): loop S tails come from T2S, S heads go to S2T, etc."""
    fr, to = LOOPS[loop]
    d = fr if kind == 'tail' else to
    out = []
    for r in runs.values():
        if r['dir'] == d and kind in r['segs'] and len(r['segs'][kind]['u']) >= 10:
            sg = dict(r['segs'][kind]); sg.update(bag=r['bag'], q=r['q'], veh=r['veh'], kind=kind, loop=loop)
            out.append(sg)
    return out


# ------------------------------------------------------------------ geometry building
def seg_poly(sg):
    """Segment track on a 1 m grid of wheel distance from the joint: (grid, xy, z).

    Per-bin medians of the fixes, so stationary jitter (stops) collapses into one bin."""
    w = sg['w']
    b = np.round(w).astype(int)
    b0, b1 = b.min(), b.max()
    nb = b1 - b0 + 1
    bi = b - b0
    X = np.c_[group_median(bi, sg['xyz'][:, 0], nb), group_median(bi, sg['xyz'][:, 1], nb),
              group_median(bi, sg['xyz'][:, 2], nb)]
    g = np.arange(b0, b1 + 1, dtype=float)
    f = np.isfinite(X[:, 0])
    X = np.c_[[np.interp(g, g[f], X[f, k]) for k in range(3)]].T
    keep = g >= 0 if sg['kind'] == 'tail' else g <= 0
    return g[keep], X[keep, :2], X[keep, 2]


def seg_cover(sg):
    return float(np.abs(sg['w']).max())


def pick_seed(members):
    """Seed track: good quality, near-maximal coverage, own arc length consistent with the wheels."""
    cov = np.array([seg_cover(m) for m in members])
    ok = [i for i in range(len(members)) if arc_ok(members[i])] or list(range(len(members)))
    ref = max(cov[i] for i in ok)
    cand = [i for i in ok if cov[i] >= 0.97 * ref]
    i = max(cand, key=lambda i: (members[i]['q'], cov[i]))
    return members[i]


def arc_ok(m):
    """Own track arc length agrees with the wheel distance within 2 % (no jitter / wheel glitch)."""
    return abs((m['u'].max() - m['u'].min()) / max(seg_cover(m), 1.0) - 1.0) < 0.02


def seg_distance(a, b):
    """p90 of the distance from track a to track b over the common u range (both ways, max)."""
    from scipy.spatial import cKDTree
    out = []
    for p, q in ((a, b), (b, a)):
        gp, Pp, _ = seg_poly(p)
        gq, Pq, _ = seg_poly(q)
        m = (gp >= gq[0]) & (gp <= gq[-1])
        if m.sum() < 10:
            return np.inf
        d, _ = cKDTree(Pq).query(Pp[m])
        out.append(np.percentile(d, 90))
    return max(out)


def cluster(segs):
    # reference-grade runs (quality >= 0.85, consistent arc) seed the clusters, longest first
    good = sorted([s for s in segs if s['q'] >= Q_MIN],
                  key=lambda s: (-int(bool(s['q'] >= 0.85 and arc_ok(s))), -seg_cover(s)))
    cl = []
    for s in good:
        for c in cl:
            if seg_distance(s, c[0]) < CLUSTER_THR:
                c.append(s)
                break
        else:
            cl.append([s])
    cl.sort(key=len, reverse=True)
    # keep the main cluster first even when shorter runs outnumber: main = the one with most members
    return cl


def template(members, n_iter=3):
    """Robust median centreline of a cluster: (xy on ~1 m arc, z, support count, u of the points)."""
    seed = pick_seed(members)
    g, P, _ = seg_poly(seed)
    kind = seed['kind']
    for it in range(n_iter + 1):
        a = arclen(P)
        rt = ev.Route('tpl', np.c_[P, np.zeros(len(P))])
        nb = len(P)
        lat_runs, z_runs = [], []
        for sg in members:
            s, dist, lat, bey = rt.project(sg['xyz'][:, :2])
            ok = (dist < (5.0 if it == 0 else 2.5)) & ~bey
            b = np.clip(np.round(np.interp(s, a, np.arange(nb))).astype(int), 0, nb - 1)
            lat_runs.append(group_median(b[ok], lat[ok], nb))
            z_runs.append(group_median(b[ok], sg['xyz'][ok, 2] - ALT_OFF, nb))
        LR = np.array(lat_runs); ZR = np.array(z_runs)
        cnt = np.isfinite(LR).sum(0)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            lat = np.nanmedian(LR, 0); z = np.nanmedian(ZR, 0)
        idx = np.arange(nb)
        f = np.isfinite(lat)
        lat = np.interp(idx, idx[f], lat[f]); z = np.interp(idx, idx[f], z[f])
        if it == n_iter:
            break
        lat = movavg(lat, LAT_SMOOTH)
        P = P + lat[:, None] * normals(P)
        # keep the anchored end fixed in u: tails start at u = 0, heads end at u = 0
        P = resample(P)
    z = movavg(z, Z_SMOOTH)
    if CURVE_K2 > 0:
        # antenna path -> track centreline: the antenna sits ANT_LAT0 + CURVE_K2 * curvature to the right (F4)
        k = curvature(P)
        P = resample(P + (ANT_LAT0 + CURVE_K2 * k)[:, None] * normals(P))
    a = arclen(P)
    u = a if kind == 'tail' else a - a[-1]
    return dict(P=P, z=np.interp(np.linspace(0, 1, len(P)), np.linspace(0, 1, len(z)), z), cnt=cnt, u=u,
                kind=kind, n=len(members), bags=[m['bag'] for m in members], seed=seed['bag'])


def join(P1, z1, P2, z2, max_overlap_d=1.0):
    """Join two forward templates: P1 then P2.  Overlap -> cut in the middle; gap -> None."""
    rt = ev.Route('p1', np.c_[P1, np.zeros(len(P1))])
    s, d, lat, bey = rt.project(P2)
    ov = (d < max_overlap_d) & ~bey
    if not ov[0]:
        gap = float(np.hypot(*(P2[0] - P1[-1])))
        if gap > 5.0:
            return None
        # (almost) touching ends: plain concatenation, the final smoothing absorbs the step
        return np.r_[P1, P2], np.r_[z1, z2], float(arclen(P1)[-1]), dict(overlap_m=-gap, overlap_lat_p50=np.nan,
                                                                          overlap_lat_max=np.nan)
    k = int(np.argmin(ov)) if not ov.all() else len(ov)
    jm = k // 2
    a1 = arclen(P1)
    keep1 = a1 < s[jm]
    info = dict(overlap_m=float(arclen(P2[:max(k, 1)])[-1]), overlap_lat_p50=float(np.median(np.abs(lat[:max(k, 1)]))),
                overlap_lat_max=float(np.max(np.abs(lat[:max(k, 1)]))))
    return np.r_[P1[keep1], P2[jm:]], np.r_[z1[keep1], z2[jm:]], float(a1[keep1][-1]), info


def hermite(p0, th0, p1, th1, m):
    t = np.linspace(0.0, 1.0, 2001)[:, None]
    m0 = m * np.array([math.cos(th0), math.sin(th0)]); m1 = m * np.array([math.cos(th1), math.sin(th1)])
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1; h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2; h11 = t ** 3 - t ** 2
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1


def balloon(p0, th0, p1, r=18.0, stem=25.0):
    """Placeholder balloon turnaround p0 (heading th0) -> p1 (heading ~ th0 + pi), R >= ~15 m.

    Used only where no run was observed (the far end of the double-track branch B)."""
    from scipy.interpolate import CubicSpline
    c, s_ = math.cos(th0), math.sin(th0)
    d = np.asarray(p1, float) - np.asarray(p0, float)
    x1, y1 = d @ [c, s_], d @ [-s_, c]
    sg = 1.0 if y1 >= 0 else -1.0
    y1 *= sg
    xs = max(8.0, x1 + 8.0)
    xc, yc = xs + stem, y1 / 2
    phi = np.radians(np.arange(200, 521, 40))
    circ = np.c_[xc + r * np.cos(phi), yc + r * np.sin(phi)]
    C = np.r_[[[0.0, 0.0], [8.0, 0.0]], circ, [[x1 + 8.0, y1], [x1, y1]]]
    t = np.r_[0.0, np.cumsum(np.hypot(*np.diff(C, axis=0).T))]
    sp = CubicSpline(t, C, bc_type=((1, [1.0, 0.0]), (1, [-1.0, 0.0])))
    Q = sp(np.linspace(0, t[-1], 4000))
    Q[:, 1] *= sg
    return np.asarray(p0, float) + np.c_[Q[:, 0] * c - Q[:, 1] * s_, Q[:, 0] * s_ + Q[:, 1] * c]


def monotone(P, win=9, min_step=0.2):
    """Drop points without forward progress along the smoothed tangent (stationary jitter, back-steps)."""
    T = np.gradient(np.c_[movavg(P[:, 0], win), movavg(P[:, 1], win)], axis=0)
    T /= np.maximum(np.hypot(T[:, 0], T[:, 1]), 1e-9)[:, None]
    keep = [0]
    for i in range(1, len(P)):
        j = keep[-1]
        if (P[i] - P[j]) @ T[j] > min_step and (P[i] - P[j]) @ T[i] > 0:
            keep.append(i)
    if keep[-1] != len(P) - 1:
        keep[-1] = len(P) - 1
    return np.array(keep)


def arc_minus_chord(P, win=10):
    a = arclen(P)
    n = len(P)
    if n <= win:
        return 0.0, 0.0
    ch = np.hypot(*(P[win:] - P[:-win]).T)
    e = (a[win:] - a[:-win]) - ch
    i = int(np.argmax(e))
    return float(e[i]), float(a[i])


def heading_end(P, back=5):
    d = P[-1] - P[-1 - back]
    return math.atan2(d[1], d[0])


def heading_start(P, fwd=5):
    d = P[fwd] - P[0]
    return math.atan2(d[1], d[0])


def arc_ext(x0, y0, yaw0, k, a):
    """Points along a constant-curvature arc from (x0, y0, yaw0), signed arc a."""
    if abs(k) < 1e-6:
        return np.c_[x0 + a * math.cos(yaw0), y0 + a * math.sin(yaw0)]
    return np.c_[x0 + (np.sin(yaw0 + k * a) - math.sin(yaw0)) / k, y0 - (np.cos(yaw0 + k * a) - math.cos(yaw0)) / k]


def finalize(P, z, unobs, Rf, Rt):
    """Anchor at the map joints, blend into the map poses (tangent-continuous), resample every ~1 m.

    ``unobs``: list of [a0, a1] arc intervals of P; returned re-mapped to the final arc."""
    P, z = resample(P, z)
    for _ in range(2):
        k = monotone(P)
        P, z = resample(P[k], z[k])
    Ps = np.c_[movavg(P[:, 0], 5), movavg(P[:, 1], 5)]
    Ps[:3], Ps[-3:] = P[:3], P[-3:]
    P, z = resample(Ps, z)
    a = arclen(P)
    # anchor: projections of the map end / start onto the branch
    rt = ev.Route('b', np.c_[P, np.zeros(len(P))])
    xe, ye, ze, yawe = Rf.pose(Rf.L)
    xs, ys, zs, yaws = Rt.pose(0.0)
    sa, _, _, _ = rt.project(np.array([[xe, ye]]))
    # the end joint must be searched near the end (loops pass close to their own start)
    tail_part = a > a[-1] - 60
    rt2 = ev.Route('b2', np.c_[P[tail_part], np.zeros(tail_part.sum())])
    sb, _, _, _ = rt2.project(np.array([[xs, ys]]))
    a0, a1 = float(sa[0]), float(a[tail_part][0] + sb[0])
    g = np.arange(a0, a1, 1.0)
    g = np.r_[g, a1] if a1 - g[-1] > 0.05 else np.r_[g[:-1], a1]
    # points beyond the polyline ends: straight continuation
    t0 = (P[1] - P[0]) / max(np.hypot(*(P[1] - P[0])), 1e-9)
    t1 = (P[-1] - P[-2]) / max(np.hypot(*(P[-1] - P[-2])), 1e-9)
    Q = np.c_[np.interp(g, a, P[:, 0]), np.interp(g, a, P[:, 1])]
    lo, hi = g < 0, g > a[-1]
    Q[lo] = P[0] + np.outer(g[lo], t0); Q[hi] = P[-1] + np.outer(g[hi] - a[-1], t1)
    zq = np.interp(g, a, z)
    u = g - a0
    Lb = u[-1]
    # start blend: map end continued along its curvature
    ke = float(np.mean(Rf.curv[-10:]))
    we = 1.0 - smoothstep(u / BLEND_XY)
    Q = we[:, None] * arc_ext(xe, ye, yawe, ke, u) + (1 - we[:, None]) * Q
    ks = float(np.mean(Rt.curv[:10]))
    ws = 1.0 - smoothstep((Lb - u) / BLEND_XY)
    Q = ws[:, None] * arc_ext(xs, ys, yaws, ks, u - Lb) + (1 - ws[:, None]) * Q
    ge = float(np.mean(Rf.grade[-20:])); gs = float(np.mean(Rt.grade[:20]))
    wze = 1.0 - smoothstep(u / BLEND_Z); wzs = 1.0 - smoothstep((Lb - u) / BLEND_Z)
    zq = wze * (ze + ge * u) + (1 - wze) * zq
    zq = wzs * (zs + gs * (u - Lb)) + (1 - wzs) * zq
    # final uniform resampling (~1 m)
    a_old = arclen(Q)
    Qf, zf = resample(Q, zq)
    Lf = arclen(Qf)[-1]
    un = [[float(np.interp(i0 - a0, u, a_old)), float(np.interp(i1 - a0, u, a_old))] for i0, i1 in unobs]
    un = [[max(0.0, x0) * Lf / a_old[-1], min(a_old[-1], x1) * Lf / a_old[-1]] for x0, x1 in un if x1 > x0]
    return Qf, zf, un


def build(runs, verbose=True):
    RR = load_routes(ev.MAPS_DIR)
    res = {'loops': {}, 'templates': {}, 'joins': {}}
    for loop in ('S', 'T'):
        fr, to = LOOPS[loop]
        tails = terminus_segments(runs, loop, 'tail')
        heads = terminus_segments(runs, loop, 'head')
        ct, ch = cluster(tails), cluster(heads)
        if verbose:
            for kind, cl in (('tail', ct), ('head', ch)):
                for i, c in enumerate(cl):
                    sd = pick_seed(c)
                    print('  loop %s %s cluster %d: %d runs, seed %s (wheel %.0f m, q %.2f)' % (
                        loop, kind, i, len(c), sd['bag'], seg_cover(sd), sd['q']))
        Tt = [template(c) for c in ct]
        Th = [template(c) for c in ch]
        res['templates'][loop] = dict(tail=Tt, head=Th)
        branches = []
        if loop == 'S':
            j = join(Tt[0]['P'], Tt[0]['z'], Th[0]['P'], Th[0]['z'])
            if j is None:
                raise RuntimeError('Shchukinskaya tail and head templates do not overlap')
            P, z, acut, info = j
            res['joins']['S'] = info
            branches.append(dict(name='A', P=P, z=z, unobs=[], n_tail=Tt[0]['n'], n_head=Th[0]['n'],
                                 bags=Tt[0]['bags'] + Th[0]['bags']))
        else:
            # identify tail/head clusters: A = the tail cluster reaching farthest (main loop to the platform)
            tA = max(Tt, key=lambda T: T['u'][-1] if T['n'] > 0 else 0)
            tB = [T for T in Tt if T is not tA]
            # head clusters: deviation from the main head (0 near the merge); B ~25 m, C ~5 m
            hA = Th[0]
            dev = []
            for T in Th:
                rt = ev.Route('hA', np.c_[hA['P'], np.zeros(len(hA['P']))])
                s, d, lat, bey = rt.project(T['P'])
                dev.append(float(np.max(d)))
            j = join(tA['P'], tA['z'], hA['P'], hA['z'])
            if j is None:
                raise RuntimeError('Tallinskaya main tail and head do not overlap')
            PA, zA, acut, info = j
            res['joins']['T_A'] = info
            branches.append(dict(name='A', P=PA, z=zA, unobs=[], n_tail=tA['n'], n_head=hA['n'],
                                 bags=tA['bags'] + hA['bags']))
            aA = arclen(PA)
            for T, dv in zip(Th[1:], dev[1:]):
                if dv > 15.0 and tB:
                    # far branch B: tail B + unobserved turnaround + head B
                    t = tB[0]
                    p0, p1 = t['P'][-1], T['P'][0]
                    br = balloon(p0, heading_end(t['P']), p1)
                    Pb = np.r_[t['P'], br[1:-1], T['P']]
                    zb = np.r_[t['z'], np.linspace(t['z'][-1], T['z'][0], len(br))[1:-1], T['z']]
                    a_t = arclen(t['P'])[-1]
                    a_b = a_t + arclen(br)[-1]
                    branches.append(dict(name='B', P=Pb, z=zb, unobs=[[a_t, a_b]], n_tail=t['n'], n_head=T['n'],
                                         bags=t['bags'] + T['bags'], dev=dv, gap=float(np.hypot(*(p1 - p0)))))
                elif dv > 3.0:
                    # parallel variant C: lateral offset profile relative to A, ramped in before its first fix
                    lo = aA[-1] - (T['u'][-1] - T['u'][0]) - 80.0
                    sel = aA >= lo
                    s, d, lat = project_window(PA, aA, T['P'], aA[-1] + T['u'], half=30.0)
                    offs = np.zeros(len(PA))
                    ac0 = s[0]
                    m = aA >= ac0
                    offs[m] = np.interp(aA[m], s, lat)
                    ramp = (aA >= ac0 - C_RAMP) & (aA < ac0)
                    offs[ramp] = lat[0] * smoothstep((aA[ramp] - (ac0 - C_RAMP)) / C_RAMP)
                    Pc = PA + offs[:, None] * normals(PA)
                    zc = zA.copy()
                    zc[m] = np.interp(aA[m], s, T['z'])
                    zc[ramp] = zA[ramp] + (T['z'][0] - zA[ramp]) * smoothstep((aA[ramp] - (ac0 - C_RAMP)) / C_RAMP)
                    zc = movavg(zc, 5)
                    del sel
                    branches.append(dict(name='C', P=Pc, z=zc, unobs=[[ac0 - C_RAMP, ac0]], n_tail=0, n_head=T['n'],
                                         bags=T['bags'], dev=dv, offset_first=float(lat[0])))
        out = []
        for b in branches:
            Q, zq, un = finalize(b['P'], b['z'], b['unobs'], RR[fr], RR[to])
            b.update(Q=Q, zq=zq, un=un, length=float(arclen(Q)[-1]))
            out.append(b)
        # most frequent first, A stays first (main)
        out = [out[0]] + sorted(out[1:], key=lambda b: (-(b['n_tail'] + b['n_head']), b['name']))
        res['loops'][loop] = dict(frm=fr, to=to, branches=out)
    return res


def write_json(res, path=OUT_JSON):
    J = {'version': 1,
         'source': ('tools/build_loops.py: median of RTK (status 2) master-antenna tracks of the unique training '
                    'bags (runs with on-map quality >= %.2f), map frame (UTM37N - (300000, 6100000)), '
                    'z = GNSS alt - %.2f m, blended into the pathgraph ends; points every ~1 m of arc. '
                    '"unobserved" = arc intervals not covered by any fix (placeholder geometry).' % (Q_MIN, ALT_OFF)),
         'loops': {}}
    for loop, L in res['loops'].items():
        J['loops'][loop] = {'from': L['frm'], 'to': L['to'], 'branches': [
            {'name': b['name'], 'n_runs': int(b['n_tail'] + b['n_head']), 'n_tail': int(b['n_tail']),
             'n_head': int(b['n_head']), 'length': round(b['length'], 2),
             'unobserved': [[round(x0, 1), round(x1, 1)] for x0, x1 in b['un']],
             'x': [round(v, 3) for v in b['Q'][:, 0]], 'y': [round(v, 3) for v in b['Q'][:, 1]],
             'z': [round(v, 3) for v in b['zq']]} for b in L['branches']]}
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(J, f, separators=(',', ':'))
    os.replace(tmp, path)
    return J


# ------------------------------------------------------------------ validation
def branch_checks(res):
    RR = load_routes(ev.MAPS_DIR)
    print('\nBranch geometry checks')
    for loop, L in res['loops'].items():
        Rf, Rt = RR[L['frm']], RR[L['to']]
        xe, ye, _, yawe = Rf.pose(Rf.L); xs, ys, _, yaws = Rt.pose(0.0)
        for b in L['branches']:
            Q = b['Q']; a = arclen(Q)
            hd = np.unwrap(np.arctan2(*np.diff(Q, axis=0).T[::-1]))
            k3 = np.abs(np.diff(hd[::3])) / 3.0
            print('  %s/%s: L=%.1f m, n_runs=%d (tail %d, head %d), start err %.2f m, end err %.2f m, '
                  'joint heading err %.2f / %.2f deg, spacing %.3f m, Rmin(3 m) %.1f m, z %.1f..%.1f, unobserved %s' % (
                      loop, b['name'], b['length'], b['n_tail'] + b['n_head'], b['n_tail'], b['n_head'],
                      math.hypot(Q[0, 0] - xe, Q[0, 1] - ye), math.hypot(Q[-1, 0] - xs, Q[-1, 1] - ys),
                      math.degrees(abs((hd[0] - yawe + math.pi) % (2 * math.pi) - math.pi)),
                      math.degrees(abs((hd[-1] - yaws + math.pi) % (2 * math.pi) - math.pi)),
                      np.median(np.diff(a)), 1.0 / max(k3.max(), 1e-9), b['zq'].min(), b['zq'].max(),
                      [[round(x, 1) for x in u] for u in b['un']]))
            e, ae = arc_minus_chord(Q)
            print('      arc - chord over 10 m: max %.3f m at arc %.0f m' % (e, ae))
    for k, v in res['joins'].items():
        print('  join %s: overlap %.1f m, |lateral| p50 %.2f max %.2f m' % (k, v['overlap_m'], v['overlap_lat_p50'], v['overlap_lat_max']))


def validate_segment(sg, branches, kind):
    """Windowed projection onto every branch; returns per-branch stats and the best branch."""
    Q = sg['xyz'][:, :2]
    w = sg['w']                                   # odometer relative to the joint (tail > 0, head < 0)
    out = {}
    for b in branches:
        P = b['Q']; a = arclen(P); Lb = a[-1]
        a_pred = w if kind == 'tail' else Lb + w
        s, d, _ = project_window(P, a, Q, a_pred)
        dist_j = s if kind == 'tail' else Lb - s
        wd = np.abs(w)
        m = wd > 30.0
        rel = (dist_j[m] - wd[m]) / wd[m]
        out[b['name']] = dict(d=d, rel=rel, relend=float(rel[np.argmax(wd[m])]) if m.any() else np.nan,
                              p95=float(np.percentile(d, 95)))
    best = min(out, key=lambda k: out[k]['p95'])
    same = [k for k in out if out[k]['p95'] < out[best]['p95'] + 0.3]
    return best, same, out


def validate(res, runs, test=None):
    print('\nValidation: off-map RTK master fixes vs branches (window +-%.0f m around the wheel arc)' % WIN)
    print('  %-16s %-9s %5s %6s %-6s %6s %6s %6s %8s %8s' % ('run', 'segment', 'q', 'len', 'branch', 'p50', 'p95', 'max', 'rel_med', 'rel_end'))
    pooled = {}
    rows = list(runs.values()) + ([test] if test else [])
    for r in rows:
        for kind in ('head', 'tail'):
            if kind not in r['segs']:
                continue
            sg = r['segs'][kind]
            if len(sg['u']) < 10:
                continue
            loop = [lp for lp, (fr, to) in LOOPS.items() if (fr if kind == 'tail' else to) == r['dir']][0]
            label = '%s %s' % (r['dir'], kind)
            best, same, st = validate_segment(sg, res['loops'][loop]['branches'], kind)
            o = st[best]
            name = '=' .join(sorted(same)) if len(same) > 1 else best
            rel = o['rel']
            print('  %-16s %-9s %5.2f %6.0f %-6s %6.2f %6.2f %6.2f %8s %8s' % (
                r['bag'][-16:], label, r['q'], np.abs(sg['w']).max(), name, np.percentile(o['d'], 50),
                o['p95'], o['d'].max(),
                '%+.2f%%' % (100 * np.median(rel)) if len(rel) else '-', '%+.2f%%' % (100 * o['relend']) if len(rel) else '-'))
            if r is not test and r['q'] >= Q_MIN:
                P = pooled.setdefault(label, dict(d=[], rel=[], relend=[], br={}))
                P['d'].append(o['d']); P['rel'].append(rel)
                if len(rel):
                    P['relend'].append(o['relend'])
                P['br'][name] = P['br'].get(name, 0) + 1
    print('\n  Pooled over runs with q >= %.2f (cross-track to the best branch; rel = (loop arc - wheel dist) / wheel dist, |wheel dist| > 30 m):' % Q_MIN)
    for label, P in sorted(pooled.items()):
        d = np.concatenate(P['d']); rel = np.concatenate(P['rel']); re = np.array(P['relend'])
        print('  %-9s runs %2d  xtrack p50 %.2f p95 %.2f max %.2f m | rel all fixes: median %+.2f%% p95|.| %.2f%% | '
              'rel at farthest fix: median %+.2f%% p95|.| %.2f%% | branches %s' % (
                  label, len(P['d']), np.percentile(d, 50), np.percentile(d, 95), d.max(),
                  100 * np.median(rel), 100 * np.percentile(np.abs(rel), 95), 100 * np.median(re),
                  100 * np.percentile(np.abs(re), 95), P['br']))


def load_test_run():
    """Organisers' test bag: master RTK run + base_link reference run (both as run dicts)."""
    if not os.path.isdir(TEST_BAG):
        return None, None
    sys.path.insert(0, TEST_LOADER)
    import load_test  # monkey-patches bagio to parse nav_msgs/Odometry
    from tram_backup_odometry import bagio
    D = bagio.load_bag(TEST_BAG)
    R = ev.routes()
    bag = os.path.basename(TEST_BAG)
    m = make_run(bag, D, R)
    o = D.get('/localization/kinematic_state')
    ref = None
    if o is not None and len(o.get('x', [])):
        so = np.argsort(o['t_hdr'], kind='stable')
        ref = make_run(bag, D, R, xyz_override=(np.asarray(o['t_hdr'], float)[so], np.c_[o['x'], o['y'], o['z']][so]))
        if ref is not None:
            ref['bag'] = bag + '_ref'
            ref['t0'] = float(np.min(D['/vehicle/front_bogie_velocity']['t_rec']))
    if m is not None:
        m['bag'] = bag + '_rtk'
    return m, ref


def test_ref_report(res, ref):
    """Cross-track of the reference's off-map parts (plain nearest distance to every branch)."""
    from scipy.spatial import cKDTree
    t0 = ref.get('t0', ref['t'][0])
    trel = ref['t'] - t0
    print('\nTest bag reference (/localization/kinematic_state, base_link = master + %.3f m ahead), direction %s' % (REF_AHEAD, ref['dir']))
    for label, m, loop in (('start (t<220 s)', trel < 220, LOOPS_BY_ROUTE_HEAD[ref['dir']]),
                           ('end (t>1228 s)', trel > 1228, LOOPS_BY_ROUTE_TAIL[ref['dir']])):
        off = m & ~ref['on']
        Q = ref['xyz'][off, :2]
        if not len(Q):
            print('  %s: no off-map samples' % label)
            continue
        for b in res['loops'][loop]['branches']:
            P = resample(b['Q'], step=0.25)
            d, _ = cKDTree(P).query(Q)
            print('  %-16s loop %s branch %s: %4d off-map samples, nearest-distance p50 %.2f p95 %.2f max %.2f m' % (
                label, loop, b['name'], len(Q), np.percentile(d, 50), np.percentile(d, 95), d.max()))


LOOPS_BY_ROUTE_HEAD = {to: lp for lp, (fr, to) in LOOPS.items()}
LOOPS_BY_ROUTE_TAIL = {fr: lp for lp, (fr, to) in LOOPS.items()}


# ------------------------------------------------------------------ plot
def plot(res, runs, path=OUT_PNG):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    R = ev.routes()
    fig, axs = plt.subplots(1, 2, figsize=(16, 8))
    cols = {'A': '#1f77b4', 'B': '#d62728', 'C': '#2ca02c'}
    for ax, loop, title in zip(axs, ('T', 'S'), ('Tallinskaya (S2T end -> T2S start)', 'Shchukinskaya (T2S end -> S2T start)')):
        L = res['loops'][loop]
        allQ = np.concatenate([b['Q'] for b in L['branches']])
        lo, hi = allQ.min(0) - 25, allQ.max(0) + 25
        for r in runs.values():
            for kind in ('head', 'tail'):
                if kind in r['segs']:
                    X = r['segs'][kind]['xyz']
                    m = (X[:, 0] > lo[0]) & (X[:, 0] < hi[0]) & (X[:, 1] > lo[1]) & (X[:, 1] < hi[1])
                    ax.plot(X[m, 0], X[m, 1], '.', ms=0.6, color='0.6', alpha=0.4, zorder=1)
        for name, rt in R.items():
            ax.plot(rt.P[:, 0], rt.P[:, 1], color='k', lw=1.2, zorder=2)
        fr, to = LOOPS[loop]
        ax.plot(*R[fr].P[-1, :2], 'ks', ms=7, zorder=5, label='%s map end' % fr)
        ax.plot(*R[to].P[0, :2], 'k^', ms=7, zorder=5, label='%s map start' % to)
        for b in L['branches']:
            Q = b['Q']; a = arclen(Q)
            c = cols.get(b['name'], 'm')
            obs = np.ones(len(Q), bool)
            for x0, x1 in b['un']:
                obs &= ~((a >= x0) & (a <= x1))
            Qo = Q.copy(); Qo[~obs] = np.nan
            ax.plot(Qo[:, 0], Qo[:, 1], color=c, lw=1.6, zorder=3,
                    label='%s: %.0f m, %d runs' % (b['name'], b['length'], b['n_tail'] + b['n_head']))
            if (~obs).any():
                Qu = Q.copy(); Qu[obs] = np.nan
                ax.plot(Qu[:, 0], Qu[:, 1], color=c, lw=1.6, ls=':', zorder=3)
            i = len(Q) // 2
            ax.annotate('', Q[i + 3], Q[i], arrowprops=dict(arrowstyle='->', color=c, lw=1.5), zorder=4)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect('equal')
        ax.grid(alpha=0.3); ax.set_title(title); ax.legend(fontsize=8, loc='best')
        ax.set_xlabel('x, m (map frame)'); ax.set_ylabel('y, m')
    fig.suptitle('Terminus loops (maps/loops.json): branches (dotted = unobserved placeholder), grey = training RTK master tracks')
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    print('plot -> %s' % os.path.relpath(path, ROOT))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=OUT_JSON)
    ap.add_argument('--no-test-bag', action='store_true')
    ap.add_argument('--no-plot', action='store_true')
    ap.add_argument('--antenna-path', action='store_true',
                    help='keep the master-antenna path (no shift to the track centreline in curves)')
    args = ap.parse_args(argv)
    global CURVE_K2, ANT_LAT0
    if args.antenna_path:
        CURVE_K2, ANT_LAT0 = 0.0, 0.0
    runs = load_runs()
    res = build(runs)
    J = write_json(res, args.out)
    print('loops -> %s' % os.path.relpath(args.out, ROOT))
    for loop, L in J['loops'].items():
        print('  %s (%s -> %s): %s' % (loop, L['from'], L['to'], ', '.join(
            '%s %.1f m (%d runs)' % (b['name'], b['length'], b['n_runs']) for b in L['branches'])))
    branch_checks(res)
    test = ref = None
    if not args.no_test_bag:
        test, ref = load_test_run()
    validate(res, runs, test)
    if ref is not None:
        validate(res, {}, ref)
        test_ref_report(res, ref)
    if not args.no_plot:
        plot(res, runs)


if __name__ == '__main__':
    main()
