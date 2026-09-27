#!/usr/bin/env python3
"""Judge-like evaluator of speed and position estimates against the GNSS reference.

Reference (what the jury most likely uses, see docs/analysis_notes.md, baseline_eval)
------------------------------------------------------------------------------------
* Position: GNSS fixes with ``status == 2`` (RTK) of the chosen antenna, converted
  to UTM 37N (lon0 = 39) minus (300000, 6100000) = the pathgraph map frame;
  z = GNSS altitude.  ``antenna`` = ``master`` (default, the map is drawn through
  the master antenna), ``rover`` (12.44 m ahead along the track) or ``mid``
  (mean of master and rover fixes paired within 0.05 s, both RTK).
* Speed: ``hypot(vx, vy)`` of ``/sensing/gnss/master/vel``, kept only where a
  status-2 master fix lies within 0.05 s.
* All stamps are header stamps.  The reference is sorted by stamp; header-stamp
  glitches of the GNSS streams are kept (judge-like) unless ``clean=True``
  (then stamp glitches, status-2 position jumps and frozen-zero speed are dropped).

Matching
--------
For every reference sample take the estimate with the nearest stamp; the pair
counts only if ``|dt| <= tol`` (0.05 s, the README tolerance).  Coverage is the
matched fraction.  Positions with ``has_pose == False`` (or non-finite) are not
candidates, i.e. they count as missing.  Several outputs with one stamp: the
last published one is used.

Metrics
-------
Speed: e = v_est - v_ref over matched samples; RMSE, MAE, bias, p99 and max |e|,
and bias / RMSE per regime.  Regimes come from the reference: 11-sample box
smoothing vs (edge-padded), a = d vs / dt; accel a > 0.15, brake a < -0.15, stopped vs < 0.1,
otherwise cruise.

Position: e2d = |xy_est - xy_ref|, e3d = |xyz_est - xyz_ref|.  Along/cross-track
errors use the map polyline of the run direction: both points are projected onto
it (arc length s, signed lateral l, left positive); along = s_est - s_ref (> 0:
estimate ahead), cross = l_est - l_ref.  Only reference samples within ``gate``
(5 m) of the polyline and not beyond its ends count ("on-map").  End drift is
e2d at the last matched sample divided by the distance travelled up to it
(odometer of the GNSS Doppler speed, all fix statuses, from the start of the
recording: runs with RTK only late in the run are not inflated); the on-map
drift is |along| at the last on-map sample divided by the on-map arc length
covered, and al_growth_pct = (along_last - along_first) / that length (signed:
the scale/drift rate independent of the start offset); both NaN below 200 m.

Score-like summary
------------------
The jury's formula is unknown (speed 30 points: RMSE/MAE + transient bias;
position 35 points: drift %, along-track MEAN/MAX/RMSE, cross-track).  For ranking
parameter variants ``score_loss`` combines normalised metrics:

    loss = sum_i w_i * min(m_i / y_i, 10) / sum_i w_i,
    then loss = cov * loss + (1 - cov) * 10   (missing matches cost the cap),

with (metric, yardstick y, weight w): v_rmse 0.05 m/s 20, max |regime bias|
0.02 m/s 10, drift_pct 0.3 % 10, al_mean_abs 2 m 7, al_rmse 3 m 7, al_max 10 m 7,
ct_rmse 0.5 m 4.  Lower is better; 1.0 means "at the yardstick on every metric".

Usage
-----
    python tools/evaluate.py <bag> --est est.npz [--antenna master|rover|mid] [--clean]
    python tools/evaluate.py --self-test [bag ...]         # reference as estimate -> exactly 0
    python tools/evaluate.py --self-test interp [bag ...]  # reference at the input stamps: matching floor
"""
import argparse
import json
import math
import os
import pickle
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_DIR = os.path.join(ROOT, 'src', 'tram_backup_odometry')
CACHE_DIR = os.path.join(ROOT, '.work', 'cache')
MAPS_DIR = os.path.join(PKG_DIR, 'maps')
DATA_DIR = os.path.join(os.path.dirname(ROOT), 'dataset', 'data')
MAP_FILES = {'T2S': 't2s.json', 'S2T': 's2t.json'}
OFFSET = np.array([300000.0, 6100000.0])

TOPIC_MFIX = '/sensing/gnss/master/fix'
TOPIC_RFIX = '/sensing/gnss/rover/fix'
TOPIC_MVEL = '/sensing/gnss/master/vel'

REGIMES = ('accel', 'brake', 'stopped', 'cruise')
SCORE_TERMS = (  # metric, yardstick, weight
    ('v_rmse', 0.05, 20.0),
    ('v_bias_trans', 0.02, 10.0),
    ('drift_pct', 0.3, 10.0),
    ('al_mean_abs', 2.0, 7.0),
    ('al_rmse', 3.0, 7.0),
    ('al_max', 10.0, 7.0),
    ('ct_rmse', 0.5, 4.0),
)
SCORE_CAP = 10.0
MIN_DRIFT_SPAN_M = 200.0  # on-map drift % needs at least this much on-map arc length

_A, _F = 6378137.0, 1 / 298.257223563


# ------------------------------------------------------------------ geometry
def utm(lat, lon, lon0=39.0, k0=0.9996, fe=500000.0):
    """Vectorised WGS84 -> UTM (Krueger series, mm accuracy)."""
    n = _F / (2 - _F)
    A = _A / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64)
    al = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16, 13 * n ** 2 / 48 - 3 * n ** 3 / 5, 61 * n ** 3 / 240)
    phi = np.radians(np.asarray(lat, float))
    lam = np.radians(np.asarray(lon, float) - lon0)
    e = math.sqrt(_F * (2 - _F))
    t = np.sinh(np.arctanh(np.sin(phi)) - e * np.arctanh(e * np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1 + t * t))
    E = eta + sum(al[j - 1] * np.cos(2 * j * xi) * np.sinh(2 * j * eta) for j in (1, 2, 3))
    N = xi + sum(al[j - 1] * np.sin(2 * j * xi) * np.cosh(2 * j * eta) for j in (1, 2, 3))
    return fe + k0 * A * E, k0 * A * N


class Route:
    """Raw pathgraph polyline (no resampling) with vectorised projection."""

    def __init__(self, name, P):
        self.name = name
        self.P = np.asarray(P, float)
        d = np.diff(self.P[:, :2], axis=0)
        keep = np.r_[True, np.hypot(d[:, 0], d[:, 1]) > 1e-6]
        self.P = self.P[keep]
        d = np.diff(self.P[:, :2], axis=0)
        self.seg = np.hypot(d[:, 0], d[:, 1])
        self.u = d / self.seg[:, None]
        self.s = np.r_[0.0, np.cumsum(self.seg)]
        self.L = float(self.s[-1])
        self.tree = cKDTree(self.P[:, :2])

    def project(self, xy):
        """-> s (extrapolated past the ends), distance, signed lateral (left +), beyond-end flag."""
        xy = np.atleast_2d(np.asarray(xy, float))
        m = len(xy)
        n = len(self.P)
        bs = np.zeros(m); bd = np.full(m, np.inf); bl = np.zeros(m); bo = np.zeros(m, bool)
        if m == 0:
            return bs, bd, bl, bo
        _, i = self.tree.query(xy)
        for k in (i - 1, i):
            k = np.clip(k, 0, n - 2)
            a = self.P[k, :2]
            r = xy - a
            u = self.u[k]
            t = np.einsum('ij,ij->i', r, u)
            lat = u[:, 0] * r[:, 1] - u[:, 1] * r[:, 0]
            lo = (k == 0) & (t < 0)
            hi = (k == n - 2) & (t > self.seg[k])
            tc = np.where(lo | hi, t, np.clip(t, 0, self.seg[k]))
            q = a + tc[:, None] * u
            dd = np.where(lo | hi, np.abs(lat), np.hypot(*(xy - q).T))
            better = dd < bd
            bd[better] = dd[better]; bs[better] = (self.s[k] + tc)[better]
            bl[better] = lat[better]; bo[better] = (lo | hi)[better]
        return bs, bd, bl, bo


_ROUTES = {}


def routes():
    if not _ROUTES:
        for name, fn in MAP_FILES.items():
            with open(os.path.join(MAPS_DIR, fn)) as f:
                pts = json.load(f)['points']
            _ROUTES[name] = Route(name, [[p['x'], p['y'], p['z']] for p in pts])
    return _ROUTES


def nearest(tq, ts):
    """Index of the nearest stamp in sorted ``ts`` for each query, and |dt|."""
    tq = np.asarray(tq, float)
    ts = np.asarray(ts, float)
    if len(ts) == 0:
        return np.zeros(len(tq), int), np.full(len(tq), np.inf)
    if len(ts) == 1:
        return np.zeros(len(tq), int), np.abs(tq - ts[0])
    j = np.clip(np.searchsorted(ts, tq), 1, len(ts) - 1)
    jl = j - 1
    pick = np.where(np.abs(ts[jl] - tq) <= np.abs(ts[j] - tq), jl, j)
    return pick, np.abs(ts[pick] - tq)


# ------------------------------------------------------------------ data / reference
def cache_path(bag):
    return os.path.join(CACHE_DIR, bag + '.pkl')


def load_cache(bag):
    with open(cache_path(bag), 'rb') as f:
        return pickle.load(f)


def fixes(D, antenna):
    """Fixes of one antenna in the map frame, sorted by header stamp (None if absent)."""
    g = D.get('/sensing/gnss/%s/fix' % antenna) or {}
    if len(g.get('lat', [])) == 0:
        return None
    E, N = utm(g['lat'], g['lon'])
    o = np.argsort(g['t_hdr'], kind='stable')
    return dict(t=np.asarray(g['t_hdr'], float)[o], t_rec=np.asarray(g['t_rec'], float)[o],
                xyz=np.c_[E - OFFSET[0], N - OFFSET[1], g['alt']][o], st=np.asarray(g['status'])[o])


def detect_direction(xy, min_moves=5):
    """Majority of on-map (< 6 m from T2S) consecutive arc-length steps: 'T2S' / 'S2T' / None."""
    if len(xy) < 2:
        return None
    s, d, _, _ = routes()['T2S'].project(xy)
    on = d < 6
    ds = np.diff(s)[on[1:] & on[:-1]]
    fwd, bwd = int((ds > 0.05).sum()), int((ds < -0.05).sum())
    if fwd + bwd < min_moves:
        return None
    return 'T2S' if fwd >= bwd else 'S2T'


def stamp_glitch_mask(t_hdr, t_rec, win=51, thr=0.3):
    """Header stamps whose (t_rec - t_hdr) offset departs from its running median by > thr s."""
    if len(t_hdr) < 3:
        return np.zeros(len(t_hdr), bool)
    from scipy.ndimage import median_filter
    off = np.asarray(t_rec, float) - np.asarray(t_hdr, float)
    return np.abs(off - median_filter(off, size=min(win, len(off)), mode='nearest')) > thr


def position_glitch_mask(t, xyz, tv, v, win=1.0, thr=1.5):
    """Fixes whose 1 s displacement disagrees with the integrated GNSS speed by > thr m."""
    if len(t) < 2 or len(tv) < 2:
        return np.zeros(len(t), bool)
    vi = np.r_[0.0, np.cumsum(np.diff(tv) * (v[1:] + v[:-1]) / 2)]
    cum = np.interp(t, tv, vi)
    ib = np.clip(np.searchsorted(t, t - win), 0, len(t) - 1)
    return np.abs(np.hypot(*(xyz[:, :2] - xyz[ib, :2]).T) - (cum - cum[ib])) > thr


def frozen_speed_mask(tv, v, tp, xyz, thr_v=0.05, thr_move=1.0, half=0.5, return_vpos=False):
    """Reference speed ~0 while the fixes move faster than thr_move m/s (frozen GNSS speed).

    The fix-derived speed is the MEDIAN of the consecutive-fix step speeds within
    [t-half, t+half] (>= 3 steps of <= 0.3 s): robust to the 1-2 m position jumps of a
    standing tram and does not flag the first second of real stops.  Samples without
    such evidence are extended over (<= 2 s, still zero speed) from flagged ones:
    frozen runs are bordered by non-RTK fixes.
    Must be computed from fixes that were NOT filtered with ``position_glitch_mask``
    using the same (frozen) speed: that mask would flag exactly the moving fixes.
    """
    tv = np.asarray(tv, float)
    v = np.asarray(v, float)
    vp = np.zeros(len(tv))
    z = np.zeros(len(tv), bool)
    if len(tp) < 4 or len(tv) == 0:
        return (z, vp) if return_vpos else z
    dts = np.diff(tp)
    good = (dts > 1e-3) & (dts <= 0.3)
    tm = ((tp[1:] + tp[:-1]) / 2)[good]
    sp = (np.hypot(*np.diff(xyz[:, :2], axis=0).T)[good] / dts[good])
    lo = np.searchsorted(tm, tv - half)
    hi = np.searchsorted(tm, tv + half)
    evid = (hi - lo) >= 3
    cand = np.where((v < thr_v) & evid)[0]
    for i in cand:
        vp[i] = np.median(sp[lo[i]:hi[i]])
    m = z.copy()
    m[cand] = vp[cand] > thr_move
    if m.any():
        _, dtn = nearest(tv, tv[m])
        ext = (v < thr_v) & ~evid & (dtn <= 2.0)
        if ext.any():
            vp[ext] = np.interp(tv[ext], tv[m], vp[m])
        m |= ext
    return (m, vp) if return_vpos else m


def load_ref(bag, antenna='master', clean=False, D=None):
    """Reference dict (tp, xyz, tv, v, dir, n_glitch) or None when there are no RTK master fixes."""
    D = D if D is not None else load_cache(bag)
    m = fixes(D, 'master')
    if m is None or not (m['st'] == 2).any():
        return None
    mm = m['st'] == 2
    if antenna == 'mid':
        r = fixes(D, 'rover')
        if r is None:
            return None
        j, dt = nearest(m['t'], r['t'])
        ok = (dt < 0.05) & mm & (r['st'][j] == 2)
        t, xyz, t_rec = m['t'][ok], (m['xyz'][ok] + r['xyz'][j[ok]]) / 2, m['t_rec'][ok]
    elif antenna in ('master', 'rover'):
        g = m if antenna == 'master' else fixes(D, 'rover')
        if g is None:
            return None
        ok = g['st'] == 2
        t, xyz, t_rec = g['t'][ok], g['xyz'][ok], g['t_rec'][ok]
    else:
        raise ValueError('antenna must be master|rover|mid')
    if len(t) == 0:  # no RTK fix of this antenna: nothing to score positions against
        return None
    gv = D.get(TOPIC_MVEL) or {}
    if len(gv.get('t_hdr', [])):
        o = np.argsort(gv['t_hdr'], kind='stable')
        tv0 = np.asarray(gv['t_hdr'], float)[o]
        v0 = np.hypot(gv['vx'], gv['vy'])[o]
        tvr0 = np.asarray(gv['t_rec'], float)[o]
    else:
        tv0 = v0 = tvr0 = np.zeros(0)
    # odometer of the GNSS (Doppler) speed over all samples, any fix status: distance travelled
    n_glitch = {}
    fz0 = np.zeros(len(tv0), bool)
    v0o = v0
    if clean:
        # frozen speed first (from stamp-clean fixes), then position glitches against the
        # integrated speed with the frozen samples replaced by the fix-derived speed
        gs = stamp_glitch_mask(t, t_rec)
        fz0, vpos0 = frozen_speed_mask(tv0, v0, t[~gs], xyz[~gs], return_vpos=True)
        v0o = np.where(fz0, vpos0, v0)
    dtv = np.clip(np.diff(tv0), 0.0, 1.0)
    odo = np.r_[0.0, np.cumsum(dtv * (v0o[1:] + v0o[:-1]) / 2)] if len(tv0) > 1 else np.zeros(len(tv0))
    j, dt = nearest(tv0, m['t'])
    okv = (dt < 0.05) & (m['st'][j] == 2)
    tv, v = tv0[okv], v0[okv]
    if clean:
        gp = gs | position_glitch_mask(t, xyz, tv0, v0o)
        gv_ = stamp_glitch_mask(tv, tvr0[okv]) | fz0[okv]
        n_glitch = dict(pos=int(gp.sum()), vel=int(gv_.sum()), vel_frozen=int(fz0[okv].sum()))
        t, xyz = t[~gp], xyz[~gp]
        tv, v = tv[~gv_], v[~gv_]
    return dict(tp=t, xyz=xyz, tv=tv, v=v, dir=detect_direction(m['xyz'][mm][:, :2]),
                antenna=antenna, n_glitch=n_glitch, odo_t=tv0, odo=odo)


def regimes(tv, v):
    if len(v) == 0:
        return np.zeros(0, dtype=object)
    vs = np.convolve(np.r_[np.full(5, v[0]), v, np.full(5, v[-1])], np.ones(11) / 11, mode='valid')  # edge-padded
    # guard against duplicate stamps in np.gradient (relative time: 1e-9 s is below the
    # float64 resolution of absolute unix stamps)
    tt = (tv - tv[0]) + np.arange(len(tv)) * 1e-9
    a = np.gradient(vs, tt) if len(v) > 1 else np.zeros(1)
    r = np.full(len(v), 'cruise', dtype=object)
    r[a > 0.15] = 'accel'
    r[a < -0.15] = 'brake'
    r[vs < 0.1] = 'stopped'
    return r


# ------------------------------------------------------------------ estimates
def as_estimate(est):
    """replay.run_bag() dict / .npz path / npz mapping -> dict(t, v, xyz, has_pose) sorted by t.

    Outputs with a non-finite stamp are dropped (they cannot be matched, and a NaN at the
    end of the sorted stamps would hide the last valid neighbour from ``nearest``)."""
    if isinstance(est, (str, os.PathLike)):
        with np.load(est, allow_pickle=True) as z:
            est = {k: z[k] for k in z.files}
    t = np.asarray(est['t'], float)
    n = len(t)
    v = np.asarray(est.get('v', np.full(n, np.nan)), float)
    xyz = np.column_stack([np.asarray(est.get(k, np.full(n, np.nan)), float) for k in ('x', 'y', 'z')])
    hp = np.asarray(est['has_pose'], bool) if 'has_pose' in est else np.ones(n, bool)
    # reported pose covariance (optional): along/cross variances in the frame of the reported yaw
    cv = np.column_stack([np.asarray(est.get(k, np.full(n, np.nan)), float)
                          for k in ('yaw', 'var_along', 'var_cross')])
    fin = np.isfinite(t)
    if not fin.all():
        t, v, xyz, hp, cv = t[fin], v[fin], xyz[fin], hp[fin], cv[fin]
        n = len(t)
    o = np.argsort(t, kind='stable')
    t, v, xyz, hp, cv = t[o], v[o], xyz[o], hp[o], cv[o]
    last = np.r_[t[1:] != t[:-1], True] if n else np.zeros(0, bool)  # several outputs per stamp: keep the last
    return dict(t=t[last], v=v[last], xyz=xyz[last], has_pose=hp[last], cov=cv[last],
                n_dup_stamps=int(n - last.sum()))


def _stats(e, prefix):
    if len(e) == 0:
        return {prefix + k: np.nan for k in ('_rmse', '_mae', '_bias', '_p99', '_max')}
    a = np.abs(e)
    return {prefix + '_rmse': float(np.sqrt(np.mean(e ** 2))), prefix + '_mae': float(a.mean()),
            prefix + '_bias': float(e.mean()), prefix + '_p99': float(np.percentile(a, 99)),
            prefix + '_max': float(a.max())}


def _nan(*keys):
    return {k: np.nan for k in keys}


# ------------------------------------------------------------------ evaluation
def evaluate(bag, est, antenna='master', tol=0.05, gate=5.0, clean_ref=False, ref=None, series=False, D=None):
    """Metrics dict for one bag (plus 'series' arrays for plotting when series=True)."""
    ref = ref if ref is not None else load_ref(bag, antenna, clean_ref, D=D)
    E = as_estimate(est)
    R = {'bag': bag, 'antenna': antenna, 'has_ref': ref is not None}
    if ref is None:
        return R
    R['dir'] = ref['dir']
    S = {}
    # ---- speed
    fin_v = np.isfinite(E['v'])
    tv_est, v_est = E['t'][fin_v], E['v'][fin_v]
    j, dt = nearest(ref['tv'], tv_est)
    ok = dt <= tol
    e = v_est[j[ok]] - ref['v'][ok] if ok.any() else np.zeros(0)
    R['v_n_ref'] = int(len(ref['tv']))
    R['v_cov'] = float(ok.mean()) if len(ok) else np.nan
    R.update(_stats(e, 'v'))
    rg = regimes(ref['tv'], ref['v'])[ok]
    for k in REGIMES:
        m = rg == k
        R['n_' + k] = int(m.sum())
        R['v_bias_' + k] = float(e[m].mean()) if m.any() else np.nan
        R['v_rmse_' + k] = float(np.sqrt(np.mean(e[m] ** 2))) if m.any() else np.nan
    trans = [abs(R['v_bias_' + k]) for k in ('accel', 'brake', 'stopped') if np.isfinite(R['v_bias_' + k])]
    R['v_bias_trans'] = max(trans) if trans else np.nan
    if series:
        v_err = np.full(len(ref['tv']), np.nan)
        v_err[ok] = e
        S.update(tv=ref['tv'], v_ref=ref['v'], v_err=v_err, v_est_t=tv_est, v_est=v_est)
    # ---- position (has_pose False / non-finite count as missing)
    pm = E['has_pose'] & np.isfinite(E['xyz']).all(axis=1)
    tpe, xyze, cve = E['t'][pm], E['xyz'][pm], E['cov'][pm]
    j, dt = nearest(ref['tp'], tpe)
    ok = dt <= tol
    R['p_n_ref'] = int(len(ref['tp']))
    R['p_cov'] = float(ok.mean()) if len(ok) else np.nan
    pos_keys = ('e2d_mean', 'e2d_rmse', 'e2d_p95', 'e2d_max', 'e3d_mean', 'e3d_rmse', 'e3d_p95', 'e3d_max',
                'ez_mean', 'ez_rmse', 'onmap_frac', 'al_mean', 'al_mean_abs', 'al_rmse', 'al_p95', 'al_max',
                'ct_mean', 'ct_rmse', 'ct_max', 'e2d_onmap_mean', 'e2d_onmap_max', 'e2d_offmap_mean',
                'e2d_offmap_max', 'dist', 'end_e2d', 'end_e3d', 'drift_pct', 'drift3d_pct', 'end_onmap_al',
                'drift_onmap_pct', 'al_growth_pct', 'p_in95', 'p_nees', 'p_sigma_med')
    R.update(_nan(*pos_keys))
    if ok.sum() == 0:
        R['score_loss'] = score_loss(R)
        if series:
            R['series'] = S
        return R
    Pe, Pr, tp = xyze[j[ok]], ref['xyz'][ok], ref['tp'][ok]
    d2 = np.hypot(*(Pe[:, :2] - Pr[:, :2]).T)
    d3 = np.linalg.norm(Pe - Pr, axis=1)
    ez = Pe[:, 2] - Pr[:, 2]
    R.update(e2d_mean=float(d2.mean()), e2d_rmse=float(np.sqrt(np.mean(d2 ** 2))), e2d_p95=float(np.percentile(d2, 95)),
             e2d_max=float(d2.max()), e3d_mean=float(d3.mean()), e3d_rmse=float(np.sqrt(np.mean(d3 ** 2))),
             e3d_p95=float(np.percentile(d3, 95)), e3d_max=float(d3.max()), ez_mean=float(ez.mean()),
             ez_rmse=float(np.sqrt(np.mean(ez ** 2))))
    # covariance consistency: horizontal error in the reported along/cross axes, normalised by the
    # reported variances (Mahalanobis^2, chi2 with 2 dof); in95 = share inside the 95 % ellipse
    # (0.95 if calibrated), nees = mean Mahalanobis^2 / 2 (1 if calibrated)
    C = cve[j[ok]]
    if np.isfinite(C).all(axis=1).any():
        dxy = Pe[:, :2] - Pr[:, :2]
        cy, sy = np.cos(C[:, 0]), np.sin(C[:, 0])
        ea = dxy[:, 0] * cy + dxy[:, 1] * sy
        ec = -dxy[:, 0] * sy + dxy[:, 1] * cy
        va = np.maximum(C[:, 1], 1e-6)
        vc = np.where(np.isfinite(C[:, 2]), np.maximum(C[:, 2], 1e-6), 0.09)
        m2 = ea * ea / va + ec * ec / vc
        m2 = m2[np.isfinite(m2)]
        if len(m2):
            R['p_in95'] = float(np.mean(m2 <= 5.991))
            R['p_nees'] = float(np.mean(np.minimum(m2, 1e4)) / 2.0)
            R['p_sigma_med'] = float(np.median(np.sqrt(va + vc)))
    al = ct = np.full(len(tp), np.nan)
    on = np.zeros(len(tp), bool)
    if ref['dir'] is not None:
        rt = routes()[ref['dir']]
        sr, dr, lr, outr = rt.project(Pr[:, :2])
        se, _, le, _ = rt.project(Pe[:, :2])
        on = (dr < gate) & ~outr
        al = np.where(on, se - sr, np.nan)
        ct = np.where(on, le - lr, np.nan)
        R['onmap_frac'] = float(on.mean())
        if on.any():
            a, c = al[on], ct[on]
            R.update(al_mean=float(a.mean()), al_mean_abs=float(np.abs(a).mean()), al_rmse=float(np.sqrt(np.mean(a ** 2))),
                     al_p95=float(np.percentile(np.abs(a), 95)), al_max=float(np.abs(a).max()),
                     ct_mean=float(c.mean()), ct_rmse=float(np.sqrt(np.mean(c ** 2))), ct_max=float(np.abs(c).max()),
                     e2d_onmap_mean=float(d2[on].mean()), e2d_onmap_max=float(d2[on].max()))
            lo = np.where(on)[0]
            R['end_onmap_al'] = float(al[lo[-1]])
            span = sr[lo[-1]] - sr[lo[0]]
            if span >= MIN_DRIFT_SPAN_M:
                R['drift_onmap_pct'] = float(100 * abs(al[lo[-1]]) / span)
                R['al_growth_pct'] = float(100 * (al[lo[-1]] - al[lo[0]]) / span)
    if (~on).any():
        R['e2d_offmap_mean'] = float(d2[~on].mean())
        R['e2d_offmap_max'] = float(d2[~on].max())
    # distance travelled: GNSS-speed odometer from the start of the recording to the last matched
    # sample (covers runs with RTK only at the end); RTK path length (1 s decimated, jumps
    # > 25 m/s dropped) between the first and the last matched sample for reference
    g = np.arange(tp[0], tp[-1], 1.0)
    ii, _ = nearest(g, ref['tp'])
    P = ref['xyz'][ii, :2]
    step = np.hypot(*np.diff(P, axis=0).T)
    dtt = np.diff(ref['tp'][ii])
    R['dist_rtk_path'] = float(step[(dtt > 0) & (step < 25 * np.maximum(dtt, 1e-3))].sum())
    dist = float(np.interp(tp[-1], ref['odo_t'], ref['odo'])) if len(ref['odo_t']) > 1 else R['dist_rtk_path']
    R.update(dist=dist, end_e2d=float(d2[-1]), end_e3d=float(d3[-1]),
             drift_pct=float(100 * d2[-1] / max(dist, 1.0)), drift3d_pct=float(100 * d3[-1] / max(dist, 1.0)))
    R['score_loss'] = score_loss(R)
    if series:
        S.update(tp=tp, xyz_ref=Pr, xyz_est=Pe, e2d=d2, e3d=d3, al=al, ct=ct, on=on,
                 ref_tp_all=ref['tp'], ref_xyz_all=ref['xyz'], est_t_pose=tpe, est_xyz_pose=xyze)
        R['series'] = S
    return R


def score_loss(R):
    """Weighted, capped, coverage-penalised combination of normalised metrics (lower is better)."""
    num = den = 0.0
    for k, y, w in SCORE_TERMS:
        m = R.get(k, np.nan)
        if m is None or not np.isfinite(m):
            continue
        num += w * min(abs(m) / y, SCORE_CAP)
        den += w
    covs = [c for c in (R.get('v_cov'), R.get('p_cov')) if c is not None and np.isfinite(c)]
    cov = min(covs) if covs else 0.0
    if den == 0:
        return SCORE_CAP
    return float(cov * num / den + (1 - cov) * SCORE_CAP)


SUMMARY_KEYS = ('score_loss', 'v_rmse', 'v_mae', 'v_bias', 'v_bias_trans', 'v_p99', 'v_max', 'v_cov',
                'v_bias_accel', 'v_bias_brake', 'v_bias_stopped', 'v_bias_cruise',
                'p_cov', 'e2d_mean', 'e2d_p95', 'e3d_mean', 'e3d_rmse', 'e3d_max', 'ez_mean',
                'al_mean', 'al_mean_abs', 'al_rmse', 'al_max', 'ct_rmse', 'ct_max',
                'e2d_onmap_mean', 'e2d_offmap_mean', 'e2d_offmap_max', 'end_e2d', 'drift_pct',
                'end_onmap_al', 'drift_onmap_pct', 'al_growth_pct', 'p_in95', 'p_nees', 'p_sigma_med')
SIGNED_KEYS = ('v_bias', 'v_bias_accel', 'v_bias_brake', 'v_bias_stopped', 'v_bias_cruise', 'al_mean',
               'ez_mean', 'end_onmap_al', 'al_growth_pct')


def aggregate(results, keys=SUMMARY_KEYS):
    """Across bags: {key: {median, mean, p90, max, n}}; signed keys also get median of the signed value."""
    out = {}
    for k in keys:
        vals = np.array([r.get(k, np.nan) for r in results], float)
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            out[k] = dict(median=np.nan, mean=np.nan, p90=np.nan, max=np.nan, n=0)
            continue
        a = np.abs(vals)
        d = dict(median=float(np.median(a)), mean=float(a.mean()), p90=float(np.percentile(a, 90)),
                 max=float(a.max()), n=int(len(a)))
        if k in SIGNED_KEYS:
            d['median_signed'] = float(np.median(vals))
            d['mean_signed'] = float(vals.mean())
        out[k] = d
    return out


# ------------------------------------------------------------------ self test / CLI
def reference_as_estimate(ref):
    """Estimate that reproduces the reference exactly at every reference stamp."""
    t = np.union1d(ref['tp'], ref['tv'])
    v = np.interp(t, ref['tv'], ref['v']) if len(ref['tv']) else np.zeros(len(t))
    xyz = np.column_stack([np.interp(t, ref['tp'], ref['xyz'][:, k]) for k in range(3)])
    # exact values at the reference stamps (interp is exact there unless stamps repeat)
    iv = np.searchsorted(t, ref['tv']); v[iv] = ref['v']
    ip = np.searchsorted(t, ref['tp']); xyz[ip] = ref['xyz']
    return dict(t=t, v=v, x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], has_pose=np.ones(len(t), bool))


INPUT_TOPICS = ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity', '/vehicle/driver_position_cmd')


def reference_at_input_stamps(ref, D):
    """Ideal estimator published on the input stamps: the reference interpolated there."""
    t = np.unique(np.concatenate([np.asarray((D.get(k) or {}).get('t_hdr', []), float) for k in INPUT_TOPICS]))
    t = t[(t >= ref['tp'][0]) & (t <= ref['tp'][-1])]
    v = np.interp(t, ref['tv'], ref['v']) if len(ref['tv']) else np.zeros(len(t))
    xyz = np.column_stack([np.interp(t, ref['tp'], ref['xyz'][:, k]) for k in range(3)])
    return dict(t=t, v=v, x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], has_pose=np.ones(len(t), bool))


def self_test(bags, antenna='master', mode='exact'):
    """mode 'exact': estimate = reference at its own stamps (must be 0);
    'interp': reference interpolated to the input-message stamps (the matching floor)."""
    rows = []
    for b in bags:
        D = load_cache(b)
        ref = load_ref(b, antenna, D=D)
        if ref is None:
            continue
        est = reference_as_estimate(ref) if mode == 'exact' else reference_at_input_stamps(ref, D)
        R = evaluate(b, est, antenna=antenna, ref=ref)
        rows.append(R)
        print('%-16s dir=%s v_cov=%.3f p_cov=%.3f v_rmse=%.2e e3d_max=%.2e al_max=%.2e ct_max=%.2e drift=%.2e%% loss=%.3f'
              % (b, R['dir'], R['v_cov'], R['p_cov'], R['v_rmse'], R['e3d_max'], R['al_max'], R['ct_max'],
                 R['drift_pct'], R['score_loss']))
    return rows


def _jsonable(x):
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items() if k != 'series'}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return None if not np.isfinite(x) else float(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='*')
    ap.add_argument('--est', help='estimate .npz (keys t, v, x, y, z, has_pose)')
    ap.add_argument('--antenna', default='master', choices=['master', 'rover', 'mid'])
    ap.add_argument('--clean', action='store_true', help='drop reference stamp/position glitches and frozen speed')
    ap.add_argument('--tol', type=float, default=0.05)
    ap.add_argument('--gate', type=float, default=5.0)
    ap.add_argument('--self-test', nargs='?', const='exact', choices=['exact', 'interp'],
                    help='feed the reference as the estimate (exact: at its stamps; interp: at the input stamps)')
    a = ap.parse_args(argv)
    if a.self_test:
        bags = a.bags or sorted(f[:-4] for f in os.listdir(CACHE_DIR) if f.endswith('.pkl'))
        rows = self_test(bags, a.antenna, a.self_test)
        agg = aggregate(rows)
        print('bags %d; over bags (median / max):' % len(rows))
        for k in ('v_cov', 'p_cov', 'v_rmse', 'v_max', 'e2d_mean', 'e3d_max', 'al_rmse', 'al_max', 'ct_max',
                  'drift_pct', 'score_loss'):
            print('  %-10s %.4g / %.4g' % (k, agg[k]['median'], agg[k]['max']))
        return 0
    if len(a.bags) != 1 or not a.est:
        ap.error('give one bag and --est, or --self-test')
    R = evaluate(a.bags[0], a.est, a.antenna, a.tol, a.gate, a.clean)
    print(json.dumps(_jsonable(R), indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
