"""Shared data of the model / estimator validation (model_openloop.py, model_closedloop.py,
model_lagcheck.py).  Source: .work/scratch_model/{common,prep}.py.

Per-bag arrays ("prep", built by model_prep.py into $CALIB_WORK/model_prep/<bag>.pkl, or on
demand by load()), for the clean RTK unique bags (bag_index eval_ok and rtk_frac >= 0.9, 49 bags):
  F, R   wheel header stamps and speed in m/s (raw km/h / K_vehicle)
  C      cmd header stamps and notch
  vel    GNSS master speed at its header stamps + judge gate (status-2 master fix within 0.05 s)
         + 'clean' gate (rover speed within 0.3 m/s when a rover vel is within 0.05 s)
  S      RTK master fixes projected on the route of the run direction (dist < 3 m, not beyond)
  direction  route of the travelled track, checked by the majority sign of ds along it
"""
import os
import pickle

import numpy as np

import _common as C
from tram_backup_odometry.core.config import Params
from tram_backup_odometry.core.track_map import load_routes

PREP = os.path.join(C.WORK, 'model_prep')
_ROUTES = None


def routes():
    global _ROUTES
    if _ROUTES is None:
        _ROUTES = load_routes(C.MAPS, Params().grade_window_m)
    return _ROUTES


def prep_bags():
    idx = C.load_index()['bags']
    return sorted(b for b, v in idx.items() if v['eval_ok'] and v['rtk_frac'] >= 0.9)


def bag_list(spec=''):
    return C.select_bags(spec, prep_bags())


def load(b):
    fn = os.path.join(PREP, b + '.pkl')
    if not os.path.exists(fn):
        d = build(b)
        os.makedirs(PREP, exist_ok=True)
        with open(fn, 'wb') as f:
            pickle.dump(d, f)
        return d
    with open(fn, 'rb') as f:
        return pickle.load(f)


def params(**kw):
    p = Params()
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def true_speed_fn(d, key='gate', max_gap=0.25):
    """Interpolated GNSS speed on gated samples; nan where the nearest gated sample is farther than max_gap."""
    m = d['vel'][key]
    t = d['vel']['t'][m]
    v = d['vel']['v'][m]
    o = np.argsort(t)
    t, v = t[o], v[o]

    def f(tq):
        tq = np.asarray(tq, float)
        out = np.interp(tq, t, v)
        j = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
        gap = np.minimum(np.abs(t[j] - tq), np.abs(tq - t[j - 1]))
        return np.where(gap <= max_gap, out, np.nan)
    return f


def true_s_fn(d, max_gap=0.6):
    t = d['S']['t']
    s = d['S']['s']
    o = np.argsort(t)
    t, s = t[o], s[o]

    def f(tq):
        tq = np.asarray(tq, float)
        out = np.interp(tq, t, s)
        j = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
        ok = (t[j] - t[j - 1] <= max_gap) & (tq >= t[0]) & (tq <= t[-1])
        return np.where(ok, out, np.nan)
    return f


def project_route(route, x, y):
    s = np.full(len(x), np.nan)
    d = np.full(len(x), np.inf)
    for i in range(len(x)):
        si, di, _, beyond = route.project(x[i], y[i])
        if not beyond:
            s[i] = si
            d[i] = di
    return s, d


def build(b):
    idx = C.load_index()['bags']
    p = Params()
    rts = routes()
    D = C.load_cache(b)
    veh = idx[b]['vehicle']
    K = {'30618': p.wheel_k_30618, '30639': p.wheel_k_30639}.get(veh, p.wheel_k)
    out = dict(bag=b, vehicle=veh, K=K)
    for key, T in (('F', C.FT), ('R', C.RT)):
        out[key] = dict(t=np.asarray(D[T]['t_hdr'], float), v=np.asarray(D[T]['v'], float) / K,
                        t_rec=np.asarray(D[T]['t_rec'], float))
    out['C'] = dict(t=np.asarray(D[C.CT]['t_hdr'], float), n=np.asarray(D[C.CT]['pos'], int),
                    t_rec=np.asarray(D[C.CT]['t_rec'], float))
    mf = D[C.MF]
    mv = D[C.MV]
    x, y = C.utm_vec(mf['lat'], mf['lon'])
    st = np.asarray(mf['status'])
    tf = np.asarray(mf['t_hdr'], float)
    o = np.argsort(tf, kind='stable')
    tf, x, y, st = tf[o], x[o], y[o], st[o]
    rtk = st == 2
    xs, ys = x[rtk][::5], y[rtk][::5]
    cnt = {nm: project_route(rts[nm], xs, ys) for nm in ('T2S', 'S2T')}
    near_T = cnt['T2S'][1] < cnt['S2T'][1]
    votes = {}
    for nm, m_ in (('T2S', near_T), ('S2T', ~near_T)):
        s_, d_ = cnt[nm]
        on = m_ & (d_ < 3)
        ds = np.diff(s_[on])
        votes[nm] = int((ds > 0.05).sum() - (ds < -0.05).sum())
    direction = max(votes, key=votes.get)
    out.update(dir_votes=votes, direction=direction, direction_index=idx[b]['direction'])
    s, d = project_route(rts[direction], x[rtk], y[rtk])
    ok = d < 3.0
    out['S'] = dict(t=tf[rtk][ok], s=s[ok])
    tv = np.asarray(mv['t_hdr'], float)
    v = np.hypot(mv['vx'], mv['vy'])
    o = np.argsort(tv, kind='stable')
    tv, v = tv[o], v[o]
    j, dt = C.nearest(tv, tf)
    gate = (dt <= 0.05) & (st[j] == 2)
    clean = gate.copy()
    rv = D.get(C.RV)
    if rv is not None and len(rv.get('t_hdr', [])) > 10:
        trv = np.asarray(rv['t_hdr'], float)
        vr = np.hypot(rv['vx'], rv['vy'])
        o = np.argsort(trv, kind='stable')
        trv, vr = trv[o], vr[o]
        jr, dtr = C.nearest(tv, trv)
        clean &= ~((dtr <= 0.05) & (np.abs(vr[jr] - v) > 0.3))
    out['vel'] = dict(t=tv, v=v, gate=gate, clean=clean)
    out['fix'] = dict(t=tf, x=x, y=y, st=st)
    return out
