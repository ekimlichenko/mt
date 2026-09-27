"""Synthetic fault injection on parsed bags (robustness tests).

A fault spec is a dict {'kind': ..., 't0': s since the first wheel stamp, 'dur': s, ...}.
apply(D, specs, seed) returns a modified deep copy of the parsed bag dict
(topic -> arrays, see tram_backup_odometry.bagio) that can be fed to
replay.run_bag(..., data=D).  Record times are kept consistent with headers
(a message removed from a topic disappears from the replay as it would on the bus).

Kinds:
  drop_both        both bogies silent                      (dur)
  drop_front/rear  one bogie silent                        (dur)
  zero_front/rear  one bogie reads 0.0                     (dur)
  freeze_front/rear one bogie repeats its last value       (dur)
  spikes           random single-sample spikes on both bogies (rate per s, amp km/h)
  noise            Gaussian noise on both bogies           (sigma km/h)
  scale_front      front bogie scale error                 (factor)
  slip_front       front bogie reads v*(1+amp) (spin)      (amp, dur)
  stamp_jump       all input headers +jump s in a window   (jump, dur)
  cmd_drop         driver command topic silent             (dur)
  cmd_freeze       driver command repeats its last value   (dur)
  nan_front        front bogie sends NaN                   (dur)
"""
import copy

import numpy as np

FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
CMD = '/vehicle/driver_position_cmd'
BOGIE = {'front': FRONT, 'rear': REAR}


def _t0(D):
    return float(min(D[FRONT]['t_hdr'][0], D[REAR]['t_hdr'][0]))


def _keep(d, mask):
    for k, v in list(d.items()):
        if isinstance(v, np.ndarray) and len(v) == len(mask):
            d[k] = v[mask]
        elif isinstance(v, list) and len(v) == len(mask):
            d[k] = [x for x, m in zip(v, mask) if m]


def _window(d, a, b):
    t = d['t_hdr']
    return (t >= a) & (t < b)


def apply(D, specs, seed=0):
    rng = np.random.default_rng(seed)
    D = copy.deepcopy(D)
    base = _t0(D)
    for sp in specs:
        kind = sp['kind']
        a = base + float(sp.get('t0', 0.0))
        b = a + float(sp.get('dur', 0.0))
        if kind == 'drop_both':
            for tp in (FRONT, REAR):
                _keep(D[tp], ~_window(D[tp], a, b))
        elif kind in ('drop_front', 'drop_rear'):
            tp = BOGIE[kind.split('_')[1]]
            _keep(D[tp], ~_window(D[tp], a, b))
        elif kind in ('zero_front', 'zero_rear'):
            tp = BOGIE[kind.split('_')[1]]
            m = _window(D[tp], a, b)
            D[tp]['v'] = np.where(m, 0.0, D[tp]['v'])
        elif kind in ('freeze_front', 'freeze_rear'):
            tp = BOGIE[kind.split('_')[1]]
            m = _window(D[tp], a, b)
            idx = np.where(m)[0]
            if len(idx):
                v = D[tp]['v'].copy()
                v[idx] = v[max(idx[0] - 1, 0)]
                D[tp]['v'] = v
        elif kind == 'nan_front':
            m = _window(D[FRONT], a, b)
            D[FRONT]['v'] = np.where(m, np.nan, D[FRONT]['v'])
        elif kind == 'spikes':
            rate, amp = float(sp.get('rate', 0.2)), float(sp.get('amp', 20.0))
            for tp in (FRONT, REAR):
                v = D[tp]['v'].astype(float).copy()
                t = D[tp]['t_hdr']
                dur = t[-1] - t[0]
                n = rng.poisson(rate * dur)
                idx = rng.integers(0, len(v), n)
                v[idx] = np.maximum(v[idx] + rng.choice([-1.0, 1.0], n) * amp, 0.0)
                D[tp]['v'] = v
        elif kind == 'noise':
            sig = float(sp.get('sigma', 1.0))
            for tp in (FRONT, REAR):
                v = D[tp]['v'].astype(float)
                D[tp]['v'] = np.maximum(v + rng.normal(0.0, sig, len(v)), 0.0)
        elif kind == 'scale_front':
            D[FRONT]['v'] = D[FRONT]['v'] * float(sp.get('factor', 1.05))
        elif kind == 'slip_front':
            m = _window(D[FRONT], a, b)
            D[FRONT]['v'] = np.where(m, D[FRONT]['v'] * (1.0 + float(sp.get('amp', 0.3))), D[FRONT]['v'])
        elif kind == 'stamp_jump':
            j = float(sp.get('jump', 1.0))
            for tp in (FRONT, REAR, CMD):
                m = _window(D[tp], a, b)
                D[tp]['t_hdr'] = np.where(m, D[tp]['t_hdr'] + j, D[tp]['t_hdr'])
        elif kind == 'cmd_drop':
            _keep(D[CMD], ~_window(D[CMD], a, b))
        elif kind == 'cmd_freeze':
            m = _window(D[CMD], a, b)
            idx = np.where(m)[0]
            if len(idx):
                p = D[CMD]['pos'].copy()
                p[idx] = p[max(idx[0] - 1, 0)]
                D[CMD]['pos'] = p
        else:
            raise ValueError('unknown fault kind %r' % kind)
    return D


def moving_windows(D, dur, n, min_v_kmh=10.0, margin_s=150.0, seed=0):
    """Pick n window starts (s since the first wheel stamp) while the tram is moving."""
    rng = np.random.default_rng(seed)
    t = D[FRONT]['t_hdr']
    v = D[FRONT]['v']
    base = _t0(D)
    ok = (v > min_v_kmh) & (t - base > margin_s) & (t < t[-1] - dur - 5.0)
    idx = np.where(ok)[0]
    if len(idx) == 0:
        return []
    pick = np.sort(rng.choice(idx, size=min(n, len(idx)), replace=False))
    starts = []
    for i in pick:
        s = float(t[i] - base)
        if all(abs(s - x) > dur + 30.0 for x in starts):
            starts.append(s)
    return starts
