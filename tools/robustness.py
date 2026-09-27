#!/usr/bin/env python3
"""Robustness suite: synthetic faults, real anomaly bags, slip-flag, adhesion and gain statistics.

Four parts, all through the same ``Pipeline`` as the ROS node (``replay.run_bag``):

1. Fault injection (``--part faults``).  A fixed set of 12 ``eval_ok`` bags
   (6 T2S / 6 S2T, both vehicles, no reference or clock artefacts) is replayed
   clean and with every scenario of ``SCENARIOS``.  Faults come from
   ``tools/inject_faults.apply`` plus local kinds (``slip_rear``,
   ``slip_both`` = common-mode slip / slide / lock of both bogies with an optional
   linear ramp, ``noise_moving`` = Gaussian noise on non-zero raw samples only;
   ``slip_front`` with a negative amplitude = slide).  The FAULTED data are fed
   to the estimator; the reference is always built from the CLEAN data
   (``ev.load_ref(D=clean)``), so it is untouched.
   Fault placement is deterministic, from the data and the clean replay:
     cruise anchor  first t >= 0.3 T with wheel speed > 5 m/s for 8 s, ref speed
                    coverage >= 90 % and clean |v_err| < 0.3 m/s over
                    [t-5, t+60+40], on-map reference at t+60+30;
     brake anchor   first braking onset (notch >= 0 -> < 0, not the -8 hold) at
                    >= 0.3 T with v > 7 m/s (else > 5) that ends in a stop
                    within 30 s; same checks over [t-5, t+30+40];
     spin episodes  up to 5 starts (targets 0.15/0.30/0.45/0.60/0.75 T, >= 40 s
                    apart), v > 5 m/s over the episode, notch >= 0;
     slide episodes up to 5 braking onsets with v > 5 m/s, episode = onset+1.5 s,
                    2 s long, v > 4 m/s over it.
   Metrics per window [a, b] (fault start/end): crash-free, finite outputs,
   output stamps = input stamps, output rate and 0.1 s-grid coverage inside
   [a, b), speed RMSE / max |error| over [a, b+10], recovery time after b
   (|v_err| < 0.1 m/s for 2 s against the reference, and |v - v_clean| < 0.1 m/s
   for 2 s against the clean replay), along-track displacement vs the clean run
   (d_s = s_route(fault) - s_route(clean) at b and b+30; d_al = al(fault) -
   al(clean) against the reference at b+30), end drift / al_rmse deltas.
   Naive baseline (no validation): v_naive(t) = mean of the latest raw sample of
   each bogie / K (for drop_both this is the zero-order hold of the last fused
   wheel speed); naive_d_s = int_a^b (v_naive - v_clean) dt.
   Slip families: recall (flag within 0.5 s of the episode start; also over the
   episodes where the clean replay has no flag in [a-0.5, b+0.5]), time to
   flag, flagged fraction inside / outside the episodes (vs the clean run),
   speed error in [ts, te+1] of the estimate vs the naive mean of the bogies.
2. Clean runs of all 60 ``eval_ok`` bags (``--part clean``): slip-flag rate
   (false-positive budget), flag episodes confirmed / not confirmed by the
   reference (the naive bogie mean departs from the GNSS speed by > 0.3 m/s),
   bogie-state shares, adhesion used (|a|/g) and gain adaptation (g_tr, g_br).
3. Real anomaly bags without injection (``--part real``): slip-flag episodes,
   bogie-state episodes, input stamp glitches, speed and position errors.
4. Figures ``docs/plots/robust_*.png`` (``--part plots``; re-runs the few
   replays it needs).

    python tools/robustness.py --jobs 4              # everything, 4 workers
    python tools/robustness.py --part faults --bags 30618_073f08d1 --scenarios drop_both_30 --jobs 1
    python tools/robustness.py --part plots          # figures only (needs summary.json)

Outputs: results/robustness/{summary.md, summary.json, per_bag.json}.
"""
import argparse
import copy
import json
import os
import sys
import time
import traceback
import zlib
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import evaluate as ev  # noqa: E402
import inject_faults as fi  # noqa: E402

sys.path.insert(0, ev.PKG_DIR)
from tram_backup_odometry.core.config import Params  # noqa: E402
from tram_backup_odometry.replay import run_bag  # noqa: E402

FRONT, REAR, CMD = fi.FRONT, fi.REAR, fi.CMD
OUT_DIR = os.path.join(ev.ROOT, 'results', 'robustness')
PLOT_DIR = os.path.join(ev.ROOT, 'docs', 'plots')
INDEX = os.path.join(ev.ROOT, '.work', 'bag_index.json')

FAULT_BAGS = (
    # T2S
    '30618_073f08d1', '30618_22c1c589', '30618_6cb3280a', '30618_d4ba7005', '30639_2b4a6347', '30639_9f0b519f',
    # S2T
    '30618_0e41eac3', '30618_437e855c', '30618_b83d854d', '30618_3b36e5cd', '30639_584b6e32', '30639_0be558e2',
)
REAL_BAGS = (
    ('slip', '2050d396'), ('slip', '33bec73f'), ('slip', '50956d6e'), ('slip', '68d1748a'),
    ('dropout', '3b3d9eb8'), ('dropout', '927002c2'), ('dropout', 'd927f360'), ('dropout', 'c31df386'),
    ('stamp', '2255aade'), ('stamp', '40ffd323'),
    ('cmd', '4d487b0d'),
)

V_MOVE = 5.0            # m/s, "moving" for fault placement
ERR_OK = 0.3            # m/s, clean |v_err| bound over an anchor window (sane reference)
POST_S = 10.0           # speed window = [a, b + POST_S]
AFTER_S = 30.0          # along-track displacement is read at b + AFTER_S
REC_THR, REC_HOLD, REC_HORIZON = 0.1, 2.0, 120.0
G0 = 9.81               # m/s^2, as in the package (adhesion_used = |a| / 9.81)
SLIP_CONFIRM = 0.3      # m/s: a clean-run flag episode is "confirmed" if the naive bogie mean is this far off
LOCAL_KINDS = ('slip_rear', 'slip_both', 'noise_moving')

# ------------------------------------------------------------------ scenarios
# anchor: cruise | brake | spin (multi-episode) | slide (multi-episode) | whole (entire run)
# faults: specs without t0 (the anchor supplies it); 'dur' also sets the window length.
SCENARIOS = [
    dict(name='drop_front_60', family='bogie', anchor='cruise', faults=[dict(kind='drop_front', dur=60.0)],
         ru='передняя тележка молчит 60 с'),
    dict(name='drop_rear_60', family='bogie', anchor='cruise', faults=[dict(kind='drop_rear', dur=60.0)],
         ru='задняя тележка молчит 60 с'),
    dict(name='zero_rear_30', family='bogie', anchor='cruise', faults=[dict(kind='zero_rear', dur=30.0)],
         ru='задняя тележка шлёт 0.0 в движении 30 с'),
    dict(name='freeze_front_30', family='bogie', anchor='cruise', faults=[dict(kind='freeze_front', dur=30.0)],
         ru='передняя тележка «замерзла» (повтор последнего значения) 30 с'),
    dict(name='nan_front_10', family='bogie', anchor='cruise', faults=[dict(kind='nan_front', dur=10.0)],
         ru='передняя тележка шлёт NaN 10 с'),
    dict(name='drop_both_5', family='drop', anchor='cruise', faults=[dict(kind='drop_both', dur=5.0)],
         ru='обе тележки молчат 5 с (движение)'),
    dict(name='drop_both_15', family='drop', anchor='cruise', faults=[dict(kind='drop_both', dur=15.0)],
         ru='обе тележки молчат 15 с (движение)'),
    dict(name='drop_both_30', family='drop', anchor='cruise', faults=[dict(kind='drop_both', dur=30.0)],
         ru='обе тележки молчат 30 с (движение)'),
    dict(name='drop_both_15_brake', family='drop', anchor='brake', faults=[dict(kind='drop_both', dur=15.0)],
         ru='обе тележки молчат 15 с с начала торможения'),
    dict(name='drop_both_30_brake', family='drop', anchor='brake', faults=[dict(kind='drop_both', dur=30.0)],
         ru='обе тележки молчат 30 с с начала торможения (остановка)'),
    dict(name='combo_drop15_cmd15', family='drop', anchor='cruise',
         faults=[dict(kind='drop_both', dur=15.0), dict(kind='cmd_drop', dur=15.0)],
         ru='обе тележки + команда молчат 15 с (чистое счисление без входов)'),
    dict(name='combo_drop15_cmd15_brake', family='drop', anchor='brake',
         faults=[dict(kind='drop_both', dur=15.0), dict(kind='cmd_drop', dur=15.0)],
         ru='обе тележки + команда молчат 15 с с начала торможения'),
    dict(name='spikes_0.5hz_20kmh', family='signal', anchor='whole', faults=[dict(kind='spikes', rate=0.5, amp=20.0)],
         ru='одиночные выбросы ±20 км/ч, 0.5 1/с на каждой тележке, весь заезд'),
    dict(name='noise_1kmh', family='signal', anchor='whole', faults=[dict(kind='noise', sigma=1.0)],
         ru='гауссов шум 1 км/ч на обеих тележках, весь заезд (и на стоянке, с обрезкой < 0)'),
    dict(name='noise_1kmh_moving', family='signal', anchor='whole', faults=[dict(kind='noise_moving', sigma=1.0)],
         ru='гауссов шум 1 км/ч только на ненулевых отсчётах (в движении), весь заезд'),
    dict(name='scale_front_1.05', family='signal', anchor='whole', faults=[dict(kind='scale_front', factor=1.05)],
         ru='ошибка масштаба передней тележки +5 %, весь заезд'),
    dict(name='slip_front_0.3', family='slip', anchor='spin', faults=[dict(kind='slip_front', amp=0.3, dur=3.0)],
         ru='боксование передней тележки +30 %, 3 с, до 5 эпизодов'),
    dict(name='slip_front_0.15', family='slip', anchor='spin', faults=[dict(kind='slip_front', amp=0.15, dur=3.0)],
         ru='боксование передней тележки +15 %, 3 с, до 5 эпизодов'),
    dict(name='slip_rear_0.3', family='slip', anchor='spin', faults=[dict(kind='slip_rear', amp=0.3, dur=3.0)],
         ru='боксование задней тележки +30 %, 3 с, до 5 эпизодов'),
    dict(name='slide_front_0.3_brake', family='slip', anchor='slide', faults=[dict(kind='slip_front', amp=-0.3, dur=2.0)],
         ru='юз передней тележки −30 % при торможении, 2 с, до 5 эпизодов'),
    dict(name='slip_both_0.3', family='slip', anchor='spin', faults=[dict(kind='slip_both', amp=0.3, dur=3.0)],
         ru='синфазное боксование обеих тележек +30 % (скачок), 3 с'),
    dict(name='slip_both_0.15_ramp1s', family='slip', anchor='spin',
         faults=[dict(kind='slip_both', amp=0.15, dur=3.0, ramp=1.0)],
         ru='синфазное боксование обеих +15 % с нарастанием 1 с, 3 с'),
    dict(name='slide_both_0.3_brake', family='slip', anchor='slide', faults=[dict(kind='slip_both', amp=-0.3, dur=2.0)],
         ru='синфазный юз обеих тележек −30 % при торможении, 2 с'),
    dict(name='lock_both_2s_brake', family='slip', anchor='slide', faults=[dict(kind='slip_both', amp=-1.0, dur=2.0)],
         ru='блокировка колёс обеих тележек (обе шлют 0) при торможении, 2 с'),
    dict(name='stamp_jump_+0.5', family='stamp', anchor='cruise', faults=[dict(kind='stamp_jump', jump=0.5, dur=5.0)],
         ru='все входные header.stamp +0.5 с на 5 с'),
    dict(name='stamp_jump_-0.5', family='stamp', anchor='cruise', faults=[dict(kind='stamp_jump', jump=-0.5, dur=5.0)],
         ru='все входные header.stamp −0.5 с на 5 с'),
    dict(name='cmd_drop_60', family='cmd', anchor='cruise', faults=[dict(kind='cmd_drop', dur=60.0)],
         ru='команда контроллера пропала на 60 с'),
    dict(name='cmd_freeze_30', family='cmd', anchor='cruise', faults=[dict(kind='cmd_freeze', dur=30.0)],
         ru='команда контроллера «замерзла» на 30 с'),
]
SCEN = {s['name']: s for s in SCENARIOS}
WHEEL_FAMILIES = ('bogie', 'drop', 'signal', 'slip')


# ------------------------------------------------------------------ helpers
def _f(x):
    """JSON-safe float."""
    if x is None:
        return None
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if np.isfinite(x) else None


def jsonable(x):
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return _f(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, np.ndarray):
        return jsonable(x.tolist())
    return x


def load_index():
    with open(INDEX) as f:
        d = json.load(f)
    b = d['bags']
    return dict(b) if isinstance(b, dict) else {r['bag']: r for r in b}


def full_name(suffix, index):
    for b in index:
        if b.endswith(suffix):
            return b
    raise KeyError(suffix)


def params_for(bag):
    p = Params()
    if bag[:5] in ('30618', '30639'):
        p.vehicle_id = bag[:5]
    return p


def base_time(D):
    return fi._t0(D)


def apply_faults(D, specs, seed):
    """inject_faults.apply for its kinds, then the local kinds (same time base, on the copy)."""
    inj = [s for s in specs if s['kind'] not in LOCAL_KINDS]
    loc = [s for s in specs if s['kind'] in LOCAL_KINDS]
    Df = fi.apply(D, inj, seed)
    base = base_time(D)
    rng = np.random.default_rng(int(seed) + 7919)
    for sp in loc:
        if sp['kind'] == 'noise_moving':
            # Gaussian noise only on non-zero raw samples: a standing tram keeps reading 0.0
            # (inject_faults 'noise' also perturbs standstill and clips at 0, i.e. rectifies it)
            sig = float(sp.get('sigma', 1.0))
            for tp in (FRONT, REAR):
                v = np.asarray(Df[tp]['v'], float)
                nz = v > 0.0
                Df[tp]['v'] = np.where(nz, np.maximum(v + rng.normal(0.0, sig, len(v)), 0.0), v)
            continue
        a = base + float(sp.get('t0', 0.0))
        b = a + float(sp.get('dur', 0.0))
        amp = float(sp.get('amp', 0.3))
        ramp = float(sp.get('ramp', 0.0))
        tps = (REAR,) if sp['kind'] == 'slip_rear' else (FRONT, REAR)
        for tp in tps:
            t = Df[tp]['t_hdr']
            inside = (t >= a) & (t < b)
            if ramp > 0:
                w = np.clip(np.minimum(t - a, b - t) / ramp, 0.0, 1.0)
            else:
                w = np.ones(len(t))
            Df[tp]['v'] = np.where(inside, Df[tp]['v'] * (1.0 + amp * w), Df[tp]['v'])
    return Df


def seed_for(bag, name, base_seed):
    return (zlib.crc32(('%s:%s' % (bag, name)).encode()) + int(base_seed)) % (2 ** 31)


def sorted_est(est):
    o = np.argsort(est['t'], kind='stable')
    return o


def value_at(t_sorted, y_sorted, tq, tol=0.25):
    """Nearest-stamp lookup with a tolerance (NaN outside it or where y is NaN)."""
    if len(t_sorted) == 0:
        return np.nan
    j, dt = ev.nearest(np.atleast_1d(tq), t_sorted)
    y = np.asarray(y_sorted)[j]
    y = np.where(dt <= tol, y, np.nan)
    return float(y[0]) if np.ndim(tq) == 0 else y


def bogie_last(D, topic, tq):
    """Latest raw sample (km/h) of a bogie at header time tq (NaN before the first); no validation."""
    d = D[topic]
    t = np.asarray(d['t_hdr'], float)
    v = np.asarray(d['v'], float)
    o = np.argsort(t, kind='stable')
    t, v = t[o], v[o]
    i = np.searchsorted(t, tq, side='right') - 1
    out = np.where(i >= 0, v[np.clip(i, 0, len(v) - 1)], np.nan)
    return out


def naive_speed(D, K, tq):
    """Naive fusion: mean of the latest raw samples of both bogies / K (no staleness, no checks)."""
    return 0.5 * (bogie_last(D, FRONT, tq) + bogie_last(D, REAR, tq)) / K


def recovery_time(tv, err, b, thr=REC_THR, hold=REC_HOLD, horizon=REC_HORIZON):
    """Seconds after b until |err| < thr for `hold` s (NaN err = bad); inf if not within horizon."""
    m = (tv >= b) & (tv <= b + horizon + hold)
    t, e = tv[m], err[m]
    if len(t) == 0:
        return np.nan
    bad = ~(np.abs(e) < thr)
    # time of the next bad sample at or after each index
    nb = np.full(len(t), np.inf)
    nxt = np.inf
    for i in range(len(t) - 1, -1, -1):
        if bad[i]:
            nxt = t[i]
        nb[i] = nxt
    for i in range(len(t)):
        if bad[i]:
            continue
        if nb[i] > t[i] + hold and t[-1] >= t[i] + hold - 0.2:
            return float(max(t[i] - b, 0.0))
    return np.inf


def episodes_from_mask(t, m, gap=0.3):
    """Contiguous runs of True in a boolean series over sorted stamps -> [(start, end)], gaps <= gap merged."""
    out = []
    if len(t) == 0:
        return out
    idx = np.where(m)[0]
    if len(idx) == 0:
        return out
    s = e = t[idx[0]]
    for i in idx[1:]:
        if t[i] - e <= gap:
            e = t[i]
        else:
            out.append((s, e))
            s = e = t[i]
    out.append((s, e))
    return out


def mode_state(modes):
    return np.array([str(m).split(':', 1)[1] if ':' in str(m) else str(m) for m in modes])


# ------------------------------------------------------------------ bag context / anchors
class Ctx:
    """Clean replay and reference of one bag, plus the 0.1 s grid used for fault placement."""

    def __init__(self, bag, gnss_limit):
        self.bag = bag
        self.D = ev.load_cache(bag)
        self.ref = ev.load_ref(bag, 'master', False, D=self.D)
        self.p = params_for(bag)
        self.K = self.p.k_for_vehicle()
        self.base = base_time(self.D)
        self.gnss_limit = gnss_limit
        self.est = run_bag(bag, params=copy.copy(self.p), data=self.D, gnss_limit_s=gnss_limit)
        self.R = ev.evaluate(bag, self.est, series=True, D=self.D, ref=self.ref) if self.ref is not None else None
        self._grid()

    def _grid(self):
        D, K, base = self.D, self.K, self.base
        tF, vF = np.asarray(D[FRONT]['t_hdr'], float), np.maximum(np.asarray(D[FRONT]['v'], float), 0)
        tR, vR = np.asarray(D[REAR]['t_hdr'], float), np.maximum(np.asarray(D[REAR]['v'], float), 0)
        oF, oR = np.argsort(tF, kind='stable'), np.argsort(tR, kind='stable')
        self.T = float(max(tF.max(), tR.max()) - base)
        g = np.arange(0.0, self.T, 0.1)
        f = np.interp(g + base, tF[oF], vF[oF]) / K
        r = np.interp(g + base, tR[oR], vR[oR]) / K
        self.g, self.vw = g, 0.5 * (f + r)
        tc = np.asarray(D[CMD]['t_hdr'], float)
        pc = np.asarray(D[CMD]['pos'], int)
        oc = np.argsort(tc, kind='stable')
        i = np.searchsorted(tc[oc], g + base, side='right') - 1
        self.notch = np.where(i >= 0, pc[oc][np.clip(i, 0, len(pc) - 1)], 0)
        # reference presence and clean error on the grid
        self.ref_ok = np.zeros(len(g), bool)
        self.err_c = np.full(len(g), np.nan)
        self.on_c = np.zeros(len(g), bool)
        if self.R is not None and 'series' in self.R:
            S = self.R['series']
            j, dt = ev.nearest(g + base, S['tv'])
            self.ref_ok = dt <= 0.1
            self.err_c = np.where(self.ref_ok, np.abs(S['v_err'][j]), np.nan)
            if len(S.get('tp', [])):
                jp, dtp = ev.nearest(g + base, S['tp'])
                self.on_c = (dtp <= 0.15) & np.asarray(S['on'], bool)[jp]
        # sorted clean outputs
        o = sorted_est(self.est)
        self.ct = self.est['t'][o]
        self.cv = self.est['v'][o]
        self.cs = self.est['diag_s_route'][o] if 'diag_s_route' in self.est else np.full(len(o), np.nan)
        self.cslip = self.est['slip'][o]
        self._sref = None

    def speed_ref(self):
        """Speed reference for slip confirmation and the real-bag checks (cached).

        kind 'rtk': RTK master cleaned as `evaluate --clean-ref`, with GNSS stamps that carry a
        sustained clock offset restamped (`restamp_gnss`); kind 'doppler': no RTK fixes, the
        Doppler speed of the master antenna (restamped, any fix status); kind None: no GNSS speed.
        Returns dict(kind, n_restamped, R, tv, v_ref, v_err, st) - st = bogie state of the output
        matched to each reference sample.
        """
        if self._sref is not None:
            return self._sref
        D2, n, runs = restamp_gnss(self.D)
        out = dict(kind=None, n_restamped=n, R=None,
                   offset_runs=[(t0 - self.base, t1 - self.base, o) for t0, t1, o in runs.get('master_vel', [])])
        ref = ev.load_ref(self.bag, 'master', True, D=D2)
        tv = v = e = None
        if ref is not None:
            R = ev.evaluate(self.bag, self.est, D=D2, ref=ref, series=True)
            S = R['series']
            tv, v, e = S['tv'], S['v_ref'], S['v_err']
            out.update(kind='rtk', R=R)
        else:
            g = D2.get(ev.TOPIC_MVEL) or {}
            if len(g.get('t_hdr', [])):
                tv = np.asarray(g['t_hdr'], float)
                o = np.argsort(tv, kind='stable')
                tv = tv[o]
                v = np.hypot(np.asarray(g['vx'], float), np.asarray(g['vy'], float))[o]
                e = np.full(len(tv), np.nan)
                fin = np.isfinite(self.cv)
                j, dt = ev.nearest(tv, self.ct[fin])
                ok = dt <= 0.05
                e[ok] = self.cv[fin][j[ok]] - v[ok]
                out['kind'] = 'doppler'
        if tv is not None:
            st_all = mode_state(self.est['mode'][sorted_est(self.est)])
            j, _ = ev.nearest(tv, self.ct)
            out.update(tv=tv, v_ref=v, v_err=e, st=st_all[j])
        self._sref = out
        return out

    def gi(self, t_rel):
        return int(np.clip(round(t_rel / 0.1), 0, len(self.g) - 1))

    def window_ok(self, t0, t1, need_on_at=None):
        i0, i1 = self.gi(t0), self.gi(t1)
        if i1 <= i0:
            return False
        if self.ref_ok[i0:i1].mean() < 0.9:
            return False
        e = self.err_c[i0:i1]
        if not np.isfinite(e).any() or np.nanmax(e) >= ERR_OK:
            return False
        if need_on_at is not None and not self.on_c[self.gi(need_on_at)]:
            return False
        return True

    def cruise_anchor(self, dur=60.0):
        T = self.T
        for t in np.arange(max(0.3 * T, 60.0), T - dur - 60.0, 1.0):
            i = self.gi(t)
            if self.vw[i] <= V_MOVE or self.vw[i:self.gi(t + 8.0)].min() <= V_MOVE:
                continue
            if self.window_ok(t - 5.0, t + dur + 40.0, need_on_at=t + dur + AFTER_S):
                return float(t)
        return None

    def brake_onsets(self, v_min, need_stop):
        n = self.notch
        on = np.where((n[1:] < 0) & (n[:-1] >= 0) & (n[1:] != -8))[0] + 1
        out = []
        for i in on:
            if self.vw[i] <= v_min:
                continue
            if need_stop and self.vw[i:self.gi(self.g[i] + 30.0)].min() >= 0.5:
                continue
            out.append(float(self.g[i]))
        return out

    def brake_anchor(self, dur=30.0):
        T = self.T
        for v_min in (7.0, V_MOVE):
            for t in self.brake_onsets(v_min, True):
                if t < max(0.3 * T, 60.0) or t > T - dur - 60.0:
                    continue
                if self.window_ok(t - 5.0, t + dur + 40.0, need_on_at=t + dur + AFTER_S):
                    return t
        return None

    def spin_episodes(self, dur=3.0, n=5):
        T = self.T
        cands = []
        for t in np.arange(max(0.1 * T, 60.0), T - 60.0, 1.0):
            i, j = self.gi(t), self.gi(t + dur + 1.0)
            if self.vw[i:j].min() <= V_MOVE or self.notch[i] < 0:
                continue
            cands.append(float(t))
        return self._spread(cands, dur, n)

    def slide_episodes(self, dur=2.0, n=5):
        cands = []
        for t in self.brake_onsets(V_MOVE, False):
            ts = t + 1.5
            if ts < max(0.1 * self.T, 60.0) or ts > self.T - 60.0:
                continue
            if self.vw[self.gi(ts):self.gi(ts + dur)].min() <= 4.0:
                continue
            cands.append(ts)
        return self._spread(cands, dur, n)

    def _spread(self, cands, dur, n):
        out = []
        for frac in np.linspace(0.15, 0.75, n):
            for t in cands:
                if t < frac * self.T or any(abs(t - x) < 40.0 for x in out):
                    continue
                if self.window_ok(t - 2.0, t + dur + 5.0):
                    out.append(t)
                    break
        return sorted(out)

    def anchors(self):
        return dict(cruise=self.cruise_anchor(), brake=self.brake_anchor(),
                    spin=self.spin_episodes(3.0), slide=self.slide_episodes(2.0), T=self.T)


def build_specs(sc, anchors):
    """-> (specs, windows as (t0, t1) relative to the bag's time base) or (None, None) if no anchor."""
    a = sc['anchor']
    if a == 'whole':
        return [dict(f) for f in sc['faults']], [(0.0, anchors['T'])]
    if a in ('cruise', 'brake'):
        t0 = anchors[a]
        if t0 is None:
            return None, None
        specs = [dict(f, t0=t0) for f in sc['faults']]
        dur = max(float(f.get('dur', 0.0)) for f in sc['faults'])
        return specs, [(t0, t0 + dur)]
    eps = anchors[a]
    if not eps:
        return None, None
    f = sc['faults'][0]
    dur = float(f.get('dur', 3.0))
    specs = [dict(f, t0=t) for t in eps]
    return specs, [(t, t + dur) for t in eps]


# ------------------------------------------------------------------ metrics
def output_checks(est, Df):
    t = np.asarray(est['t'], float)
    v = np.asarray(est['v'], float)
    hp = np.asarray(est['has_pose'], bool)
    xyz = np.column_stack([np.asarray(est[k], float) for k in ('x', 'y', 'z')]) if len(t) else np.zeros((0, 3))
    var = np.column_stack([np.asarray(est[k], float) for k in ('var_v', 'var_along')]) if len(t) else np.zeros((0, 2))
    stamps = np.concatenate([np.asarray((Df.get(tp) or {}).get('t_hdr', []), float) for tp in (FRONT, REAR, CMD)])
    return dict(n_out=int(len(t)),
                n_nonfinite_v=int((~np.isfinite(v)).sum()), n_neg_v=int((v < 0).sum()),
                n_nonfinite_pose=int((hp & ~np.isfinite(xyz).all(axis=1)).sum()),
                n_nonfinite_var=int((hp & ~np.isfinite(var).all(axis=1)).sum()),
                n_nonfinite_stamp=int((~np.isfinite(t)).sum()),
                stamps_from_input=float(np.isin(t, stamps).mean()) if len(t) else None,
                pose_frac=float(hp.mean()) if len(hp) else 0.0,
                v_max_out=_f(np.nanmax(v)) if len(v) else None)


def window_metrics(ctx, est_f, R_f, Df, a, b, want_naive, slip_family):
    """Metrics of one fault window [a, b] (absolute stamps)."""
    w = {}
    t_out = np.asarray(est_f['t'], float)
    dur = b - a
    inw = (t_out >= a) & (t_out < b)
    w['out_rate_hz'] = float(inw.sum() / dur) if dur > 0 else None
    if dur > 0:
        nb = int(np.ceil(dur / 0.1 - 1e-9))
        bins = np.unique(np.floor((t_out[inw] - a) / 0.1).astype(int))
        w['out_cov01'] = float(len(bins[(bins >= 0) & (bins < nb)]) / nb)
    Sf = R_f['series'] if R_f is not None else None
    Sc = ctx.R['series'] if ctx.R is not None else None
    if Sf is not None:
        tv = Sf['tv']
        m = (tv >= a) & (tv <= b + POST_S)
        ef, ec = Sf['v_err'][m], Sc['v_err'][m]
        w['n_ref'] = int(m.sum())
        w['ref_cov'] = float(np.isfinite(ef).mean()) if m.any() else None
        if np.isfinite(ef).any():
            w['v_rmse'] = float(np.sqrt(np.nanmean(ef ** 2)))
            w['v_max'] = float(np.nanmax(np.abs(ef)))
        if np.isfinite(ec).any():
            w['v_rmse_clean'] = float(np.sqrt(np.nanmean(ec ** 2)))
            w['v_max_clean'] = float(np.nanmax(np.abs(ec)))
        w['rec_s'] = recovery_time(tv, Sf['v_err'], b)
        w['rec_s_clean'] = recovery_time(Sc['tv'], Sc['v_err'], b)
        if want_naive and m.any():
            vn = naive_speed(Df, ctx.K, tv[m])
            en = vn - Sf['v_ref'][m]
            w['naive_nonfinite'] = int((~np.isfinite(en)).sum())
            w['naive_nonfinite_frac'] = float(np.mean(~np.isfinite(en)))
            # a naive fusion that outputs NaN has no finite error: do not hide the NaN with nanmean
            if np.isfinite(en).any() and w['naive_nonfinite'] == 0:
                w['naive_v_rmse'] = float(np.sqrt(np.nanmean(en ** 2)))
                w['naive_v_max'] = float(np.nanmax(np.abs(en)))
        # along-track error vs reference at b + AFTER_S (on-map only)
        tq = b + AFTER_S
        for key, S in (('al_f', Sf), ('al_c', Sc)):
            if len(S.get('tp', [])):
                j, dt = ev.nearest([tq], S['tp'])
                w[key] = float(S['al'][j[0]]) if dt[0] <= 0.2 and np.isfinite(S['al'][j[0]]) else None
        if w.get('al_f') is not None and w.get('al_c') is not None:
            w['d_al_30'] = w['al_f'] - w['al_c']
    # along-track displacement vs the clean run (estimate vs estimate)
    o = sorted_est(est_f)
    ft, fs, fv, fslip = est_f['t'][o], est_f['diag_s_route'][o], est_f['v'][o], est_f['slip'][o]
    for key, tq in (('d_s_b', b), ('d_s_b1', b + 1.0), ('d_s_30', b + AFTER_S), ('d_s_pre', a - 1.0)):
        sf = value_at(ft, fs, tq)
        sc = value_at(ctx.ct, ctx.cs, tq)
        w[key] = sf - sc if np.isfinite(sf) and np.isfinite(sc) else None
    # recovery against the clean replay (independent of the reference): |v - v_clean| on the clean stamps
    mc = (ctx.ct >= b) & (ctx.ct <= b + REC_HORIZON + REC_HOLD)
    if mc.any():
        vf_c = value_at(ft, fv, ctx.ct[mc])
        w['rec_vs_clean'] = recovery_time(ctx.ct[mc], np.atleast_1d(vf_c) - ctx.cv[mc], b)
    # first output at or after the end of the fault: what the estimate publishes when inputs return
    k1 = np.searchsorted(ft, b, side='left')
    if k1 < len(ft):
        t1 = ft[k1]
        w['first_after_dt'] = float(t1 - b)
        sc1, vc1 = value_at(ctx.ct, ctx.cs, t1), value_at(ctx.ct, ctx.cv, t1)
        w['first_after_d_s'] = float(fs[k1] - sc1) if np.isfinite(sc1) and np.isfinite(fs[k1]) else None
        w['first_after_d_v'] = float(fv[k1] - vc1) if np.isfinite(vc1) and np.isfinite(fv[k1]) else None
        va = np.asarray(est_f['var_along'], float)[o]
        w['first_after_sd_along'] = _f(np.sqrt(va[k1])) if np.isfinite(va[k1]) else None
        m2 = (ft >= t1) & (ft <= t1 + 3.0)
        w['max_sd_along_3s'] = _f(np.sqrt(np.nanmax(va[m2]))) if m2.any() and np.isfinite(va[m2]).any() else None
    # the same in PUBLICATION order (what a consumer receives first) and the worst output of the first second
    t_em = np.asarray(est_f['t'], float)
    s_em = np.asarray(est_f['diag_s_route'], float)
    va_em = np.asarray(est_f['var_along'], float)
    ke = np.where((t_em >= b) & (t_em <= b + 1.0))[0]
    if len(ke):
        ds_em = s_em[ke] - np.atleast_1d(value_at(ctx.ct, ctx.cs, t_em[ke]))
        if np.isfinite(ds_em[0]):
            w['first_emit_d_s'] = float(ds_em[0])
            w['first_emit_sd_along'] = _f(np.sqrt(va_em[ke[0]]))
        if np.isfinite(ds_em).any():
            j = int(np.nanargmax(np.abs(ds_em)))
            w['worst1s_d_s'] = float(ds_em[j])
            w['worst1s_sd_along'] = _f(np.sqrt(va_em[ke[j]]))
            w['worst1s_z'] = _f(abs(ds_em[j]) / np.sqrt(va_em[ke[j]])) if va_em[ke[j]] > 0 else None
    # largest step between consecutive outputs from the fault start to 2 s after its end
    m = (ft >= a - 0.5) & (ft <= b + 2.0)
    if m.sum() > 1:
        dv = np.abs(np.diff(fv[m]))
        k = int(np.nanargmax(dv))
        w['out_step_max'] = float(dv[k])
        w['out_step_at'] = float(ft[m][k + 1] - a)
    mc = (ctx.ct >= a - 0.5) & (ctx.ct <= b + 2.0)
    if mc.sum() > 1:
        w['out_step_max_clean'] = float(np.nanmax(np.abs(np.diff(ctx.cv[mc]))))
    if want_naive:
        m = (ctx.ct >= a) & (ctx.ct <= b)
        if m.sum() > 1:
            tt = ctx.ct[m]
            vn = naive_speed(Df, ctx.K, tt)
            d = vn - ctx.cv[m]
            w['naive_d_s_b'] = float(np.trapezoid(d, tt)) if np.isfinite(d).all() else None
    if slip_family:
        m0 = (ft >= a) & (ft <= a + 0.5)
        w['flag_0p5'] = bool(fslip[m0].any())
        # the clean replay already flags around this episode: the credit is not the injection's
        mc0 = (ctx.ct >= a - 0.5) & (ctx.ct <= b + 0.5)
        w['clean_flag_in_ep'] = bool(ctx.cslip[mc0].any())
        m1 = (ft >= a) & (ft <= b + 0.5) & fslip
        w['ttf'] = float(ft[m1][0] - a) if m1.any() else None
        mi = (ft >= a) & (ft < b)
        w['frac_in'] = float(fslip[mi].mean()) if mi.any() else None
        # episode speed error [a, b+1]: estimate vs naive mean vs clean estimate
        if Sf is not None:
            tv = Sf['tv']
            m = (tv >= a) & (tv <= b + 1.0)
            if m.any():
                ef, ec = Sf['v_err'][m], Sc['v_err'][m]
                en = naive_speed(Df, ctx.K, tv[m]) - Sf['v_ref'][m]
                for k, e in (('ep_rmse', ef), ('ep_rmse_clean', ec), ('ep_rmse_naive', en)):
                    w[k] = float(np.sqrt(np.nanmean(e ** 2))) if np.isfinite(e).any() else None
                for k, e in (('ep_max', ef), ('ep_max_naive', en)):
                    w[k] = float(np.nanmax(np.abs(e))) if np.isfinite(e).any() else None
        # distance error of the episode: displacement increment vs the clean run
        s1 = value_at(ft, fs, b + 5.0) - value_at(ctx.ct, ctx.cs, b + 5.0)
        s0 = value_at(ft, fs, a - 1.0) - value_at(ctx.ct, ctx.cs, a - 1.0)
        w['ep_d_s'] = float(s1 - s0) if np.isfinite(s1) and np.isfinite(s0) else None
        m = (ctx.ct >= a) & (ctx.ct <= b + 5.0)
        if m.sum() > 1:
            d = naive_speed(Df, ctx.K, ctx.ct[m]) - ctx.cv[m]
            w['ep_d_s_naive'] = float(np.trapezoid(d, ctx.ct[m])) if np.isfinite(d).all() else None
    return w


def run_scenario(ctx, sc, anchors, base_seed):
    specs, wins = build_specs(sc, anchors)
    rec = dict(scenario=sc['name'], family=sc['family'], anchor=sc['anchor'])
    if specs is None:
        rec['status'] = 'skipped'
        rec['error'] = 'no anchor'
        return rec
    rec['specs'] = specs
    rec['windows_rel'] = wins
    seed = seed_for(ctx.bag, sc['name'], base_seed)
    try:
        Df = apply_faults(ctx.D, specs, seed)
        c0 = time.process_time()
        est = run_bag(ctx.bag, params=copy.copy(ctx.p), data=Df, gnss_limit_s=ctx.gnss_limit)
        rec['cpu_s'] = time.process_time() - c0
    except Exception as e:  # a crash is a result
        rec['status'] = 'crash'
        rec['error'] = '%s: %s' % (type(e).__name__, e)
        rec['traceback'] = traceback.format_exc()
        return rec
    rec['status'] = 'ok'
    rec.update(output_checks(est, Df))
    rec['finite'] = (rec['n_nonfinite_v'] == 0 and rec['n_nonfinite_pose'] == 0 and rec['n_nonfinite_var'] == 0
                     and rec['n_nonfinite_stamp'] == 0)
    R = None
    if ctx.ref is not None:
        try:
            R = ev.evaluate(ctx.bag, est, series=True, D=ctx.D, ref=ctx.ref)
        except Exception as e:
            rec['eval_error'] = '%s: %s' % (type(e).__name__, e)
    if R is not None:
        Rc = ctx.R
        for k in ('v_rmse', 'v_max', 'al_rmse', 'al_max', 'e3d_mean', 'drift_pct', 'drift_onmap_pct', 'ct_rmse',
                  'p_in95', 'v_cov', 'p_cov'):
            rec[k] = _f(R.get(k))
            rec[k + '_clean'] = _f(Rc.get(k))
        for k in ('al_rmse', 'drift_pct', 'e3d_mean', 'v_rmse'):
            if rec[k] is not None and rec[k + '_clean'] is not None:
                rec['d_' + k] = rec[k] - rec[k + '_clean']
    base = ctx.base
    want_naive = sc['family'] in WHEEL_FAMILIES
    slip_family = sc['family'] == 'slip'
    rec['windows'] = []
    for (t0, t1) in wins:
        a, b = base + t0, base + t1
        if sc['anchor'] == 'whole':
            ts = np.asarray(est['t'], float)
            a, b = float(np.nanmin(ts)), float(np.nanmax(ts))
        try:
            w = window_metrics(ctx, est, R, Df, a, b, want_naive and sc['anchor'] != 'whole', slip_family)
        except Exception as e:  # a bug of this script, not of the estimator
            w = dict(metrics_error='%s: %s' % (type(e).__name__, e), traceback=traceback.format_exc())
        w['t0_rel'], w['t1_rel'] = t0, t1
        rec['windows'].append(w)
    # slip flag outside the injected episodes (and their hold), fault vs clean on the same stamps
    o = sorted_est(est)
    ft, fslip = est['t'][o], est['slip'][o]
    out_f = np.ones(len(ft), bool)
    out_c = np.ones(len(ctx.ct), bool)
    if sc['anchor'] != 'whole':
        for (t0, t1) in wins:
            a, b = base + t0, base + t1 + ctx.p.slip_hold_s + 1.0
            out_f &= ~((ft >= a) & (ft <= b))
            out_c &= ~((ctx.ct >= a) & (ctx.ct <= b))
    rec['slip_frac_out'] = float(fslip[out_f].mean()) if out_f.any() else None
    rec['slip_frac_out_clean'] = float(ctx.cslip[out_c].mean()) if out_c.any() else None
    rec['slip_frac'] = float(np.mean(est['slip'])) if len(est['slip']) else None
    rec['slip_frac_clean'] = float(np.mean(ctx.cslip)) if len(ctx.cslip) else None
    stc = mode_state(ctx.est['mode'])
    rec['disagree_frac_clean'] = float(np.mean(stc == 'disagree')) if len(stc) else None
    rec['slip_episodes_out'] = len(episodes_from_mask(ft[out_f], fslip[out_f], 0.3))
    rec['slip_episodes_out_clean'] = len(episodes_from_mask(ctx.ct[out_c], ctx.cslip[out_c], 0.3))
    if any(sp['kind'] == 'spikes' for sp in specs):
        # spikes are drawn per sample independently: two adjacent spiked samples of one bogie form
        # a 0.2 s level, which the spike filter accepts by design (a level that persists)
        n_sp = n_adj = 0
        for tp in (FRONT, REAR):
            d = np.asarray(Df[tp]['v'], float) != np.asarray(ctx.D[tp]['v'], float)
            n_sp += int(d.sum())
            n_adj += int((d[1:] & d[:-1]).sum())
        rec['n_spiked_samples'] = n_sp
        rec['n_adjacent_spike_pairs'] = n_adj
    st = mode_state(est['mode'])
    rec['state_frac'] = {s: float(np.mean(st == s)) for s in np.unique(st)}
    rec['disagree_frac'] = float(np.mean(st == 'disagree')) if len(st) else None
    # end-of-run along-track displacement vs clean
    ts_end = min(ft[-1], ctx.ct[-1]) - 1.0
    sf, scl = value_at(ft, est['diag_s_route'][o], ts_end), value_at(ctx.ct, ctx.cs, ts_end)
    rec['d_s_end'] = float(sf - scl) if np.isfinite(sf) and np.isfinite(scl) else None
    # adaptive model gains: does the fault drive them (a wrong command can, distorted wheels should not)
    for k, key in (('gtr', 'diag_gain_tr'), ('gbr', 'diag_gain_br')):
        for suf, E in (('', est), ('_clean', ctx.est)):
            x = np.asarray(E[key], float)
            x = x[np.isfinite(x)]
            rec[k + '_min' + suf] = _f(x.min()) if len(x) else None
            rec[k + '_max' + suf] = _f(x.max()) if len(x) else None
    return rec


def clean_summary(ctx):
    R = ctx.R or {}
    return {k: _f(R.get(k)) for k in ('v_rmse', 'v_max', 'al_rmse', 'al_max', 'e3d_mean', 'drift_pct', 'ct_rmse',
                                        'p_in95')}


# ------------------------------------------------------------------ tasks (run in workers)
def task_faults(bag, scen_names, cfg):
    t0 = time.time()
    out = dict(bag=bag, part='faults')
    try:
        ctx = Ctx(bag, cfg['gnss_limit'])
    except Exception as e:
        out.update(status='crash', error='%s: %s' % (type(e).__name__, e), traceback=traceback.format_exc())
        return out
    anc = ctx.anchors()
    out['anchors'] = anc
    out['clean'] = clean_summary(ctx)
    out['clean'].update(output_checks(ctx.est, ctx.D))
    out['K'] = ctx.K
    out['scenarios'] = {}
    for name in scen_names:
        out['scenarios'][name] = run_scenario(ctx, SCEN[name], anc, cfg['seed'])
    out['wall_s'] = time.time() - t0
    return out


def clock_glitch(e):
    """Slip episode within +-1 s of an input stamp glitch or of a GNSS clock-offset start / end."""
    return bool(e.get('input_stamp_glitch') or e.get('gnss_clock_step'))


def clock_glitch_label(e):
    p = []
    if e.get('input_stamp_glitch'):
        p.append('сбой штампов входа (%d отсч.)' % e['input_stamp_glitch'])
    if e.get('gnss_clock_step'):
        p.append('смена сдвига часов GNSS')
    return ', '.join(p)


def slip_episode_table(ctx, gap=0.3, max_list=40):
    """Slip-flag episodes of the clean replay with bogie values and reference evidence."""
    est = ctx.est
    o = sorted_est(est)
    t, slip, v = est['t'][o], est['slip'][o], est['v'][o]
    vf, vr = est['diag_v_front'][o], est['diag_v_rear'][o]
    eps = episodes_from_mask(t, slip, gap)
    # confirmation against the cleaned + restamped reference (a frozen GNSS speed or a shifted
    # GNSS clock would otherwise "confirm" an episode); Doppler speed when there is no RTK
    sr = ctx.speed_ref()
    S = sr if sr['kind'] is not None else None
    # input stamp glitches (clock steps of the wheel / command headers): the outputs carry the
    # glitched stamps by contract, so neither the estimate nor the naive mean can be judged there
    gl = []
    for tp in (FRONT, REAR, CMD):
        d = ctx.D.get(tp) or {}
        if len(d.get('t_hdr', [])) >= 3:
            th = np.asarray(d['t_hdr'], float)
            tr = np.asarray(d['t_rec'], float)
            mk = ev.stamp_glitch_mask(th, tr)
            # a header AHEAD of its receipt by > 0.3 s (vs the bag median) cannot be a delivery delay:
            # a slow header-clock drift that the running median of stamp_glitch_mask follows
            mk |= (th - tr) - np.median(th - tr) > 0.3
            gl += [th[mk], tr[mk] + np.median(th - tr)]
    gl = np.concatenate(gl) if gl else np.zeros(0)
    # starts / ends of sustained GNSS clock offsets: the reference switches its time base there, and in
    # the logs the wheel stream jumps at the same moments (content skip or header step)
    gcs = np.array([ctx.base + x for r in (sr.get('offset_runs') or []) for x in r[:2]])
    rows = []
    for (s, e) in eps:
        m = (t >= s) & (t <= e)
        pre = (t >= s - 1.0) & (t < s)
        row = dict(start_rel=s - ctx.base, dur=e - s,
                   v_est_pre=_f(np.nanmean(v[pre])) if pre.any() else None,
                   v_est_min=_f(np.nanmin(v[m])), v_est_max=_f(np.nanmax(v[m])),
                   v_front_min=_f(np.nanmin(vf[m])) if np.isfinite(vf[m]).any() else None,
                   v_front_max=_f(np.nanmax(vf[m])) if np.isfinite(vf[m]).any() else None,
                   v_rear_min=_f(np.nanmin(vr[m])) if np.isfinite(vr[m]).any() else None,
                   v_rear_max=_f(np.nanmax(vr[m])) if np.isfinite(vr[m]).any() else None,
                   max_abs_front_rear=_f(np.nanmax(np.abs(vf[m] - vr[m]))) if np.isfinite(vf[m] - vr[m]).any() else None,
                   input_stamp_glitch=int(((gl >= s - 1.0) & (gl <= e + 1.0)).sum()),
                   gnss_clock_step=int(((gcs >= s - 1.0) & (gcs <= e + 1.0)).sum()))
        # raw bogie values (m/s) over the episode
        mr = (np.asarray(ctx.D[FRONT]['t_hdr']) >= s) & (np.asarray(ctx.D[FRONT]['t_hdr']) <= e)
        if mr.any():
            row['raw_front_min'] = _f(np.min(ctx.D[FRONT]['v'][mr]) / ctx.K)
            row['raw_front_max'] = _f(np.max(ctx.D[FRONT]['v'][mr]) / ctx.K)
        mr = (np.asarray(ctx.D[REAR]['t_hdr']) >= s) & (np.asarray(ctx.D[REAR]['t_hdr']) <= e)
        if mr.any():
            row['raw_rear_min'] = _f(np.min(ctx.D[REAR]['v'][mr]) / ctx.K)
            row['raw_rear_max'] = _f(np.max(ctx.D[REAR]['v'][mr]) / ctx.K)
        if S is not None:
            mv = (S['tv'] >= s - 0.5) & (S['tv'] <= e + 0.5)
            if mv.any():
                vref = S['v_ref'][mv]
                row['v_ref_min'] = _f(np.min(vref))
                row['v_ref_max'] = _f(np.max(vref))
                ee = S['v_err'][mv]
                row['est_max_err'] = _f(np.nanmax(np.abs(ee))) if np.isfinite(ee).any() else None
                en = naive_speed(ctx.D, ctx.K, S['tv'][mv]) - vref
                row['naive_max_err'] = _f(np.nanmax(np.abs(en))) if np.isfinite(en).any() else None
                row['confirmed'] = bool(row['naive_max_err'] is not None and row['naive_max_err'] > SLIP_CONFIRM)
                row['ref_kind'] = S['kind']
        rows.append(row)
    return rows


def state_episodes(ctx, min_dur=0.5, gap=0.5):
    o = sorted_est(ctx.est)
    t = ctx.est['t'][o]
    st = mode_state(ctx.est['mode'][o])
    rows = []
    for s_name in ('single_front', 'single_rear', 'model_only', 'disagree'):
        for (s, e) in episodes_from_mask(t, st == s_name, gap):
            if e - s >= min_dur or s_name == 'disagree':
                rows.append(dict(state=s_name, start_rel=s - ctx.base, dur=e - s))
    rows.sort(key=lambda r: r['start_rel'])
    return rows


def adhesion_gain_stats(ctx):
    est = ctx.est
    v = est['v']
    mv = v > 1.0
    ad = est['diag_adhesion_used']
    gtr, gbr, bias = est['diag_gain_tr'], est['diag_gain_br'], est['diag_bias']
    slip = est['slip']
    d = {}
    if mv.any():
        a = ad[mv & np.isfinite(ad)]
        d.update(adh_p50=_f(np.percentile(a, 50)), adh_p95=_f(np.percentile(a, 95)), adh_p99=_f(np.percentile(a, 99)),
                 adh_max=_f(a.max()), adh_frac_gt_0p15=_f(np.mean(a > 0.15)))
        ms = mv & slip & np.isfinite(ad)
        d['adh_max_slip'] = _f(ad[ms].max()) if ms.any() else None
    for k, x in (('gtr', gtr), ('gbr', gbr), ('bias', bias)):
        x = x[np.isfinite(x)]
        if len(x):
            d[k + '_min'] = _f(x.min())
            d[k + '_max'] = _f(x.max())
            d[k + '_final'] = _f(x[-1])
            d[k + '_p05'] = _f(np.percentile(x, 5))
            d[k + '_p95'] = _f(np.percentile(x, 95))
    # cross-check: kinematic acceleration of the published speed (1 s central difference on a
    # 0.1 s grid) against the model acceleration `a` that adhesion_used is computed from
    o = sorted_est(est)
    t, vv, am = est['t'][o], est['v'][o], est['a'][o]
    fin = np.isfinite(t) & np.isfinite(vv) & np.isfinite(am)
    if fin.sum() > 50:
        g = np.arange(t[fin][0], t[fin][-1], 0.1)
        vg = np.interp(g, t[fin], vv[fin])
        ak = vg[10:] - vg[:-10]
        gc = g[5:-5]
        mk = vg[5:-5] > 1.0
        if mk.any():
            amg = np.interp(gc, t[fin], am[fin])
            x = np.abs(ak[mk]) / G0
            d.update(adhk_p99=_f(np.percentile(x, 99)), adhk_max=_f(x.max()),
                     a_diff_p95=_f(np.percentile(np.abs(amg[mk] - ak[mk]), 95)),
                     a_sign_wrong=_f(np.mean((np.abs(ak[mk]) > 0.3) & (amg[mk] * ak[mk] < 0))))
    return d


GNSS_TOPICS = ('/sensing/gnss/master/fix', '/sensing/gnss/master/vel', '/sensing/gnss/rover/fix', '/sensing/gnss/rover/vel')
RESTAMP_THR = 0.5
RESTAMP_MIN_S = 5.0      # a clock offset is sustained over >= 5 s at the topic's normal rate
RESTAMP_EXT = 0.1        # s: a sustained run is extended over its slewing edges while |offset| > this


def restamp_gnss(D, thr=RESTAMP_THR, min_span=RESTAMP_MIN_S, ext=RESTAMP_EXT):
    """Copy of D whose GNSS header stamps with a sustained clock offset are moved to t_rec + median offset.

    `evaluate --clean-ref` drops short stamp glitches (running median over ~5 s) but not a GNSS
    clock offset that lasts a minute (30618_40ffd323: +1 s over 85-145 s, -1 s over 446-506 s of
    t_rec); against such a reference an exact estimate shows |a| * 1 s of speed error.  Only runs
    of offset samples spanning >= min_span of t_rec at no more than 1.5x the topic's median rate are
    moved: the burst at the start of every bag and the bursts after a delivery stall (many samples
    with one t_rec) have a correct header stamp and a late t_rec, and are left as they are.  The
    offset is slewed in and out over ~10 s (40ffd323: -1 s -> 0 over 500-512 s), so an accepted run
    is extended over the adjacent samples while |offset| > ext.
    Only the reference changes: the estimator has already run on the original data.
    Returns (D2, n, runs): n = restamped samples per topic, runs = [(t_rec start, t_rec end, offset)].
    """
    D2 = dict(D)
    n, runs = {}, {}
    for tp in GNSS_TOPICS:
        d = D.get(tp) or {}
        th = np.asarray(d.get('t_hdr', []), float)
        if len(th) < 3:
            continue
        key = tp.split('/')[-2] + '_' + tp.split('/')[-1]
        tr = np.asarray(d['t_rec'], float)
        off = th - tr
        med = float(np.median(off))
        rate = (len(tr) - 1) / max(tr.max() - tr.min(), 1e-9)
        idx = np.where(np.abs(off - med) > thr)[0]
        fix = np.zeros(len(th), bool)
        rr = []
        if len(idx):
            cuts = np.where(np.abs(np.diff(tr[idx])) > 1.0)[0]
            for grp in np.split(idx, cuts + 1):
                span = tr[grp].max() - tr[grp].min()
                if span >= min_span and (len(grp) - 1) / span <= 1.5 * rate:
                    lo, hi = grp.min(), grp.max()
                    while lo > 0 and abs(off[lo - 1] - med) > ext:
                        lo -= 1
                    while hi < len(off) - 1 and abs(off[hi + 1] - med) > ext:
                        hi += 1
                    sel = np.arange(lo, hi + 1)
                    sel = sel[np.abs(off[sel] - med) > ext]
                    fix[sel] = True
                    rr.append((float(tr[sel].min()), float(tr[sel].max()), float(np.median(off[grp]) - med)))
        if fix.any():
            d2 = dict(d)
            d2['t_hdr'] = np.where(fix, tr + med, th)
            D2[tp] = d2
        n[key] = int(fix.sum())
        runs[key] = rr
    return D2, n, runs


def speed_err_stats(e, st=None):
    """RMSE / max / counts of a speed-error array (NaN = no reference), optionally per bogie state."""
    ok = np.isfinite(e)
    d = dict(n=int(ok.sum()))
    if not ok.any():
        return d
    ee = e[ok]
    d.update(v_rmse=_f(np.sqrt(np.mean(ee ** 2))), v_max=_f(np.max(np.abs(ee))),
             n_gt0p5=int((np.abs(ee) > 0.5).sum()), n_gt1=int((np.abs(ee) > 1.0).sum()))
    if st is not None:
        s_ok = st[ok]
        d['by_state'] = {s: dict(n=int((s_ok == s).sum()), v_rmse=_f(np.sqrt(np.mean(ee[s_ok == s] ** 2))),
                                 v_max=_f(np.max(np.abs(ee[s_ok == s]))))
                         for s in np.unique(s_ok)}
    return d


def input_stamp_glitches(D):
    out = {}
    for name, tp in (('front', FRONT), ('rear', REAR), ('cmd', CMD)):
        d = D.get(tp) or {}
        if len(d.get('t_hdr', [])) < 3:
            continue
        m = ev.stamp_glitch_mask(np.asarray(d['t_hdr'], float), np.asarray(d['t_rec'], float))
        th = np.asarray(d['t_hdr'], float)
        out[name] = dict(n_glitch=int(m.sum()), n_nonmono=int((np.diff(th) < 0).sum()),
                         max_back_s=_f(-np.min(np.diff(th))) if (np.diff(th) < 0).any() else 0.0)
    return out


def task_clean(bag, cfg, detail=False, category=None):
    out = dict(bag=bag, part='real' if detail else 'clean', category=category)
    try:
        ctx = Ctx(bag, cfg['gnss_limit'])
    except Exception as e:
        out.update(status='crash', error='%s: %s' % (type(e).__name__, e), traceback=traceback.format_exc())
        return out
    out['status'] = 'ok'
    out['K'] = ctx.K
    out['duration_s'] = ctx.T
    out.update(output_checks(ctx.est, ctx.D))
    out['metrics'] = clean_summary(ctx) if ctx.R is not None else None
    est = ctx.est
    v = est['v']
    out['slip_frac'] = float(est['slip'].mean())
    out['slip_frac_moving'] = float(est['slip'][v > 1.0].mean()) if (v > 1.0).any() else None
    eps = slip_episode_table(ctx)
    out['slip_episodes_n'] = len(eps)
    out['slip_time_s'] = float(sum(e['dur'] for e in eps))
    out['slip_stamp_n'] = int(sum(1 for e in eps if clock_glitch(e)))
    out['slip_confirmed_n'] = int(sum(1 for e in eps if e.get('confirmed') is True and not clock_glitch(e)))
    out['slip_unconfirmed_n'] = int(sum(1 for e in eps if e.get('confirmed') is False and not clock_glitch(e)))
    out['slip_noref_n'] = int(sum(1 for e in eps if 'confirmed' not in e and not clock_glitch(e)))
    st = mode_state(est['mode'])
    out['state_frac'] = {s: float(np.mean(st == s)) for s in np.unique(st)}
    out['adh'] = adhesion_gain_stats(ctx)
    out['gnss_restamped'] = ctx.speed_ref()['n_restamped']
    out['gnss_offset_runs'] = ctx.speed_ref()['offset_runs']
    if detail:
        out['slip_episodes'] = eps
        out['state_episodes'] = state_episodes(ctx)
        out['stamp_glitches'] = input_stamp_glitches(ctx.D)
        if ctx.R is not None:
            S = ctx.R['series']
            e = S['v_err']
            out['v_max_err_at_rel'] = _f(S['tv'][np.nanargmax(np.abs(e))] - ctx.base) if np.isfinite(e).any() else None
            out['n_ref_v'] = int(np.isfinite(e).sum())
            out['n_err_gt1'] = int((np.abs(e) > 1.0).sum())
            out['n_err_gt0p5'] = int((np.abs(e) > 0.5).sum())
            # the same estimate against the cleaned reference (stamp glitches, frozen GNSS speed and
            # position jumps removed, as `evaluate.py --clean-ref`): single-sample reference artefacts
            try:
                Rc = ev.evaluate(bag, est, D=ctx.D, clean_ref=True)
                out['metrics_cleanref'] = {k: _f(Rc.get(k)) for k in ('v_rmse', 'v_max', 'al_rmse', 'drift_pct')}
            except Exception as ex:
                out['metrics_cleanref_error'] = '%s: %s' % (type(ex).__name__, ex)
        # cleaned + restamped RTK reference, or the Doppler speed when the bag has no RTK
        sr = ctx.speed_ref()
        out['ref_kind'] = sr['kind']
        out['gnss_restamped'] = sr['n_restamped']
        if sr['kind'] is not None:
            e = sr['v_err']
            out['speed_fix'] = speed_err_stats(e, sr['st'])
            if np.isfinite(e).any():
                out['speed_fix']['v_max_at_rel'] = _f(sr['tv'][np.nanargmax(np.abs(e))] - ctx.base)
        if sr['R'] is not None:
            out['metrics_fixref'] = {k: _f(sr['R'].get(k)) for k in ('v_rmse', 'v_max', 'al_rmse', 'drift_pct')}
    else:
        out['slip_episodes'] = [dict(start_rel=e['start_rel'], dur=e['dur'], confirmed=e.get('confirmed'),
                                     naive_max_err=e.get('naive_max_err'), est_max_err=e.get('est_max_err'),
                                     input_stamp_glitch=e.get('input_stamp_glitch'), gnss_clock_step=e.get('gnss_clock_step'))
                                for e in eps]
    return out


# ------------------------------------------------------------------ aggregation
def _agg(vals, absval=True):
    x = np.array([v for v in vals if v is not None and np.isfinite(v)], float)
    if absval:
        x = np.abs(x)
    if len(x) == 0:
        return dict(n=0, median=None, p90=None, max=None, mean=None)
    return dict(n=int(len(x)), median=float(np.median(x)), p90=float(np.percentile(x, 90)), max=float(x.max()),
                mean=float(x.mean()), min=float(x.min()))


def aggregate_faults(fault_results):
    agg = {}
    for sc in SCENARIOS:
        name = sc['name']
        recs = [r['scenarios'][name] for r in fault_results if 'scenarios' in r and name in r['scenarios']]
        ran = [r for r in recs if r.get('status') in ('ok', 'crash')]
        ok = [r for r in recs if r.get('status') == 'ok']
        wins = [w for r in ok for w in r.get('windows', [])]
        A = dict(name=name, family=sc['family'], anchor=sc['anchor'], ru=sc['ru'], faults=sc['faults'],
                 n_bags=len(ran), n_skipped=len(recs) - len(ran), n_crash=len(ran) - len(ok),
                 n_finite=sum(1 for r in ok if r.get('finite')),
                 n_neg_v=sum(r.get('n_neg_v', 0) for r in ok),
                 stamps_from_input_min=_f(min([r['stamps_from_input'] for r in ok if r.get('stamps_from_input') is not None],
                                              default=np.nan)),
                 n_windows=len(wins))
        for k in ('out_rate_hz', 'out_cov01', 'v_rmse', 'v_max', 'v_rmse_clean', 'v_max_clean', 'rec_s', 'rec_s_clean',
                  'rec_vs_clean', 'naive_v_rmse', 'naive_v_max', 'd_al_30', 'd_s_b', 'd_s_b1', 'd_s_30', 'naive_d_s_b',
                  'out_step_max', 'out_step_max_clean', 'first_after_dt', 'first_after_d_s', 'first_after_d_v',
                  'first_after_sd_along', 'max_sd_along_3s', 'naive_nonfinite', 'first_emit_d_s', 'first_emit_sd_along',
                  'worst1s_d_s', 'worst1s_sd_along', 'worst1s_z',
                  'ep_rmse', 'ep_rmse_clean', 'ep_rmse_naive', 'ep_max', 'ep_max_naive', 'ep_d_s', 'ep_d_s_naive',
                  'ttf', 'frac_in'):
            vals = [w.get(k) for w in wins]
            if k in ('rec_s', 'rec_s_clean', 'rec_vs_clean'):
                A[k + '_n_inf'] = int(sum(1 for v in vals if v is not None and np.isinf(v)))
            A[k] = _agg([v for v in vals if v is None or not np.isinf(v)])
        for k in ('v_rmse', 'v_rmse_clean', 'al_rmse', 'al_rmse_clean', 'd_al_rmse', 'drift_pct', 'drift_pct_clean',
                  'd_drift_pct', 'd_e3d_mean', 'd_s_end', 'slip_frac_out', 'slip_frac_out_clean', 'v_max',
                  'v_max_clean', 'e3d_mean', 'e3d_mean_clean', 'p_in95', 'slip_episodes_out', 'slip_episodes_out_clean',
                  'n_spiked_samples', 'n_adjacent_spike_pairs', 'slip_frac', 'slip_frac_clean', 'disagree_frac',
                  'disagree_frac_clean', 'gtr_min', 'gtr_min_clean', 'gtr_max', 'gtr_max_clean', 'gbr_min', 'gbr_min_clean',
                  'gbr_max', 'gbr_max_clean'):
            A['run_' + k] = _agg([r.get(k) for r in ok], absval=False)
        for k in ('d_s_b', 'd_s_b1', 'naive_d_s_b', 'd_al_30', 'first_after_d_s', 'd_s_30', 'first_emit_d_s',
                  'worst1s_d_s'):
            A[k + '_signed'] = _agg([w.get(k) for w in wins], absval=False)
        for suf in ('', '_clean'):
            A['n_gain_lo' + suf] = int(sum(1 for r in ok if min(r.get('gtr_min' + suf) or 1.0, r.get('gbr_min' + suf) or 1.0) <= 0.801))
            A['n_gain_hi' + suf] = int(sum(1 for r in ok if max(r.get('gtr_max' + suf) or 1.0, r.get('gbr_max' + suf) or 1.0) >= 1.299))
        # signed deltas also as |.|
        for k in ('d_al_rmse', 'd_drift_pct', 'd_e3d_mean', 'd_s_end'):
            A['run_abs_' + k] = _agg([r.get(k) for r in ok], absval=True)
        if sc['family'] == 'slip':
            fl = [w.get('flag_0p5') for w in wins if w.get('flag_0p5') is not None]
            A['recall_0p5'] = float(np.mean(fl)) if fl else None
            A['n_flagged_0p5'] = int(sum(fl))
            A['n_flagged_any'] = int(sum(1 for w in wins if w.get('ttf') is not None))
            # recall over the episodes where the clean replay has no flag of its own around the episode
            fx = [w.get('flag_0p5') for w in wins if w.get('flag_0p5') is not None and not w.get('clean_flag_in_ep')]
            A['n_windows_noclean'] = len(fx)
            A['recall_0p5_noclean'] = float(np.mean(fx)) if fx else None
            A['n_clean_flag_in_ep'] = int(sum(1 for w in wins if w.get('clean_flag_in_ep')))
            A['n_flagged_any_noclean'] = int(sum(1 for w in wins if w.get('ttf') is not None and not w.get('clean_flag_in_ep')))
        agg[name] = A
    return agg


def aggregate_clean(clean_results):
    ok = [r for r in clean_results if r.get('status') == 'ok']
    tot_h = sum(r['duration_s'] for r in ok) / 3600.0
    n_ep = sum(r['slip_episodes_n'] for r in ok)
    d = dict(n_bags=len(ok), n_crash=len(clean_results) - len(ok), hours=tot_h,
             slip_frac=_agg([r['slip_frac'] for r in ok]),
             slip_frac_moving=_agg([r['slip_frac_moving'] for r in ok]),
             slip_frac_pooled=float(np.sum([r['slip_frac'] * r['n_out'] for r in ok]) / max(1, np.sum([r['n_out'] for r in ok]))),
             slip_time_s=float(sum(r['slip_time_s'] for r in ok)),
             slip_episodes=n_ep, slip_episodes_per_h=n_ep / tot_h if tot_h > 0 else None,
             slip_confirmed=int(sum(r['slip_confirmed_n'] for r in ok)),
             slip_unconfirmed=int(sum(r['slip_unconfirmed_n'] for r in ok)),
             slip_noref=int(sum(r['slip_noref_n'] for r in ok)),
             slip_stamp=int(sum(r.get('slip_stamp_n', 0) for r in ok)),
             slip_stamp_bags=sorted(r['bag'] for r in ok if r.get('slip_stamp_n')),
             gnss_restamped_bags=sorted(r['bag'] for r in ok if r.get('gnss_offset_runs')),
             gnss_offset_runs={r['bag']: r['gnss_offset_runs'] for r in ok if r.get('gnss_offset_runs')},
             gnss_restamped_vel=int(sum((r.get('gnss_restamped') or {}).get('master_vel', 0) for r in ok)),
             bags_with_flag=int(sum(1 for r in ok if r['slip_episodes_n'] > 0)))
    unconf = [e for r in ok for e in r['slip_episodes'] if e.get('confirmed') is False and not clock_glitch(e)]
    d['unconfirmed_time_s'] = float(sum(e['dur'] for e in unconf))
    d['unconfirmed_frac_pooled'] = d['unconfirmed_time_s'] / (tot_h * 3600.0) if tot_h > 0 else None
    states = {}
    for r in ok:
        for s, f in r['state_frac'].items():
            states.setdefault(s, []).append(f)
    d['state_frac_mean'] = {s: float(np.mean(v + [0.0] * (len(ok) - len(v)))) for s, v in states.items()}
    A = [r['adh'] for r in ok]
    for k in ('adh_p50', 'adh_p95', 'adh_p99', 'adh_max', 'adh_frac_gt_0p15', 'adh_max_slip',
              'adhk_p99', 'adhk_max', 'a_diff_p95', 'a_sign_wrong',
              'gtr_min', 'gtr_max', 'gtr_final', 'gbr_min', 'gbr_max', 'gbr_final', 'bias_min', 'bias_max'):
        d[k] = _agg([a.get(k) for a in A], absval=False)
    d['gtr_range'] = _agg([a['gtr_max'] - a['gtr_min'] for a in A if a.get('gtr_max') is not None], absval=False)
    d['gbr_range'] = _agg([a['gbr_max'] - a['gbr_min'] for a in A if a.get('gbr_max') is not None], absval=False)
    d['gain_bound_hits'] = dict(
        gtr_lo=int(sum(1 for a in A if (a.get('gtr_min') or 1.0) <= 0.801)),
        gtr_hi=int(sum(1 for a in A if (a.get('gtr_max') or 1.0) >= 1.299)),
        gbr_lo=int(sum(1 for a in A if (a.get('gbr_min') or 1.0) <= 0.801)),
        gbr_hi=int(sum(1 for a in A if (a.get('gbr_max') or 1.0) >= 1.299)))
    return d


# ------------------------------------------------------------------ markdown
def fmt(x, nd=3):
    if x is None:
        return '–'
    if isinstance(x, (bool, np.bool_)):
        return 'да' if x else 'нет'
    if isinstance(x, (int, np.integer)):
        return str(int(x))
    if not np.isfinite(x):
        return '∞' if x > 0 else '–'
    ax = abs(x)
    if ax >= 100:
        return '%.0f' % x
    if ax >= 10:
        return '%.1f' % x
    if ax >= 1:
        return '%.2f' % x
    return ('%.' + str(nd) + 'f') % x


def mm(A, k, nd=3, stat=('median', 'max')):
    a = A.get(k) or {}
    return ' / '.join(fmt(a.get(s), nd) for s in stat)


REPRO_CMD = 'OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python tools/robustness.py --jobs 4'


def _nz(A, k, stat):
    return (A.get(k) or {}).get(stat)


def write_markdown(S, path):
    L = []
    cfg = S['config']
    L.append('# Устойчивость: инъекция отказов, реальные аномалии, флаг проскальзывания\n')
    L.append('Сгенерировано `tools/robustness.py` (%s, %s с на %d процессах). Все числа ниже получены этим скриптом на текущем коде '
             'пакета; ничего не переносилось из старых прогонов.\n' % (S['generated'], fmt(cfg.get('wall_s'), 0), cfg.get('jobs', 4)))
    L.append('Воспроизведение (из каталога `solution/`, Python с numpy/scipy/matplotlib, кэши бэгов в `.work/cache`):\n')
    L.append('```bash\n%s\n```\n' % REPRO_CMD)
    L.append('Части по отдельности: `--part faults|clean|real|plots`; один сценарий на одном заезде: '
             '`python tools/robustness.py --part faults --bags 30618_073f08d1 --scenarios drop_both_30 --jobs 1 --out /tmp/rb`; '
             'список сценариев: `--list`.\n')
    L.append('## 0. Метод\n')
    L.append('- **Оценщик** — `Pipeline` через `replay.run_bag`, т. е. ровно тот код, что работает в узле ROS 2; сообщения подаются в '
             'порядке записи (как `ros2 bag play`) со своими `header.stamp`. GNSS подаётся только первые %.0f с (начальная привязка), '
             'дальше — только колёса, команда контроллера и карта.' % cfg['gnss_limit'])
    L.append('- **Отказ вносится в копию данных** (`tools/inject_faults.apply` + локальные виды `slip_rear`, `slip_both`, '
             '`noise_moving` в `robustness.apply_faults`): меняются значения `v` и/или `header.stamp`, пропавшие сообщения удаляются '
             'целиком, порядок доставки не меняется. **Эталон всегда строится из чистых данных** (RTK master, скорость GNSS; '
             '`evaluate.load_ref(D=чистые)`), поэтому прогон с отказом и чистый прогон оцениваются по одному и тому же эталону.')
    L.append('- **Парное сравнение.** Каждый прогон с отказом сравнивается с чистым прогоном того же заезда: Δ-метрики '
             '(Δs, Δal, Δдрейф, восстановление к чистому прогону) не зависят от собственной ошибки оценки и артефактов эталона.')
    L.append('- **Наивный базис** — фузия колёс без проверок: среднее последних сырых отсчётов двух тележек / K, без контроля '
             'свежести, согласия и NaN (при `drop_both` это удержание последнего значения, ZOH).')
    L.append('- **Флаг `slip`** оценивается тремя способами: recall и время до флага на введённых эпизодах (синтетика), доля '
             'и число эпизодов флага на 60 чистых заездах `eval_ok` (бюджет ложных срабатываний, с проверкой по эталону), '
             'разбор эпизодов на реальных аномальных заездах.')
    L.append('- **Сцепление и адаптация**: `diag_adhesion_used` = |a|/g, где a — ускорение модели процесса EKF (то же, что '
             '`Output.a`), с проверкой против dv/dt выходной скорости, и онлайн-коэффициенты модели `diag_gain_tr`, '
             '`diag_gain_br` (состояния EKF), `diag_bias` — на чистых прогонах 60 заездов и под каждым отказом (табл. 7a).\n')
    if 'faults' in S:
        F = S['faults']
        agg = F['agg']
        n_runs = sum(A['n_bags'] for A in agg.values())
        L.append('## 1. Синтетические отказы\n')
        L.append('Заезды (%d: 6 T2S / 6 S2T, обе машины, без артефактов эталона и часов): %s. Сценариев: %d, прогонов с отказом: %d.\n'
                 % (len(F['bags']), ', '.join('`%s`' % b for b in F['bags']), sum(1 for A in agg.values() if A['n_bags']), n_runs))
        L.append('Привязка отказов детерминирована (по данным и чистому прогону), отказ никогда не ставится на стоянке: '
                 '**cruise** — первый момент после 0.3 длины заезда, когда скорость колёс > 5 м/с не менее 8 с, эталон покрывает '
                 '≥ 90 %% окна и чистая ошибка < %.1f м/с на [t−5, t+длит.+40] с; **brake** — первое начало торможения (ручка ≥ 0 → < 0, '
                 'не −8) при v > 7 м/с (иначе > 5), заканчивающееся остановкой в пределах 30 с; **spin** — до 5 эпизодов (≥ 40 с '
                 'друг от друга) при v > 5 м/с и ручке ≥ 0; **slide** — до 5 эпизодов через 1.5 с после начала торможения при '
                 'v > 5 м/с (в эпизоде v > 4 м/с); **whole** — весь заезд. Моменты — в раскрывающейся таблице в конце раздела.\n' % ERR_OK)
        L.append('Обозначения. Окно скорости = [начало отказа, конец + %.0f с]. «Восст. (эталон)» — время после конца отказа до '
                 '|ошибки скорости| < %.1f м/с в течение %.0f с; «восст. (к чистому)» — то же для |v − v_чистый|, т. е. сколько '
                 'оценка возвращается к своему чистому прогону (не зависит от эталона). Δs — смещение вдоль пути (`s_route`) '
                 'относительно чистого прогона того же заезда (оценка минус оценка): в конце отказа (b) и через 1 с (b+1). '
                 'Δal@+30 — изменение ошибки вдоль пути по эталону через 30 с после конца отказа (с отказом минус без, на карте; '
                 'включает коррекцию по уклонам TRN). Δдрейф — изменение итогового дрейфа, п.п. Значения — медиана / максимум модуля '
                 'по заездам (для многоэпизодных сценариев — по эпизодам).\n' % (POST_S, REC_THR, REC_HOLD))
        # table 1: health
        L.append('### Таблица 1. Работоспособность: падения, конечность, контракт штампов, частота выходов\n')
        L.append('| сценарий | описание | заездов (окон) | без падений | конечные выходы | stamp = вход (мин) | частота в окне, Гц (мед / мин) | покрытие 0.1 с (мед / мин) | макс. скачок выхода, м/с (отказ / чистый) |')
        L.append('|---|---|---|---|---|---|---|---|---|')
        for name, A in agg.items():
            if A['n_bags'] == 0:
                continue
            whole = A['anchor'] == 'whole'
            L.append('| `%s` | %s | %d (%d) | %d/%d | %d/%d | %s | %s | %s | %s |' % (
                name, A['ru'], A['n_bags'], A['n_windows'], A['n_bags'] - A['n_crash'], A['n_bags'], A['n_finite'],
                A['n_bags'] - A['n_crash'], fmt(A['stamps_from_input_min'], 3),
                ('%s (весь заезд)' % fmt(_nz(A, 'out_rate_hz', 'median'), 1)) if whole else
                '%s / %s' % (fmt(_nz(A, 'out_rate_hz', 'median'), 1), fmt(_nz(A, 'out_rate_hz', 'min'), 1)),
                '–' if whole else '%s / %s' % (fmt(_nz(A, 'out_cov01', 'median'), 3), fmt(_nz(A, 'out_cov01', 'min'), 3)),
                '–' if whole else '%s / %s' % (fmt(_nz(A, 'out_step_max', 'max'), 2), fmt(_nz(A, 'out_step_max_clean', 'max'), 2))))
        L.append('')
        L.append('«stamp = вход» — минимальная по заездам доля выходов, чей `header.stamp` совпадает со штампом входного сообщения '
                 '(контракт: выход на каждое входное сообщение с его штампом; сообщение с уже опубликованным штампом повторно не '
                 'публикуется, а передняя и задняя тележки приходят с общим штампом, поэтому пара тележек даёт один выход). '
                 'Частота и покрытие — по выходам внутри окна отказа; в чистых данных ≈ 29–30 Гц = команда 20 Гц + пара тележек '
                 '≈ 9–10 Гц, поэтому без колёс остаётся 20 Гц, а без команды (`cmd_drop_60`) — ≈ 9.7 Гц. '
                 'При `stamp_jump` выходы по контракту несут сдвинутые штампы входов, поэтому часть 0.1-с ячеек окна пустеет. '
                 'В комбинированных сценариях (нет ни колёс, ни команды) входов нет вовсе, и выходов в окне нет по построению; '
                 'после возврата входов оценка продолжается. «Макс. скачок выхода» — наибольшая разность скоростей соседних выходов '
                 'на [начало − 0.5 с, конец + 2 с].\n')
        # table 2: accuracy in the fault window
        L.append('### Таблица 2. Точность в окне отказа и восстановление (сценарии с окном)\n')
        L.append('| сценарий | v_rmse окна, м/с (отказ: мед / макс; чистый: мед) | max \\|ошибка v\\| окна, м/с (отказ: мед / макс; чистый: мед) | восст. (эталон), с | восст. (к чистому), с | \\|Δs\\| в конце, м (b) | \\|Δs\\| b+1 с, м | \\|Δal@+30\\|, м | \\|Δдрейф\\|, п.п. |')
        L.append('|---|---|---|---|---|---|---|---|---|')
        for name, A in agg.items():
            if A['n_bags'] == 0 or A['anchor'] == 'whole':
                continue
            rec = mm(A, 'rec_s', 1)
            if A.get('rec_s_n_inf'):
                rec += ' (%d > %.0f с)' % (A['rec_s_n_inf'], REC_HORIZON)
            rec += ' (чистый %s)' % fmt(_nz(A, 'rec_s_clean', 'median'), 2)
            rvc = mm(A, 'rec_vs_clean', 2)
            if A.get('rec_vs_clean_n_inf'):
                rvc += ' (%d > %.0f с)' % (A['rec_vs_clean_n_inf'], REC_HORIZON)
            L.append('| `%s` | %s; %s | %s; %s | %s | %s | %s | %s | %s | %s |' % (
                name, mm(A, 'v_rmse'), fmt(_nz(A, 'v_rmse_clean', 'median')), mm(A, 'v_max'),
                fmt(_nz(A, 'v_max_clean', 'median')), rec, rvc, mm(A, 'd_s_b', 2), mm(A, 'd_s_b1', 2), mm(A, 'd_al_30', 2),
                mm(A, 'run_abs_d_drift_pct', 3)))
        L.append('')
        # table 3: whole-run scenarios
        L.append('### Таблица 3. Искажения сигнала на весь заезд\n')
        L.append('| сценарий | описание | v_rmse заезда, м/с (отказ / чистый, мед) | max \\|ошибка v\\| заезда, м/с (отказ / чистый: мед; макс) | доля выходов с флагом slip (отказ / чистый, мед; макс) | доля `disagree` (отказ / чистый, мед) | \\|Δs\\| к концу заезда, м (мед / макс) | \\|Δдрейф\\|, п.п. (мед / макс) | \\|Δal_rmse\\|, м (мед / макс) |')
        L.append('|---|---|---|---|---|---|---|---|---|')
        for name, A in agg.items():
            if A['n_bags'] == 0 or A['anchor'] != 'whole':
                continue
            L.append('| `%s` | %s | %s / %s | %s / %s; %s / %s | %s / %s; %s | %s / %s | %s | %s | %s |' % (
                name, A['ru'], fmt(_nz(A, 'run_v_rmse', 'median')), fmt(_nz(A, 'run_v_rmse_clean', 'median')),
                fmt(_nz(A, 'run_v_max', 'median'), 2), fmt(_nz(A, 'run_v_max_clean', 'median'), 2),
                fmt(_nz(A, 'run_v_max', 'max'), 2), fmt(_nz(A, 'run_v_max_clean', 'max'), 2),
                fmt(_nz(A, 'run_slip_frac', 'median'), 4), fmt(_nz(A, 'run_slip_frac_clean', 'median'), 4),
                fmt(_nz(A, 'run_slip_frac', 'max'), 4),
                fmt(_nz(A, 'run_disagree_frac', 'median'), 4), fmt(_nz(A, 'run_disagree_frac_clean', 'median'), 4),
                mm(A, 'run_abs_d_s_end', 2), mm(A, 'run_abs_d_drift_pct', 3), mm(A, 'run_abs_d_al_rmse', 2)))
        L.append('')
        # table 4: vs naive
        L.append('### Таблица 4. Модель против наивной фузии колёс\n')
        L.append('Δs(конец) — смещение вдоль пути к концу отказа относительно чистого прогона: у оценки — разность `s_route` '
                 'через 1 с после конца отказа, у наивной — ∫(v_naive − v_clean)dt по окну отказа. «NaN» — наивная фузия выдаёт '
                 'NaN (доля отсчётов окна в скобках), её ошибка не определена.\n')
        L.append('| сценарий | окон | v_rmse окна, м/с: оценка / наивная (мед) | max \\|ошибка\\|, м/с: оценка / наивная (мед; макс) | \\|Δs(конец)\\|, м: оценка / наивная (мед; макс) |')
        L.append('|---|---|---|---|---|')
        for name, A in agg.items():
            if A['family'] not in ('drop', 'bogie') or A['n_windows'] == 0:
                continue
            if (_nz(A, 'naive_nonfinite', 'max') or 0) > 0:
                nv = 'NaN (%d отсч. на окно, мед)' % int(_nz(A, 'naive_nonfinite', 'median') or 0)
                L.append('| `%s` | %d | %s / %s | %s / %s; %s / %s | %s / %s; %s / %s |' % (
                    name, A['n_windows'], fmt(_nz(A, 'v_rmse', 'median')), nv, fmt(_nz(A, 'v_max', 'median')), 'NaN',
                    fmt(_nz(A, 'v_max', 'max')), 'NaN', fmt(_nz(A, 'd_s_b1', 'median'), 2), 'NaN',
                    fmt(_nz(A, 'd_s_b1', 'max'), 2), 'NaN'))
                continue
            L.append('| `%s` | %d | %s / %s | %s / %s; %s / %s | %s / %s; %s / %s |' % (
                name, A['n_windows'], fmt(_nz(A, 'v_rmse', 'median')), fmt(_nz(A, 'naive_v_rmse', 'median')),
                fmt(_nz(A, 'v_max', 'median')), fmt(_nz(A, 'naive_v_max', 'median')), fmt(_nz(A, 'v_max', 'max')),
                fmt(_nz(A, 'naive_v_max', 'max')),
                fmt(_nz(A, 'd_s_b1', 'median'), 2), fmt(_nz(A, 'naive_d_s_b', 'median'), 2), fmt(_nz(A, 'd_s_b1', 'max'), 2),
                fmt(_nz(A, 'naive_d_s_b', 'max'), 2)))
        L.append('')
        # table 5: return of the inputs
        L.append('### Таблица 5. Возврат входов после пропадания (сценарии `drop`)\n')
        L.append('Δs — `s_route` выхода минус `s_route` чистого прогона в тот же момент, со знаком (медиана; мин…макс). '
                 '«Первый опубликованный» — первый выход со штампом ≥ конца отказа в порядке публикации (что получит потребитель); '
                 '«худший за 1 с» — выход с наибольшим |Δs| среди опубликованных за первую секунду, с его σ_along = √var_along '
                 'и отношением |Δs|/σ_along; b+1 — через 1 с после конца отказа (см. табл. 2).\n')
        L.append('| сценарий | окон | Δs первого опубликованного, м | σ_along первого, м (мед / макс) | худший за 1 с: Δs, м | худший за 1 с: σ_along, м (мед) / \\|Δs\\|/σ (макс) | Δs b+1 с, м | макс. σ_along за 3 с, м (мед / макс) |')
        L.append('|---|---|---|---|---|---|---|---|')
        for name, A in agg.items():
            if A['family'] != 'drop' or A['n_windows'] == 0:
                continue
            f1, w1, b1 = (A.get(k + '_signed') or {} for k in ('first_emit_d_s', 'worst1s_d_s', 'd_s_b1'))
            L.append('| `%s` | %d | %s; %s…%s | %s | %s; %s…%s | %s / %s | %s; %s…%s | %s |' % (
                name, A['n_windows'], fmt(f1.get('median'), 2), fmt(f1.get('min'), 2), fmt(f1.get('max'), 2),
                mm(A, 'first_emit_sd_along', 2), fmt(w1.get('median'), 2), fmt(w1.get('min'), 2), fmt(w1.get('max'), 2),
                fmt(_nz(A, 'worst1s_sd_along', 'median'), 2), fmt(_nz(A, 'worst1s_z', 'max'), 2),
                fmt(b1.get('median'), 2), fmt(b1.get('min'), 2), fmt(b1.get('max'), 2), mm(A, 'max_sd_along_3s', 2)))
        L.append('')
        # table 6: slip flag on injected episodes
        L.append('### Таблица 6. Проскальзывание / юз: флаг на введённых эпизодах\n')
        L.append('Recall — доля введённых эпизодов, для которых флаг `slip` поднят в пределах 0.5 с от начала; «без флага в '
                 'чистом» — то же только по эпизодам, где чистый прогон сам не поднимает флаг на [начало − 0.5, конец + 0.5] с '
                 '(заслуга отказа, а не совпадение); t_flag — время до первого флага (в пределах эпизода + 0.5 с); доля флага — '
                 'доля выходов внутри эпизода с флагом; вне эпизодов — доля выходов с флагом вне эпизодов (+ удержание), с отказом / '
                 'в чистом прогоне.\n')
        L.append('| сценарий | эпизодов | recall ≤ 0.5 с | recall ≤ 0.5 с без флага в чистом (эпизодов) | флаг хоть раз | t_flag, с (мед / макс) | доля флага в эпизоде (мед) | флаг вне эпизодов: отказ / чистый |')
        L.append('|---|---|---|---|---|---|---|---|')
        for name, A in agg.items():
            if A['family'] != 'slip' or A['n_windows'] == 0:
                continue
            L.append('| `%s` | %d | %s | %s (%d) | %d/%d | %s | %s | %s / %s |' % (
                name, A['n_windows'], fmt(A.get('recall_0p5'), 3), fmt(A.get('recall_0p5_noclean'), 3),
                A.get('n_windows_noclean', 0), A.get('n_flagged_any', 0), A['n_windows'], mm(A, 'ttf', 2),
                fmt(_nz(A, 'frac_in', 'median'), 2), fmt(_nz(A, 'run_slip_frac_out', 'median'), 4),
                fmt(_nz(A, 'run_slip_frac_out_clean', 'median'), 4)))
        L.append('')
        L.append('### Таблица 7. Проскальзывание / юз: ошибка скорости и пути\n')
        L.append('Ошибка скорости — на [начало, конец + 1 с] по эталону; Δs эпизода — приращение смещения вдоль пути за '
                 '[начало − 1 с, конец + 5 с] относительно чистого прогона (у наивной — ∫(v_naive − v_clean)dt).\n')
        L.append('| сценарий | v_rmse эпизода, м/с: оценка / наивная / чистый (мед) | max \\|ошибка\\|, м/с: оценка / наивная (мед; макс) | \\|Δs эпизода\\|, м: оценка / наивная (мед; макс) | восст. (к чистому), с (мед / макс) |')
        L.append('|---|---|---|---|---|')
        for name, A in agg.items():
            if A['family'] != 'slip' or A['n_windows'] == 0:
                continue
            L.append('| `%s` | %s / %s / %s | %s / %s; %s / %s | %s / %s; %s / %s | %s |' % (
                name, fmt(_nz(A, 'ep_rmse', 'median')), fmt(_nz(A, 'ep_rmse_naive', 'median')),
                fmt(_nz(A, 'ep_rmse_clean', 'median')), fmt(_nz(A, 'ep_max', 'median')), fmt(_nz(A, 'ep_max_naive', 'median')),
                fmt(_nz(A, 'ep_max', 'max')), fmt(_nz(A, 'ep_max_naive', 'max')),
                fmt(_nz(A, 'ep_d_s', 'median'), 2), fmt(_nz(A, 'ep_d_s_naive', 'median'), 2), fmt(_nz(A, 'ep_d_s', 'max'), 2),
                fmt(_nz(A, 'ep_d_s_naive', 'max'), 2), mm(A, 'rec_vs_clean', 2)))
        L.append('')
        L.append('### Таблица 7a. Адаптивные коэффициенты модели под отказом (весь заезд)\n')
        L.append('Минимум и максимум `gain_tr` / `gain_br` за заезд (медиана по заездам), с отказом / в чистом прогоне того же заезда; '
                 '«у границы» — число заездов, где хоть один коэффициент дошёл до 0.8 (или 1.3).\n')
        L.append('| сценарий | gain_tr мин: отказ / чистый | gain_br мин: отказ / чистый | gain_tr макс: отказ / чистый | gain_br макс: отказ / чистый | у границы 0.8: отказ / чистый (заездов) | у границы 1.3: отказ / чистый |')
        L.append('|---|---|---|---|---|---|---|')
        for name, A in agg.items():
            if not A['n_bags']:
                continue
            L.append('| `%s` | %s / %s | %s / %s | %s / %s | %s / %s | %d / %d (%d) | %d / %d |' % (
                name, fmt(_nz(A, 'run_gtr_min', 'median')), fmt(_nz(A, 'run_gtr_min_clean', 'median')),
                fmt(_nz(A, 'run_gbr_min', 'median')), fmt(_nz(A, 'run_gbr_min_clean', 'median')),
                fmt(_nz(A, 'run_gtr_max', 'median')), fmt(_nz(A, 'run_gtr_max_clean', 'median')),
                fmt(_nz(A, 'run_gbr_max', 'median')), fmt(_nz(A, 'run_gbr_max_clean', 'median')),
                A.get('n_gain_lo', 0), A.get('n_gain_lo_clean', 0), A['n_bags'] - A['n_crash'],
                A.get('n_gain_hi', 0), A.get('n_gain_hi_clean', 0)))
        L.append('')
        # anchors
        L.append('<details><summary>Моменты отказов по заездам (с от первого колёсного штампа)</summary>\n')
        L.append('| заезд | направление | cruise | brake | spin | slide |')
        L.append('|---|---|---|---|---|---|')
        for b, a in F['anchors'].items():
            a = a or {}
            L.append('| `%s` | %s | %s | %s | %s | %s |' % (b, F['dirs'].get(b, ''), fmt(a.get('cruise'), 1), fmt(a.get('brake'), 1),
                                                         ', '.join('%.0f' % x for x in a.get('spin') or []),
                                                         ', '.join('%.1f' % x for x in a.get('slide') or [])))
        L.append('\n</details>\n')
    if 'clean' in S:
        C = S['clean']
        L.append('## 2. Флаг проскальзывания на чистых прогонах (%d заездов eval_ok, %.1f ч)\n' % (C['n_bags'], C['hours']))
        L.append('Эпизод флага «подтверждён», если наивное среднее тележек отличается от скорости GNSS больше чем на %.1f м/с внутри '
                 'эпизода (±0.5 с) — т. е. колёса действительно врали; «не подтверждён» — верхняя оценка ложных срабатываний '
                 '(в него попадают и настоящие малые проскальзывания < %.1f м/с). Для подтверждения используется эталон, очищенный '
                 'как `evaluate.py --clean-ref` (сбои штампов, «замёрзшая» скорость GNSS, скачки позиции) и с исправленными '
                 'длительными сдвигами часов GNSS: серия штампов, отличающихся от времени записи больше чем на %.1f с против '
                 'медианы заезда, длиной ≥ %.0f с при обычной частоте получает штамп «время записи + медианный сдвиг» (всплески '
                 'в начале записи и после задержки доставки — это запоздавшее время записи при верном штампе — не трогаются). '
                 'Иначе «замёрзшая» или сдвинутая на 1 с скорость GNSS сама «подтверждает» эпизод. Эпизод, в пределах ±1 с '
                 'от которого есть сбой часов, считается отдельно: сбой штампов входа (колёса или команда: '
                 '`evaluate.stamp_glitch_mask` или штамп впереди времени записи > 0.3 с) — выходы там по контракту несут '
                 'сдвинутые штампы; начало или конец длительного сдвига часов GNSS — эталон меняет шкалу времени, а поток '
                 'колёс в записях в эти же моменты скачет (пропуск ≈ 1 с содержимого или скачок штампа). В обоих случаях '
                 'сравнение с эталоном по штампу теряет смысл.\n'
                 % (SLIP_CONFIRM, SLIP_CONFIRM, RESTAMP_THR, RESTAMP_MIN_S))
        L.append('### Таблица 8. Ложные срабатывания флага `slip`\n')
        L.append('| показатель | значение |')
        L.append('|---|---|')
        L.append('| падений / заездов | %d / %d |' % (C['n_crash'], C['n_bags'] + C['n_crash']))
        L.append('| доля выходов с флагом (все заезды вместе) | %s |' % fmt(C['slip_frac_pooled'], 5))
        L.append('| доля выходов с флагом по заездам (медиана / p90 / макс) | %s / %s / %s |' % (
            fmt(C['slip_frac']['median'], 5), fmt(C['slip_frac']['p90'], 5), fmt(C['slip_frac']['max'], 5)))
        L.append('| то же в движении v > 1 м/с (медиана / макс) | %s / %s |' % (fmt(C['slip_frac_moving']['median'], 5),
                                                                          fmt(C['slip_frac_moving']['max'], 5)))
        L.append('| эпизодов флага всего (в час) | %d (%s/ч) |' % (C['slip_episodes'], fmt(C['slip_episodes_per_h'], 2)))
        L.append('| заездов с хотя бы одним эпизодом | %d из %d |' % (C['bags_with_flag'], C['n_bags']))
        L.append('| подтверждены эталоном / не подтверждены / в окне сбоя часов / нет эталона | %d / %d / %d / %d |' % (
            C['slip_confirmed'], C['slip_unconfirmed'], C.get('slip_stamp', 0), C['slip_noref']))
        L.append('| суммарное время флага, с (из них не подтверждено) | %s (%s) |' % (fmt(C['slip_time_s'], 1), fmt(C['unconfirmed_time_s'], 1)))
        L.append('| доля времени неподтверждённого флага (FP rate по времени) | %s |' % fmt(C['unconfirmed_frac_pooled'], 6))
        L.append('| неподтверждённых эпизодов в час (FP rate по эпизодам) | %s |' % fmt(
            C['slip_unconfirmed'] / C['hours'] if C['hours'] else None, 2))
        if 'gnss_offset_runs' in C:
            L.append('| заездов с длительным сдвигом часов GNSS (серии: начало–конец по времени записи, с; сдвиг, с) | %d: %s |' % (
                len(C['gnss_offset_runs']), '; '.join('`%s` %s' % (b, ', '.join('%.0f–%.0f (%+.2f)' % tuple(x) for x in rr))
                                                     for b, rr in sorted(C['gnss_offset_runs'].items()))))
        L.append('')
        L.append('### Таблица 9. Сцепление и адаптация модели (чистые прогоны, %d заездов)\n' % C['n_bags'])
        L.append('`adhesion_used` = |a|/g, где a — ускорение модели процесса EKF на последнем шаге прогноза (тяга/торможение по '
                 'ручке контроллера × `gain_tr`/`gain_br` + уклон и кривизна по карте + `bias`; то же `a`, что в `Output.a`), '
                 'т. е. доля сцепления, которую модель считает использованной; учитываются выходы при v > 1 м/с. `gain_tr`, `gain_br` — '
                 'онлайн-коэффициенты тяги и тормоза (состояния EKF, случайное блуждание, обновление только при согласии с колёсами '
                 'в пределах 3σ, границы [0.8, 1.3], старт 1.0); `bias` — смещение ускорения модели (Гаусс–Марков). Значения по '
                 'заездам: медиана / p90 / мин / макс.\n')
        L.append('| величина | медиана | p90 | мин | макс |')
        L.append('|---|---|---|---|---|')
        for k, ru in (('adh_p50', 'adhesion_used, медиана по заезду'), ('adh_p95', 'adhesion_used, p95 по заезду'),
                      ('adh_p99', 'adhesion_used, p99 по заезду'),
                      ('adh_max', 'adhesion_used, максимум по заезду'), ('adh_frac_gt_0p15', 'доля времени с adhesion_used > 0.15'),
                      ('adh_max_slip', 'adhesion_used, максимум под флагом slip'),
                      ('adhk_p99', 'для сравнения: \\|dv/dt\\|/g выходной скорости (1 с), p99 по заезду'),
                      ('adhk_max', 'для сравнения: \\|dv/dt\\|/g выходной скорости (1 с), максимум по заезду'),
                      ('a_diff_p95', '\\|a модели − dv/dt\\|, м/с², p95 по заезду'),
                      ('a_sign_wrong', 'доля времени, когда знак a модели противоположен dv/dt (\\|dv/dt\\| > 0.3 м/с²)'),
                      ('gtr_final', 'gain_tr в конце заезда'), ('gtr_min', 'gain_tr, минимум'), ('gtr_max', 'gain_tr, максимум'),
                      ('gtr_range', 'gain_tr, размах за заезд'),
                      ('gbr_final', 'gain_br в конце заезда'), ('gbr_min', 'gain_br, минимум'), ('gbr_max', 'gain_br, максимум'),
                      ('gbr_range', 'gain_br, размах за заезд'),
                      ('bias_min', 'bias, минимум, м/с²'), ('bias_max', 'bias, максимум, м/с²')):
            a = C.get(k) or {}
            L.append('| %s | %s | %s | %s | %s |' % (ru, fmt(a.get('median'), 3), fmt(a.get('p90'), 3), fmt(a.get('min'), 3),
                                                      fmt(a.get('max'), 3)))
        L.append('')
        if C.get('gain_bound_hits'):
            gb = C['gain_bound_hits']
            L.append('Заездов, где коэффициент хоть раз упирался в границу: gain_tr ≤ 0.801 — %d, ≥ 1.299 — %d; gain_br ≤ 0.801 — %d, '
                     '≥ 1.299 — %d.\n' % (gb['gtr_lo'], gb['gtr_hi'], gb['gbr_lo'], gb['gbr_hi']))
        L.append('Доли состояний тележек (среднее по заездам): %s.\n' % ', '.join(
            '%s %s' % (s, fmt(f, 5)) for s, f in sorted(C['state_frac_mean'].items(), key=lambda kv: -kv[1])))
    if 'real' in S:
        L.append('## 3. Реальные аномальные заезды (без инъекции)\n')
        L.append('Три варианта эталона скорости: «как есть» — RTK master без очистки (как во всех остальных таблицах); «очищ.» — '
                 '`evaluate.py --clean-ref` (сбои штампов, «замёрзшая» скорость GNSS, скачки позиции); «испр.» — очищенный эталон, '
                 'у которого вдобавок исправлены длительные сдвиги часов GNSS: серия штампов, отличающихся от времени записи '
                 'больше чем на %.1f с против медианы заезда, длиной ≥ %.0f с получает штамп «время записи + медианный сдвиг» '
                 '(`--clean-ref` сравнивает с бегущей медианой ≈ 5 с и сдвиг длиной в минуту не видит). В заездах без RTK (`rtk_frac` = 0) эталон скорости — '
                 'доплеровская скорость GNSS master при любом статусе решения (с тем же исправлением штампов), позиционного '
                 'эталона там нет. Разница между вариантами — артефакты эталона, а не ошибка оценки. Оценщик во всех случаях '
                 'один и тот же (GNSS — только первые 10 с).\n' % (RESTAMP_THR, RESTAMP_MIN_S))
        L.append('### Таблица 10. Точность\n')
        L.append('| заезд | категория | длит., с | падение/NaN | эталон скорости | v_rmse, м/с: как есть / очищ. / испр. | max \\|ошибка v\\|, м/с: как есть (t, с) / очищ. / испр. (t, с) | отсчётов \\|ошибка v\\| > 0.5 / > 1 м/с: как есть → испр. (из) | в `single_*`/`disagree`: v_rmse / max, м/с (отсчётов) | al_rmse, м: как есть / испр. | дрейф, % |')
        L.append('|---|---|---|---|---|---|---|---|---|---|---|')
        kind_ru = {'rtk': 'RTK', 'doppler': 'доплер GNSS (без RTK)', None: 'нет GNSS'}
        for r in S['real']:
            if r.get('status') != 'ok':
                L.append('| `%s` | %s | – | %s | | | | | | | |' % (r['bag'], r.get('category'), r.get('error')))
                continue
            m = r.get('metrics') or {}
            mc = r.get('metrics_cleanref') or {}
            mf = r.get('metrics_fixref') or {}
            sp = r.get('speed_fix') or {}
            deg = [(x['n'], x['v_rmse'], x['v_max']) for s_, x in (sp.get('by_state') or {}).items()
                   if s_ != 'both' and x.get('n')]
            nd = sum(x[0] for x in deg)
            degs = ('%s / %s (%d)' % (fmt(np.sqrt(sum(n_ * r_ ** 2 for n_, r_, _ in deg) / nd)), fmt(max(x[2] for x in deg), 2), nd)
                    if nd else '–')
            L.append('| `%s` | %s | %.0f | %s | %s | %s / %s / %s | %s (%s) / %s / %s (%s) | %s / %s → %s / %s (%s) | %s | %s / %s | %s |' % (
                r['bag'], r['category'], r['duration_s'],
                'нет' if (r['n_nonfinite_v'] == 0 and r['n_nonfinite_pose'] == 0) else 'ЕСТЬ', kind_ru.get(r.get('ref_kind'), '?'),
                fmt(m.get('v_rmse')), fmt(mc.get('v_rmse')), fmt(sp.get('v_rmse')),
                fmt(m.get('v_max'), 2), fmt(r.get('v_max_err_at_rel'), 1), fmt(mc.get('v_max'), 2),
                fmt(sp.get('v_max'), 2), fmt(sp.get('v_max_at_rel'), 1),
                fmt(r.get('n_err_gt0p5')), fmt(r.get('n_err_gt1')), fmt(sp.get('n_gt0p5')), fmt(sp.get('n_gt1')), fmt(sp.get('n')),
                degs, fmt(m.get('al_rmse'), 2), fmt(mf.get('al_rmse'), 2), fmt(m.get('drift_pct'), 2)))
        L.append('')
        L.append('### Таблица 11. Флаг, состояния тележек, штампы\n')
        L.append('Эпизоды флага подтверждаются по эталону «испр.» (или по доплеровской скорости без RTK), как в разд. 2.\n')
        L.append('| заезд | категория | эпизодов slip (подтв. / не подтв. / сбой часов / без эталона) | время slip, с | доли состояний тележек | сбои штампов входа (front / rear / cmd) | длительный сдвиг часов GNSS: начало–конец, с (сдвиг, с) |')
        L.append('|---|---|---|---|---|---|---|')
        for r in S['real']:
            if r.get('status') != 'ok':
                continue
            sf = ', '.join('%s %s' % (s, fmt(f, 4)) for s, f in sorted(r['state_frac'].items(), key=lambda kv: -kv[1]))
            sg = r.get('stamp_glitches') or {}
            sgs = ' / '.join(str(sg.get(k, {}).get('n_glitch', '–')) for k in ('front', 'rear', 'cmd'))
            gr = r.get('gnss_offset_runs') or []
            grs = (', '.join('%.0f–%.0f (%+.2f)' % tuple(x) for x in gr) if gr else 'нет') if r.get('gnss_restamped') else 'нет GNSS'
            L.append('| `%s` | %s | %d (%d / %d / %d / %d) | %s | %s | %s | %s |' % (
                r['bag'], r['category'], r['slip_episodes_n'], r['slip_confirmed_n'], r['slip_unconfirmed_n'],
                r.get('slip_stamp_n', 0), r['slip_noref_n'], fmt(r['slip_time_s'], 1), sf, sgs, grs))
        L.append('')
        L.append('### Таблица 12. Эпизоды флага `slip` на реальных заездах\n')
        L.append('Скорости в м/с: оценка (до эпизода; мин–макс в эпизоде), сырые тележки мин–макс, эталон мин–макс (±0.5 с; «испр.» '
                 'RTK или доплер без RTK), максимальная ошибка оценки и наивного среднего тележек против него.\n')
        L.append('| заезд | начало, с | длит., с | v оценки до | v оценки | передняя (сырая) | задняя (сырая) | эталон | max ошибка: оценка / наивная | подтверждён |')
        L.append('|---|---|---|---|---|---|---|---|---|---|')
        for r in S['real']:
            for e in (r.get('slip_episodes') or [])[:25]:
                L.append('| `%s` | %.1f | %.2f | %s | %s–%s | %s–%s | %s–%s | %s–%s | %s / %s | %s |' % (
                    r['bag'], e['start_rel'], e['dur'], fmt(e.get('v_est_pre'), 2), fmt(e.get('v_est_min'), 2),
                    fmt(e.get('v_est_max'), 2), fmt(e.get('raw_front_min'), 2), fmt(e.get('raw_front_max'), 2),
                    fmt(e.get('raw_rear_min'), 2), fmt(e.get('raw_rear_max'), 2), fmt(e.get('v_ref_min'), 2),
                    fmt(e.get('v_ref_max'), 2), fmt(e.get('est_max_err'), 2), fmt(e.get('naive_max_err'), 2),
                    clock_glitch_label(e) if clock_glitch(e) else
                    (fmt(e.get('confirmed')) + (' (доплер)' if e.get('ref_kind') == 'doppler' else ''))
                    if 'confirmed' in e else 'нет эталона'))
            if len(r.get('slip_episodes') or []) > 25:
                L.append('| `%s` | … ещё %d эпизодов в summary.json | | | | | | | | |' % (r['bag'], len(r['slip_episodes']) - 25))
        L.append('')
        L.append('### Таблица 13. Эпизоды деградации тележек (не `both`, ≥ 0.5 с; `disagree` — все; до 12 самых длинных на заезд)\n')
        L.append('| заезд | состояние | начало, с | длит., с |')
        L.append('|---|---|---|---|')
        for r in S['real']:
            eps = r.get('state_episodes') or []
            eps = sorted(eps, key=lambda e: -e['dur'])[:12]
            for e in sorted(eps, key=lambda e: e['start_rel']):
                L.append('| `%s` | %s | %.1f | %.2f |' % (r['bag'], e['state'], max(e['start_rel'], 0.0), e['dur']))
        L.append('')
    if S.get('findings'):
        L.append('## 4. Наблюдения\n')
        for f in S['findings']:
            L.append('- ' + f)
        L.append('')
    if S.get('figures'):
        L.append('## 5. Рисунки\n')
        for f, cap in S['figures']:
            L.append('- `%s` — %s' % (os.path.relpath(f, ev.ROOT), cap))
        L.append('')
    with open(path, 'w') as fh:
        fh.write('\n'.join(L))


# ------------------------------------------------------------------ plots
def _plot_setup():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.3, 'legend.fontsize': 8})
    return plt


C_REF, C_EST, C_CLEAN, C_NAIVE, C_F, C_R, C_FLAG = '#1f77b4', '#d62728', '#7f7f7f', '#ff7f0e', '#2ca02c', '#9467bd', '#e377c2'


def _series_run(ctx, specs, seed):
    Df = apply_faults(ctx.D, specs, seed) if specs else ctx.D
    est = run_bag(ctx.bag, params=copy.copy(ctx.p), data=Df, gnss_limit_s=ctx.gnss_limit)
    return Df, est


def _gapped(t, *ys, gap=0.5):
    """Insert NaN rows where consecutive stamps are more than `gap` apart (no line across missing outputs)."""
    t = np.asarray(t, float)
    if len(t) < 2:
        return (t,) + tuple(np.asarray(y, float) for y in ys)
    k = np.where(np.diff(t) > gap)[0] + 1
    tt = np.insert(t, k, np.nan)
    return (tt,) + tuple(np.insert(np.asarray(y, float), k, np.nan) for y in ys)


def _raw(D, topic, K):
    t = np.asarray(D[topic]['t_hdr'], float)
    o = np.argsort(t, kind='stable')
    return t[o], np.asarray(D[topic]['v'], float)[o] / K


def _ref_speed(ctx):
    return ctx.ref['tv'], ctx.ref['v']


def plot_drop(ctx_list, path, seed):
    """Columns: (ctx, scenario name). Speed est vs ref vs naive; along-track displacement vs clean."""
    plt = _plot_setup()
    n = len(ctx_list)
    fig, axs = plt.subplots(2, n, figsize=(5.2 * n, 7.2), sharex='col', squeeze=False,
                            gridspec_kw=dict(height_ratios=[2, 1]))
    for c, (ctx, name) in enumerate(ctx_list):
        sc = SCEN[name]
        anc = ctx.anchors()
        specs, wins = build_specs(sc, anc)
        Df, est = _series_run(ctx, specs, seed_for(ctx.bag, name, seed))
        a, b = ctx.base + wins[0][0], ctx.base + wins[0][1]
        lo, hi = a - 15, b + 45
        o = sorted_est(est)
        t, v, s = est['t'][o], est['v'][o], est['diag_s_route'][o]
        tv, vr = _ref_speed(ctx)
        m = (tv >= lo) & (tv <= hi)
        ax = axs[0, c]
        ax.axvspan(0, b - a, color='0.85', label='отказ', zorder=0)
        ax.plot(tv[m] - a, vr[m], color=C_REF, lw=2.2, label='эталон GNSS')
        mc = (ctx.ct >= lo) & (ctx.ct <= hi)
        ax.plot(ctx.ct[mc] - a, ctx.cv[mc], color=C_CLEAN, lw=1, ls='--', label='оценка без отказа')
        mf = (t >= lo) & (t <= hi)
        tg, vg = _gapped(t[mf] - a, v[mf])
        ax.plot(tg, vg, color=C_EST, lw=1.4, label='оценка с отказом')
        # outputs isolated by a gap of the inputs (e.g. the first one after a total silence)
        iso = np.where(np.isnan(tg))[0]
        if len(iso):
            ax.plot(tg[iso + 1], vg[iso + 1], 'o', ms=4, mfc='none', color=C_EST)
        tq = ctx.ct[mc]
        vn = naive_speed(Df, ctx.K, tq)
        ax.plot(tq - a, vn, color=C_NAIVE, lw=1.2, ls=':', label='наивная (ZOH колёс)')
        ax.set_title('%s: %s\n%s' % (ctx.bag, name, sc['ru']), fontsize=9)
        ax.set_ylabel('скорость, м/с')
        ax = axs[1, c]
        sc_ = value_at(ctx.ct, ctx.cs, t[mf])
        # publication order around the end of the fault: the first published output may carry a stale state
        tg, dg = _gapped(t[mf] - a, s[mf] - sc_)
        ax.plot(tg, dg, color=C_EST, label='оценка: s_route − s_route(чистый)')
        te, se = np.asarray(est['t'], float), np.asarray(est['diag_s_route'], float)
        me = np.where((te >= b) & (te <= b + 1.0))[0]
        if len(me):
            k0 = me[0]
            d0 = se[k0] - value_at(ctx.ct, ctx.cs, te[k0])
            ax.plot([te[k0] - a], [d0], 'x', ms=7, mew=2, color=C_EST, label='первый опубликованный после отказа')
        mn = (ctx.ct >= a) & (ctx.ct <= hi)
        tn = ctx.ct[mn]
        dn = naive_speed(Df, ctx.K, tn) - ctx.cv[mn]
        dn = np.where(tn <= b, dn, 0.0)
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (dn[1:] + dn[:-1]) * np.diff(tn))])
        ax.plot(tn - a, cum, color=C_NAIVE, ls=':', label='наивная: ∫(v_naive − v_clean)dt')
        ax.axvspan(0, b - a, color='0.85', zorder=0)
        ax.axhline(0, color='k', lw=0.6)
        ax.set_xlabel('время от начала отказа, с')
        ax.set_ylabel('смещение вдоль пути, м')
    # one legend for the figure (the same entries in every column), below the panels, so that no
    # marker or curve is hidden behind a legend box
    hs, ls = [], []
    for ax in axs.ravel():
        for h, l in zip(*ax.get_legend_handles_labels()):
            if l not in ls:
                hs.append(h)
                ls.append(l)
    fig.legend(hs, ls, loc='lower center', ncol=4, fontsize=9, frameon=False)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_slip(ctx_list, path, seed):
    plt = _plot_setup()
    n = len(ctx_list)
    fig, axs = plt.subplots(2, n, figsize=(5.2 * n, 5.8), sharex='col', squeeze=False,
                            gridspec_kw=dict(height_ratios=[3, 1]))
    for c, (ctx, name, k_ep) in enumerate(ctx_list):
        sc = SCEN[name]
        specs, wins = build_specs(sc, ctx.anchors())
        Df, est = _series_run(ctx, specs, seed_for(ctx.bag, name, seed))
        k_ep = min(k_ep, len(wins) - 1)
        a, b = ctx.base + wins[k_ep][0], ctx.base + wins[k_ep][1]
        lo, hi = a - 4, b + 6
        o = sorted_est(est)
        t, v, sl = est['t'][o], est['v'][o], est['slip'][o]
        ax = axs[0, c]
        ax.axvspan(0, b - a, color='0.88', label='введённый эпизод', zorder=0)
        for tp, col, lab in ((FRONT, C_F, 'передняя (сырая)'), (REAR, C_R, 'задняя (сырая)')):
            tt, vv = _raw(Df, tp, ctx.K)
            m = (tt >= lo) & (tt <= hi)
            ax.step(tt[m] - a, vv[m], where='post', color=col, lw=1, label=lab)
        tv, vr = _ref_speed(ctx)
        m = (tv >= lo) & (tv <= hi)
        ax.plot(tv[m] - a, vr[m], color=C_REF, lw=2.2, alpha=0.8, label='эталон GNSS')
        m = (t >= lo) & (t <= hi)
        ax.plot(t[m] - a, v[m], color=C_EST, lw=1.5, label='оценка')
        vn = naive_speed(Df, ctx.K, t[m])
        ax.plot(t[m] - a, vn, color=C_NAIVE, lw=1, ls=':', label='наивное среднее')
        ax.set_title('%s: %s\n%s' % (ctx.bag, name, sc['ru']), fontsize=9)
        ax.set_ylabel('скорость, м/с')
        ax.legend(loc='best', fontsize=7)
        ax = axs[1, c]
        ax.fill_between(t[m] - a, 0, sl[m].astype(float), step='post', color=C_FLAG, alpha=0.7, label='флаг slip')
        ax.axvspan(0, b - a, color='0.88', zorder=0)
        ax.set_ylim(-0.1, 1.1)
        ax.set_yticks([0, 1])
        ax.set_ylabel('slip')
        ax.set_xlabel('время от начала эпизода, с')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


STATE_CODE = {'both': 0, 'disagree': 1, 'single_front': 2, 'single_rear': 3, 'model_only': 4}


def plot_single(ctx_list, path, seed):
    plt = _plot_setup()
    n = len(ctx_list)
    fig, axs = plt.subplots(2, n, figsize=(5.2 * n, 5.8), sharex='col', squeeze=False,
                            gridspec_kw=dict(height_ratios=[3, 1.2]))
    for c, (ctx, name) in enumerate(ctx_list):
        sc = SCEN[name]
        specs, wins = build_specs(sc, ctx.anchors())
        Df, est = _series_run(ctx, specs, seed_for(ctx.bag, name, seed))
        a, b = ctx.base + wins[0][0], ctx.base + wins[0][1]
        lo, hi = a - 10, b + 15
        o = sorted_est(est)
        t, v = est['t'][o], est['v'][o]
        st = mode_state(est['mode'][o])
        ax = axs[0, c]
        ax.axvspan(0, b - a, color='0.88', label='отказ', zorder=0)
        for tp, col, lab in ((FRONT, C_F, 'передняя (сырая)'), (REAR, C_R, 'задняя (сырая)')):
            tt, vv = _raw(Df, tp, ctx.K)
            m = (tt >= lo) & (tt <= hi)
            ax.plot(tt[m] - a, vv[m], '.', ms=2, color=col, label=lab)
        tv, vr = _ref_speed(ctx)
        m = (tv >= lo) & (tv <= hi)
        ax.plot(tv[m] - a, vr[m], color=C_REF, lw=2.2, alpha=0.8, label='эталон GNSS')
        m = (t >= lo) & (t <= hi)
        ax.plot(t[m] - a, v[m], color=C_EST, lw=1.3, label='оценка')
        ax.set_title('%s: %s\n%s' % (ctx.bag, name, sc['ru']), fontsize=9)
        ax.set_ylabel('скорость, м/с')
        ax.legend(loc='best', fontsize=7)
        ax = axs[1, c]
        code = np.array([STATE_CODE.get(s, 5) for s in st[m]])
        ax.plot(t[m] - a, code, drawstyle='steps-post', color='k', lw=1)
        ax.axvspan(0, b - a, color='0.88', zorder=0)
        ax.set_yticks(list(STATE_CODE.values()))
        ax.set_yticklabels(list(STATE_CODE.keys()), fontsize=7)
        ax.set_ylim(-0.5, 4.5)
        ax.set_xlabel('время от начала отказа, с')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_stamp(ctx, names, path, seed):
    plt = _plot_setup()
    n = len(names)
    fig, axs = plt.subplots(3, n, figsize=(5.4 * n, 7.2), sharex='col', squeeze=False,
                            gridspec_kw=dict(height_ratios=[1, 2, 1.3]))
    for c, name in enumerate(names):
        sc = SCEN[name]
        specs, wins = build_specs(sc, ctx.anchors())
        Df, est = _series_run(ctx, specs, seed_for(ctx.bag, name, seed))
        a, b = ctx.base + wins[0][0], ctx.base + wins[0][1]
        lo, hi = a - 5, b + 8
        ax = axs[0, c]
        for tp, col, lab in ((FRONT, C_F, 'передняя'), (CMD, 'k', 'команда')):
            th = np.asarray(Df[tp]['t_hdr'], float)
            tr = np.asarray(Df[tp]['t_rec'], float)
            m = (tr >= lo) & (tr <= hi)
            ax.plot(tr[m] - a, th[m] - tr[m], '.', ms=2, color=col, label=lab)
        ax.axvspan(0, b - a, color='0.88', zorder=0)
        ax.set_ylabel('header − запись, с')
        ax.set_title('%s: %s\n%s' % (ctx.bag, name, sc['ru']), fontsize=9)
        ax.legend(loc='best', fontsize=7)
        o = sorted_est(est)
        t, v = est['t'][o], est['v'][o]
        ax = axs[1, c]
        tv, vr = _ref_speed(ctx)
        m = (tv >= lo) & (tv <= hi)
        ax.plot(tv[m] - a, vr[m], color=C_REF, lw=2.2, alpha=0.8, label='эталон GNSS (по штампу)')
        m = (t >= lo) & (t <= hi)
        ax.plot(t[m] - a, v[m], '.', ms=2.5, color=C_EST, label='выход (по своему header.stamp)')
        mc = (ctx.ct >= lo) & (ctx.ct <= hi)
        ax.plot(ctx.ct[mc] - a, ctx.cv[mc], color=C_CLEAN, lw=1, ls='--', label='выход без отказа')
        ax.axvspan(0, b - a, color='0.88', zorder=0)
        ax.set_ylabel('скорость, м/с')
        ax.legend(loc='best', fontsize=7)
        ax = axs[2, c]
        R = ev.evaluate(ctx.bag, est, series=True, D=ctx.D, ref=ctx.ref)
        S = R['series']
        m = (S['tv'] >= lo) & (S['tv'] <= hi)
        ax.plot(S['tv'][m] - a, S['v_err'][m], color=C_EST, lw=1, label='с отказом')
        Sc = ctx.R['series']
        ax.plot(Sc['tv'][m] - a, Sc['v_err'][m], color=C_CLEAN, lw=1, ls='--', label='без отказа')
        ax.axvspan(0, b - a, color='0.88', zorder=0)
        ax.set_ylabel('ошибка v, м/с')
        ax.set_xlabel('время от начала сдвига штампов (по штампу), с')
        ax.legend(loc='best', fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_real(items, path):
    """items: (ctx, episode start rel) -> speeds around a real flag episode."""
    plt = _plot_setup()
    n = len(items)
    fig, axs = plt.subplots(2, n, figsize=(5.0 * n, 5.8), sharex='col', squeeze=False,
                            gridspec_kw=dict(height_ratios=[3, 1]))
    for c, (ctx, t_rel, dur, label) in enumerate(items):
        a = ctx.base + t_rel
        lo, hi = a - 4, a + max(dur, 1.0) + 6
        est = ctx.est
        o = sorted_est(est)
        t, v, sl = est['t'][o], est['v'][o], est['slip'][o]
        ax = axs[0, c]
        for tp, col, lab in ((FRONT, C_F, 'передняя (сырая)'), (REAR, C_R, 'задняя (сырая)')):
            tt, vv = _raw(ctx.D, tp, ctx.K)
            m = (tt >= lo) & (tt <= hi)
            ax.step(tt[m] - a, vv[m], where='post', color=col, lw=1, label=lab)
        if ctx.ref is not None:
            tv, vr = _ref_speed(ctx)
            m = (tv >= lo) & (tv <= hi)
            ax.plot(tv[m] - a, vr[m], color=C_REF, lw=2.2, alpha=0.8, label='эталон GNSS')
        m = (t >= lo) & (t <= hi)
        ax.plot(t[m] - a, v[m], color=C_EST, lw=1.5, label='оценка')
        ax.plot(t[m] - a, naive_speed(ctx.D, ctx.K, t[m]), color=C_NAIVE, lw=1, ls=':', label='наивное среднее')
        ax.set_title('%s @ %.1f с: %s' % (ctx.bag, t_rel, label), fontsize=9)
        ax.set_ylabel('скорость, м/с')
        ax.legend(loc='best', fontsize=7)
        ax = axs[1, c]
        ax.fill_between(t[m] - a, 0, sl[m].astype(float), step='post', color=C_FLAG, alpha=0.7)
        ax.set_ylim(-0.1, 1.1)
        ax.set_yticks([0, 1])
        ax.set_ylabel('slip')
        ax.set_xlabel('время от начала эпизода флага, с')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_overview(S, path):
    plt = _plot_setup()
    agg = S['faults']['agg']
    names = [n for n, A in agg.items() if A['anchor'] != 'whole' and A['n_windows'] > 0]
    fig, axs = plt.subplots(1, 2, figsize=(12, 0.34 * len(names) + 1.6), sharey=True)
    y = np.arange(len(names))
    for ax, (k, kn, lab) in zip(axs, (('v_max', 'naive_v_max', 'max |ошибка скорости| в окне, м/с'),
                                      ('d_s_b1', 'naive_d_s_b', '|Δs| к концу отказа (оценка: +1 с), м'))):
        med = [agg[n][k]['median'] or np.nan for n in names]
        mx = [agg[n][k]['max'] or np.nan for n in names]
        medn = [(agg[n].get(kn) or {}).get('median') or np.nan for n in names]
        ax.barh(y + 0.2, med, height=0.38, color=C_EST, label='оценка (медиана)')
        ax.plot(mx, y + 0.2, '|', color='k', ms=8, label='оценка (максимум)')
        ax.barh(y - 0.2, medn, height=0.38, color=C_NAIVE, alpha=0.8, label='наивная фузия (медиана)')
        for yi, n in zip(y, names):
            if ((agg[n].get('naive_nonfinite') or {}).get('max') or 0) > 0:
                ax.text(0.02, yi - 0.2, 'наивная: NaN', va='center', fontsize=7, color=C_NAIVE)
        ax.set_xscale('symlog', linthresh=0.1)
        ax.set_xlabel(lab)
        ax.legend(loc='lower right', fontsize=7)
    axs[0].set_yticks(y)
    axs[0].set_yticklabels(names, fontsize=8)
    axs[0].invert_yaxis()
    fig.suptitle('Синтетические отказы: 12 заездов, оценка против наивной фузии колёс', fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_adhesion(clean, path):
    plt = _plot_setup()
    ok = [r for r in clean if r.get('status') == 'ok']
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.6))
    ad99 = [r['adh'].get('adh_p99') for r in ok if r['adh'].get('adh_p99') is not None]
    admax = [r['adh'].get('adh_max') for r in ok if r['adh'].get('adh_max') is not None]
    axs[0].hist(ad99, bins=20, color=C_REF, alpha=0.8, label='p99 по заезду')
    axs[0].hist(admax, bins=20, color=C_EST, alpha=0.5, label='максимум по заезду')
    axs[0].set_xlabel('adhesion_used = |a|/g')
    axs[0].set_ylabel('заездов')
    axs[0].legend()
    for ax, k, lab in ((axs[1], 'gtr', 'gain_tr (тяга)'), (axs[2], 'gbr', 'gain_br (тормоз)')):
        rows = sorted([r for r in ok if r['adh'].get(k + '_min') is not None], key=lambda r: r['adh'][k + '_final'])
        x = np.arange(len(rows))
        ax.vlines(x, [r['adh'][k + '_min'] for r in rows], [r['adh'][k + '_max'] for r in rows], color='0.6',
                  label='диапазон за заезд')
        ax.plot(x, [r['adh'][k + '_final'] for r in rows], 'o', ms=3, color=C_EST, label='в конце заезда')
        ax.axhline(1.0, color='k', lw=0.6)
        ax.set_xlabel('заезды (сортировка по конечному значению)')
        ax.set_ylabel(lab)
        ax.legend(fontsize=7)
    fig.suptitle('Сцепление и адаптация модели на чистых прогонах (%d заездов)' % len(ok), fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def make_plots(S, cfg):
    PLOT_DIR = cfg.get('plot_dir') or globals()['PLOT_DIR']
    os.makedirs(PLOT_DIR, exist_ok=True)
    figs = []
    seed = cfg['seed']
    fb = S['faults']['bags'] if 'faults' in S else list(FAULT_BAGS)
    anc = S['faults']['anchors'] if 'faults' in S else {}
    cache = {}

    def ctx_of(bag):
        if bag not in cache:
            cache[bag] = Ctx(bag, cfg['gnss_limit'])
        return cache[bag]

    def first_with(key):
        for b in fb:
            a = anc.get(b) or {}
            if a.get(key):
                return b
        return fb[0]

    b_cr, b_br = first_with('cruise'), first_with('brake')
    p = os.path.join(PLOT_DIR, 'robust_drop_both.png')
    plot_drop([(ctx_of(b_cr), 'drop_both_30'), (ctx_of(b_br), 'drop_both_15_brake'),
               (ctx_of(b_br), 'combo_drop15_cmd15_brake')], p, seed)
    figs.append((p, 'обе тележки молчат: 30 с в движении, 15 с при торможении, 15 с при торможении без команды; '
                    'оценка против эталона и наивного удержания (ZOH); ниже — смещение вдоль пути относительно чистого прогона'))
    b_sp, b_sl = first_with('spin'), first_with('slide')
    p = os.path.join(PLOT_DIR, 'robust_slip_injection.png')
    plot_slip([(ctx_of(b_sp), 'slip_front_0.3', 1), (ctx_of(b_sp), 'slip_both_0.3', 1),
               (ctx_of(b_sl), 'slide_front_0.3_brake', 0), (ctx_of(b_sl), 'slide_both_0.3_brake', 1)], p, seed)
    figs.append((p, 'введённое боксование передней тележки, синфазное боксование обеих, юз передней и синфазный юз обеих '
                    'при торможении; флаг slip'))
    p = os.path.join(PLOT_DIR, 'robust_single_bogie.png')
    plot_single([(ctx_of(b_cr), 'drop_rear_60'), (ctx_of(b_cr), 'zero_rear_30'), (ctx_of(b_cr), 'freeze_front_30')], p, seed)
    figs.append((p, 'отказы одной тележки: пропадание задней 60 с, нули задней 30 с, замерзание передней 30 с; состояние фузии'))
    p = os.path.join(PLOT_DIR, 'robust_stamp_jump.png')
    plot_stamp(ctx_of(b_cr), ['stamp_jump_+0.5', 'stamp_jump_-0.5'], p, seed)
    figs.append((p, 'сдвиг всех входных штампов на ±0.5 с на 5 с: штампы, скорость на выходе, ошибка'))
    # real slip episodes: the longest confirmed (or longest) episode of each slip bag with a reference
    items = []
    for r in S.get('real', []):
        if r.get('category') != 'slip' or r.get('status') != 'ok' or not r.get('slip_episodes'):
            continue
        eps = [e for e in r['slip_episodes'] if e.get('confirmed')] or r['slip_episodes']
        e = max(eps, key=lambda e: (e.get('naive_max_err') or 0.0, e['dur']))

        def _v(k):
            x = e.get(k)
            return np.nan if x is None else float(x)
        items.append((ctx_of(r['bag']), e['start_rel'], e['dur'],
                      'передняя %.1f–%.1f, задняя %.1f–%.1f м/с' % (_v('raw_front_min'), _v('raw_front_max'),
                                                                  _v('raw_rear_min'), _v('raw_rear_max'))))
        if len(items) == 3:
            break
    if items:
        p = os.path.join(PLOT_DIR, 'robust_real_slip.png')
        plot_real(items, p)
        figs.append((p, 'реальные эпизоды проскальзывания/юза (без инъекции): сырые тележки, эталон, оценка, флаг'))
    if 'faults' in S:
        p = os.path.join(PLOT_DIR, 'robust_overview.png')
        plot_overview(S, p)
        figs.append((p, 'сводка по сценариям: max |ошибка скорости| и |Δs| к концу отказа, оценка против наивной фузии'))
    if S.get('clean_rows'):
        p = os.path.join(PLOT_DIR, 'robust_adhesion_gains.png')
        plot_adhesion(S['clean_rows'], p)
        figs.append((p, 'распределение adhesion_used и диапазоны адаптивных коэффициентов тяги/тормоза по 60 заездам'))
    return figs


# ------------------------------------------------------------------ findings
def findings(S):
    """Observations for summary.md: numbers from this run, mechanisms from the code (see the replays in the text)."""
    out = []

    def g(A, k, stat='median'):
        return (A.get(k) or {}).get(stat)

    if 'faults' in S:
        agg = S['faults']['agg']
        got = {n: A for n, A in agg.items() if A['n_bags']}
        tot = sum(A['n_bags'] for A in got.values())
        n_crash = sum(A['n_crash'] for A in got.values())
        nonfin = [n for n, A in got.items() if A['n_finite'] < A['n_bags'] - A['n_crash']]
        smin = min((A['stamps_from_input_min'] for A in got.values() if A.get('stamps_from_input_min') is not None), default=None)
        out.append('**Работоспособность.** Прогонов с отказами: %d (%d заездов × %d сценариев); падений: %d; прогонов с нечисловыми '
                   'выходами: %d; отрицательных скоростей: %d; доля выходов со штампом входного сообщения — не ниже %s во всех прогонах.'
                   % (tot, len(S['faults']['bags']), len(got), n_crash, len(nonfin), sum(A['n_neg_v'] for A in got.values()),
                      fmt(smin, 3)))
        # single bogie
        parts = []
        for n in ('drop_front_60', 'drop_rear_60', 'zero_rear_30', 'freeze_front_30', 'nan_front_10'):
            A = got.get(n)
            if A and A['n_windows']:
                parts.append('`%s` v_rmse окна %s (чистый %s), max %s м/с, |Δs| %s м (макс %s)' % (
                    n, fmt(g(A, 'v_rmse')), fmt(g(A, 'v_rmse_clean')), fmt(g(A, 'v_max', 'max'), 2),
                    fmt(g(A, 'd_s_b1'), 2), fmt(g(A, 'd_s_b1', 'max'), 2)))
        if parts:
            out.append('**Одна тележка.** Отказ одной тележки (молчание, нули в движении, «замерзание», NaN) не влияет на точность: '
                       + '; '.join(parts) + '. Нули и «замерзание» отбраковываются через `stuck_zero_other_mps` / `frozen_s` '
                       '(переход в `single_*` за ≈ 1.5 с, см. `robust_single_bogie.png`), NaN не доходит до фильтра; наивная '
                       'фузия в тех же окнах даёт ошибку в метры в секунду (табл. 4), а при NaN — NaN.')
        # both bogies
        parts = []
        for n in ('drop_both_5', 'drop_both_15', 'drop_both_30', 'drop_both_15_brake', 'drop_both_30_brake'):
            A = got.get(n)
            if A and A['n_windows']:
                parts.append('`%s` %s м (макс %s) против %s м (макс %s)' % (
                    n, fmt(g(A, 'd_s_b1'), 2), fmt(g(A, 'd_s_b1', 'max'), 2), fmt(g(A, 'naive_d_s_b'), 2),
                    fmt(g(A, 'naive_d_s_b', 'max'), 2)))
        if parts:
            A30 = got.get('drop_both_30') or {}
            out.append('**Обе тележки молчат (команда есть).** Модель тяги/торможения по ручке контроллера ведёт скорость без колёс; '
                       '|Δs| к концу отказа (медиана по заездам) против наивного удержания последней скорости: ' + '; '.join(parts)
                       + '. После возврата колёс оценка возвращается к чистому прогону за %s с (медиана, `drop_both_30`); '
                       'Δs частично исправляется через ковариацию v–s (s-коррекция), остаток — через привязку к уклонам (TRN).'
                       % fmt(g(A30, 'rec_vs_clean'), 2))
        A, B = got.get('combo_drop15_cmd15'), got.get('combo_drop15_cmd15_brake')
        if A and A['n_windows']:
            f1 = A.get('first_emit_d_s_signed') or {}
            f1b = (B or {}).get('first_emit_d_s_signed') or {}
            out.append('**Все входы молчат 15 с (колёса + команда).** Выходов в окне нет (выход публикуется только на входное сообщение). '
                       'Первый опубликованный после паузы выход несёт состояние ДО паузы с новым штампом: Δs медиана %s м '
                       '(%s…%s; при торможении %s…%s) при σ_along %s м (|Δs|/σ до %s). Причина — `TimeGuard` считает консенсус штампов '
                       'в сообщениях, а не во времени: первое сообщение после 15-с паузы помечается как «скачок вперёд» и выход '
                       'формируется без продвижения фильтра; следующим сообщением фильтр перекрывает паузу с постоянной скоростью '
                       '(`max_gap_s` = 5 с, σ_along ≈ %s м), и через 1 с Δs = %s м (медиана; в движении это эквивалент ZOH, при '
                       'торможении — переоценка пути). Это замечание к пакету (см. `issues`), а не к метрике.' % (
                           fmt(f1.get('median'), 1), fmt(f1.get('min'), 1), fmt(f1.get('max'), 1), fmt(f1b.get('min'), 1),
                           fmt(f1b.get('max'), 1), fmt(g(A, 'first_emit_sd_along'), 1), fmt(g(A, 'worst1s_z', 'max'), 1),
                           fmt(g(A, 'max_sd_along_3s'), 1), fmt((A.get('d_s_b1_signed') or {}).get('median'), 1)))
        # slip on one bogie
        parts = []
        for n in ('slip_front_0.3', 'slip_front_0.15', 'slip_rear_0.3', 'slide_front_0.3_brake'):
            A = got.get(n)
            if A and A['n_windows']:
                parts.append('`%s` recall %s, t_flag %s с, v_rmse эпизода %s против %s у наивного среднего' % (
                    n, fmt(A.get('recall_0p5'), 2), fmt(g(A, 'ttf'), 2), fmt(g(A, 'ep_rmse')), fmt(g(A, 'ep_rmse_naive'))))
        if parts:
            out.append('**Проскальзывание / юз одной тележки** распознаётся по расхождению тележек: ' + '; '.join(parts)
                       + '. Оценка идёт по второй тележке и модели, ошибка скорости остаётся на уровне чистого прогона.')
        A, B = got.get('slip_both_0.3'), got.get('slip_both_0.15_ramp1s')
        if A and A['n_windows']:
            out.append('**Синфазное боксование обеих тележек** (скачок +30 %%, 3 с): recall %s (флаг по ускорению > '
                       '`slip_acc_flag` = 2.5 м/с²). Колёса согласованы, поэтому флаг держится только на переходе (≈ 1.4 с с '
                       '`slip_hold_s` = 1 с), а затем завышенный уровень принимается как измерение `both` и сдерживается лишь '
                       'ограничением скорости изменения (`acc_max` = 1.8 м/с²): оценка ползёт вверх и к концу эпизода превышает '
                       'истину на ≈ 3 м/с. Конец эпизода обрабатывается неверно (замечание к пакету, см. `issues`): возврат колёс '
                       'к истинной скорости (−4 м/с за такт) отбрасывается фильтром выбросов `WheelFusion.add_sample` на обеих '
                       'тележках, а `measure` линейно экстраполирует обе тележки по наклону этого скачка вниз (≈ −19 м/с²) до '
                       'времени фильтра — «согласованное» измерение оказывается ниже обеих сырых тележек (в разобранном случае '
                       '`30618_3b36e5cd`, эпизод 4: 10.3 м/с при сырых 13.7), и поскольку отсечение по `acc_max` длилось дольше '
                       '`clip_resync_s` = 3 с, EKF принимает это экстраполированное значение целиком (resync): скачок выхода до %s м/с '
                       '(табл. 1), оценка на ≈ 3 м/с ниже истины и возвращается к ней ≈ 2 с. Другой вариант того же дефекта — на '
                       '`robust_slip_injection.png` (`30618_073f08d1`, эпизод 2): новый (истинный) отсчёт отброшен, resync '
                       'принимает значение, экстраполированное по прежнему наклону вверх (10.7 м/с при истинных 8.2), а флаг slip, '
                       'поднятый самим возвратом, ослабляет измерение, и ≈ 1.1 с оценка идёт по модели вверх до 11.4 м/с (на ≈ 2.2 '
                       'м/с выше истины), затем за ≈ 0.3 с сходится к колёсам. Итог: v_rmse эпизода %s против %s у '
                       'наивного среднего, |Δs| эпизода %s м (макс %s) против %s м у наивной. Плавное синфазное боксование с '
                       'нарастанием ниже порога ускорения (`slip_both_0.15_ramp1s`: recall %s, флаг хоть раз %d/%d) без внешнего '
                       'датчика неотличимо от реального разгона: оценка идёт за колёсами, |Δs| эпизода %s м ≈ наивной (%s м).' % (
                           fmt(A.get('recall_0p5'), 2), fmt(g(A, 'out_step_max', 'max'), 2), fmt(g(A, 'ep_rmse')),
                           fmt(g(A, 'ep_rmse_naive')), fmt(g(A, 'ep_d_s'), 2), fmt(g(A, 'ep_d_s', 'max'), 2),
                           fmt(g(A, 'ep_d_s_naive'), 2),
                           fmt((B or {}).get('recall_0p5'), 3), (B or {}).get('n_flagged_any', 0), (B or {}).get('n_windows', 0),
                           fmt(g(B or {}, 'ep_d_s'), 2), fmt(g(B or {}, 'ep_d_s_naive'), 2)))
        A, B = got.get('slide_both_0.3_brake'), got.get('lock_both_2s_brake')
        if A and A['n_windows']:
            out.append('**Синфазный юз обеих тележек −30 %% (2 с)**: флаг поднимается сразу (recall %s, t_flag %s с), но через '
                       '`slip_hold_s` = 1 с согласованный и плавный заниженный уровень колёс принимается (при `both` без флага '
                       'допустимое замедление `dec_max_consistent` = 6 м/с²), а возврат колёс к истинной скорости в конце эпизода сам '
                       'выглядит как боксование (+30 %% за такт) и отбрасывается ещё ≈ 1 с. Итог: v_rmse эпизода %s (наивная %s), '
                       '|Δs| эпизода %s м (макс %s) против %s м у наивной — здесь оценка по пути не лучше наивной. Полная блокировка '
                       '(`lock_both_2s_brake`, обе тележки шлют 0) распознаётся правилом `lock_zero_dec`: recall %s, v_rmse эпизода %s '
                       'против %s, |Δs| %s м против %s м.' % (
                           fmt(A.get('recall_0p5'), 2), fmt(g(A, 'ttf'), 2), fmt(g(A, 'ep_rmse')), fmt(g(A, 'ep_rmse_naive')),
                           fmt(g(A, 'ep_d_s'), 2), fmt(g(A, 'ep_d_s', 'max'), 2), fmt(g(A, 'ep_d_s_naive'), 2),
                           fmt((B or {}).get('recall_0p5'), 2), fmt(g(B or {}, 'ep_rmse')), fmt(g(B or {}, 'ep_rmse_naive')),
                           fmt(g(B or {}, 'ep_d_s'), 2), fmt(g(B or {}, 'ep_d_s_naive'), 2)))
        A, B = got.get('stamp_jump_+0.5'), got.get('stamp_jump_-0.5')
        if A and A['n_windows']:
            out.append('**Сдвиг штампов ±0.5 с.** `TimeGuard` не принимает скачок за истину: внутри окна выходы по контракту '
                       'несут сдвинутые штампы, поэтому их ошибка против эталона «по штампу» ≈ a·0.5 с (max %s / %s м/с); после '
                       'возврата штампов оценка совпадает с чистым прогоном через %s / %s с (медиана), |Δs| %s / %s м.' % (
                           fmt(g(A, 'v_max', 'max'), 2), fmt(g(B or {}, 'v_max', 'max'), 2), fmt(g(A, 'rec_vs_clean'), 2),
                           fmt(g(B or {}, 'rec_vs_clean'), 2), fmt(g(A, 'd_s_b1'), 2), fmt(g(B or {}, 'd_s_b1'), 2)))
        A, B = got.get('cmd_drop_60'), got.get('cmd_freeze_30')
        if A and A['n_windows']:
            B = B or {}
            out.append('**Команда контроллера.** Пропажа команды на 60 с при исправных колёсах безвредна: |Δal@+30| %s м (макс %s); '
                       'коэффициенты модели не сдвигаются (минимум gain_tr за заезд %s против %s в чистом, gain_br %s против %s). '
                       '«Замёрзшая» команда 30 с опаснее: модель получает неверную тягу/торможение, и адаптивные `gain_tr`/`gain_br` '
                       'и `bias` подстраиваются под неверный вход — минимум gain_tr за заезд %s (чистый %s), gain_br %s (чистый %s), '
                       'до границы 0.8 дошли в %d из %d заездов (в чистых прогонах тех же заездов — в %d, табл. 7a); это смещение '
                       'остаётся и после отказа: |Δal@+30| %s м (макс %s), |Δs| к концу заезда %s м (макс %s). Скорость при этом '
                       'верна (колёса есть), страдает только привязка по уклонам (TRN использует ускорение модели).' % (
                           fmt(g(A, 'd_al_30'), 2), fmt(g(A, 'd_al_30', 'max'), 2),
                           fmt(g(A, 'run_gtr_min'), 3), fmt(g(A, 'run_gtr_min_clean'), 3),
                           fmt(g(A, 'run_gbr_min'), 3), fmt(g(A, 'run_gbr_min_clean'), 3),
                           fmt(g(B, 'run_gtr_min'), 3), fmt(g(B, 'run_gtr_min_clean'), 3),
                           fmt(g(B, 'run_gbr_min'), 3), fmt(g(B, 'run_gbr_min_clean'), 3),
                           B.get('n_gain_lo', 0), B.get('n_bags', 0) - B.get('n_crash', 0), B.get('n_gain_lo_clean', 0),
                           fmt(g(B, 'd_al_30'), 2), fmt(g(B, 'd_al_30', 'max'), 2), fmt(g(B, 'run_abs_d_s_end'), 2),
                           fmt(g(B, 'run_abs_d_s_end', 'max'), 2)))
        parts = []
        for n in ('spikes_0.5hz_20kmh', 'noise_1kmh', 'noise_1kmh_moving', 'scale_front_1.05'):
            A = got.get(n)
            if A and A['n_bags']:
                parts.append('`%s`: v_rmse %s (чистый %s), флаг slip на %s выходов (чистый %s), |Δs| к концу заезда %s м (макс %s), '
                             '|Δдрейф| %s п.п.' % (
                                 n, fmt(g(A, 'run_v_rmse')), fmt(g(A, 'run_v_rmse_clean')), fmt(g(A, 'run_slip_frac'), 3),
                                 fmt(g(A, 'run_slip_frac_clean'), 4), fmt(g(A, 'run_abs_d_s_end'), 1),
                                 fmt(g(A, 'run_abs_d_s_end', 'max'), 1), fmt(g(A, 'run_abs_d_drift_pct'), 3)))
        if parts:
            Sp = got.get('spikes_0.5hz_20kmh') or {}
            out.append('**Искажения на весь заезд.** ' + '; '.join(parts) + ' Одиночные выбросы гасятся фильтром выбросов '
                       '(`spike_kmh` = 4): точность не меняется, флаг slip — на %s выходов (медиана; макс %s) против %s в чистом '
                       'прогоне, т. е. почти только в момент выброса. Белый шум' % (
                           fmt(g(Sp, 'run_slip_frac'), 4), fmt(g(Sp, 'run_slip_frac', 'max'), 4),
                           fmt(g(Sp, 'run_slip_frac_clean'), 4))
                       + ' 1 км/ч (σ = 0.28 м/с на отсчёт — в ~10 раз больше '
                       'собственного шума датчиков, расхождение тележек в чистых данных ≈ 0.02–0.08 м/с) превышает допуск согласия '
                       '(`agree_abs` = 0.2 м/с): состояние `disagree` и флаг slip держатся почти всё время движения; скорость остаётся '
                       'точной (v_rmse ≈ 0.15 м/с), но флаг теряет смысл, а на трогании флаг + ограничение `acc_max` могут держать '
                       'оценку у нуля до `clip_resync_s` = 3 с. Вариант `noise_1kmh` дополнительно добавляет шум на стоянке, где '
                       'обрезка по нулю даёт положительное смещение, отсюда систематический Δs > 0. Ошибка масштаба одной тележки '
                       '+5 % при v > 4 м/с превышает допуск согласия (`agree_abs` = 0.2 м/с / `agree_rel` = 3 %): большую часть '
                       'движения тележки «не согласны», флаг slip поднят, а в состоянии `disagree` берётся тележка, ближайшая к '
                       'прогнозу фильтра, так что оценка часть времени идёт по завышенной передней; остаток исправляет TRN. '
                       'Онлайн-оценки отношения масштабов тележек в пакете нет (замечание в `issues`).')
    if 'clean' in S:
        C = S['clean']
        gor = C.get('gnss_offset_runs') or {}
        out.append('**Флаг slip на %d чистых заездах** (%.1f ч): %s выходов с флагом, %d эпизодов (%s/ч) в %d заездах; подтверждены '
                   'эталоном %d, не подтверждены %d (%s/ч; %s с из %.1f ч, доля времени %s), в окне сбоя часов %d%s, без '
                   'эталона %d. Подтверждение — по '
                   'очищенному эталону с исправленными длительными сдвигами часов GNSS; такие сдвиги (≈ ±1 с на 18–60 с) есть в %d '
                   'из %d заездов (%s), и `evaluate.py --clean-ref` их не исправляет (замечание в `issues`).' % (
                       C['n_bags'], C['hours'], fmt(C['slip_frac_pooled'], 5), C['slip_episodes'], fmt(C['slip_episodes_per_h'], 2),
                       C['bags_with_flag'], C['slip_confirmed'], C['slip_unconfirmed'],
                       fmt(C['slip_unconfirmed'] / C['hours'] if C['hours'] else None, 2), fmt(C['unconfirmed_time_s'], 1),
                       C['hours'], fmt(C['unconfirmed_frac_pooled'], 6), C.get('slip_stamp', 0),
                       (' (%s: флаг поднят скачком в потоке колёс в момент сбоя часов — скачок или дрейф штампов колёс/команды '
                        'либо начало/конец сдвига часов GNSS; выходы или эталон там сдвинуты по времени, и ни оценку, ни наивное '
                        'среднее по эталону судить нельзя)' % ', '.join('`%s`' % b for b in C.get('slip_stamp_bags') or []))
                       if C.get('slip_stamp') else '', C['slip_noref'], len(gor), C['n_bags'],
                       ', '.join('`%s`' % b for b in sorted(gor))))
        gb = C.get('gain_bound_hits') or {}
        rows = [r for r in S.get('clean_rows') or [] if r.get('adh')]
        top_k = max(rows, key=lambda r: r['adh'].get('adhk_max') or 0) if rows else None
        top_s = max(rows, key=lambda r: r['adh'].get('a_sign_wrong') or 0) if rows else None
        txt = ('**Сцепление и адаптация.** `adhesion_used` = |a|/g, где a — ускорение модели процесса (тяга/торможение по ручке × '
               'коэффициенты + уклон и кривизна по карте + `bias`), а не измеренное: медиана p99 по заездам %s, максимум %s — все '
               '< 0.2 (в пределах сцепления сухого рельса; `acc_max` = 1.8 м/с² ≈ 0.18 g). Проверка по самой выходной скорости '
               '(|dv/dt|/g за 1 с): p99 %s (медиана по заездам), максимум по заезду — медиана %s, наибольший %s (`%s`). ' % (
                   fmt(C['adh_p99']['median'], 3), fmt(C['adh_max']['max'], 3), fmt((C.get('adhk_p99') or {}).get('median'), 3),
                   fmt((C.get('adhk_max') or {}).get('median'), 3), fmt((C.get('adhk_max') or {}).get('max'), 3),
                   top_k['bag'] if top_k else '–'))
        if top_k and top_k['bag'].endswith('616ec56b') and (top_k['adh'].get('adhk_max') or 0) > 0.3:
            txt += ('Это реальное резкое торможение без тормозной позиции ручки (≈ 1022 с: ручка 15 → 0, скорость по GNSS 8.2 → '
                    '1.9 м/с за 2 с, до ≈ 4.2 м/с² ≈ 0.43 g — по-видимому, экстренный или рельсовый тормоз вне контроллера): модель '
                    'даёт a ≈ −0.4…−0.7 м/с², оценка идёт за колёсами с отставанием до ≈ 0.9 м/с, флаг slip не поднимается. '
                    'То есть `adhesion_used` показывает сцепление, «заложенное» моделью, и торможение вне команды не видит. ')
        txt += ('Невязка a модели и dv/dt: p95 %s м/с² (медиана по заездам; макс %s), противоположный знак — %s времени (медиана; '
                'макс %s, `%s`). gain_tr в конце заезда %s…%s (медиана %s), gain_br %s…%s (медиана %s); у границы 0.8 хоть раз: '
                'gain_tr %s, gain_br %s заездов, у 1.3 — %s / %s. Коэффициенты адаптируются только при согласии с колёсами (3σ), '
                'без отсечения по скорости изменения и без флага slip.' % (
                    fmt((C.get('a_diff_p95') or {}).get('median'), 2), fmt((C.get('a_diff_p95') or {}).get('max'), 2),
                    fmt((C.get('a_sign_wrong') or {}).get('median'), 4), fmt((C.get('a_sign_wrong') or {}).get('max'), 4),
                    top_s['bag'] if top_s else '–',
                    fmt(C['gtr_final']['min'], 3), fmt(C['gtr_final']['max'], 3), fmt(C['gtr_final']['median'], 3),
                    fmt(C['gbr_final']['min'], 3), fmt(C['gbr_final']['max'], 3), fmt(C['gbr_final']['median'], 3),
                    gb.get('gtr_lo', '–'), gb.get('gbr_lo', '–'), gb.get('gtr_hi', '–'), gb.get('gbr_hi', '–')))
        if 'faults' in S:
            sh, fr = [], []
            for n, A in S['faults']['agg'].items():
                if not A['n_bags']:
                    continue
                d = [(g(A, 'run_%s_min' % k) or 1.0) - (g(A, 'run_%s_min_clean' % k) or 1.0) for k in ('gtr', 'gbr')]
                if not (A.get('n_gain_lo', 0) > A.get('n_gain_lo_clean', 0) or max(abs(x) for x in d) > 0.05):
                    continue
                item = '`%s` (мин. gain_tr %s против %s, gain_br %s против %s; у 0.8: %d против %d)' % (
                    n, fmt(g(A, 'run_gtr_min'), 3), fmt(g(A, 'run_gtr_min_clean'), 3), fmt(g(A, 'run_gbr_min'), 3),
                    fmt(g(A, 'run_gbr_min_clean'), 3), A.get('n_gain_lo', 0), A.get('n_gain_lo_clean', 0))
                # minimum ABOVE the clean one and no bound hits: adaptation frozen, not driven off
                (fr if min(d) > -0.05 and A.get('n_gain_lo', 0) <= A.get('n_gain_lo_clean', 0) else sh).append(item)
            txt += (' Под отказами (табл. 7a) коэффициенты уводятся от чистого прогона (медиана минимума за заезд ниже на > 0.05 '
                    'или граница 0.8 чаще, чем в чистом) только в сценариях: %s.' % (', '.join(sh) if sh else 'нет'))
            if fr:
                txt += (' Наоборот, почти не адаптируются (минимум выше чистого: флаг slip поднят почти всё время и блокирует '
                        'адаптацию, коэффициенты остаются у начальных значений): %s.' % ', '.join(fr))
            txt += ' Остальные отказы коэффициенты не трогают.'
        out.append(txt)
    if 'real' in S:
        rs = [r for r in S['real'] if r.get('status') == 'ok']
        nc = sum(1 for r in S['real'] if r.get('status') != 'ok')
        nf = sum(1 for r in rs if r['n_nonfinite_v'] or r['n_nonfinite_pose'])
        byb = {r['bag'][-8:]: r for r in rs}
        rtk = [r for r in rs if r.get('ref_kind') == 'rtk' and (r.get('speed_fix') or {}).get('n')]
        dop = [r for r in rs if r.get('ref_kind') == 'doppler' and (r.get('speed_fix') or {}).get('n')]
        nog = [r['bag'] for r in rs if r.get('ref_kind') is None]

        def rng(R, k, nd=3):
            x = [(r.get('speed_fix') or {}).get(k) for r in R]
            x = [v for v in x if v is not None]
            return '%s…%s' % (fmt(min(x), nd), fmt(max(x), nd)) if x else '–'
        txt = ('**Реальные аномальные заезды** (%d): падений %d, нечисловых выходов %d. Точность по исправленному эталону '
               '(табл. 10): v_rmse %s м/с на %d заездах с RTK и %s м/с по доплеровской скорости на %d заездах без RTK, max '
               '|ошибка v| %s м/с%s.' % (
                   len(S['real']), nc, nf, rng(rtk, 'v_rmse'), len(rtk), rng(dop, 'v_rmse'), len(dop), rng(rtk + dop, 'v_max', 2),
                   ('; без GNSS вовсе (нет ни эталона, ни начальной привязки GNSS) — %s: оценщик работает, проверены только '
                    'конечность и флаг' % ', '.join('`%s`' % b for b in nog)) if nog else ''))
        # GNSS clock offsets
        off = [r for r in rs if r.get('gnss_offset_runs')]
        if off:
            parts = []
            for r in off:
                m = r.get('metrics') or {}
                sp = r.get('speed_fix') or {}
                runs = ', '.join('%.0f–%.0f с (%+.1f с)' % tuple(x) for x in r['gnss_offset_runs'])
                if m.get('v_rmse') is not None:
                    parts.append('`%s` — %s: v_rmse как есть %s → испр. %s, отсчётов > 0.5 м/с %s → %s' % (
                        r['bag'], runs, fmt(m.get('v_rmse')), fmt(sp.get('v_rmse')), fmt(r.get('n_err_gt0p5')),
                        fmt(sp.get('n_gt0p5'))))
                else:
                    parts.append('`%s` — %s (доплер, эталон исправлен до сравнения)' % (r['bag'], runs))
            txt += (' Сдвиги часов GNSS (штамп сдвинут на ≈ 1 с на десятки секунд) портят эталон, а не оценку: ' + '; '.join(parts)
                    + '. Одиночные большие ошибки «как есть» (до %s м/с) — артефакты эталона (скорость GNSS «замерзает» или '
                    'скачет); очищенный эталон их убирает.' % fmt(max((r.get('metrics') or {}).get('v_max') or 0 for r in rs), 2))
        # slip bags
        sl = [r for r in rs if r.get('category') == 'slip']
        ce = [e for r in sl for e in (r.get('slip_episodes') or []) if e.get('confirmed') is True and not clock_glitch(e)]
        if sl:
            txt += (' На 4 заездах с проскальзыванием %d эпизодов флага, %d подтверждены эталоном (наивное среднее тележек '
                    'отличается от эталона > %.1f м/с); на подтверждённых max ошибка оценки %s…%s м/с против %s…%s у наивного '
                    'среднего.' % (
                        sum(r['slip_episodes_n'] for r in sl), len(ce), SLIP_CONFIRM,
                        fmt(min(e['est_max_err'] for e in ce), 2), fmt(max(e['est_max_err'] for e in ce), 2),
                        fmt(min(e['naive_max_err'] for e in ce), 2), fmt(max(e['naive_max_err'] for e in ce), 2)))
            worst = max(ce, key=lambda e: e['est_max_err']) if ce else None
            r50 = byb.get('50956d6e')
            if worst and r50 and any(e is worst for e in r50.get('slip_episodes') or []):
                txt += (' Худший — `30639_50956d6e`, %.0f с: синфазный юз обеих тележек до остановки (колёса показывают 0 примерно '
                        'на 2 с раньше GNSS, тележки согласны между собой); флаг поднят, но опереться не на что, и оценка идёт за '
                        'колёсами: %s м/с против %s у наивной — тот же предел, что у `slide_both` в синтетике.' % (
                            worst['start_rel'], fmt(worst['est_max_err'], 2), fmt(worst['naive_max_err'], 2)))
        # dropout bags
        dr = [r for r in rs if r.get('category') == 'dropout']
        if dr:
            fr, rm, rb_ = [], [], []
            for r in dr:
                sf = r.get('state_frac') or {}
                fr.append(sum(v for k, v in sf.items() if k.startswith('single')))
                bs = (r.get('speed_fix') or {}).get('by_state') or {}
                pr = ['%s / %s' % (fmt(x['v_rmse']), fmt((bs.get('both') or {}).get('v_rmse')))
                      for k, x in bs.items() if k.startswith('single') and x.get('n', 0) >= 50]
                rm += ['`%s` %s' % (r['bag'][-8:], p_) for p_ in pr]
            txt += (' На заездах с пропаданием тележки в `single_*` %s…%s %% времени; v_rmse в `single_*` / в `both` того же '
                    'заезда, м/с: %s — на одном уровне (на `3b3d9eb8` и `c31df386` эталон — доплер без RTK).'
                    % (fmt(100 * min(fr), 1), fmt(100 * max(fr), 1), '; '.join(rm) if rm else '–'))
        r4 = byb.get('4d487b0d')
        if r4:
            a4 = r4.get('adh') or {}
            mx = max([(r.get('adh') or {}).get('a_sign_wrong') or 0 for r in (S.get('clean_rows') or []) + rs] + [0])
            txt += (' `30618_4d487b0d` (аномалия команды): скорость верна (v_rmse испр. %s м/с), но команда расходится с движением '
                    '(тормозные позиции ручки при разгоне): знак a модели противоположен dv/dt %s времени%s, p95 невязки %s м/с², '
                    'gain_br уходит до %s. al_rmse %s м — не от команды: заезд начинается '
                    '≈ 590 м до начала карты, при въезде на карту ошибка вдоль пути ≈ 26 м, и TRN снимает её только к ≈ 420 с '
                    '(тот же сдвиг и при обнулённой ручке).' % (
                        fmt((r4.get('speed_fix') or {}).get('v_rmse')), fmt(a4.get('a_sign_wrong'), 3),
                        ' (наибольшая доля среди всех заездов)' if (a4.get('a_sign_wrong') or 0) >= mx else '',
                        fmt(a4.get('a_diff_p95'), 2),
                        fmt(a4.get('gbr_min'), 3), fmt((r4.get('metrics') or {}).get('al_rmse'), 1)))
        out.append(txt)
    return out


# ------------------------------------------------------------------ main

def _relpaths(x):
    """Replace absolute paths inside the repository by repo-relative ones (no machine-specific paths in results)."""
    root = os.path.abspath(ev.ROOT) + os.sep
    if isinstance(x, dict):
        return {k: _relpaths(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_relpaths(v) for v in x]
    if isinstance(x, str) and x.startswith(root):
        return x[len(root):]
    if isinstance(x, str) and x == root[:-1]:
        return '.'
    return x

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--part', default='all',
                    help='all | faults | clean | real | plots | md (comma list; md = rewrite summary.md from summary.json)')
    ap.add_argument('--bags', default=None, help='fault bags (comma list; default: the fixed 12)')
    ap.add_argument('--scenarios', default=None, help='scenario names (comma list; default: all)')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--gnss-limit', type=float, default=10.0)
    ap.add_argument('--out', default=OUT_DIR)
    ap.add_argument('--plots', default=PLOT_DIR, help='figure directory')
    ap.add_argument('--list', action='store_true', help='list scenarios and exit')
    a = ap.parse_args(argv)
    if a.list:
        for s in SCENARIOS:
            print('%-28s %-7s %-7s %s' % (s['name'], s['family'], s['anchor'], s['ru']))
        return 0
    parts = set(a.part.split(','))
    if 'all' in parts:
        parts = {'faults', 'clean', 'real', 'plots'}
    os.makedirs(a.out, exist_ok=True)
    cfg = dict(gnss_limit=a.gnss_limit, seed=a.seed, plot_dir=a.plots,
               command='python tools/robustness.py ' + ' '.join(argv if argv is not None else sys.argv[1:]))
    index = load_index()
    fault_bags = a.bags.split(',') if a.bags else list(FAULT_BAGS)
    fault_bags = [b if b in index else full_name(b, index) for b in fault_bags]
    scen_names = a.scenarios.split(',') if a.scenarios else [s['name'] for s in SCENARIOS]
    for n in scen_names:
        if n not in SCEN:
            raise SystemExit('unknown scenario %r (see --list)' % n)
    eval_bags = sorted(b for b, r in index.items() if r.get('eval_ok'))
    real = [(cat, full_name(s, index)) for cat, s in REAL_BAGS]

    sum_path = os.path.join(a.out, 'summary.json')
    per_path = os.path.join(a.out, 'per_bag.json')
    S = {}
    per = {}
    if os.path.exists(sum_path) and not ({'faults', 'clean', 'real'} >= parts and len(parts) >= 3):
        with open(sum_path) as f:
            S = json.load(f)
        if os.path.exists(per_path):
            with open(per_path) as f:
                per = json.load(f)
    runs_tasks = bool({'faults', 'clean', 'real'} & parts)
    prev_cfg = S.get('config') or {}
    if runs_tasks or 'generated' not in S:
        S['generated'] = time.strftime('%Y-%m-%d %H:%M')
    S['config'] = dict(cfg, jobs=a.jobs, post_s=POST_S, after_s=AFTER_S, rec_thr=REC_THR, rec_hold=REC_HOLD,
                       v_move=V_MOVE, err_ok=ERR_OK, slip_confirm=SLIP_CONFIRM)

    tasks = []
    if 'faults' in parts:
        tasks += [('faults', b) for b in fault_bags]
    if 'real' in parts:
        tasks += [('real', b, cat) for cat, b in real]
    if 'clean' in parts:
        tasks += [('clean', b) for b in eval_bags]
    results = {'faults': [], 'real': [], 'clean': []}
    t_start = time.time()
    if tasks:
        env_threads = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
        os.environ.update(env_threads)
        with ProcessPoolExecutor(max_workers=max(1, a.jobs)) as ex:
            futs = {}
            for tk in tasks:
                if tk[0] == 'faults':
                    futs[ex.submit(task_faults, tk[1], scen_names, cfg)] = tk
                elif tk[0] == 'real':
                    futs[ex.submit(task_clean, tk[1], cfg, True, tk[2])] = tk
                else:
                    futs[ex.submit(task_clean, tk[1], cfg, False, None)] = tk
            for i, fu in enumerate(as_completed(futs)):
                tk = futs[fu]
                try:
                    r = fu.result()
                except Exception as e:
                    r = dict(bag=tk[1], status='crash', error='%s: %s' % (type(e).__name__, e))
                results[tk[0]].append(r)
                print('[%3d/%d] %6.1f s  %-6s %s %s' % (i + 1, len(tasks), time.time() - t_start, tk[0], tk[1],
                                                       r.get('status', 'ok') if tk[0] != 'faults' else
                                                       '%d scen' % len(r.get('scenarios', {}))), flush=True)
    if 'faults' in parts:
        fr = sorted(results['faults'], key=lambda r: fault_bags.index(r['bag']))
        S['faults'] = dict(bags=fault_bags, scenarios=scen_names,
                           anchors={r['bag']: r.get('anchors') for r in fr},
                           dirs={b: index[b].get('direction') for b in fault_bags},
                           clean={r['bag']: r.get('clean') for r in fr},
                           agg=aggregate_faults(fr),
                           scenario_defs={s['name']: dict(family=s['family'], anchor=s['anchor'], faults=s['faults'], ru=s['ru'])
                                          for s in SCENARIOS if s['name'] in scen_names})
        per['faults'] = {r['bag']: r for r in fr}
        crashes = [(r['bag'], n, s.get('error'), s.get('traceback')) for r in fr for n, s in r.get('scenarios', {}).items()
                   if s.get('status') == 'crash']
        S['faults']['crashes'] = [dict(bag=b, scenario=n, error=e) for b, n, e, _ in crashes]
        for b, n, e, tb in crashes:
            print('CRASH %s %s: %s\n%s' % (b, n, e, tb))
    if 'clean' in parts:
        cr = sorted(results['clean'], key=lambda r: r['bag'])
        S['clean'] = aggregate_clean(cr)
        S['clean_rows'] = [{k: r.get(k) for k in ('bag', 'status', 'slip_frac', 'slip_frac_moving', 'slip_episodes_n',
                                                  'slip_confirmed_n', 'slip_unconfirmed_n', 'slip_stamp_n', 'slip_noref_n', 'adh', 'state_frac', 'duration_s',
                                                  'n_out', 'n_nonfinite_v', 'n_nonfinite_pose', 'gnss_restamped',
                                                  'gnss_offset_runs')} for r in cr]
        per['clean'] = {r['bag']: r for r in cr}
    if 'real' in parts:
        order = [b for _, b in real]
        S['real'] = sorted(results['real'], key=lambda r: order.index(r['bag']))
    if runs_tasks:
        S['config']['wall_s'] = time.time() - t_start
    else:   # plots / md only: keep the timing and command of the run that produced the numbers
        for k in ('wall_s', 'command', 'jobs'):
            if k in prev_cfg:
                S['config'][k] = prev_cfg[k]
    S['findings'] = findings(S)
    if 'plots' in parts:
        try:
            S['figures'] = make_plots(S, cfg)
        except Exception:
            traceback.print_exc()
            S['figures_error'] = traceback.format_exc()
    S = _relpaths(jsonable(S))
    with open(sum_path, 'w') as f:
        json.dump(S, f, indent=1, ensure_ascii=False)
    with open(per_path, 'w') as f:
        json.dump(jsonable(per), f, indent=1, ensure_ascii=False)
    write_markdown(S, os.path.join(a.out, 'summary.md'))
    print('wrote %s (%.0f s)' % (a.out, time.time() - t_start))
    return 0


if __name__ == '__main__':
    sys.exit(main())
