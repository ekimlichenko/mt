"""Robust fusion of the front and rear bogie speed sensors into one speed measurement.

Each bogie publishes a raw speed in km/h at about 10 Hz with its own header stamp;
v = max(raw, 0) / K  [m/s], with K = params.k_for_vehicle().  Per bogie b the
two latest accepted samples (t0, v0), (t1, v1) are kept.

Sample validation (``add_sample``)
    rejected:  non-finite, raw > wheel_max_kmh, raw < wheel_min_kmh, t < t1 (out of order);
    clamped:   small negatives (roll-back, down to wheel_min_kmh) -> 0;
    spike:     |v - v_b(t)| > a_phys * (t - t1) + spike_kmh / K, the other bogie does not
               confirm the new value (same tolerance) and the previous sample of b was
               accepted -> rejected.  a_phys = max(acc_max, dec_max_consistent); a level
               that persists for a second sample is always accepted.

Per-bogie value and validity at time t (``measure``)
    v_b(t)  = max(0, v1 + s * clip(t - t1, t0 - t1, wheel_extrap_max_s)),
              s = (v1 - v0) / (t1 - t0)  (0 if t1 - t0 < 0.02 s); negative offsets interpolate.
    stale:       t - t1 > wheel_stale_s
    frozen:      the same non-zero raw value for >= frozen_s while the other bogie's range
                 over that span > frozen_other_dv_mps; invalid until the value changes.
    stuck zero:  v1 == 0 while the other valid bogie reads > stuck_zero_other_mps (the 30639
                 dropouts start with the dropped bogie sending 0.0 at departure).

Fusion of the valid bogies F, R  (tol = max(agree_abs_mps, agree_rel * max(F, R)))
    The bogies share header stamps, but one of a pair arrives < 1 ms before the other, so
    at the first arrival the other bogie is ~0.1 s old.  Two comparisons are made:
        own:      |F(t) - R(t)| with each bogie's own extrapolation
        aligned:  the older bogie moved by the newer bogie's change since its stamp,
                  v_old(t) = v_old(t1_old) + v_new(t) - v_new(t1_old)  (wheel_align_other;
                  only when the newer bogie brackets t1_old, else aligned = own)
    both within tol -> mean of the aligned values, sigma = meas_sigma, 'both'
    exactly one within tol -> mean of the agreeing pair, sigma = meas_sigma_slip, 'both',
                  no slip flag (an extrapolation artefact or the first sample of a slip;
                  the pair's second sample resolves it)
    neither   -> 'disagree', slip: the bogie closer to a reference speed, which is the
                  fused speed of the last full agreement held for slip_anchor_s (a spinning
                  bogie reads high, a sliding one low; both leave the last common value
                  faster than the true speed), then v_pred, then the last fused value
    one valid -> that bogie, sigma = meas_sigma_single, 'single_front' / 'single_rear'
    none      -> None

Slip flag
    set by a disagreement, or by the common-mode test on the fused speed:
        a = (v(t) - v(t_w)) / (t - t_w),  t_w = newest fused stamp with t - t_w >= slip_acc_win_s
        a > slip_acc_flag  or  a < -slip_dec_flag   (slip_dec_flag defaults to slip_acc_flag)
    and held for slip_hold_s after the last trigger.  If the fused speed drops to exactly 0
    while slip is set (both wheels locked: 30639_50956d6e at 1040 s reads 0 for 1.8 s while
    the tram decelerates from 1.6 m/s), slip is kept for v0 / lock_zero_dec seconds from
    that moment, v0 = the last agreed speed before the slip episode, or until the speed
    becomes non-zero.  While slip is set, sigma = meas_sigma_slip; the fused value itself
    is never dropped or altered.
"""
import math
from dataclasses import dataclass

NAN = float('nan')


@dataclass
class Measurement:
    v: float            # fused speed, m/s (>= 0)
    sigma: float        # measurement std, m/s
    state: str          # 'both' | 'disagree' | 'single_front' | 'single_rear'
    slip: bool          # slip/slide suspected (disagreement or implausible accel)
    front_ok: bool
    rear_ok: bool
    v_front: float      # extrapolated per-bogie values at t (nan if not valid)
    v_rear: float


class _Bogie:
    __slots__ = ('name', 't0', 'v0', 't1', 'v1', 'raw1', 'frz_t', 'oth_lo', 'oth_hi',
                 'spike_prev', 'n', 'rejected', 'spikes', 'stale', 'stuck_zero', 'frozen', 'age')

    def __init__(self, name):
        self.name = name
        self.t0 = self.v0 = self.t1 = self.v1 = self.raw1 = None
        self.frz_t = None
        self.oth_lo = self.oth_hi = None
        self.spike_prev = False
        self.n = self.rejected = self.spikes = 0
        self.stale = True
        self.stuck_zero = self.frozen = False
        self.age = math.inf

    def value_at(self, t, extrap_max):
        dt = t - self.t1
        if self.t0 is None:
            return self.v1
        span = self.t1 - self.t0
        if span < 0.02:
            return self.v1
        if dt > extrap_max:
            dt = extrap_max
        elif dt < -span:
            dt = -span
        v = self.v1 + (self.v1 - self.v0) / span * dt
        return v if v > 0.0 else 0.0


class WheelFusion:
    def __init__(self, params, k_scale: float):
        g = lambda name, default: float(getattr(params, name, default))
        self.k = float(k_scale)
        self.stale_s = g('wheel_stale_s', 0.5)
        self.extrap_max_s = g('wheel_extrap_max_s', 0.2)
        self.max_kmh = g('wheel_max_kmh', 100.0)
        self.min_kmh = g('wheel_min_kmh', -5.0)
        self.stuck_zero_mps = g('stuck_zero_other_mps', 0.5)
        self.frozen_s = g('frozen_s', 1.5)
        self.frozen_dv = g('frozen_other_dv_mps', 0.3)
        self.spike_mps = g('spike_kmh', 4.0) / self.k
        self.spike_acc = max(g('acc_max', 1.8), g('dec_max_consistent', 6.0))
        self.agree_abs = g('agree_abs_mps', 0.2)
        self.agree_rel = g('agree_rel', 0.03)
        self.slip_hold_s = g('slip_hold_s', 1.0)
        self.acc_flag = g('slip_acc_flag', 2.5)
        self.dec_flag = g('slip_dec_flag', self.acc_flag)
        self.acc_win_s = g('slip_acc_win_s', 0.3)
        self.sig = g('meas_sigma', 0.03)
        self.sig_single = g('meas_sigma_single', 0.045)
        self.sig_slip = g('meas_sigma_slip', 0.35)
        self.align = bool(getattr(params, 'wheel_align_other', True))
        self.anchor_s = g('slip_anchor_s', 3.0)
        self.lock_dec = g('lock_zero_dec', 1.0)
        self._b = {'front': _Bogie('front'), 'rear': _Bogie('rear')}
        self._hist = []                # fused (t, v) for the common-mode test, oldest first
        self._slip_until = -math.inf
        self._anchor = None            # (t, v) of the last full agreement
        self._slip_v0 = 0.0            # anchor speed at the start of the current slip episode
        self._lock_until = -math.inf   # end of the wheel-lock hold (fused speed stuck at 0)
        self._last_v = None
        self._last_state = None
        self._last_slip = False
        self.n_disagree = 0
        self.n_uncertain = 0
        self.n_accel_flag = 0
        self.n_lock = 0

    # ------------------------------------------------------------------ input
    def add_sample(self, bogie: str, t: float, raw_kmh: float) -> bool:
        """Store one raw sample (km/h) of 'front' or 'rear'; False if it was rejected."""
        b = self._b.get(bogie)
        if b is None:
            return False
        t = float(t)
        raw = float(raw_kmh)
        if (not math.isfinite(t) or not math.isfinite(raw) or raw > self.max_kmh
                or raw < self.min_kmh or (b.t1 is not None and t < b.t1)):
            b.rejected += 1
            return False
        v = raw / self.k if raw > 0.0 else 0.0
        o = self._b['rear' if bogie == 'front' else 'front']

        if b.t1 is not None and not b.spike_prev and t - b.t1 <= self.stale_s:
            tol = self.spike_mps + self.spike_acc * (t - b.t1)
            if abs(v - b.value_at(t, self.extrap_max_s)) > tol:
                confirmed = (o.t1 is not None and abs(t - o.t1) <= self.stale_s
                             and abs(v - o.value_at(t, self.extrap_max_s)) <= tol)
                if not confirmed:
                    b.spike_prev = True
                    b.spikes += 1
                    return False
        b.spike_prev = False

        if b.t1 is not None and t == b.t1:
            b.v1 = v                               # same stamp again: newest value wins
        else:
            b.t0, b.v0 = b.t1, b.v1
            b.t1, b.v1 = t, v
        if raw == 0.0 or raw != b.raw1:            # a new value starts a new frozen-run
            b.frz_t = t
            b.oth_lo = b.oth_hi = o.v1 if (o.t1 is not None and t - o.t1 <= self.stale_s) else None
        b.raw1 = raw
        b.n += 1
        if o.frz_t is not None:                    # track the other bogie's range during its run
            if o.oth_lo is None:
                o.oth_lo = o.oth_hi = v
            elif v < o.oth_lo:
                o.oth_lo = v
            elif v > o.oth_hi:
                o.oth_hi = v
        return True

    # ----------------------------------------------------------------- output
    def _bogie_at(self, b, t):
        """(value, fresh) of one bogie at t, before the cross-bogie rules."""
        if b.t1 is None:
            b.age = math.inf
            b.stale = True
            return NAN, False
        b.age = t - b.t1
        b.stale = b.age > self.stale_s
        if b.stale:
            return NAN, False
        return b.value_at(t, self.extrap_max_s), True

    def _is_frozen(self, b):
        return (b.raw1 != 0.0 and b.frz_t is not None and b.t1 - b.frz_t >= self.frozen_s
                and b.oth_lo is not None and b.oth_hi - b.oth_lo > self.frozen_dv)

    def measure(self, t: float, v_pred: float):
        """Fused speed measurement at t, or None if no bogie is valid (or t is not finite)."""
        t = float(t)
        if not math.isfinite(t):
            # a NaN time would pass every comparison as False: all bogies 'fresh', v = 0, and a
            # NaN entry in the acceleration history that is never trimmed again
            return None
        F = self._b['front']
        R = self._b['rear']
        fv, fok = self._bogie_at(F, t)
        rv, rok = self._bogie_at(R, t)

        F.frozen = fok and self._is_frozen(F)
        R.frozen = rok and self._is_frozen(R)
        fok = fok and not F.frozen
        rok = rok and not R.frozen
        F.stuck_zero = R.stuck_zero = False
        if fok and rok:
            if F.v1 == 0.0 and rv > self.stuck_zero_mps:
                F.stuck_zero, fok = True, False
            elif R.v1 == 0.0 and fv > self.stuck_zero_mps:
                R.stuck_zero, rok = True, False

        disagree = full_agree = False
        if fok and rok:
            tol = max(self.agree_abs, self.agree_rel * max(fv, rv))
            fa, ra = self._aligned(F, R, fv, rv, t) if self.align else (fv, rv)
            ok_own = abs(fv - rv) <= tol
            ok_al = abs(fa - ra) <= tol
            if ok_own and ok_al:
                v, sigma, state = 0.5 * (fa + ra), self.sig, 'both'
                full_agree = True
            elif ok_own or ok_al:
                # only one of the two comparisons fails: an extrapolation artefact of the older
                # bogie, or the first sample of a slip of the newer one.  The pair's second
                # sample (same stamp, < 1 ms later) resolves it; until then use the agreeing
                # mean with the slip sigma, without raising or holding the slip flag.
                v = 0.5 * (fa + ra) if ok_al else 0.5 * (fv + rv)
                sigma, state = self.sig_slip, 'both'
                self.n_uncertain += 1
            else:
                disagree = True
                self.n_disagree += 1
                ref = self._slip_reference(t, v_pred)
                if ref is None:
                    v = 0.5 * (fv + rv)
                else:
                    v = fv if abs(fv - ref) <= abs(rv - ref) else rv
                sigma, state = self.sig_slip, 'disagree'
            if v < 0.0:
                v = 0.0
        elif fok:
            v, sigma, state = fv, self.sig_single, 'single_front'
        elif rok:
            v, sigma, state = rv, self.sig_single, 'single_rear'
        else:
            self._last_state = None
            return None

        acc_bad = self._accel_implausible(t, v)       # always called: keeps the history
        if disagree or acc_bad:
            if t >= self._slip_until:                   # a new slip episode begins
                self._slip_v0 = self._anchor[1] if self._anchor is not None else 0.0
            self._slip_until = t + self.slip_hold_s
        elif full_agree:
            self._anchor = (t, v)
        slip = disagree or t < self._slip_until
        # wheel lock: the fused speed reaching zero during a slip episode is not trusted
        # for as long as a service stop from the pre-slip speed would take
        if v > 0.0:
            self._lock_until = -math.inf
        elif slip and self._lock_until == -math.inf and self._slip_v0 > 0.0 and self.lock_dec > 0.0:
            self._lock_until = t + self._slip_v0 / self.lock_dec
            self.n_lock += 1
        if t < self._lock_until:
            slip = True
        if slip:
            sigma = self.sig_slip
        self._last_v = v
        self._last_state = state
        self._last_slip = slip
        return Measurement(v, sigma, state, slip, fok, rok,
                           fv if fok else NAN, rv if rok else NAN)

    def _slip_reference(self, t, v_pred):
        """Reference speed for choosing a bogie in a disagreement.

        Within slip_anchor_s of the last full agreement: the speed of that agreement, held.
        A spinning bogie reads above and a sliding one below the true speed, and both move
        away from the last common value faster than the true speed does, so the bogie nearer
        the held value is the one that is not slipping ("lower in traction, higher in
        braking").  The anchor does not follow the fused output, so it cannot lock onto the
        slipping bogie; extrapolating it with the pre-disagreement acceleration was worse on
        the recordings, because that acceleration already contains half the slip.
        Later (or with no anchor): v_pred, else the last fused value, else None.
        """
        an = self._anchor
        if an is not None and 0.0 <= t - an[0] <= self.anchor_s:
            return an[1]
        if v_pred is not None and math.isfinite(v_pred):
            return v_pred
        return self._last_v

    def _aligned(self, F, R, fv, rv, t):
        """Replace the older bogie's own extrapolation by the newer bogie's change since then.

        The two bogies share header stamps but arrive one after the other, so at the first
        arrival the other bogie is ~0.1 s old.  Its own two-sample slope is noisy; the newer
        bogie, sampled at both instants, gives that change with less noise:
            v_old(t) = v_old(t1_old) + v_new(t) - v_new(t1_old),
        used when the newer bogie brackets t1_old (t0_new <= t1_old < t1_new <= t).
        """
        if F.t1 > R.t1:
            new, old, nv, ov = F, R, fv, rv
        elif R.t1 > F.t1:
            new, old, nv, ov = R, F, rv, fv
        else:
            return fv, rv
        if new.t0 is None or new.t0 > old.t1 or new.t1 > t:
            return fv, rv
        ov = old.v1 + nv - new.value_at(old.t1, self.extrap_max_s)
        return (nv, ov) if new is F else (ov, nv)

    def _accel_implausible(self, t, v):
        """Common-mode test: mean acceleration of the fused speed over >= slip_acc_win_s."""
        h = self._hist
        if h and t <= h[-1][0]:
            if t == h[-1][0]:
                h[-1] = (t, v)
            else:
                return False                       # out-of-order call: no history update
        else:
            h.append((t, v))
        cut = t - self.acc_win_s
        while len(h) > 2 and h[1][0] <= cut:
            h.pop(0)
        t_w, v_w = h[0]
        if t_w > cut:
            return False
        a = (v - v_w) / (t - t_w)
        if a > self.acc_flag or a < -self.dec_flag:
            self.n_accel_flag += 1
            return True
        return False

    # ------------------------------------------------------------ diagnostics
    def health(self) -> dict:
        out = {}
        for name, b in self._b.items():
            out[name] = {'stale': b.stale, 'stuck_zero': b.stuck_zero, 'frozen': b.frozen,
                         'age': b.age, 'n': b.n, 'rejected': b.rejected, 'spikes': b.spikes}
        out['state'] = self._last_state
        out['slip'] = self._last_slip
        out['n_disagree'] = self.n_disagree
        out['n_uncertain'] = self.n_uncertain
        out['n_accel_flag'] = self.n_accel_flag
        out['n_lock'] = self.n_lock
        return out
