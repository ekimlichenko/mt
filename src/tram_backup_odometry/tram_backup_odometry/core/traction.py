"""Notch-driven longitudinal dynamics of the tram (the model half of the filter).

The driver controller position (notch n in -15..+15) is the only control input.
All accelerations are in m/s^2, speeds in m/s, times in s.

Static maps (fitted offline on 68 unique bags, see docs/analysis_notes.md, D1/D2)
------------------------------------------------------------------------------
Traction (n > 0), a per-notch plateau capped by a common envelope that decays
slower than constant power above ~6 m/s:

    a_tr(n, v) = min(b0 + a0*n / (1 + max(v - v1, 0)/va),  A0 * min(1, vb/v)^pw) - r0

Service brake (n < 0), linear in |n| with a low-speed fade:

    a_br(n, v) = max(-(d0 + d1*|n|) * (1 - f*exp(-v/vf)),  cap) - r0

Notch ``hold8_notch`` (-8) is a closed-loop hold position whose acceleration is
not predictable from the notch (D6).  It is modelled as a = 0 above ``hold8_v``
and ``hold8_low_acc`` below it, with a large process noise.

Transients (D3)
---------------
Each branch sees the notch after a dead time (traction ``lag_tr_dead_s``, brake
``lag_br_dead_s``) and then a first-order lag.  The lag is integrated exactly
for a notch held constant over the step:

    z <- u + (z - u) * exp(-dt / tau)

The lag states hold the *un-gained* traction/brake levels ``u_tr``, ``u_br``;
the online gains multiply the lagged levels:

    a_ng = gain_tr * u_tr + gain_br * u_br + (-r0 if neither branch is active)

With constant gains this is identical to lagging the gained command (the
reference ``openloop_step``), and it makes d a / d gain = u directly available
to the estimator for gain adaptation.

Grade and curvature are applied by the estimator (they need the map position):

    a = a_ng + c_grade * 9.81 * grade(s) - curve_k * |curv(s)|

where c_grade is the mode's grade coefficient (traction/brake/coast; hold8 uses
``grade_c_hold8``, default = the brake value; 0 would mean the -8 hold loop
fully compensates the grade).

Standstill latch (D8)
---------------------
Motion starts ~1.2 s after the first traction notch.  The latch is active when
v < ``standstill_v`` and traction has not been held for ``start_delay_s``; the
estimator then forces v = 0 and a = 0 (unless the wheels say otherwise).

Without any command (none received yet, or the stream is older than
``cmd_stale_s``) the model runs in coast mode with the inflated process noise
``sigma_a_nocmd``.
"""
import bisect
import math
from dataclasses import dataclass


_MAX_UNSTEPPED = 1 << 16          # command buffer cap before the first step()


@dataclass
class ModelOut:
    a_ng: float       # model accel WITHOUT grade & curve terms (incl. gains), m/s^2
    c_grade: float    # grade coefficient of the active mode (a += c_grade * 9.81 * grade)
    sigma_a: float    # process noise std for this mode, m/s^2
    mode: str         # 'traction' | 'brake' | 'coast' | 'hold8' | 'standstill'
    latch: bool       # standstill latch active -> estimator forces a = 0 and v = 0
    u_tr: float       # lagged traction level before gain, m/s^2
    u_br: float       # lagged brake level before gain, m/s^2
    notch: int = 0    # notch in force at t (no dead time); 0 without a command
    has_cmd: bool = False


class TractionModel:
    """Traction/brake model with dead time, first-order lags and the standstill latch."""

    def __init__(self, params):
        p = params
        self.tr = (p.tr_b0, p.tr_a0, p.tr_v1, p.tr_va, p.tr_A0, p.tr_vb, p.tr_pw)
        self.br = (p.br_d0, p.br_d1, p.br_f, p.br_vf, p.br_cap)
        self.r0 = p.res_r0
        self.c_tr = p.grade_c_traction
        self.c_br = p.grade_c_brake
        self.c_coast = p.grade_c_coast
        self.c_hold8 = getattr(p, 'grade_c_hold8', p.grade_c_brake)
        self.dead_tr = p.lag_tr_dead_s
        self.tau_tr = p.lag_tr_tau_s
        self.dead_br = p.lag_br_dead_s
        self.tau_br = p.lag_br_tau_s
        self.hold8_notch = int(p.hold8_notch)
        self.hold8_v = p.hold8_v
        self.hold8_low_acc = p.hold8_low_acc
        self.start_delay = p.start_delay_s
        self.standstill_v = p.standstill_v
        self.sig = {'traction': p.sigma_a_traction, 'coast': p.sigma_a_coast,
                    'brake': p.sigma_a_brake, 'hold8': p.sigma_a_hold8,
                    'standstill': p.sigma_a_brake}
        self.sig_brake_high = p.sigma_a_brake_high
        self.sig_nocmd = getattr(p, 'sigma_a_nocmd', 0.5)
        self.cmd_stale = getattr(p, 'cmd_stale_s', 1.0)
        self.history_s = getattr(p, 'cmd_history_s', 3.0)
        self.gain_tr = p.gain_tr
        self.gain_br = p.gain_br
        # state
        self.zt = 0.0          # lagged traction level (un-gained)
        self.zb = 0.0          # lagged brake level (un-gained)
        self.hold_t = 0.0      # time traction has been held while below standstill_v
        self._ct = []          # command stamps, sorted
        self._cn = []          # notches, aligned with _ct
        self._alpha_cache = (None, 0.0, 0.0)
        self._t_step = None    # time of the last step (history pruning reference)

    # ------------------------------------------------------------ commands
    def add_cmd(self, t, notch):
        """Insert a command; late / out-of-order stamps are placed by time."""
        n = int(notch)
        ct = self._ct
        if not ct or t >= ct[-1]:
            ct.append(t)
            self._cn.append(n)
        else:
            i = bisect.bisect_right(ct, t)
            ct.insert(i, t)
            self._cn.insert(i, n)
        if len(ct) > 256:
            if self._t_step is not None:
                # keep the history window behind the model time plus the command in force at its start
                i = bisect.bisect_right(ct, min(ct[-1], self._t_step) - self.history_s) - 1
            elif len(ct) > _MAX_UNSTEPPED:
                i = len(ct) - _MAX_UNSTEPPED // 2      # never stepped: hard memory cap only
            else:
                i = 0
            if i > 0:
                del ct[:i]
                del self._cn[:i]

    def _cmd(self, t):
        """(notch, stamp) of the last command <= t, or (0, None)."""
        ct = self._ct
        if not ct:
            return 0, None
        if t >= ct[-1]:
            return self._cn[-1], ct[-1]
        i = bisect.bisect_right(ct, t) - 1
        if i < 0:
            return 0, None
        return self._cn[i], ct[i]

    def notch_at(self, t):
        """Notch in force at t (last command <= t, no dead time), 0 if none."""
        return self._cmd(t)[0]

    def has_cmd(self, t):
        _, tc = self._cmd(t)
        return tc is not None and t - tc <= self.cmd_stale

    # ------------------------------------------------------------ static maps
    def a_traction(self, n, v):
        b0, a0, v1, va, A0, vb, pw = self.tr
        plateau = b0 + a0 * n / (1.0 + max(v - v1, 0.0) / va)
        envelope = A0 * min(1.0, vb / max(v, 1e-3)) ** pw
        return min(plateau, envelope) - self.r0

    def a_brake(self, n, v):
        if n == self.hold8_notch:
            return 0.0 if v > self.hold8_v else self.hold8_low_acc
        d0, d1, f, vf, cap = self.br
        return max(-(d0 + d1 * abs(n)) * (1.0 - f * math.exp(-v / vf)), cap) - self.r0

    # ------------------------------------------------------------ propagation
    def _alphas(self, dt):
        c = self._alpha_cache
        if c[0] != dt:
            c = (dt, 1.0 - math.exp(-dt / self.tau_tr), 1.0 - math.exp(-dt / self.tau_br))
            self._alpha_cache = c
        return c[1], c[2]

    def _eval(self, t, dt, v, commit):
        n_now, tc = self._cmd(t)
        has = tc is not None and t - tc <= self.cmd_stale
        if has:
            n_tr = self._cmd(t - self.dead_tr)[0]
            n_br = self._cmd(t - self.dead_br)[0]
        else:
            n_now = n_tr = n_br = 0
        ut = self.a_traction(n_tr, v) if n_tr > 0 else 0.0
        ub = self.a_brake(n_br, v) if n_br < 0 else 0.0
        zt, zb = self.zt, self.zb
        if dt > 0.0:
            at, ab = self._alphas(dt)
            zt += at * (ut - zt)
            zb += ab * (ub - zb)
        hold_t = self.hold_t + dt if (n_tr > 0 and v < self.standstill_v) else 0.0
        latch = v < self.standstill_v and (n_tr <= 0 or hold_t < self.start_delay - 1e-6)
        if commit:
            self.zt, self.zb, self.hold_t = zt, zb, hold_t
            self._t_step = t

        traction, brake = n_tr > 0, n_br < 0
        a_ng = self.gain_tr * zt + self.gain_br * zb
        if not traction and not brake:
            a_ng -= self.r0
        if traction:
            c, mode = self.c_tr, 'traction'
        elif brake:
            if n_br == self.hold8_notch:
                c, mode = self.c_hold8, 'hold8'
            else:
                c, mode = self.c_br, 'brake'
        else:
            c, mode = self.c_coast, 'coast'
        if not has:
            sig = self.sig_nocmd
        elif mode == 'brake' and n_br <= -9:
            sig = self.sig_brake_high
        else:
            sig = self.sig[mode]
        if latch:
            mode = 'standstill'
        return ModelOut(a_ng, c, sig, mode, latch, zt, zb, n_now, has)

    def step(self, t, dt, v):
        """Advance the lag states from t - dt to t with the current speed estimate v."""
        return self._eval(t, max(dt, 0.0), v, True)

    def peek(self, t, v):
        """Model output at t without changing the state (dt = 0)."""
        return self._eval(t, 0.0, v, False)

    def reset_state(self):
        self.zt = self.zb = self.hold_t = 0.0
