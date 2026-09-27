"""Speed / odometer EKF: notch-driven dynamic model + fused wheel speed.

State
-----
    x = [v, b, g_tr, g_br, s]

    v     speed along the track, m/s (>= 0)
    b     model-bias acceleration, first-order Gauss-Markov (tau = bias_tau_s,
          stationary std = bias_sigma), m/s^2
    g_tr  traction gain, random walk (gain_sigma / sqrt(s)), bounded [gain_min, gain_max]
    g_br  brake gain, idem
    s     odometer: distance integrated since reset(), m (NOT the route arc length)

Process model (sub-steps dt <= max_step_s)
------------------------------------------
    a     = g_tr*u_tr + g_br*u_br + r_coast                    (TractionModel -> a_ng)
            + c_grade * 9.81 * grade(route_offset + s + lookahead)
            - curve_k * |curv(route_offset + s + lookahead)|
            + b
    v'    = max(v + a*dt, 0)
    s'    = s + (v + v')/2 * dt            (exact stopping distance when v' hits 0)
    b'    = phi*b,  phi = exp(-dt/bias_tau_s)
    g'    = g

The integrator relates the odometer to the map with the attribute
``route_offset`` (s_route = route_offset + s); it may be changed at any time
(initialisation, TRN corrections) and only affects the grade/curvature lookup.

Jacobian (non-identity entries):  dv'/db = dt, dv'/dg_tr = u_tr*dt,
dv'/dg_br = u_br*dt, db'/db = phi, ds'/dv = dt, ds'/d(b, g) = dt/2 * dv'/d(b, g).
Process noise:  Q_vv = sigma_a(mode)^2 * accel_corr_s * dt  (white acceleration
with PSD sigma_a^2*accel_corr_s; its s-coupling dt^2/2, dt^3/3 is included),
Q_bb = bias_sigma^2 * (1 - phi^2), Q_gg = gain_sigma^2 * dt.

Standstill latch: when the model latches (v < standstill_v and traction not
held for start_delay_s) and the wheels do not report motion (no fresh
measurement, or the last one is below standstill_v), v and a are forced to 0.
When the model is latched but the wheels report motion (e.g. starts in the
closed-loop -8 mode) the model acceleration is dropped (a = b) and the process
noise is raised to ``sigma_a_unlatched``.
The latched covariance depends on whether the wheels confirm the standstill
(latch_needs_wheels):
  * a fresh wheel sample (age <= meas_fresh_s) below standstill_v: the v row/column
    of P is zeroed and P_vv = standstill_sigma_v^2;
  * no fresh wheel data (dropout): v = 0 but the v/s covariance is frozen at its
    value when the model stopped.  The model may stop early (over-predicted brake at
    low speed) while the tram still rolls, so the stop is not "known";
  * no fresh wheel data and the -8 hold notch (hold8_rest_grow): P is propagated
    with the hold8 noise as if moving (Q_vv and the s coupling ds'/dv = dt).  The
    -8 hold loop can start the tram without any traction notch (47 of 1503
    departures from standstill, 3.1 %, had no traction notch in the preceding 5 s;
    tools/calibration/standstill_hold8.py).

Measurement update (H = [1 0 0 0 0], z = fused wheel speed)
-----------------------------------------------------------
Rate-limit clip against the last posterior (v_p at t_p):

    z <- clip(z, v_p - dec*dt_p, v_p + acc_max*dt_p),   dt_p = max(t - t_p, clip_min_dt_s)
    dec = dec_max_consistent if (meas.state == 'both' and not meas.slip) else dec_max_slip

clip_min_dt_s (0.1 s) is the per-bogie sample period: two bogies interleave at
~10 Hz each, so the fused value can legitimately step by ~a*0.1 s between two
updates only 0.02-0.05 s apart; a 0.02 s floor clipped 3.9 % of all updates of a
simple bogie mean on real data (0.01 % with 0.1 s).

The clipped value keeps the measurement's own sigma (the wheel fusion already
inflates it in slip states; ``meas_sigma_clip`` can raise it further), so the
posterior follows an implausible measurement at the rate limit instead of
anchoring to the model.  Inflating every clipped value would make the next
measurement fall even further outside the window (positive feedback lock-out).
Measurements are never locked out: after long gaps dt_p is large (no clip), and
if clipping persists for clip_resync_s the raw measurement is accepted and P_vv
is re-inflated.

Gain states are updated only when gain_adapt, meas.state == 'both', no slip,
the mode is not hold8/standstill and the innovation is within gain_gate_sigma;
otherwise they are "consider" states (Schmidt update: gain entries of K are 0
and their covariance block is left unchanged).  The odometer can be corrected
by the speed innovation through the v-s cross covariance (s_correction):
after a wheel dropout the returning speed error implies the distance error
accumulated during the dropout.

Output uncertainty
------------------
    var_v = P_vv
    var_s = P_ss + (odo_scale_sigma * distance)^2

The second term is the systematic wheel-scale uncertainty (per-run scale spread
~0.3-1 %, docs D11/A2); it is a std growing linearly with distance, not a
random walk.
"""
import math

from .traction import TractionModel

G = 9.81
V, B, GT, GB, S = 0, 1, 2, 3, 4
N = 5


class SpeedEstimator:
    """EKF over [v, b, g_tr, g_br] plus the odometer s (see module docstring)."""

    def __init__(self, params, model: TractionModel, grade_fn=None, curv_fn=None, lookahead_m=0.0):
        p = params
        self.p = p
        self.model = model
        self.grade_fn = grade_fn
        self.curv_fn = curv_fn
        self.lookahead = float(lookahead_m)
        self.route_offset = 0.0          # s_route = route_offset + s (set by the integrator)
        self.curve_k = p.curve_k
        self.max_step = p.max_step_s
        self.max_gap = p.max_gap_s
        self.bias_tau = p.bias_tau_s
        self.bias_sigma = p.bias_sigma
        self.bias_on = bool(getattr(p, 'bias_enable', True))
        self.gain_adapt = bool(p.gain_adapt)
        self.gain_sigma = p.gain_sigma
        self.gain_min = p.gain_min
        self.gain_max = p.gain_max
        self.gain_sigma0 = getattr(p, 'gain_sigma0', 0.07)
        self.gain_gate = getattr(p, 'gain_gate_sigma', 3.0)
        self.accel_corr = getattr(p, 'accel_corr_s', 1.0)
        self.sig_unlatched = getattr(p, 'sigma_a_unlatched', p.sigma_a_hold8)
        self.gap_sigma_a = getattr(p, 'gap_sigma_a', 0.5)
        self.acc_max = p.acc_max
        self.dec_cons = p.dec_max_consistent
        self.dec_slip = p.dec_max_slip
        self.sig_clip = getattr(p, 'meas_sigma_clip', 0.0)
        self.clip_resync = getattr(p, 'clip_resync_s', 3.0)
        self.clip_min_dt = getattr(p, 'clip_min_dt_s', 0.1)
        self.standstill_v = p.standstill_v
        self.standstill_sigma = getattr(p, 'standstill_sigma_v', 0.01)
        self.meas_fresh = getattr(p, 'meas_fresh_s', p.wheel_stale_s)
        self.s_correction = bool(getattr(p, 's_correction', True))
        self.hold8_rest_grow = bool(getattr(p, 'hold8_rest_grow', True))
        self.latch_needs_wheels = bool(getattr(p, 'latch_needs_wheels', True))
        self.scale_sigma = getattr(p, 'odo_scale_sigma', 0.003)
        self.extrap_max = getattr(p, 'extrap_max_s', 1.0)
        self.init_sigma_v = getattr(p, 'init_sigma_v', 1.0)
        self.t = None
        self.reset(None, 0.0)

    # ------------------------------------------------------------ state
    def reset(self, t, v0):
        self.t = t
        v0 = float(v0)
        self.v = v0 if v0 > 0.0 else 0.0            # also maps NaN to 0
        self.b = 0.0
        self.g_tr = self.p.gain_tr
        self.g_br = self.p.gain_br
        self.s = 0.0
        self.dist = 0.0
        P = [[0.0] * N for _ in range(N)]
        P[V][V] = self.init_sigma_v ** 2
        P[B][B] = self.bias_sigma ** 2 if self.bias_on else 0.0
        g0 = self.gain_sigma0 ** 2 if self.gain_adapt else 0.0
        P[GT][GT] = P[GB][GB] = g0
        self.P = P
        self.last_a = 0.0
        self.last_out = None
        self._t_post = None        # last posterior (for the rate-limit clip)
        self._v_post = self.v
        self._t_z = None           # last measurement
        self._z = 0.0
        self._clip_since = None
        self._stopped = False

    @property
    def var_v(self):
        return self.P[V][V]

    @property
    def var_s(self):
        return self.P[S][S] + (self.scale_sigma * self.dist) ** 2

    @property
    def s_route(self):
        return self.route_offset + self.s

    # ------------------------------------------------------------ prediction
    def predict_to(self, t):
        """Propagate the state to t; returns the ModelOut of the last sub-step."""
        if self.t is None:
            self.reset(t, self.v)
        T = t - self.t
        if not T > 0.0:                 # backwards, equal or NaN stamp: no change
            if self.last_out is None:
                self.last_out = self.model.peek(self.t, self.v)
            return self.last_out
        if T > self.max_gap:
            self._gap(t, T)
            return self.last_out
        n = max(1, int(math.ceil(T / self.max_step - 1e-9)))
        dt = T / n
        t0 = self.t
        for k in range(1, n):
            self._substep(t0 + k * dt, dt)
        self._substep(t, dt)
        return self.last_out

    def _meas_fresh(self, t):
        return self._t_z is not None and t - self._t_z <= self.meas_fresh

    def _meas_moving(self, t):
        return self._t_z is not None and t - self._t_z <= self.meas_fresh and self._z >= self.standstill_v

    def _substep(self, tk, dt):
        m = self.model
        m.gain_tr = self.g_tr
        m.gain_br = self.g_br
        out = m.step(tk, dt, self.v)
        self.last_out = out
        self.t = tk
        phi = math.exp(-dt / self.bias_tau) if self.bias_on else 0.0
        if out.latch and not self._meas_moving(tk):
            self._stopped = True
            self.v = 0.0
            self.b *= phi
            self.last_a = 0.0
            if self._meas_fresh(tk) or not self.latch_needs_wheels:
                # a fresh wheel zero confirms the standstill
                self._propagate(dt, phi, 0.0, 0.0, 0.0, 0.0, 0.0, stopped=True)
            elif self.hold8_rest_grow and out.notch == self.model.hold8_notch:
                # -8 at rest with no wheel data: the hold loop may start the tram at any time
                # -> v stays 0 but P grows with the hold8 noise (incl. the s coupling)
                self._propagate(dt, phi, 0.0, 0.0, 0.0, out.sigma_a ** 2 * self.accel_corr, dt)
            else:
                # unconfirmed standstill (no wheel data): v = 0, the v/s covariance is frozen
                self._propagate(dt, phi, 0.0, 0.0, 0.0, 0.0, 0.0)
            return
        self._stopped = False
        if out.latch:
            # wheels report motion although the model is latched: model unusable
            a_model, ut, ub, sig = 0.0, 0.0, 0.0, max(out.sigma_a, self.sig_unlatched)
        else:
            a_model, ut, ub, sig = out.a_ng, out.u_tr, out.u_br, out.sigma_a
            if self.grade_fn is not None or self.curv_fn is not None:
                # float(): map lookups may return numpy scalars, which would turn the whole
                # state into numpy scalars (~2x slower per call); a NaN lookup is ignored.
                sr = self.route_offset + self.s + self.lookahead
                if self.grade_fn is not None:
                    g = float(self.grade_fn(sr))
                    if g == g:
                        a_model += out.c_grade * G * g
                if self.curv_fn is not None:
                    k = float(self.curv_fn(sr))
                    if k == k:
                        a_model -= self.curve_k * abs(k)
        a = a_model + self.b
        v0 = self.v
        v1 = v0 + a * dt
        if v1 < 0.0:
            ds = v0 * v0 / (-2.0 * a) if a < 0.0 else 0.0
            v1 = 0.0
        else:
            ds = 0.5 * (v0 + v1) * dt
        self.v = v1
        self.s += ds
        self.dist += ds
        self.last_a = a
        self.b *= phi
        fb = dt if self.bias_on else 0.0
        self._propagate(dt, phi, fb, ut * dt, ub * dt, sig * sig * self.accel_corr, dt)

    def _propagate(self, dt, phi, fb, fg_t, fg_b, q_v, dt_s, stopped=False):
        """P <- F P F^T + Q for the sparse Jacobian of the module docstring.

        fb, fg_t, fg_b = dv'/db, dv'/dg_tr, dv'/dg_br; dt_s = ds'/dv (0 when stopped);
        q_v = PSD of the white acceleration noise."""
        P = self.P
        h = 0.5 * dt_s      # ds'/d(dv') for the trapezoid
        # A = F P (row operations)
        r0, r1, r2, r3, r4 = P
        A0 = [r0[j] + fb * r1[j] + fg_t * r2[j] + fg_b * r3[j] for j in range(N)]
        A1 = [phi * x for x in r1]
        A4 = [r4[j] + dt_s * r0[j] + h * (fb * r1[j] + fg_t * r2[j] + fg_b * r3[j]) for j in range(N)]
        A = (A0, A1, r2, r3, A4)
        # P' = A F^T (column operations)
        new = []
        for Ai in A:
            c0 = Ai[0] + fb * Ai[1] + fg_t * Ai[2] + fg_b * Ai[3]
            new.append([c0, phi * Ai[1], Ai[2], Ai[3],
                        Ai[4] + dt_s * Ai[0] + h * (fb * Ai[1] + fg_t * Ai[2] + fg_b * Ai[3])])
        if self.bias_on:
            new[B][B] += self.bias_sigma ** 2 * (1.0 - phi * phi)
        if self.gain_adapt:
            qg = self.gain_sigma ** 2 * dt
            new[GT][GT] = min(new[GT][GT] + qg, self.gain_sigma0 ** 2)
            new[GB][GB] = min(new[GB][GB] + qg, self.gain_sigma0 ** 2)
        if stopped:
            for j in range(N):
                new[V][j] = new[j][V] = 0.0
            new[V][V] = self.standstill_sigma ** 2
        else:
            new[V][V] += q_v * dt
            new[S][V] += 0.5 * q_v * dt * dt_s
            new[V][S] += 0.5 * q_v * dt * dt_s
            new[S][S] += q_v * dt * dt_s * dt_s / 3.0
        self.P = new

    def _gap(self, t, T):
        """Input gap longer than max_gap_s: jump the time, keep v, inflate P."""
        m = self.model
        out = m.step(t, T, self.v)
        self.last_out = out
        self.t = t
        phi = math.exp(-T / self.bias_tau) if self.bias_on else 0.0
        self.b *= phi
        self.last_a = 0.0
        if out.latch and self.v == 0.0:
            self._stopped = True
            confirmed = self._meas_fresh(t) or not self.latch_needs_wheels
            self._propagate(T, phi, 0.0, 0.0, 0.0, 0.0, 0.0, stopped=confirmed)
            return
        self._stopped = False
        ds = self.v * T
        self.s += ds
        self.dist += ds
        # constant speed through the gap; the bias is not integrated
        self._propagate(T, phi, 0.0, 0.0, 0.0, self.gap_sigma_a ** 2 * T, T)

    # ------------------------------------------------------------ update
    def update(self, meas):
        """Wheel measurement at the filter time self.t; returns innovation info."""
        if meas is None or self.t is None:
            return {'used': False}
        t = self.t
        z_raw = float(meas.v)
        R = float(meas.sigma) ** 2
        if not (math.isfinite(z_raw) and math.isfinite(R)):
            return {'used': False}      # a NaN/inf would poison the state permanently
        z_raw = max(z_raw, 0.0)
        R = max(R, 1e-8)                # S > 0 even with a zero sigma at a collapsed P_vv
        consistent = meas.state == 'both' and not meas.slip
        # rate-limit clip against the last posterior
        z = z_raw
        clipped = False
        if self._t_post is not None:
            dtp = max(t - self._t_post, self.clip_min_dt)
            dec = self.dec_cons if consistent else self.dec_slip
            hi = self._v_post + self.acc_max * dtp
            lo = self._v_post - dec * dtp
            if z > hi:
                z, clipped = hi, True
            elif z < lo:
                z, clipped = max(lo, 0.0), True
        resync = False
        if clipped:
            if self._clip_since is None:
                self._clip_since = t
            if t - self._clip_since >= self.clip_resync:
                z, clipped, resync = z_raw, False, True
            else:
                R = max(R, self.sig_clip ** 2)
        else:
            self._clip_since = None
        v_pred = self.v
        innov = z - v_pred
        P = self.P
        if resync:
            P[V][V] += innov * innov
        S_ = P[V][V] + R
        out = self.last_out
        mode = out.mode if out is not None else 'coast'
        adapt = (self.gain_adapt and consistent and not clipped and not resync
                 and mode not in ('hold8', 'standstill') and not (out is not None and out.latch)
                 and innov * innov <= self.gain_gate ** 2 * S_)
        active = [True, True, adapt, adapt, self.s_correction]
        col = [P[i][V] for i in range(N)]
        K = [col[i] / S_ if active[i] else 0.0 for i in range(N)]
        self.v += K[V] * innov
        self.b += K[B] * innov
        self.g_tr += K[GT] * innov
        self.g_br += K[GB] * innov
        self.s += K[S] * innov
        for i in range(N):
            ci = col[i] / S_
            Pi = P[i]
            for j in range(N):
                if active[i] or active[j]:
                    Pi[j] -= ci * col[j]
        # bounds
        if self.g_tr < self.gain_min:
            self.g_tr = self.gain_min
        elif self.g_tr > self.gain_max:
            self.g_tr = self.gain_max
        if self.g_br < self.gain_min:
            self.g_br = self.gain_min
        elif self.g_br > self.gain_max:
            self.g_br = self.gain_max
        if self.v < 0.0:
            self.v = 0.0
        latched = out is not None and out.latch
        if latched and z < self.standstill_v:
            self.v = 0.0
            for j in range(N):
                P[V][j] = P[j][V] = 0.0
            P[V][V] = self.standstill_sigma ** 2
            self._stopped = True
        self._t_post = t
        self._v_post = self.v
        self._t_z = t
        self._z = z_raw
        return {'used': True, 'z': z, 'z_raw': z_raw, 'v_pred': v_pred, 'innov': innov,
                'sigma': math.sqrt(S_), 'nis': innov * innov / S_, 'clipped': clipped,
                'resync': resync, 'gain_update': adapt}

    # ------------------------------------------------------------ output
    def extrapolate(self, t_stamp):
        """(v, s) at t_stamp without changing the state: constant last acceleration,
        |t_stamp - t| limited to extrap_max_s, v >= 0 with the exact stop distance."""
        if self.t is None:
            return self.v, self.s
        dt = t_stamp - self.t
        if dt > self.extrap_max:
            dt = self.extrap_max
        elif dt < -self.extrap_max:
            dt = -self.extrap_max
        v, a = self.v, (0.0 if self._stopped else self.last_a)
        ve = v + a * dt
        if ve >= 0.0:
            return ve, self.s + 0.5 * (v + ve) * dt
        tz = -v / a                   # a != 0 here; same sign as dt
        return 0.0, self.s + 0.5 * v * tz
