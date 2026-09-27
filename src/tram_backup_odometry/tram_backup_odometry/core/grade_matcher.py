"""Grade matching (terrain-referenced navigation along the track).

The longitudinal dynamics contain a strong position-dependent term: gravity
along the track, ``c_grade * g * grade(s)``, up to ~0.35 m/s^2 on the ±35 per
mille ramps.  Comparing the measured acceleration with the model acceleration
therefore tells where on the map the tram is.  This module estimates the error
of the odometry-based route position ``s_route`` with a 2-D point-mass
(grid) filter over

    s_true = s_route + delta0 + eps * (s_route - s_anchor)

where ``delta0`` is the position error at the anchor (the first update) and
``eps`` the residual wheel-scale error.  The grid is regular in delta0
(``trn_delta_step_m`` over ±``trn_delta_range_m``) and eps (``trn_eps_step``
over ±``trn_eps_range``): about 241 x 31 = 7.5k cells.

Observation (every ``trn_update_every_m`` metres of travel)
------------------------------------------------------------
Over the causal window [t - W, t] (W = ``trn_win_s``) with samples (t_i, v_i):

* a_obs = least-squares slope of v_i over t_i.  The LS slope is a weighted
  mean of the true acceleration: slope = sum_i c_i v_i with
  c_i = (t_i - mean t) / S_tt, and for v_i = v_0 + int a, this equals
  int a(tau) K(tau) dtau with K(tau) = sum_{t_i > tau} c_i >= 0, sum = 1
  (a parabola-like kernel centred in the window).
* The model side is weighted with the *same* kernel, so both sides see the
  same time response:  a_ng_k = sum_j w_j a_ng(j),  c_k = sum_j w_j c(j),
  s_k = sum_j w_j s(j)  (w_j = K on sample interval j times its length,
  trapezoidal values).
* For every hypothesis h = (delta0, eps):
      s_h   = s_k + delta0 + eps * (s_k - s_anchor)
      pred  = a_ng_k + c_k * g * grade(s_h + lookahead)  [- curve_k*|curv(s_h + lookahead)|]
      r_h   = a_obs - pred
  ``lookahead`` (per direction) moves the grade point from the master
  antenna (the reference of s) to the effective centre of the vehicle mass.
  The grade profile is the route's (``grade_window_m`` smoothing) unless
  ``trn_grade_window_m`` > 0 gives the matcher its own moving-average window
  of dz/ds (a sharper profile carries more position information; the
  estimator keeps the route's smoother grade).
* Log-likelihood: truncated quadratic (robust to model outliers such as
  unannounced braking), tempered because the model residual is correlated
  over ``trn_corr_len_m`` (its integral correlation length on the data is
  ~20 m for a 1 s window); each update is weighted by the distance ds it
  represents (travel of s_k since the previous update, capped at corr_len):
      ll_h = -0.5 * (ds / corr_len) * min(r_h^2, trunc^2) / sigma^2
  so the total weight is (travelled distance)/corr_len independent samples,
  whatever the message rate and skipped windows.  sigma is ``trn_sigma``,
  or ``trn_sigma_t2s`` / ``trn_sigma_s2t`` when set (> 0) for the direction.
  Only corr_len * sigma^2 (and trunc) shape the posterior.
  Hypotheses whose grade point is off the map get the mean ll of the on-map
  ones (no information either way).
* Posterior: log p <- prior + forget * (log p - prior) + ll   (forget = 1: none),
  priors N(0, sigma0^2) on delta0 and N(0, trn_eps_prior^2) on eps.

No update while the inputs are unusable (``ok`` False anywhere in the window:
slip, bogie disagreement, hold8/standstill, strong notches), below
``trn_min_v``, or with s_route off the map.  Non-finite inputs empty the
window.  A stamp older than the newest one is ignored, unless it is older by
more than a window (clock jump, or the newest stamp was a forward glitch):
then the window restarts on the new clock.

Output
------
The posterior moments of delta0 and eps are cached after each update, so
``correction(s)`` is O(1):  D(s) = delta0 + eps * (s - s_anchor),
mean = E[delta0] + (s - a) E[eps],
var  = Var[delta0] + (s - a)^2 Var[eps] + 2 (s - a) Cov[delta0, eps].
"""
import math
from collections import deque

import numpy as np

from .track_map import _moving_average

G = 9.81


class GradeMatcher:
    """Point-mass filter over (delta0, eps): s_true = s_route + delta0 + eps*(s_route - s_anchor)."""

    def __init__(self, params, route, direction, sigma0):
        p = params
        self.route = route
        direction = str(direction).upper()
        self.enabled = bool(getattr(p, 'trn_enable', True))
        self.look = float(p.trn_lookahead_t2s_m if direction == 'T2S' else p.trn_lookahead_s2t_m)
        self.win = float(p.trn_win_s)
        self.every = float(p.trn_update_every_m)
        self.min_v = float(p.trn_min_v)
        # optional per-direction residual std (trn_sigma_t2s / trn_sigma_s2t), else trn_sigma
        sig_dir = getattr(p, 'trn_sigma_' + str(direction).lower(), None)
        self.sigma = float(sig_dir) if (sig_dir is not None and sig_dir > 0) else float(p.trn_sigma)
        self.trunc2 = float(p.trn_trunc) ** 2
        self.corr_len = max(float(p.trn_corr_len_m), 1e-6)
        self.forget = float(getattr(p, 'trn_forget', 1.0))
        self.apply_sigma = float(p.trn_apply_sigma_m)
        self.min_samples = int(getattr(p, 'trn_min_samples', 5))
        self.curve_k = float(p.curve_k) if getattr(p, 'trn_use_curv', True) else 0.0
        self.sigma0 = float(sigma0)
        if math.isnan(self.sigma0):
            self.sigma0 = math.inf                   # unknown: flat prior over the delta0 grid

        nd = max(0, int(round(p.trn_delta_range_m / p.trn_delta_step_m)))
        self.delta = np.arange(-nd, nd + 1) * float(p.trn_delta_step_m)
        if p.trn_eps_range > 0 and p.trn_eps_step > 0:
            ne = int(round(p.trn_eps_range / p.trn_eps_step))
            self.eps = np.arange(-ne, ne + 1) * float(p.trn_eps_step)
        else:
            self.eps = np.zeros(1)
        eps_prior = max(float(p.trn_eps_prior), 1e-9)
        self.prior = (-0.5 * (self.delta[None, :] / max(self.sigma0, 1e-3)) ** 2
                      - 0.5 * (self.eps[:, None] / eps_prior) ** 2)
        self.logp = self.prior.copy()

        # map profiles on the route grid.  The matcher may use its own grade smoothing
        # (trn_grade_window_m, moving average of dz/ds) instead of the route's grade_window_m.
        self._sg = route.sg
        gw = getattr(p, 'trn_grade_window_m', None)
        if gw is not None and gw > 0 and getattr(route, 'z', None) is not None:
            self._grade = _moving_average(np.gradient(np.asarray(route.z, float), route.ds), gw / route.ds)
        else:
            self._grade = np.asarray(route.grade, float)
        self._acurv = np.abs(np.asarray(route.curv, float))

        self._hist = deque()
        self._t_bad = -math.inf
        self.anchor = None
        self._s_prev = None
        self._s_next = -math.inf
        self._last_s = None
        self.n_updates = 0
        self.last_obs = None
        self._moments(self.logp)

    # ------------------------------------------------------------------ input
    def add(self, t, s_route, v_meas, a_ng, c_grade, ok):
        """Feed one wheel update (s_route: uncorrected route arc length of the reference point)."""
        if not (math.isfinite(t) and math.isfinite(s_route) and math.isfinite(v_meas)
                and math.isfinite(a_ng) and math.isfinite(c_grade)):
            self._hist.clear()
            self._t_bad = t if math.isfinite(t) else self._t_bad
            return
        self._last_s = s_route
        h = self._hist
        t_newest = max(h[-1][0], self._t_bad) if h else self._t_bad
        if t < t_newest - self.win:
            # clock jumped back by more than a window (or the newest stamp was a forward
            # glitch): restart the window on the new clock instead of waiting for it to catch up
            h.clear()
            self._t_bad = -math.inf
        elif h and t < h[-1][0]:
            return                                   # slightly out of order: ignore
        if h and t == h[-1][0]:
            h.pop()                                  # same stamp (the other bogie): keep the newest fusion
        h.append((t, s_route, v_meas, a_ng, c_grade))
        while h and t - h[0][0] > self.win + 1e-6:
            h.popleft()
        if not ok:
            self._t_bad = t
        if not self.enabled or s_route < self._s_next:
            return
        if t - self._t_bad <= self.win or v_meas < self.min_v:
            return
        if not (0.0 <= s_route <= self.route.L):
            return
        obs = self.window_obs()
        if obs is None:
            return
        self._s_next = s_route + self.every
        self.update(*obs)

    def window_obs(self):
        """(s_k, a_obs, a_ng_k, c_k) of the current window, or None if it is too thin."""
        h = self._hist
        n = len(h)
        if n < self.min_samples or h[-1][0] - h[0][0] < 0.8 * self.win:
            return None
        ts = [x[0] for x in h]
        tm = sum(ts) / n
        stt = sum((ti - tm) ** 2 for ti in ts)
        if stt <= 0.0:
            return None
        if min(x[2] for x in h) < self.min_v:
            return None
        slope = sum((x[0] - tm) * x[2] for x in h) / stt
        # kernel K_j = sum_{i>=j} c_i on interval (j-1, j); weights w_j = K_j * dt_j
        k = 0.0
        a_k = c_k = s_k = wsum = 0.0
        for j in range(n - 1, 0, -1):
            k += (h[j][0] - tm) / stt
            w = k * (h[j][0] - h[j - 1][0])
            a_k += w * (h[j][3] + h[j - 1][3])
            c_k += w * (h[j][4] + h[j - 1][4])
            s_k += w * (h[j][1] + h[j - 1][1])
            wsum += w
        if wsum <= 0.0:
            return None
        f = 0.5 / wsum
        return s_k * f, slope, a_k * f, c_k * f

    # ------------------------------------------------------------------ filter
    def update(self, s_k, a_obs, a_ng_k, c_k):
        """One likelihood update with a window observation."""
        if self.anchor is None:
            self.anchor = s_k
            ds = self.every
        else:
            ds = min(max(s_k - self._s_prev, 0.0), self.corr_len)
        self._s_prev = s_k
        pos = (s_k + self.look + self.delta[None, :]) + self.eps[:, None] * (s_k - self.anchor)
        g = np.interp(pos, self._sg, self._grade, left=np.nan, right=np.nan)
        pred = a_ng_k + (c_k * G) * g
        if self.curve_k > 0.0:
            pred -= self.curve_k * np.interp(pos, self._sg, self._acurv)
        r = a_obs - pred
        q = np.minimum(r * r, self.trunc2)
        valid = np.isfinite(q)
        if not valid.any():
            return
        if not valid.all():
            q[~valid] = q[valid].mean()
        ll = (-0.5 * (ds / self.corr_len) / self.sigma ** 2) * q
        if self.forget < 1.0:
            self.logp = self.prior + self.forget * (self.logp - self.prior) + ll
        else:
            self.logp += ll
        self.logp -= self.logp.max()
        self._moments(self.logp)
        self.n_updates += 1
        self.last_obs = (s_k, a_obs, a_ng_k, c_k)

    def _moments(self, logp):
        w = np.exp(logp)
        w /= w.sum()
        wd = w.sum(axis=0)            # marginal of delta0
        we = w.sum(axis=1)            # marginal of eps
        d, e = self.delta, self.eps
        md = float(wd @ d)
        me = float(we @ e)
        self._md = md
        self._me = me
        self._vd = max(float(wd @ (d * d)) - md * md, 0.0)
        self._ve = max(float(we @ (e * e)) - me * me, 0.0)
        self._cde = float(e @ w @ d) - md * me

    # ------------------------------------------------------------------ output
    def correction(self, s_route):
        """(mean, std, eps_mean) of s_true - s_route at s_route."""
        x = 0.0 if self.anchor is None else s_route - self.anchor
        var = self._vd + x * x * self._ve + 2.0 * x * self._cde
        return self._md + self._me * x, math.sqrt(max(var, 0.0)), self._me

    def converged(self):
        if self._last_s is None or self.n_updates == 0:
            return False
        return self.correction(self._last_s)[1] < self.apply_sigma

    def posterior(self):
        """Normalised posterior grid (eps x delta0), for diagnostics."""
        w = np.exp(self.logp)
        return w / w.sum()
