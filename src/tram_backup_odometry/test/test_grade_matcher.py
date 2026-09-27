"""Unit tests of core/grade_matcher.py on a synthetic grade profile.

A straight 4 km route gets a random smooth height profile (grades up to ~35
per mille).  A tram is simulated along it with a speed profile that sweeps
6..14 m/s; its acceleration is  a = a_ng + c * g * grade(s + lookahead)  plus
correlated model noise.  The odometry has an injected initial offset and a
wheel-scale error; the matcher must recover both.

Runs under pytest or standalone:  python test_grade_matcher.py
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tram_backup_odometry.core.config import Params                # noqa: E402
from tram_backup_odometry.core.grade_matcher import GradeMatcher    # noqa: E402
from tram_backup_odometry.core.track_map import Route               # noqa: E402

G = 9.81
LOOK = 10.0
C_GRADE = -0.9


def make_route(seed=1, length=4000.0):
    rng = np.random.default_rng(seed)
    n = int(length) + 1
    # smooth random grade: white noise low-passed twice with a 60 m moving average
    g = rng.normal(0.0, 1.0, n + 400)
    k = np.ones(60) / 60.0
    g = np.convolve(np.convolve(g, k, 'same'), k, 'same')[200:200 + n]
    g *= 0.035 / np.abs(g).max()
    z = 150.0 + np.cumsum(g)
    pts = [dict(x=100.0 + i, y=200.0, z=float(z[i]), curv=0.0) for i in range(n)]
    return Route('T2S', pts, grade_window_m=11.0)


def simulate(route, delta0, eps, seed=2, noise=0.04, dt=0.1, s_start=20.0):
    """Returns a list of add() arguments: (t, s_route, v_meas, a_ng, c_grade, ok) and the truth."""
    rng = np.random.default_rng(seed)
    t, s, v = 0.0, s_start, 8.0
    e_model = 0.0
    rows = []
    truth = []
    odo = 0.0
    while s < route.L - 60.0:
        a_ng = 0.35 * math.sin(0.05 * t) + 0.15 * math.sin(0.23 * t)
        if v > 14.0:
            a_ng -= 0.3
        if v < 6.0:
            a_ng += 0.3
        e_model = 0.8 * e_model + rng.normal(0.0, noise * math.sqrt(1 - 0.8 ** 2))
        a = a_ng + C_GRADE * G * route.grade_at(s + LOOK) + e_model
        v = max(v + a * dt, 0.5)
        s += v * dt
        t += dt
        v_meas = v / (1.0 + eps)            # wheel scale error: odometry = true distance / (1 + eps)
        odo += v_meas * dt
        s_route = s_start - delta0 + odo    # s_true - s_route = delta0 + eps * odo
        rows.append((t, s_route, v_meas + rng.normal(0.0, 0.01), a_ng, C_GRADE, True))
        truth.append(s)
    return rows, np.array(truth)


def params(**kw):
    p = Params()
    p.trn_lookahead_t2s_m = LOOK
    p.trn_sigma = 0.05
    p.trn_trunc = 0.2
    p.trn_corr_len_m = 10.0
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def run(route, rows, p, sigma0=30.0):
    m = GradeMatcher(p, route, 'T2S', sigma0)
    for r in rows:
        m.add(*r)
    return m


def test_recovers_shift_and_scale():
    route = make_route()
    delta0, eps = 23.0, 0.007
    rows, truth = simulate(route, delta0, eps)
    m = run(route, rows, params())
    assert m.n_updates > 1000
    s_route = rows[-1][1]
    mean, std, eps_hat = m.correction(s_route)
    err = s_route + mean - truth[-1]
    assert abs(err) < 3.0, (err, std)
    assert std < 4.0
    assert abs(eps_hat - eps) < 0.002, eps_hat
    assert m.converged()
    # the correction also holds in the middle of the run (uses eps)
    k = len(rows) // 2
    mid, _, _ = m.correction(rows[k][1])
    assert abs(rows[k][1] + mid - truth[k]) < 3.0


def test_delta_only_grid():
    route = make_route(seed=3)
    rows, truth = simulate(route, -17.0, 0.0, seed=4)
    m = run(route, rows, params(trn_eps_range=0.0), sigma0=25.0)
    assert m.eps.shape == (1,)
    mean, std, eps_hat = m.correction(rows[-1][1])
    assert abs(rows[-1][1] + mean - truth[-1]) < 3.0
    assert eps_hat == 0.0


def test_no_update_when_unusable():
    route = make_route()
    rows, _ = simulate(route, 10.0, 0.0)
    p = params()
    # ok False everywhere
    m = run(route, [r[:5] + (False,) for r in rows], p)
    assert m.n_updates == 0
    mean, std, eps_hat = m.correction(1000.0)
    assert abs(mean) < 1e-9 and abs(eps_hat) < 1e-12 and abs(std - 30.0) < 0.1, (mean, std, eps_hat)
    assert not m.converged()
    # standstill / slow: below trn_min_v
    m = run(route, [(r[0], r[1], 0.5 * p.trn_min_v, r[3], r[4], True) for r in rows], p)
    assert m.n_updates == 0
    # off the map
    m = run(route, [(r[0], r[1] + route.L + 10.0, r[2], r[3], r[4], True) for r in rows], p)
    assert m.n_updates == 0
    # a single bad sample blocks updates for one window only
    bad = [r if i != 500 else r[:5] + (False,) for i, r in enumerate(rows)]
    m1 = run(route, rows, p)
    m2 = run(route, bad, p)
    assert 0 < m1.n_updates - m2.n_updates <= 12


def test_update_spacing_and_cost():
    route = make_route()
    rows, _ = simulate(route, 0.0, 0.0)
    p = params()
    m = run(route, rows, p)
    travel = rows[-1][1] - rows[0][1]
    # one update per trn_update_every_m plus the overshoot of the message spacing (< 1.5 m here)
    assert travel / (p.trn_update_every_m + 1.5) < m.n_updates <= travel / p.trn_update_every_m + 1
    n = 300
    t0 = time.perf_counter()
    for i in range(n):
        m.update(1000.0 + 2 * i, 0.1, 0.0, C_GRADE)
    per = (time.perf_counter() - t0) / n
    assert per < 5e-3, per      # target < 1 ms on the vehicle PC; generous bound for CI


def test_per_direction_sigma():
    route = make_route()
    p = params()
    assert GradeMatcher(p, route, 'T2S', 10.0).sigma == p.trn_sigma      # no per-direction value
    p.trn_sigma_t2s = 0.09
    p.trn_sigma_s2t = 0.05
    assert GradeMatcher(p, route, 'T2S', 10.0).sigma == 0.09
    assert GradeMatcher(p, route, 'S2T', 10.0).sigma == 0.05
    p.trn_sigma_s2t = 0.0                                                  # <= 0: fall back
    assert GradeMatcher(p, route, 'S2T', 10.0).sigma == p.trn_sigma


def test_own_grade_window():
    route = make_route()                                   # route grade smoothed over 11 m
    p = params()
    p.trn_grade_window_m = 0.0                             # 0: reuse the route's grade
    assert np.array_equal(GradeMatcher(p, route, 'T2S', 10.0)._grade, route.grade)
    p.trn_grade_window_m = 11.0
    assert np.allclose(GradeMatcher(p, route, 'T2S', 10.0)._grade, route.grade)
    p.trn_grade_window_m = 3.0
    g3 = GradeMatcher(p, route, 'T2S', 10.0)._grade
    assert g3.shape == route.grade.shape and not np.allclose(g3, route.grade)


def test_window_obs_matches_ls_slope():
    route = make_route()
    p = params()
    m = GradeMatcher(p, route, 'T2S', 10.0)
    # v = 3 + 0.5 t, a_ng constant: a_obs must be exactly 0.5, a_ng_k exactly a_ng
    for i in range(11):
        t = 0.1 * i
        m.add(t, 100.0 + 3 * t + 0.25 * t * t, 3.0 + 0.5 * t, -0.2, C_GRADE, True)
    s_k, a_obs, a_ng_k, c_k = m.window_obs()
    assert abs(a_obs - 0.5) < 1e-9 and abs(a_ng_k + 0.2) < 1e-9 and abs(c_k - C_GRADE) < 1e-9
    assert 100.0 < s_k < 100.0 + 3.0 + 0.25


def test_bad_stamps_and_values():
    route = make_route()
    rows, truth = simulate(route, 23.0, 0.007)
    p = params()
    n_ref = run(route, rows, p).n_updates
    k = len(rows) // 3

    def check(feed, min_frac):
        m = run(route, feed, p)
        mean, std, eps_hat = m.correction(rows[-1][1])
        assert math.isfinite(mean) and math.isfinite(std) and math.isfinite(eps_hat)
        assert m.n_updates > min_frac * n_ref, (m.n_updates, n_ref)
        assert abs(rows[-1][1] + mean - truth[-1]) < 3.0
    # non-finite values sprinkled in
    feed = [r if i % 97 else (r[0], r[1], float('nan'), r[3], r[4], r[5]) for i, r in enumerate(rows)]
    feed = [r if i % 89 else (r[0], r[1], r[2], float('inf'), r[4], r[5]) for i, r in enumerate(feed)]
    check(feed, 0.7)                  # each non-finite sample costs one window refill
    # a single forward-glitched stamp (+100 s, ok False) must not stall the matcher until the clock catches up
    check(rows[:k] + [(rows[k][0] + 100.0,) + rows[k][1:5] + (False,)] + rows[k:], 0.97)
    check(rows[:k] + [(rows[k][0] + 100.0, rows[k][1], float('nan')) + rows[k][3:]] + rows[k:], 0.97)
    # a backward clock jump restarts the window on the new clock
    check(rows[:k] + [(r[0] - 1000.0,) + r[1:] for r in rows[k:]], 0.97)
    # small arrival-order swaps are ignored
    feed = list(rows)
    for i in range(10, len(feed) - 1, 37):
        feed[i], feed[i + 1] = feed[i + 1], feed[i]
    check(feed, 0.95)


def test_sigma0_and_direction_inputs():
    route = make_route()
    p = params(trn_lookahead_s2t_m=3.0)
    m = GradeMatcher(p, route, 'T2S', float('nan'))       # unknown sigma0: flat prior, not NaN
    mean, std, _ = m.correction(0.0)
    assert abs(mean) < 1e-9 and math.isfinite(std) and std > 60.0
    assert GradeMatcher(p, route, 't2s', 10.0).look == p.trn_lookahead_t2s_m
    assert GradeMatcher(p, route, 'S2T', 10.0).look == 3.0


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
