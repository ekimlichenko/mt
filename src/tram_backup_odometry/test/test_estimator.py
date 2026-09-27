"""Unit tests of core/estimator.py (speed / odometer EKF).

Runs under pytest or standalone:  python test_estimator.py
"""
import math
import os
import sys
import time
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tram_backup_odometry.core.config import Params                   # noqa: E402
from tram_backup_odometry.core.estimator import SpeedEstimator        # noqa: E402
from tram_backup_odometry.core.traction import TractionModel          # noqa: E402

DT = 0.05
T0 = 1_700_000_000.0


@dataclass
class Meas:
    v: float
    sigma: float = 0.03
    state: str = 'both'
    slip: bool = False


def make(grade_fn=None, curv_fn=None, lookahead=0.0, **over):
    p = Params()
    for k, v in over.items():
        setattr(p, k, v)
    m = TractionModel(p)
    return p, m, SpeedEstimator(p, m, grade_fn, curv_fn, lookahead)


def cmds(m, t0, t1, notch, dt=DT):
    t = t0
    while t <= t1 + 1e-9:
        m.add_cmd(t, notch)
        t += dt


def run(est, t0, t1, dt=DT, meas_fn=None):
    t = t0
    for k in range(1, int(round((t1 - t0) / dt)) + 1):
        t = t0 + k * dt
        est.predict_to(t)
        if meas_fn is not None:
            z = meas_fn(t)
            if z is not None:
                est.update(z)
    return t


def _ref_step(p, v, n_tr, n_br, zt, zb, hold_t, grade, curv, dt=DT):
    """openloop_step of docs/analysis_notes.md (reference propagation), returns (..., a)."""
    def a_tr(n, v):
        return min(p.tr_b0 + p.tr_a0 * n / (1 + max(v - p.tr_v1, 0) / p.tr_va),
                   p.tr_A0 * min(1.0, p.tr_vb / max(v, 1e-3)) ** p.tr_pw) - p.res_r0

    def a_br(n, v):
        if n == p.hold8_notch:
            return 0.0 if v > p.hold8_v else p.hold8_low_acc
        return max(-(p.br_d0 + p.br_d1 * abs(n)) * (1 - p.br_f * math.exp(-v / p.br_vf)), p.br_cap) - p.res_r0
    ut = a_tr(n_tr, v) if n_tr > 0 else 0.0
    ub = a_br(n_br, v) if n_br < 0 else 0.0
    zt += (1 - math.exp(-dt / p.lag_tr_tau_s)) * (ut - zt)
    zb += (1 - math.exp(-dt / p.lag_br_tau_s)) * (ub - zb)
    if n_tr > 0:
        c = p.grade_c_traction
    elif n_br == p.hold8_notch:
        c = p.grade_c_brake
    elif n_br < 0:
        c = p.grade_c_brake
    else:
        c = p.grade_c_coast
    coast = -p.res_r0 if (n_tr <= 0 and n_br >= 0) else 0.0
    a = zt + zb + coast + c * 9.81 * grade - p.curve_k * abs(curv)
    hold_t = hold_t + dt if (n_tr > 0 and v < p.standstill_v) else 0.0
    if v < p.standstill_v and (n_tr <= 0 or hold_t < p.start_delay_s - 1e-6):
        return 0.0, zt, zb, hold_t, None                    # latched: no motion in this step
    return max(v + a * dt, 0.0), zt, zb, hold_t, a


def test_equivalent_to_reference_without_measurements():
    """gains = 1, no measurements, 20 Hz, constant grade / curvature: v and s equal the
    reference openloop_step propagation (the bias stays 0 without measurements)."""
    import random
    random.seed(11)
    p, m, est = make(grade_fn=lambda s: 0.012, curv_fn=lambda s: 0.004)
    est.reset(0.0, 0.0)
    m.add_cmd(0.0, 0)
    n, v, zt, zb, hold, s = 0, 0.0, 0.0, 0.0, 0.0, 0.0
    moved = 0.0
    for k in range(1, 12000):
        t = k * DT
        if random.random() < 0.08:
            n = max(-15, min(15, n + random.choice([-2, -1, 1, 1, 2])))
        if random.random() < 0.003:
            n = 0
        m.add_cmd(t, n)
        n_tr, n_br = m.notch_at(t - p.lag_tr_dead_s), m.notch_at(t - p.lag_br_dead_s)
        v0 = v
        v, zt, zb, hold, a = _ref_step(p, v, n_tr, n_br, zt, zb, hold, 0.012, 0.004)
        if a is None:
            pass
        elif v0 + a * DT < 0.0:
            s += v0 * v0 / (-2.0 * a)                        # exact stopping distance
        else:
            s += 0.5 * (v0 + v) * DT
        est.predict_to(t)
        assert abs(est.v - v) < 1e-9, (k, est.v, v)
        assert abs(est.s - s) < 1e-6, (k, est.s, s)
        moved = max(moved, v)
    assert moved > 5.0


def test_latch_release_after_start_delay():
    p, m, est = make()
    est.reset(T0, 0.0)
    m.add_cmd(T0, 0)
    cmds(m, T0 + 1.0, T0 + 6.0, 4)
    t, t_move = T0, None
    while t < T0 + 5.0:
        t += DT
        est.predict_to(t)
        if est.v > 0.0 and t_move is None:
            t_move = t - T0
    expected = 1.0 + p.lag_tr_dead_s + p.start_delay_s
    assert abs(t_move - expected) <= 2 * DT, t_move
    assert est.v > 1.0                          # accelerating after release
    assert est.last_out.mode == 'traction'


def test_latch_forces_zero_without_motion_and_yields_to_wheels():
    p, m, est = make()
    est.reset(T0, 0.0)
    cmds(m, T0, T0 + 10.0, -15)
    run(est, T0, T0 + 2.0, meas_fn=lambda t: Meas(0.0))
    assert est.v == 0.0 and est.last_out.latch
    assert est.var_v <= 0.02 ** 2
    # wheels report motion while the model is latched (e.g. a -8 hop start): follow the wheels
    run(est, T0 + 2.0, T0 + 4.0, meas_fn=lambda t: Meas(0.8 * (t - T0 - 2.0)))
    assert abs(est.v - 1.6) < 0.15, est.v


def test_standstill_covariance_needs_wheel_confirmation():
    """Latched at rest: v is held at 0.  P collapses only when a fresh wheel zero confirms
    the standstill; without wheel data it is frozen (service brake) or grows (-8 hold)."""
    # -8 at rest, wheels lost: P grows (the hold loop can start the tram)
    p, m, est = make()
    cmds(m, T0 - 3.0, T0 + 20.0, p.hold8_notch)
    est.reset(T0 - 3.0, 0.0)
    run(est, T0 - 3.0, T0, meas_fn=lambda t: Meas(0.0))
    assert est.v == 0.0 and est.var_v <= 0.01 ** 2 + 1e-15       # standstill_sigma_v default
    s0 = est.var_s
    run(est, T0, T0 + 10.0)                         # wheels lost (> meas_fresh_s)
    assert est.v == 0.0 and est.last_out.latch
    assert est.var_v > 0.5 * p.sigma_a_hold8 ** 2 * 9.0, est.var_v
    assert est.var_s > s0 + 10.0
    est.update(Meas(0.0))                           # a wheel zero confirms the standstill
    assert est.v == 0.0 and est.var_v <= 0.01 ** 2 + 1e-15
    # service brake: the model brakes to a stop without wheel data -> P frozen at the stop
    p, m, est = make()
    cmds(m, T0 - 3.0, T0 + 30.0, -6)
    est.reset(T0 - 3.0, 3.0)
    run(est, T0 - 3.0, T0, meas_fn=lambda t: Meas(max(3.0 - 1.0 * (t - T0 + 3.0), 0.0) + 0.3))
    run(est, T0, T0 + 10.0)                         # wheels lost; the model stops
    assert est.v == 0.0 and est.last_out.latch
    var_stop, vs_stop = est.var_v, est.var_s
    assert var_stop > 0.05 ** 2                     # not the standstill sigma
    run(est, est.t, T0 + 20.0)
    assert est.var_v == var_stop and abs(est.var_s - vs_stop) < 1e-9
    est.update(Meas(0.0))
    assert est.var_v <= 0.01 ** 2 + 1e-15
    # with wheels present the standstill is confirmed at once
    p, m, est = make()
    cmds(m, T0 - 3.0, T0 + 20.0, -5)
    est.reset(T0 - 3.0, 0.0)
    run(est, T0 - 3.0, T0 + 5.0, meas_fn=lambda t: Meas(0.0))
    assert est.v == 0.0 and est.var_v <= 0.01 ** 2 + 1e-15
    # switched off: the latch alone collapses P (reference behaviour)
    p, m, est = make(latch_needs_wheels=False)
    cmds(m, T0 - 3.0, T0 + 20.0, p.hold8_notch)
    est.reset(T0 - 3.0, 0.0)
    run(est, T0 - 3.0, T0 + 10.0)
    assert est.var_v <= 0.01 ** 2 + 1e-15


def test_hold8_speed_hold_without_measurements():
    p, m, est = make()
    cmds(m, T0 - 3.0, T0 + 20.0, p.hold8_notch)
    est.reset(T0 - 3.0, 6.0)
    run(est, T0 - 3.0, T0, meas_fn=lambda t: Meas(6.0))
    var0 = est.var_v
    run(est, T0, T0 + 10.0)
    assert est.last_out.mode == 'hold8'
    assert abs(est.v - 6.0) < 0.02                # a = 0 above hold8_v on level track
    # hold8 process noise is much larger than the brake noise
    p2, m2, est2 = make()
    cmds(m2, T0 - 3.0, T0 + 20.0, -4)
    est2.reset(T0 - 3.0, 6.0)
    run(est2, T0 - 3.0, T0, meas_fn=lambda t: Meas(6.0 - 0.7 * (t - T0 + 3.0)))
    var0b = est2.var_v
    run(est2, T0, T0 + 2.0)
    assert (est.var_v - var0) / 10.0 > 5 * (est2.var_v - var0b) / 2.0


def test_lag_step_response_of_filter_acceleration():
    """Coast -> notch +3 in the low-speed plateau: after the dead time the traction
    level rises from 0 as 1 - exp(-t/tau) (the coast resistance drops out at the switch)."""
    p, m, est = make()
    m.add_cmd(T0 - 5.0, 0)
    cmds(m, T0, T0 + 5.0, 3)
    est.reset(T0 - 1.0, 1.0)
    run(est, T0 - 1.0, T0)
    assert abs(est.last_a + p.res_r0) < 1e-12
    t_dead = T0 + p.lag_tr_dead_s
    run(est, T0, t_dead + p.lag_tr_tau_s, dt=0.01)
    a_tau = est.last_a
    run(est, est.t, T0 + 3.0, dt=0.01)
    a_inf = est.last_a
    assert est.v < p.tr_v1                              # still on the plateau
    assert abs(a_inf - m.a_traction(3, est.v)) < 1e-3
    assert abs(a_tau / a_inf - (1 - math.exp(-1))) < 0.03, a_tau / a_inf


def test_gain_adaptation_and_bounds():
    # the "true" tram accelerates 2x the model in traction -> g_tr grows; with the bias state
    # off the gain must carry the whole mismatch and saturates at gain_max
    for bias_on in (True, False):
        p, m, est = make(gain_sigma=0.05, bias_enable=bias_on)
        cmds(m, T0 - 5.0, T0 + 200.0, 3)
        a_true = 2.0 * m.a_traction(3, 2.0)
        est.reset(T0, 2.0)
        state = {'v': 2.0, 't': T0}

        def meas(t):
            state['v'] += a_true * (t - state['t'])
            state['t'] = t
            if state['v'] > 3.0:            # stay in the low-speed plateau
                state['v'] = 2.0
                est.v = 2.0
            return Meas(state['v'])
        run(est, T0, T0 + 120.0, meas_fn=meas)
        assert est.g_tr > 1.0, (bias_on, est.g_tr)     # the bias state absorbs part of it
        assert est.g_tr <= p.gain_max + 1e-12
        assert p.gain_min <= est.g_br <= p.gain_max
        if not bias_on:
            assert est.g_tr > p.gain_max - 0.02, est.g_tr
            assert est.b == 0.0


def test_no_gain_update_on_slip_or_hold8():
    for kw in ({'slip': True}, {'state': 'single_front'}):
        p, m, est = make(gain_sigma=0.05)
        cmds(m, T0 - 5.0, T0 + 30.0, 5)
        est.reset(T0, 3.0)
        run(est, T0, T0 + 20.0, meas_fn=lambda t, kw=kw: Meas(3.0 + 1.2 * (t - T0), **kw))
        assert est.g_tr == p.gain_tr and est.g_br == p.gain_br
    p, m, est = make(gain_sigma=0.05)
    cmds(m, T0 - 5.0, T0 + 30.0, p.hold8_notch)
    est.reset(T0, 6.0)
    run(est, T0, T0 + 20.0, meas_fn=lambda t: Meas(6.0 + 0.1 * (t - T0)))
    assert est.g_tr == p.gain_tr and est.g_br == p.gain_br


def test_gap_handling():
    p, m, est = make()
    cmds(m, T0 - 2.0, T0, 0)
    est.reset(T0 - 2.0, 8.0)
    run(est, T0 - 2.0, T0, meas_fn=lambda t: Meas(8.0))
    v0, s0, var0, t_before = est.v, est.s, est.var_v, est.t
    gap = p.max_gap_s + 5.0
    est.predict_to(t_before + gap)
    assert est.t == t_before + gap
    assert est.v == v0                              # speed kept through the gap
    assert abs(est.s - (s0 + v0 * gap)) < 1e-4
    assert est.var_v > var0 + (0.3 * gap) ** 2      # strongly inflated
    info = est.update(Meas(5.0))
    assert not info['clipped']                      # never locked out after a gap
    assert abs(est.v - 5.0) < 0.05


def test_clip_consistent_vs_disagreeing_bogies():
    def drop(state, slip):
        p, m, est = make()
        cmds(m, T0 - 3.0, T0 + 5.0, 0)
        est.reset(T0 - 3.0, 10.0)
        run(est, T0 - 3.0, T0, meas_fn=lambda t: Meas(10.0))
        # measured speed falls at 5 m/s^2 for 1 s
        run(est, T0, T0 + 1.0,
            meas_fn=lambda t: Meas(10.0 - 5.0 * (t - T0), sigma=0.03 if state == 'both' else 0.35,
                                   state=state, slip=slip))
        return est
    est_ok = drop('both', False)                # genuine emergency braking: followed
    assert abs(est_ok.v - 5.0) < 0.5, est_ok.v   # (lag of an unmodelled 5 m/s^2 decel)
    est_slip = drop('disagree', True)           # slide suspected: clipped, the model dominates
    assert est_slip.v > 7.5, est_slip.v
    info = est_slip.update(Meas(4.0, sigma=0.35, state='disagree', slip=True))
    assert info['clipped'] and info['z'] > 4.0


def test_clip_resync_never_locks_out():
    p, m, est = make(clip_resync_s=2.0)
    cmds(m, T0 - 3.0, T0 + 10.0, 0)
    est.reset(T0 - 3.0, 10.0)
    run(est, T0 - 3.0, T0, meas_fn=lambda t: Meas(10.0))
    run(est, T0, T0 + 3.0, meas_fn=lambda t: Meas(3.0, sigma=0.35, state='disagree', slip=True))
    assert abs(est.v - 3.0) < 0.3, est.v


def test_extrapolate_symmetry_and_stop():
    p, m, est = make()
    cmds(m, T0 - 3.0, T0 + 3.0, 5)
    est.reset(T0 - 3.0, 5.0)
    run(est, T0 - 3.0, T0)
    v, s, a, t = est.v, est.s, est.last_a, est.t
    assert a > 0.1
    tp, tm = t + 0.3, t - 0.3
    dp, dm = tp - t, tm - t                                 # exact float offsets at 1.7e9
    vp, sp = est.extrapolate(tp)
    vm, sm = est.extrapolate(tm)
    assert abs((vp - v) / dp - a) < 1e-9 and abs((vm - v) / dm - a) < 1e-9
    assert abs((sp - s) - (v * dp + 0.5 * a * dp * dp)) < 1e-9
    assert abs((sm - s) - (v * dm + 0.5 * a * dm * dm)) < 1e-9
    assert abs((sp - s) + (sm - s) - a * 0.09) < 1e-6      # only the 2nd-order term survives
    assert est.v == v and est.s == s and est.t == t         # state untouched
    # beyond +-extrap_max_s the acceleration is not extrapolated further
    v5, _ = est.extrapolate(t + 5.0)
    assert abs(v5 - (v + getattr(p, 'extrap_max_s', 1.0) * a)) < 1e-9
    # decelerating to a stop: v >= 0 and the exact stop distance v^2 / 2|a|
    est.v, est.last_a = 0.5, -1.0
    v2, s2 = est.extrapolate(t + 1.0)
    assert v2 == 0.0 and abs(s2 - (s + 0.125)) < 1e-12


def test_route_offset_and_lookahead_drive_grade_lookup():
    seen = []

    def grade(s):
        seen.append(s)
        return 0.0
    p, m, est = make(grade_fn=grade, lookahead=14.0)
    cmds(m, T0 - 1.0, T0 + 1.0, 0)
    est.reset(T0, 5.0)
    est.route_offset = 1000.0
    est.predict_to(T0 + DT)
    assert abs(seen[-1] - (1000.0 + 0.0 + 14.0)) < 1e-9
    est.route_offset = 1500.0 - est.s
    est.predict_to(T0 + 2 * DT)
    assert abs(seen[-1] - 1514.0) < 1e-9
    assert abs(est.s_route - (est.route_offset + est.s)) < 1e-12


def test_grade_decelerates_uphill():
    p, m, est = make(grade_fn=lambda s: 0.03)
    cmds(m, T0 - 1.0, T0 + 12.0, 0)
    est.reset(T0, 8.0)
    run(est, T0, T0 + 10.0)
    a_exp = -p.res_r0 + p.grade_c_coast * 9.81 * 0.03
    assert abs(est.last_a - a_exp) < 1e-9
    assert abs(est.v - (8.0 + 10.0 * a_exp)) < 1e-6


def test_variances_and_scale_term():
    p, m, est = make(odo_scale_sigma=0.003)
    cmds(m, T0 - 1.0, T0 + 200.0, 0)
    est.reset(T0, 10.0)
    run(est, T0, T0 + 100.0, meas_fn=lambda t: Meas(10.0 - 0.0 * t))
    assert est.var_v < 0.03 ** 2
    d = est.dist
    assert d > 900
    assert est.var_s >= (0.003 * d) ** 2
    # during a dropout var_v and var_s grow
    vv, vs = est.var_v, est.var_s
    run(est, est.t, est.t + 10.0)
    assert est.var_v > vv + 0.01 and est.var_s > vs + 1.0


def test_s_correction_after_dropout():
    """A constant model bias during a dropout: the returning speed error corrects s."""
    for corr in (True, False):
        p, m, est = make(s_correction=corr, bias_enable=False)
        cmds(m, T0 - 5.0, T0 + 100.0, 0)
        est.reset(T0 - 5.0, 10.0)
        # truth decelerates at 0.2 m/s^2 more than the model (coast -0.04)
        a_true = -0.04 - 0.2
        truth = lambda t: 10.0 + a_true * (t - T0 + 5.0)
        run(est, T0 - 5.0, T0, meas_fn=lambda t: Meas(truth(t)))
        s_true0 = est.s
        run(est, T0, T0 + 10.0)
        s_true = s_true0 + 10.0 * (truth(T0) + truth(T0 + 10.0)) / 2
        err_before = est.s - s_true
        est.update(Meas(truth(T0 + 10.0)))
        err_after = est.s - s_true
        if corr:
            assert abs(err_after) < 0.5 * abs(err_before), (err_before, err_after)
        else:
            assert abs(err_after - err_before) < 1e-9


def test_v_never_negative_and_bias_decays():
    p, m, est = make()
    cmds(m, T0 - 1.0, T0 + 30.0, -10)
    est.reset(T0, 3.0)
    est.b = 0.2
    run(est, T0, T0 + 10.0)
    assert est.v == 0.0
    run(est, est.t, est.t + 20.0)
    assert abs(est.b) < 0.2 * math.exp(-25.0 / p.bias_tau_s) + 1e-6


def test_robust_to_bad_inputs():
    # sub-ns positive step (small stamps): no division by zero in the sub-step count
    p, m, est = make()
    est.reset(0.1, 1.0)
    est.predict_to(0.1 + 1e-12)
    assert est.t > 0.1
    # NaN / inf speed or sigma is rejected and leaves the state untouched
    p, m, est = make()
    cmds(m, T0 - 1.0, T0 + 2.0, 0)
    est.reset(T0, 5.0)
    est.predict_to(T0 + DT)
    snap = (est.v, est.s, est.b, est.g_tr, [r[:] for r in est.P])
    for bad in (Meas(float('nan')), Meas(float('inf')), Meas(5.0, sigma=float('nan'))):
        assert est.update(bad)['used'] is False
    assert (est.v, est.s, est.b, est.g_tr, est.P) == snap
    est.predict_to(float('nan'))                   # a NaN stamp is ignored
    assert est.t == T0 + DT and math.isfinite(est.v)
    # zero sigma on a collapsed standstill covariance: no division by zero
    p, m, est = make(standstill_sigma_v=0.0)
    cmds(m, T0 - 1.0, T0 + 2.0, -5)
    est.reset(T0, 0.0)
    run(est, T0, T0 + 1.0, meas_fn=lambda t: Meas(0.0, sigma=0.0))
    assert est.v == 0.0 and math.isfinite(est.var_v)
    # a NaN map lookup is ignored instead of poisoning v forever
    p, m, est = make(grade_fn=lambda s: float('nan'), curv_fn=lambda s: float('nan'))
    cmds(m, T0 - 1.0, T0 + 2.0, 0)
    est.reset(T0, 5.0)
    run(est, T0, T0 + 1.0)
    assert math.isfinite(est.v) and abs(est.last_a + p.res_r0) < 1e-12


def test_numpy_map_scalars_do_not_leak_into_state():
    """Route.grade_at/curv_at return numpy.float64; the state must stay Python floats
    (numpy scalar arithmetic doubles the per-call cost)."""
    try:
        import numpy as np
    except ImportError:          # numpy is always present on the target; skip otherwise
        return
    p, m, est = make(grade_fn=lambda s: np.float64(0.01), curv_fn=lambda s: np.float64(0.001))
    cmds(m, T0 - 1.0, T0 + 3.0, 4)
    est.reset(T0, 5.0)
    run(est, T0, T0 + 2.0, meas_fn=lambda t: Meas(5.0))
    assert type(est.v) is float and type(est.s) is float and type(est.last_a) is float
    assert all(type(x) is float for row in est.P for x in row)


def test_per_call_cost():
    p, m, est = make(grade_fn=lambda s: 0.01 * math.sin(s / 100.0), curv_fn=lambda s: 0.002)
    est.reset(T0, 5.0)
    t = T0
    for i in range(100):
        m.add_cmd(T0 - 5.0 + i * DT, 3)
    n = 5000
    t0 = time.perf_counter()
    for i in range(n):
        t += 0.1
        m.add_cmd(t, 3 if (i // 100) % 2 else -3)
        est.predict_to(t)
        est.update(Meas(5.0 + math.sin(0.01 * i)))
    per_call = (time.perf_counter() - t0) / n
    assert per_call < 100e-6, per_call            # typical ~15 us


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
