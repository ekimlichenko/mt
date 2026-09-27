"""Unit tests of core/traction.py (notch -> acceleration model).

Runs under pytest or standalone:  python test_traction.py
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tram_backup_odometry.core.config import Params                   # noqa: E402
from tram_backup_odometry.core.traction import TractionModel          # noqa: E402

DT = 0.05


def make(**over):
    p = Params()
    for k, v in over.items():
        setattr(p, k, v)
    return p, TractionModel(p)


def feed_const(m, t0, t1, notch, dt=DT):
    t = t0
    while t <= t1 + 1e-9:
        m.add_cmd(t, notch)
        t += dt


# ---------------------------------------------------------------- reference
def _ref_step(v, n_tr, n_br, zt, zb, hold_t, grade, curv):
    """openloop_step.py of docs/analysis_notes.md (20 Hz reference propagation)."""
    def a_tr(n, v):
        b0, a0, v1, va, A0, vb, pw = 0.0968, 0.1131, 3.49, 6.90, 1.048, 5.90, 0.571
        return min(b0 + a0 * n / (1 + max(v - v1, 0) / va), A0 * min(1.0, vb / max(v, 1e-3)) ** pw) - 0.04

    def a_br(n, v):
        if n == -8:
            return 0.0 if v > 2.5 else -0.85
        d0, d1, f, vf = 0.193, 0.1123, 0.327, 2.98
        return max(-(d0 + d1 * abs(n)) * (1 - f * math.exp(-v / vf)), -1.7) - 0.04
    ut = a_tr(n_tr, v) if n_tr > 0 else 0.0
    ub = a_br(n_br, v) if n_br < 0 else 0.0
    zt += (1 - math.exp(-DT / 0.25)) * (ut - zt)
    zb += (1 - math.exp(-DT / 0.30)) * (ub - zb)
    mode_c = -0.886 if n_tr > 0 else (-0.812 if n_br < 0 else -0.978)
    coast = -0.04 if (n_tr <= 0 and n_br >= 0) else 0.0
    a = zt + zb + coast + mode_c * 9.81 * grade - 3.0 * abs(curv)
    hold_t = hold_t + DT if (n_tr > 0 and v < 0.05) else 0.0
    if v < 0.05 and (n_tr <= 0 or hold_t < 1.2 - 1e-6):
        return 0.0, zt, zb, hold_t
    return max(v + a * DT, 0.0), zt, zb, hold_t


def test_static_maps():
    p, m = make()
    # plateau below v1: b0 + a0*n - r0
    assert abs(m.a_traction(5, 0.0) - (0.0968 + 0.1131 * 5 - 0.04)) < 1e-12
    # envelope above vb: A0*(vb/v)^pw - r0
    assert abs(m.a_traction(15, 20.0) - (1.048 * (5.9 / 20.0) ** 0.571 - 0.04)) < 1e-12
    # brake linear in |n| with fade, capped
    exp = -(0.193 + 0.1123 * 7) * (1 - 0.327 * math.exp(-10.0 / 2.98)) - 0.04
    assert abs(m.a_brake(-7, 10.0) - exp) < 1e-12
    assert abs(m.a_brake(-15, 10.0) - (-1.7 - 0.04)) < 1e-12
    # traction decreases with speed, brake magnitude increases with |n|
    assert m.a_traction(10, 12.0) < m.a_traction(10, 4.0)
    assert m.a_brake(-6, 8.0) < m.a_brake(-3, 8.0) < 0


def test_equivalent_to_reference_step():
    """With gains = 1, 20 Hz steps and no measurements, v matches openloop_step exactly."""
    random.seed(3)
    p, m = make()
    grade, curv = 0.012, 0.004
    n, v_ref, zt, zb, hold = 0, 0.0, 0.0, 0.0, 0.0
    v = 0.0
    m.add_cmd(0.0, 0)
    for i in range(1, 8000):
        t = i * DT
        if random.random() < 0.08:
            n = max(-15, min(15, n + random.choice([-2, -1, 1, 1, 2])))
        if random.random() < 0.003:
            n = 0
        m.add_cmd(t, n)
        n_tr, n_br = m.notch_at(t - p.lag_tr_dead_s), m.notch_at(t - p.lag_br_dead_s)
        v_ref, zt, zb, hold = _ref_step(v_ref, n_tr, n_br, zt, zb, hold, grade, curv)
        out = m.step(t, DT, v)
        if out.latch:
            v = 0.0
        else:
            a = out.a_ng + out.c_grade * 9.81 * grade - p.curve_k * abs(curv)
            v = max(v + a * DT, 0.0)
        assert abs(v - v_ref) < 1e-9, (i, v, v_ref)
    assert v_ref > 1.0          # the sequence actually moved the tram


def test_traction_dead_time_and_lag():
    p, m = make()
    m.add_cmd(0.0, 0)
    m.add_cmd(1.0, 5)
    feed_const(m, 1.0, 5.0, 5)
    target = m.a_traction(5, 3.0)
    t = 0.0
    while t < 1.0 + p.lag_tr_dead_s - 1e-6:
        t += 0.01
        out = m.step(t, 0.01, 3.0)
        if t < 1.0 + p.lag_tr_dead_s - 0.011:
            assert out.u_tr == 0.0 and out.mode == 'coast'
    # after the dead time the lag starts: 63 % after one time constant
    t_start = 1.0 + p.lag_tr_dead_s
    while t < t_start + p.lag_tr_tau_s - 1e-9:
        t += 0.01
        out = m.step(t, 0.01, 3.0)
    frac = out.u_tr / target
    assert out.mode == 'traction'
    assert abs(frac - (1 - math.exp(-1))) < 0.05, frac


def test_brake_dead_time_and_lag():
    p, m = make()
    m.add_cmd(0.0, 0)
    feed_const(m, 1.0, 5.0, -4)
    target = m.a_brake(-4, 8.0)
    t = 0.0
    while t < 1.0 + p.lag_br_dead_s - 0.02:
        t += 0.01
        out = m.step(t, 0.01, 8.0)
    assert out.u_br == 0.0
    while t < 1.0 + p.lag_br_dead_s + p.lag_br_tau_s - 1e-9:
        t += 0.01
        out = m.step(t, 0.01, 8.0)
    assert out.mode == 'brake'
    assert abs(out.u_br / target - (1 - math.exp(-1))) < 0.06
    # steady state
    while t < 4.0:
        t += 0.05
        out = m.step(t, 0.05, 8.0)
    assert abs(out.u_br - target) < 1e-3
    assert abs(out.c_grade - p.grade_c_brake) < 1e-12
    assert abs(out.sigma_a - p.sigma_a_brake) < 1e-12


def test_hold8_mode():
    p, m = make()
    feed_const(m, 0.0, 12.0, p.hold8_notch)
    t = 0.0
    while t < 5.0:
        t += DT
        out = m.step(t, DT, 6.0)
    assert out.mode == 'hold8' and not out.latch
    assert abs(out.u_br) < 1e-6                 # a = 0 above hold8_v
    assert abs(out.sigma_a - p.sigma_a_hold8) < 1e-12
    for _ in range(100):
        t += DT
        out = m.step(t, DT, 2.0)                # below hold8_v -> hold8_low_acc
    assert abs(out.u_br - p.hold8_low_acc) < 1e-3


def test_brake_high_sigma():
    p, m = make()
    feed_const(m, 0.0, 2.0, -12)
    out = m.step(1.0, DT, 5.0)
    assert out.mode == 'brake' and abs(out.sigma_a - p.sigma_a_brake_high) < 1e-12


def test_standstill_latch_release():
    p, m = make()
    m.add_cmd(0.0, 0)
    feed_const(m, 0.5, 5.0, 3)
    t, released = 0.0, None
    while t < 4.0:
        t += DT
        out = m.step(t, DT, 0.0)
        if not out.latch and released is None:
            released = t
    # traction seen after the dead time, then held for start_delay_s
    assert abs(released - (0.5 + p.lag_tr_dead_s + p.start_delay_s)) <= DT + 1e-9, released
    # coast / brake at rest keeps the latch
    p, m = make()
    feed_const(m, 0.0, 3.0, -5)
    assert m.step(1.0, DT, 0.0).latch
    assert m.step(1.05, DT, 0.0).mode == 'standstill'
    # moving: no latch whatever the notch
    assert not m.step(1.1, DT, 1.0).latch


def test_no_cmd_and_stale_cmd():
    p, m = make(sigma_a_nocmd=0.5, cmd_stale_s=1.0)
    out = m.step(1.0, DT, 5.0)
    assert out.mode == 'coast' and not out.has_cmd
    assert abs(out.sigma_a - 0.5) < 1e-12
    assert abs(out.a_ng + p.res_r0) < 1e-12
    m.add_cmd(1.0, 6)
    out = m.step(1.5, DT, 5.0)
    assert out.has_cmd and out.mode == 'traction'
    out = m.step(2.6, DT, 5.0)                   # last cmd 1.6 s old -> stale -> no-cmd coast
    assert not out.has_cmd and out.mode == 'coast'


def test_out_of_order_commands_and_history():
    p, m = make()
    for t, n in [(1.0, 1), (3.0, 3), (2.0, 2), (2.5, -1), (0.5, 0)]:
        m.add_cmd(t, n)
    assert m.notch_at(0.4) == 0
    assert m.notch_at(0.7) == 0
    assert m.notch_at(1.2) == 1
    assert m.notch_at(2.2) == 2
    assert m.notch_at(2.7) == -1
    assert m.notch_at(9.0) == 3
    # long history is pruned but the command in force within the window is kept
    for i in range(2000):
        m.add_cmd(10.0 + i * DT, 4 if i < 1990 else 5)
        m.step(10.0 + i * DT, DT, 5.0)
    t_last = 10.0 + 1999 * DT
    assert len(m._ct) < 400
    assert m.notch_at(t_last - 2.9) == 4
    assert m.notch_at(t_last) == 5


def test_peek_does_not_change_state():
    p, m = make()
    feed_const(m, 0.0, 3.0, 7)
    for i in range(1, 20):
        m.step(i * DT, DT, 4.0)
    snap = (m.zt, m.zb, m.hold_t)
    o1 = m.peek(1.0, 4.0)
    assert (m.zt, m.zb, m.hold_t) == snap
    assert abs(o1.u_tr - m.zt) < 1e-15


def test_gains_scale_lagged_levels():
    p, m = make()
    feed_const(m, 0.0, 5.0, 6)
    for i in range(1, 60):
        out = m.step(i * DT, DT, 5.0)
    m.gain_tr = 1.2
    o2 = m.peek(60 * DT, 5.0)
    assert abs(o2.a_ng - 1.2 * o2.u_tr) < 1e-12
    assert abs(out.a_ng - out.u_tr) < 1e-12


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
