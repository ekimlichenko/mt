"""Unit tests of core/wheel_fusion.py on synthetic bogie signals.

Runs under pytest or standalone:  python test_wheel_fusion.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tram_backup_odometry.core.config import Params                   # noqa: E402
from tram_backup_odometry.core.wheel_fusion import WheelFusion        # noqa: E402

K = 3.6          # raw km/h -> m/s in these tests
T0 = 1_700_000_000.0


def make(**over):
    p = Params()
    for k, v in over.items():
        setattr(p, k, v)
    return p, WheelFusion(p, K)


def feed(wf, t, vf=None, vr=None, v_pred=None):
    """Add front and/or rear samples (m/s) at t, then measure at t."""
    if vf is not None:
        wf.add_sample('front', t, vf * K)
    if vr is not None:
        wf.add_sample('rear', t, vr * K)
    return wf.measure(t, v_pred)


def test_no_data_gives_none():
    _, wf = make()
    assert wf.measure(T0, 0.0) is None


def test_agreeing_bogies_are_averaged():
    p, wf = make()
    for i in range(20):
        m = feed(wf, T0 + 0.1 * i, 10.0 + 0.01 * i, 10.02 + 0.01 * i, 10.0)
    assert m.state == 'both' and not m.slip and m.front_ok and m.rear_ok
    assert abs(m.v - (10.01 + 0.19)) < 1e-9
    assert m.sigma == p.meas_sigma
    assert abs(m.v_front - 10.19) < 1e-9 and abs(m.v_rear - 10.21) < 1e-9


def test_sample_validation():
    p, wf = make()
    assert wf.add_sample('front', T0, 36.0)
    assert not wf.add_sample('front', T0 + 0.1, float('nan'))
    assert not wf.add_sample('front', T0 + 0.1, float('inf'))
    assert not wf.add_sample('front', T0 + 0.1, p.wheel_max_kmh + 1.0)
    assert not wf.add_sample('front', T0 + 0.1, p.wheel_min_kmh - 1.0)
    assert not wf.add_sample('front', T0 - 0.1, 36.0)          # out of order
    assert not wf.add_sample('side', T0 + 0.1, 36.0)
    assert not wf.add_sample('front', float('nan'), 36.0)
    _, wf = make()
    assert wf.add_sample('rear', T0, -0.3)                      # roll-back: clamped to 0
    m = wf.measure(T0, 0.0)
    assert m.v == 0.0 and m.state == 'single_rear'
    assert m.sigma == Params().meas_sigma_single


def test_stale_bogie_is_dropped():
    p, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 5.0, 5.0)
    t_last = T0 + 0.9
    for i in range(1, 12):                                      # rear goes silent
        t = t_last + 0.1 * i
        m = feed(wf, t, vf=5.0)
        rear_age = t - t_last
        if rear_age > p.wheel_stale_s + 1e-9:
            assert m.state == 'single_front' and not m.rear_ok and math.isnan(m.v_rear)
            assert wf.health()['rear']['stale']
        else:
            assert m.state == 'both'


def test_extrapolation_is_capped():
    p, wf = make()
    wf.add_sample('front', T0, 10.0 * K)
    wf.add_sample('front', T0 + 0.1, 10.1 * K)                  # slope +1 m/s^2
    m = wf.measure(T0 + 0.1 + 0.4, 10.0)                        # 0.4 s ahead, still fresh
    assert abs(m.v - (10.1 + 1.0 * p.wheel_extrap_max_s)) < 1e-5
    m = wf.measure(T0 + 0.15, 10.0)
    assert abs(m.v - 10.15) < 1e-5
    m = wf.measure(T0 + 0.05, 10.0)                             # between samples: interpolated
    assert abs(m.v - 10.05) < 1e-5
    _, wf = make()
    wf.add_sample('front', T0, 1.0 * K)
    wf.add_sample('front', T0 + 0.1, 0.2 * K)                   # braking to a stop
    assert wf.measure(T0 + 0.3, 0.0).v == 0.0                   # never negative


def test_dropout_30639_style():
    # Standstill, then departure at 0.8 m/s^2.  The rear bogie keeps sending 0.0 for 0.5 s
    # after the front starts to move, then goes silent for 20 s (the 30639 rear dropouts).
    p, wf = make()
    for i in range(50):
        m = feed(wf, T0 + 0.1 * i, 0.0, 0.0, 0.0)
    assert m.state == 'both' and m.v == 0.0
    t_dep = T0 + 5.0
    v_prev = 0.0
    worst = 0.0
    stuck_seen = False
    for i in range(200):
        t = t_dep + 0.1 * i
        vf = 0.8 * (t - t_dep)
        vr = 0.0 if t - t_dep <= 0.5 else None
        m = feed(wf, t, vf, vr, v_prev)
        v_prev = m.v
        worst = max(worst, abs(m.v - vf))
        stuck_seen |= wf.health()['rear']['stuck_zero']
        if vf > p.stuck_zero_other_mps and vr is not None:
            assert m.state == 'single_front' and not m.rear_ok
        if t - t_dep > 0.5 + p.wheel_stale_s + 1e-9:
            assert m.state == 'single_front' and wf.health()['rear']['stale']
    assert stuck_seen
    assert abs(m.v - 0.8 * 19.9) < 1e-5 and m.front_ok and not m.rear_ok
    assert worst <= p.stuck_zero_other_mps + 1e-9               # never worse than the zero rule


def test_frozen_value_is_dropped_until_it_changes():
    p, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 5.0, 5.0)
    frozen_seen = False
    for i in range(10, 40):                                     # rear frozen at 5.0, front accelerates
        t = T0 + 0.1 * i
        vf = 5.0 + 0.5 * (t - (T0 + 0.9))
        m = feed(wf, t, vf, 5.0, vf)
        if wf.health()['rear']['frozen']:
            frozen_seen = True
            assert m.state == 'single_front' and not m.rear_ok
    assert frozen_seen
    t = T0 + 4.0
    m = feed(wf, t, 6.55, 6.54, 6.55)                           # the rear value changes again
    assert m.state == 'both' and m.rear_ok and not wf.health()['rear']['frozen']


def test_frozen_needs_the_other_to_move():
    _, wf = make()
    for i in range(40):                                         # steady cruise, identical values
        m = feed(wf, T0 + 0.1 * i, 10.0, 10.0, 10.0)
    assert m.state == 'both' and not wf.health()['rear']['frozen']


def test_disagreement_picks_bogie_near_prediction_and_holds_slip():
    p, wf = make()
    tol = max(p.agree_abs_mps, p.agree_rel * 10.0) + 1e-6

    def front(i):                               # the correct bogie, with realistic sensor noise
        return 10.0 + 0.005 * ((i * 7) % 3 - 1)

    for i in range(10):
        feed(wf, T0 + 0.1 * i, front(i), 10.0, 10.0)
    # rear slides: drops 3 m/s over 1 s and recovers; the front stays right
    t_s = T0 + 1.0
    t_last_dis = None
    for i in range(0, 20):
        t = t_s + 0.1 * i
        vr = 10.0 - 3.0 * min(i, 20 - i) / 10.0
        m = feed(wf, t, front(i), vr, 10.0)
        if abs(front(i) - vr) > tol:
            t_last_dis = t
            assert m.state == 'disagree' and m.slip and m.v == m.v_front
            assert m.sigma == p.meas_sigma_slip
    assert t_last_dis is not None
    t_agree = t_s + 2.0
    for i in range(0, 25):
        t = t_agree + 0.1 * i
        m = feed(wf, t, front(i), 10.0, 10.0)
        assert m.state == 'both'
        assert m.slip == (t < t_last_dis + p.slip_hold_s)
    assert not m.slip and m.sigma == p.meas_sigma
    # the other way round: front spins, rear right, prediction near the rear
    for i in range(5):
        m = feed(wf, t + 0.1 * (i + 1), 12.0 + 0.01 * i, 10.1 - 0.01 * i, 10.05)
    assert m.state == 'disagree' and m.v == m.v_rear


def test_common_mode_accel_flag_keeps_the_value():
    # the deployed slip_dec_flag (6 m/s^2) sits above real uncontrolled braking (-5.15 m/s^2);
    # the mechanism is exercised here with the acceleration threshold on both sides
    p, wf = make(slip_dec_flag=2.5)
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 12.0, 12.0, 12.0)
    flagged = []
    for i in range(1, 11):                                      # both bogies lock: -4 m/s^2
        t = T0 + 0.9 + 0.1 * i
        v = 12.0 - 4.0 * 0.1 * i
        m = feed(wf, t, v, v, v)
        assert m.state == 'both' and abs(m.v - v) < 1e-9        # never altered or dropped
        flagged.append(m.slip)
        if m.slip:
            assert m.sigma == p.meas_sigma_slip
    # the mean deceleration over the last >= slip_acc_win_s must exceed slip_acc_flag:
    # after 0.1 s it is 1.33 m/s^2, after 0.2 s 2.67 m/s^2 (flagged) and 4 m/s^2 from then on
    assert flagged[0] is False and all(flagged[1:])
    # ordinary service braking at -1.2 m/s^2 is not flagged
    _, wf = make()
    for i in range(60):
        v = max(15.0 - 1.2 * 0.1 * i, 0.0)
        m = feed(wf, T0 + 0.1 * i, v, v, v)
        assert not m.slip


def test_single_sample_spike_is_rejected():
    _, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 8.0, 8.0, 8.0)
    assert not wf.add_sample('front', T0 + 1.0, (8.0 + 5.0) * K)   # +18 km/h in 0.1 s
    wf.add_sample('rear', T0 + 1.0, 8.0 * K)
    m = wf.measure(T0 + 1.0, 8.0)
    assert m.state == 'both' and abs(m.v - 8.0) < 1e-9
    assert wf.add_sample('front', T0 + 1.1, 8.0 * K)
    assert wf.health()['front']['spikes'] == 1
    # a level change that persists is accepted on its second sample
    assert not wf.add_sample('front', T0 + 1.2, 14.0 * K)
    assert wf.add_sample('front', T0 + 1.3, 14.0 * K)


def test_spike_confirmed_by_other_bogie_is_accepted():
    _, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 8.0, 8.0, 8.0)
    assert not wf.add_sample('rear', T0 + 1.0, 10.0 * K)       # a 2 m/s jump in 0.1 s: rejected
    assert wf.add_sample('rear', T0 + 1.05, 10.05 * K)         # it persists: accepted
    assert wf.add_sample('front', T0 + 1.1, 10.1 * K)          # same jump, confirmed by the rear
    assert wf.health()['front']['spikes'] == 0


def test_timing_per_call():
    _, wf = make()
    n = 20000
    t0 = time.perf_counter()
    for i in range(n):
        t = T0 + 0.05 * i
        v = 10.0 + math.sin(0.01 * i)
        wf.add_sample('front' if i % 2 else 'rear', t, v * K)
        wf.measure(t, v)
    per_pair = (time.perf_counter() - t0) / n
    assert per_pair < 200e-6                                     # loose bound; typical ~5-10 us


def test_first_arrival_uses_aligned_value_and_gap_is_not_slip():
    # pairs share stamps; the rear of each pair arrives first.  Braking at -1.3 m/s^2; the
    # front has +-0.06 m/s of noise on the two samples before a lost pair (0.2 s gap).  After
    # the gap the own-slope extrapolation of the older front is 0.3 m/s off the fresh rear
    # (a false disagreement); the aligned comparison sees the true 0.06 m/s difference, so
    # the sample is the agreeing aligned mean (only its sigma is raised), not a slip.
    p, wf = make()
    front_noise = {28: 0.06, 29: -0.06}
    prev = None
    worst = 0.0
    for i in range(40):
        if i == 30:
            continue                                            # lost pair
        t = T0 + 0.1 * i
        v = 10.0 - 1.3 * 0.1 * i
        wf.add_sample('rear', t, v * K)
        m = wf.measure(t, prev)                                 # first arrival: front is older
        prev = m.v
        if i > 0:                                               # (no front sample at i = 0)
            assert not m.slip and m.state == 'both'
            assert m.sigma == (p.meas_sigma_slip if i == 31 else p.meas_sigma)
            worst = max(worst, abs(m.v - v))
        wf.add_sample('front', t, (v + front_noise.get(i, 0.0)) * K)
        m = wf.measure(t, prev)
        prev = m.v
        assert not m.slip and m.state == 'both' and m.sigma == p.meas_sigma
    assert worst < 0.05
    assert wf.health()['n_disagree'] == 0 and wf.health()['n_uncertain'] == 1


def test_creeping_spin_does_not_capture_the_choice():
    # the front (arriving first in each pair) starts to spin while the tram accelerates at
    # 0.7 m/s^2.  At the first arrival of the diverging pair only the own comparison fails,
    # so the output is the (contaminated) agreeing mean; v_pred = previous fused value then
    # sits next to the front.  The held anchor of the last full agreement keeps the rear.
    p, wf = make()
    excess = {20: 0.1, 21: 0.3, 22: 0.6, 23: 1.0, 24: 1.5, 25: 2.0, 26: 2.5, 27: 3.0}
    prev = None
    for i in range(28):
        t = T0 + 0.1 * i
        v_true = 7.0 + 0.7 * 0.1 * i
        wf.add_sample('front', t, (v_true + excess.get(i, 0.0)) * K)
        m = wf.measure(t, prev)
        prev = m.v
        wf.add_sample('rear', t, v_true * K)
        m = wf.measure(t, prev)
        prev = m.v
        if i >= 21:
            assert m.state == 'disagree' and m.slip and m.v == m.v_rear
    assert abs(m.v - v_true) < 1e-9


def test_wheel_lock_to_zero_keeps_slip():
    # the rear slides and both bogies then lock at 0 while the tram still moves
    # (30639_50956d6e at 1040 s): slip is held for v0 / lock_zero_dec after the lock
    p, wf = make()
    for i in range(20):
        m = feed(wf, T0 + 0.1 * i, 6.0, 6.0, 6.0)
    t = T0 + 2.0
    seq = [5.9, 5.0, 3.8, 2.4, 1.0, 0.0]                        # rear: -9 m/s^2 slide
    for i, vr in enumerate(seq):
        m = feed(wf, t + 0.1 * i, 6.0 - 0.1 * i, vr, 6.0)
    assert m.slip
    t_zero = t + 0.1 * len(seq)
    hold = 5.95 / getattr(p, 'lock_zero_dec', 1.0)              # v0 = last agreed speed
    for i in range(0, 100):                                     # both read 0 from now on
        tt = t_zero + 0.1 * i
        m = feed(wf, tt, 0.0, 0.0, 0.0)
        if tt - t_zero < hold - 0.05:
            assert m.slip and m.sigma == p.meas_sigma_slip
        if tt - t_zero > hold + 0.05:
            assert not m.slip and m.sigma == p.meas_sigma
    # a genuine stop without a slip is never held
    _, wf = make()
    for i in range(60):
        v = max(3.0 - 1.0 * 0.1 * i, 0.0)
        m = feed(wf, T0 + 0.1 * i, v, v, v)
        assert not m.slip


def test_non_finite_measure_time_is_ignored():
    # a NaN time used to return a 'both' measurement with v = 0 and left a NaN entry in the
    # acceleration history that was never trimmed, which disabled the common-mode flag for good
    p, wf = make(slip_dec_flag=2.5)
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 12.0, 12.0, 12.0)
    assert wf.measure(float('nan'), 12.0) is None
    assert wf.measure(float('inf'), 12.0) is None
    flagged = []
    for i in range(1, 11):                                      # both bogies lock: -4 m/s^2
        v = 12.0 - 0.4 * i
        m = feed(wf, T0 + 0.9 + 0.1 * i, v, v, v)
        assert m.state == 'both' and abs(m.v - v) < 1e-9
        flagged.append(m.slip)
    assert all(flagged[1:])
    assert len(wf._hist) <= 6                                   # still trimmed to the window


def test_disagreement_after_the_anchor_uses_v_pred():
    # contract rule once the last full agreement is older than slip_anchor_s: the bogie
    # closer to v_pred wins.  slip_anchor_s < 0 gives that rule from the first disagreement.
    def front(i):                                               # realistic dither (not frozen)
        return 10.0 + 0.005 * ((i * 7) % 3 - 1)

    _, wf = make(slip_anchor_s=3.0)
    for i in range(10):
        feed(wf, T0 + 0.1 * i, front(i), 10.0, 10.0)
    for i in range(10, 70):                                     # rear silent for 6 s
        feed(wf, T0 + 0.1 * i, vf=front(i), v_pred=10.0)
    m = feed(wf, T0 + 7.0, front(70), 11.5, 11.4)               # rear back 1.5 m/s higher
    assert m.state == 'disagree' and m.slip and m.v == m.v_rear
    m = feed(wf, T0 + 7.1, front(71), 11.5, 10.1)               # v_pred near the front
    assert m.state == 'disagree' and m.v == m.v_front

    _, wf = make(slip_anchor_s=-1.0)
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 10.0, 10.0, 10.0)
    m = feed(wf, T0 + 1.0, 10.0, 11.5, 11.4)                    # the held anchor would pick F
    assert m.state == 'disagree' and m.v == m.v_rear


def test_disagreement_within_the_anchor_ignores_v_pred():
    # within slip_anchor_s of the last agreement the held agreed speed decides, even when
    # v_pred (e.g. the previous fused value) has drifted towards the spinning bogie
    _, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 10.0, 10.0, 10.0)
    m = feed(wf, T0 + 1.0, 11.0, 10.02, 11.0)
    assert m.state == 'disagree' and m.v == m.v_rear


def test_sample_older_than_the_last_one_is_rejected():
    # 30639_9c362687 at 1332 s: an early (future) copy was accepted, the regular sample that
    # follows is older than it; the bogie keeps its newest sample and the other still counts
    _, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 12.0, 12.0, 12.0)
    assert wf.add_sample('front', T0 + 1.3, 12.0 * K)
    assert not wf.add_sample('front', T0 + 1.0, 12.0 * K)
    assert wf.add_sample('rear', T0 + 1.0, 12.0 * K)
    m = wf.measure(T0 + 1.0, 12.0)
    assert m.state == 'both' and abs(m.v - 12.0) < 1e-9
    assert wf.health()['front']['rejected'] == 1


def test_same_stamp_resend_replaces_the_value():
    _, wf = make()
    for i in range(10):
        feed(wf, T0 + 0.1 * i, 8.0, 8.0, 8.0)
    assert wf.add_sample('front', T0 + 1.0, 8.2 * K)
    assert wf.add_sample('front', T0 + 1.0, 8.1 * K)            # same stamp again
    wf.add_sample('rear', T0 + 1.0, 8.1 * K)
    m = wf.measure(T0 + 1.0, 8.1)
    assert abs(m.v_front - 8.1) < 1e-9 and abs(m.v - 8.1) < 1e-9


def test_random_inputs_keep_the_output_invariants():
    import random
    p = Params()
    sigmas = (p.meas_sigma, p.meas_sigma_single, p.meas_sigma_slip)
    for seed in range(60):
        rnd = random.Random(seed)
        wf = WheelFusion(p, K)
        t, v, prev = T0, rnd.uniform(0.0, 15.0), None
        for _ in range(300):
            t += rnd.choice([0.0, 0.001, 0.1, 0.1, 0.1, 0.2, 0.7, 3.0])
            v = max(0.0, v + rnd.gauss(0.0, 0.2))
            raw = v * K * rnd.choice([1.0, 1.0, 1.0, 1.3, 0.5, 0.0])
            r = rnd.random()
            if r < 0.02:
                raw = float('nan')
            elif r < 0.04:
                raw = -rnd.uniform(0.0, 10.0)
            elif r < 0.05:
                raw = 1e6
            wf.add_sample(rnd.choice(['front', 'rear']), t + rnd.choice([0.0, -0.05, -1.0]), raw)
            m = wf.measure(t + rnd.choice([0.0, 0.0, 0.05, -0.05, 0.3]),
                           rnd.choice([prev, None, float('nan'), v]))
            if m is None:
                continue
            assert math.isfinite(m.v) and m.v >= 0.0 and m.sigma in sigmas
            assert m.state in ('both', 'disagree', 'single_front', 'single_rear')
            assert m.front_ok == math.isfinite(m.v_front) and m.rear_ok == math.isfinite(m.v_rear)
            assert (m.state == 'disagree') <= m.slip
            prev = m.v
        assert len(wf._hist) < 50


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
