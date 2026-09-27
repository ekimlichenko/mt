"""Unit tests of core/preprocess.py (TimeGuard) on synthetic message sequences.

The streams mimic the vehicle: 'front' and 'rear' at 10 Hz with identical stamps
that arrive ~40 ms after their header time, 'cmd' at 20 Hz with ~1 ms latency.

Runs under pytest or standalone:  python test_preprocess.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tram_backup_odometry.core.config import Params                 # noqa: E402
from tram_backup_odometry.core.preprocess import TimeGuard          # noqa: E402

T0 = 1_700_000_000.0


def stream(t_start, t_end):
    """[(t_arrival, stream, t_hdr)] in arrival order for a clean vehicle clock."""
    ev = []
    k = 0
    while True:
        t = t_start + 0.05 * k
        if t > t_end:
            break
        ev.append((t + 0.001, 'cmd', t))
        if k % 2 == 0:
            ev.append((t + 0.040, 'front', t))
            ev.append((t + 0.0401, 'rear', t))
        k += 1
    ev.sort(key=lambda e: e[0])
    return ev


def run(guard, ev):
    return [(s, th) + guard.correct(s, th) for _, s, th in ev]


def flags(out):
    return [o[3] for o in out]


def test_clean_streams_are_untouched():
    g = TimeGuard(Params())
    assert g.now == -math.inf
    out = run(g, stream(T0, T0 + 30.0))
    assert all(f == 0 for f in flags(out))
    assert all(te == th for _, th, te, _ in out)
    assert g.now == max(th for _, _, th in stream(T0, T0 + 30.0))


def test_wheel_latency_is_not_late():
    # wheel stamps trail the cmd clock by up to 0.1 s in the recordings
    g = TimeGuard(Params())
    ev = [(a + (0.1 if s != 'cmd' else 0.0), s, th) for a, s, th in stream(T0, T0 + 10.0)]
    ev.sort(key=lambda e: e[0])
    assert all(f == 0 for f in flags(run(g, ev)))


def test_one_off_forward_glitch_on_one_stream():
    g = TimeGuard(Params())
    ev = stream(T0, T0 + 20.0)
    i = next(i for i, e in enumerate(ev) if e[1] == 'cmd' and e[2] > T0 + 10.0)
    true_t = ev[i][2]
    ev[i] = (ev[i][0], 'cmd', true_t + 1.0)
    out = run(g, ev)
    assert flags(out).count(1) == 1 and flags(out).count(2) == 0
    s, th, te, f = out[i]
    assert f == 1 and th == true_t + 1.0
    assert true_t - 0.15 <= te <= true_t          # re-timed to the other streams' clock
    assert all(o[3] == 0 for j, o in enumerate(out) if j != i)
    assert g.now == max(e[2] for j, e in enumerate(ev) if j != i)


def test_front_rear_jump_together_reestablishes_consensus():
    # a whole-clock step: both bogies (identical stamps) and cmd jump +1 s and stay there
    g = TimeGuard(Params())
    ev = [(a, s, th + (1.0 if th > T0 + 10.0 else 0.0)) for a, s, th in stream(T0, T0 + 20.0)]
    out = run(g, ev)
    assert flags(out).count(2) == 0
    assert 1 <= flags(out).count(1) <= 2
    assert abs(g.now - (T0 + 21.0)) < 0.06
    # after the step every stamp is accepted as is
    tail = [o for o in out if o[1] > T0 + 11.5]
    assert all(o[3] == 0 and o[2] == o[1] for o in tail)


def test_interleaved_plus_one_second_on_all_streams():
    # 2255aade-like episode: for 3 s every stream also delivers a second copy stamped +1 s
    # (right header, wrong arrival order).  The +1 s clock wins by consensus, the old copies
    # are flagged late, and the effective clock never goes backwards.
    g = TimeGuard(Params())
    base = stream(T0, T0 + 20.0)
    ev = []
    for a, s, th in base:
        ev.append((a, s, th))
        if T0 + 10.0 <= th < T0 + 13.0:
            ev.append((a + 0.0005, s, th + 1.0))
    ev.sort(key=lambda e: e[0])
    now_prev = -math.inf
    out = []
    for _, s, th in ev:
        te, f = g.correct(s, th)
        out.append((s, th, te, f))
        assert g.now >= now_prev
        if f in (0, 1):
            assert te >= now_prev - g.late_tol_s
        now_prev = g.now
    fl = flags(out)
    assert fl.count(3) == 0
    assert fl.count(2) > 0                                   # the stale copies
    late = [o for o in out if o[3] == 2]
    assert all(T0 + 10.0 <= o[1] < T0 + 14.0 for o in late)
    tail = [o for o in out if o[1] > T0 + 14.5]              # after the episode: clean again
    assert tail and all(o[3] == 0 for o in tail)


def test_backward_step_is_late():
    g = TimeGuard(Params())
    ev = stream(T0, T0 + 20.0)
    i = next(i for i, e in enumerate(ev) if e[1] == 'front' and e[2] > T0 + 10.0)
    ev[i] = (ev[i][0], 'front', ev[i][2] - 1.0)
    out = run(g, ev)
    assert out[i][3] == 2 and out[i][2] == out[i][1]         # t_eff = t_hdr for late samples
    assert flags(out).count(2) == 1 and flags(out).count(1) == 0


def test_persistent_offset_resyncs():
    p = Params()
    g = TimeGuard(p)
    n_resync = getattr(p, 'time_resync_n', 20)
    ev = [(a, s, th + (1.0 if (s == 'cmd' and th > T0 + 10.0) else 0.0))
          for a, s, th in stream(T0, T0 + 20.0)]
    out = run(g, ev)
    cmd = [o for o in out if o[0] == 'cmd' and o[1] > T0 + 11.0]
    assert [o[3] for o in cmd[:n_resync]] == [1] * n_resync
    for s, th, te, f in cmd[:n_resync]:
        assert te < th and abs(te - (th - 1.0)) <= 0.101     # re-timed onto the 10 Hz wheel clock
    assert cmd[n_resync][3] == 0 and cmd[n_resync][2] == cmd[n_resync][1]
    assert g.resyncs == 1
    assert all(o[3] == 0 for o in cmd[n_resync:])


def test_invalid_stamps_are_dropped():
    g = TimeGuard(Params())
    run(g, stream(T0, T0 + 2.0))
    now = g.now
    for bad in (float('nan'), float('inf'), -float('inf'), 0.0):
        te, f = g.correct('front', bad)
        assert f == 3
    assert g.now == now
    assert g.correct('front', now + 0.05)[1] == 0            # the stream is still usable


def test_first_message_and_single_stream():
    g = TimeGuard(Params())
    assert g.correct('front', T0) == (T0, 0)
    # a lone stream has no reference: even a +5 s jump is accepted (nothing contradicts it)
    assert g.correct('front', T0 + 5.0) == (T0 + 5.0, 0)
    assert g.now == T0 + 5.0


def test_random_stamps_keep_the_clock_invariants():
    # jitter, one-off +-1 s / +5 s glitches, invalid stamps, out-of-order arrivals
    import random
    for seed in range(200):
        rnd = random.Random(seed)
        g = TimeGuard(Params())
        now_prev = -math.inf
        for _, s, th in stream(T0, T0 + 20.0):
            r = rnd.random()
            if r < 0.02:
                th = rnd.choice([float('nan'), float('inf'), 0.0, -th])
            elif r < 0.06:
                th = th + rnd.choice([1.0, -1.0, 0.4, 5.0])
            else:
                th = th + rnd.uniform(-0.12, 0.0) * (s != 'cmd')
            te, f = g.correct(s, th)
            assert f in (0, 1, 2, 3)
            if f == 3:
                assert g.now == now_prev
                continue
            assert g.now >= now_prev                            # the filter clock never goes back
            if f == 0:
                assert te == th and te >= now_prev - g.late_tol_s
            elif f == 1:
                # re-timed onto the filter clock, always a backward shift (contract: t_eff < t_hdr),
                # also after two independent glitches of different streams formed a false consensus
                assert te < th and te == g.now
            else:
                assert te == th and te < now_prev - g.late_tol_s and g.now == now_prev
            now_prev = g.now


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
