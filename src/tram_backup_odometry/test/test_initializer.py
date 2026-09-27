"""Unit tests of core/initializer.py on a synthetic straight double track.

T2S runs east along y = Y0 from X0 to X0 + 1000; S2T runs west on the parallel
track 3.5 m to the north (the left side of T2S), like the real line.  Fixes are
generated in the map frame and converted to lat/lon with a numerical inverse
of frames.latlon_to_map, so the public add_fix path is exercised.

Runs under pytest or standalone:  python test_initializer.py
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tram_backup_odometry.core.config import Params               # noqa: E402
from tram_backup_odometry.core.frames import latlon_to_map        # noqa: E402
from tram_backup_odometry.core.initializer import Initializer     # noqa: E402
from tram_backup_odometry.core.track_map import Route             # noqa: E402

X0, Y0, LEN, SEP = 99000.0, 85000.0, 1000.0, 3.5
Z_MAP, Z_OFF = 170.0, 3.10
T0 = 1_700_000_000.0


def make_routes():
    t2s = [dict(x=X0 + i, y=Y0, z=Z_MAP, curv=0.0) for i in range(int(LEN) + 1)]
    s2t = [dict(x=X0 + LEN - i, y=Y0 + SEP, z=Z_MAP, curv=0.0) for i in range(int(LEN) + 1)]
    return {'T2S': Route('T2S', t2s), 'S2T': Route('S2T', s2t)}


ROUTES = make_routes()


def xy_to_latlon(x, y):
    """Newton inverse of frames.latlon_to_map (finite-difference Jacobian)."""
    lat, lon = 55.8, 37.4
    for _ in range(20):
        ex, ey = latlon_to_map(lat, lon)
        rx, ry = x - ex, y - ey
        if math.hypot(rx, ry) < 1e-5:
            break
        h = 1e-6
        a = latlon_to_map(lat + h, lon); b = latlon_to_map(lat, lon + h)
        j11, j21 = (a[0] - ex) / h, (a[1] - ey) / h
        j12, j22 = (b[0] - ex) / h, (b[1] - ey) / h
        det = j11 * j22 - j12 * j21
        lat += (j22 * rx - j12 * ry) / det
        lon += (-j21 * rx + j11 * ry) / det
    return lat, lon


def feed(ini, master_xy, heading, n=30, dt=0.1, status=2, rover=True, rover_status=None,
         noise=0.0, speed=0.0, t_start=T0, odo0=0.0, seed=0, finalize=True):
    """Feed n epochs of master (+ rover 12.44 m ahead along heading); returns the finalize time."""
    rnd = random.Random(seed)
    c, s = math.cos(heading), math.sin(heading)
    t = t_start
    for i in range(n):
        t = t_start + i * dt
        odo = odo0 + speed * i * dt
        mx = master_xy[0] + c * speed * i * dt + rnd.gauss(0.0, noise)
        my = master_xy[1] + s * speed * i * dt + rnd.gauss(0.0, noise)
        lat, lon = xy_to_latlon(mx, my)
        ini.add_fix('master', t, lat, lon, Z_MAP + Z_OFF, status, odo)
        if rover:
            lat, lon = xy_to_latlon(mx + 12.44 * c, my + 12.44 * s)
            ini.add_fix('rover', t, lat, lon, Z_MAP + Z_OFF, status if rover_status is None else rover_status, odo)
        ini.maybe_finalize(t)
    if finalize:
        t_end = t_start + 3.0 + 0.01
        assert ini.maybe_finalize(t_end)
    return t


def new(**over):
    p = Params()
    for k, v in over.items():
        setattr(p, k, v)
    return Initializer(p, ROUTES), p


def test_xy_latlon_roundtrip():
    lat, lon = xy_to_latlon(X0 + 123.4, Y0 - 56.7)
    x, y = latlon_to_map(lat, lon)
    assert abs(x - (X0 + 123.4)) < 1e-4 and abs(y - (Y0 - 56.7)) < 1e-4


def test_onmap_start_t2s_and_s2t():
    ini, p = new()
    feed(ini, (X0 + 300.0, Y0), 0.0)
    r = ini.result()
    assert ini.done and r.final
    assert r.direction == 'T2S' and r.kind == 'onmap' and r.head is None and r.rtk
    assert abs(r.s_offset - 300.0) < 0.05
    assert r.sigma0 == p.sigma0_rtk_onmap_m
    assert abs(r.yaw0) < 1e-6
    assert abs(r.start_xyz[0] - (X0 + 300.0)) < 1e-3 and abs(r.start_xyz[2] - (Z_MAP + Z_OFF)) < 1e-6

    ini, _ = new()
    feed(ini, (X0 + 700.0, Y0 + SEP), math.pi)
    r = ini.result()
    assert r.direction == 'S2T' and r.kind == 'onmap'
    assert abs(r.s_offset - 300.0) < 0.05


def test_odometer_relation_moving_start():
    # moving at 5 m/s on T2S, odometer already at 1000 m when GNSS starts
    ini, _ = new()
    feed(ini, (X0 + 200.0, Y0), 0.0, speed=5.0, odo0=1000.0)
    r = ini.result()
    assert r.direction == 'T2S' and r.kind == 'onmap'
    assert abs(r.s_offset - (200.0 - 1000.0)) < 0.05


def test_offmap_start_bridge():
    ini, p = new()
    start = (X0 - 150.0, Y0 - 40.0)
    feed(ini, start, math.radians(20.0), odo0=7.0)
    r = ini.result()
    assert r.direction == 'T2S' and r.kind == 'bridge' and r.head is not None
    h = r.head
    chord = math.hypot(X0 - start[0], Y0 - start[1])
    assert h['L'] > chord
    assert abs(r.s_offset - (-h['L'] - 7.0)) < 1e-9
    assert abs(h['x'][0] - start[0]) < 0.05 and abs(h['y'][0] - start[1]) < 0.05
    assert abs(h['x'][-1] - X0) < 0.05 and abs(h['y'][-1] - Y0) < 0.05
    assert abs(h['s'][-1]) < 1e-9 and abs(h['s'][0] + h['L']) < 1e-9
    assert abs(h['z'][0] - Z_MAP) < 1e-6 and abs(h['z'][-1] - Z_MAP) < 1e-6
    assert abs(r.yaw0 - math.radians(20.0)) < 1e-3
    assert r.sigma0 == p.sigma0_bridge_t2s_m
    # tangent length scales the bridge: a larger k gives a longer path
    ini2, _ = new(bridge_tangent_k_t2s=2.0)
    feed(ini2, start, math.radians(20.0), odo0=7.0)
    assert ini2.result().head['L'] > h['L']


def test_start_beyond_route_end_picks_nearest_start():
    # tram past the T2S end, heading on (east): the next run is S2T, whose start is here
    ini, p = new()
    feed(ini, (X0 + LEN + 60.0, Y0 + 20.0), 0.0)
    r = ini.result()
    assert r.direction == 'S2T' and r.kind == 'bridge'
    assert r.sigma0 == p.sigma0_bridge_s2t_m
    assert r.s_offset < -60.0


def test_midroute_heading_decides_track():
    # standing between the tracks with noisy non-RTK fixes: distances are ambiguous,
    # the baseline heading picks the track
    for heading, want in ((0.0, 'T2S'), (math.pi, 'S2T')):
        ini, _ = new()
        feed(ini, (X0 + 500.0, Y0 + 0.5 * SEP + 0.2), heading, status=0, noise=0.3)
        r = ini.result()
        assert r.direction == want and r.kind == 'onmap', (heading, r)
        want_s = 500.0 if want == 'T2S' else LEN - 500.0
        assert abs(r.s_offset - want_s) < 1.0
        assert not r.rtk


def test_near_rule_parallel_bypass():
    # 5 m right of S2T (away from T2S): outside init_onmap_dist_m, heading agrees with S2T
    ini, p = new()
    feed(ini, (X0 + 400.0, Y0 + SEP + 5.0), math.pi)
    r = ini.result()
    assert r.direction == 'S2T' and r.kind == 'onmap' and r.diag['rule'] == 'near_heading'
    assert r.sigma0 >= p.sigma0_nortk_m


def test_far_rule_interior_offset():
    # non-RTK fixes 15 m off the track but along it (street canyon): the heading picks the
    # route, s from the projection, sigma grows with the offset
    ini, p = new()
    feed(ini, (X0 + 400.0, Y0 - 15.0), 0.0, status=0)
    r = ini.result()
    assert r.direction == 'T2S' and r.kind == 'onmap' and r.diag['rule'] == 'far_heading'
    assert abs(r.s_offset - 400.0) < 0.05
    assert abs(r.sigma0 - math.hypot(p.sigma0_nortk_m, 15.0)) < 1e-6
    ini, _ = new()
    feed(ini, (X0 + 400.0, Y0 + SEP + 15.0), math.pi, status=0)
    assert ini.result().direction == 'S2T' and ini.result().diag['rule'] == 'far_heading'
    # beyond init_far_dist_m the tram is off the map: bridge to the nearest start
    ini, _ = new()
    feed(ini, (X0 + 400.0, Y0 - 45.0), 0.0)
    assert ini.result().kind == 'bridge' and ini.result().diag['rule'] == 'offmap'


def test_motion_overrides_wrong_baseline():
    # moving east on T2S with RTK; the rover has a false fix of the right length pointing
    # north-west (moving-baseline false fix): the motion direction wins
    ini, _ = new()
    bad = math.radians(120.0)
    for i in range(30):
        t = T0 + 0.1 * i
        mx = X0 + 300.0 + 4.0 * 0.1 * i
        lat, lon = xy_to_latlon(mx, Y0)
        ini.add_fix('master', t, lat, lon, Z_MAP + Z_OFF, 2, 4.0 * 0.1 * i)
        lat, lon = xy_to_latlon(mx + 12.44 * math.cos(bad), Y0 + 12.44 * math.sin(bad))
        ini.add_fix('rover', t, lat, lon, Z_MAP + Z_OFF, 2, 4.0 * 0.1 * i)
    assert ini.maybe_finalize(T0 + 3.5)
    r = ini.result()
    assert r.diag['heading_src'] == 'motion_override'
    assert r.direction == 'T2S' and r.kind == 'onmap' and abs(r.s_offset - 300.0) < 0.05
    # standing: no motion to check against, the baseline is used as is
    ini, _ = new()
    feed(ini, (X0 + 300.0, Y0), math.pi)
    assert ini.result().diag['heading_src'] == 'baseline_rtk2'


def test_no_rtk_uses_all_and_inflates_sigma():
    ini, p = new()
    feed(ini, (X0 + 300.0, Y0), 0.0, status=0, noise=0.8)
    r = ini.result()
    assert r.direction == 'T2S' and r.kind == 'onmap' and not r.rtk
    assert r.sigma0 == max(p.sigma0_rtk_onmap_m, p.sigma0_nortk_m)
    assert abs(r.s_offset - 300.0) < 1.0


def test_rtk_fixes_preferred():
    # 25 non-RTK fixes 6 m ahead along the track, 5 RTK fixes at the true place
    ini, _ = new()
    for i in range(30):
        t = T0 + 0.1 * i
        rtk = i >= 25
        mx = X0 + 300.0 + (0.0 if rtk else 6.0)
        lat, lon = xy_to_latlon(mx, Y0)
        ini.add_fix('master', t, lat, lon, Z_MAP + Z_OFF, 2 if rtk else 0, 0.0)
    assert ini.maybe_finalize(T0 + 3.5)
    r = ini.result()
    assert r.rtk and abs(r.s_offset - 300.0) < 0.05


def test_no_rover():
    # on-map, nearest track wins
    ini, _ = new()
    feed(ini, (X0 + 300.0, Y0 + SEP), 0.0, rover=False)
    r = ini.result()
    assert r.direction == 'S2T' and r.kind == 'onmap' and r.diag['heading_src'] == 'none'
    # between the tracks, standing: route whose start is nearer (T2S starts at X0)
    ini, _ = new()
    feed(ini, (X0 + 300.0, Y0 + 0.5 * SEP), 0.0, rover=False)
    assert ini.result().direction == 'T2S' and ini.result().diag['rule'] == 'onmap_start'
    # between the tracks, moving west: motion direction -> S2T
    ini, _ = new()
    feed(ini, (X0 + 300.0, Y0 + 0.5 * SEP), math.pi, rover=False, speed=3.0)
    r = ini.result()
    assert r.direction == 'S2T' and r.diag['heading_src'] == 'motion' and r.diag['rule'] == 'onmap_heading'
    # off-map without heading: bridge along the chord, inflated sigma
    ini, p = new()
    feed(ini, (X0 - 120.0, Y0 - 30.0), 0.0, rover=False)
    r = ini.result()
    assert r.kind == 'bridge' and r.direction == 'T2S' and r.diag['heading_src'] == 'none'
    assert r.sigma0 > p.sigma0_bridge_t2s_m


def test_rover_only():
    ini2, _ = new()
    for i in range(30):
        lat, lon = xy_to_latlon(X0 + 312.44, Y0)
        ini2.add_fix('rover', T0 + 0.1 * i, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
    assert ini2.maybe_finalize(T0 + 3.5)
    r = ini2.result()
    assert r.diag['rover_only'] and r.direction == 'T2S' and r.kind == 'onmap'
    assert abs(r.s_offset - 300.0) < 0.05
    assert abs(r.start_xyz[0] - (X0 + 300.0)) < 0.05


def test_rover_baseline_outlier_rejected():
    # rover 30 m away (wrong fix): no heading from pairs
    ini, _ = new()
    for i in range(30):
        t = T0 + 0.1 * i
        lat, lon = xy_to_latlon(X0 + 300.0, Y0)
        ini.add_fix('master', t, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
        lat, lon = xy_to_latlon(X0 + 270.0, Y0)
        ini.add_fix('rover', t, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
    assert ini.maybe_finalize(T0 + 3.5)
    r = ini.result()
    assert r.diag['heading_src'] == 'none' and r.direction == 'T2S'


def test_provisional_then_final_once():
    ini, _ = new()
    assert ini.result() is None
    lat, lon = xy_to_latlon(X0 + 300.0, Y0)
    ini.add_fix('master', T0, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
    r = ini.result()
    assert r is not None and not r.final and r.direction == 'T2S'
    for i in range(1, 30):
        ini.add_fix('master', T0 + 0.1 * i, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
        assert not ini.maybe_finalize(T0 + 0.1 * i)
    assert not ini.done
    assert ini.maybe_finalize(T0 + 3.0)
    assert ini.done and ini.result().final
    assert not ini.maybe_finalize(T0 + 4.0)
    # later fixes are ignored
    lat2, lon2 = xy_to_latlon(X0 + 600.0, Y0)
    ini.add_fix('master', T0 + 5.0, lat2, lon2, Z_MAP + Z_OFF, 2, 0.0)
    assert abs(ini.result().s_offset - 300.0) < 0.05


def test_window_waits_for_min_fixes_until_timeout():
    ini, p = new()
    lat, lon = xy_to_latlon(X0 + 300.0, Y0)
    ini.maybe_finalize(T0)
    ini.add_fix('master', T0 + 1.0, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
    assert not ini.maybe_finalize(T0 + 4.5)                    # window closed, 1 < init_min_fixes
    assert ini.maybe_finalize(T0 + p.init_timeout_s)
    assert ini.result().kind == 'onmap'


def test_timeout_without_fix():
    ini, p = new()
    assert not ini.maybe_finalize(T0)
    assert not ini.maybe_finalize(T0 + 5.0)
    # unusable fixes do not count
    lat, lon = xy_to_latlon(X0 + 300.0, Y0)
    ini.add_fix('master', T0 + 5.1, lat, lon, Z_MAP + Z_OFF, -1, 0.0)
    ini.add_fix('master', T0 + 5.2, float('nan'), lon, Z_MAP + Z_OFF, 2, 0.0)
    ini.add_fix('master', T0 + 5.3, 0.0, 0.0, Z_MAP + Z_OFF, 2, 0.0)
    assert ini.result() is None
    assert ini.maybe_finalize(T0 + p.init_timeout_s)
    r = ini.result()
    assert ini.done and r.final and r.kind == 'none' and r.direction == ''
    assert math.isnan(r.s_offset)
    assert not ini.maybe_finalize(T0 + 20.0)


def test_glitched_first_stamp_cannot_block_the_window():
    # the first fix carries a far-future header stamp: the timeout (caller's clock) still closes
    ini, p = new()
    lat, lon = xy_to_latlon(X0 + 300.0, Y0)
    ini.add_fix('master', T0 + 1e6, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
    done_at = None
    for i in range(1, 200):
        ini.add_fix('master', T0 + 0.1 * i, lat, lon, Z_MAP + Z_OFF, 2, 0.0)
        if ini.maybe_finalize(T0 + 0.1 * i):
            done_at = 0.1 * i
            break
    assert done_at is not None and done_at <= p.init_timeout_s + 0.2
    r = ini.result()
    assert r.final and r.direction == 'T2S' and abs(r.s_offset - 300.0) < 0.05


def test_missing_altitude_falls_back_to_map_height():
    for alt in (float('nan'), None):
        ini, p = new()
        lat, lon = xy_to_latlon(X0 + 300.0, Y0)
        for i in range(30):
            ini.add_fix('master', T0 + 0.1 * i, lat, lon, alt, 2, 0.0)
        assert ini.maybe_finalize(T0 + 3.1)
        r = ini.result()
        assert r.kind == 'onmap' and abs(r.start_xyz[2] - (Z_MAP + p.z_offset_m)) < 1e-6
        ini, p = new()
        lat, lon = xy_to_latlon(X0 - 100.0, Y0 - 20.0)
        for i in range(30):
            ini.add_fix('master', T0 + 0.1 * i, lat, lon, alt, 2, 0.0)
        assert ini.maybe_finalize(T0 + 3.1)
        r = ini.result()
        assert r.kind == 'bridge' and abs(r.start_xyz[2] - (Z_MAP + p.z_offset_m)) < 1e-6
        assert all(math.isfinite(z) for z in r.head['z'])


def test_long_bridge_sigma_grows_with_length():
    # far outside the calibrated terminus approaches (e.g. a depot): sigma0 scales with L
    ini, p = new()
    feed(ini, (X0 - 3000.0, Y0 - 2000.0), 0.3)
    r = ini.result()
    assert r.kind == 'bridge' and r.head['L'] > 3000.0
    assert abs(r.sigma0 - 0.03 * r.head['L']) < 1e-6 and r.sigma0 > p.sigma0_bridge_t2s_m
    # a short approach keeps the calibrated constant
    ini, p = new()
    feed(ini, (X0 - 150.0, Y0 - 40.0), math.radians(20.0))
    assert ini.result().sigma0 == p.sigma0_bridge_t2s_m


def test_out_of_order_stamps_and_numpy_scalars():
    import numpy as np
    ini, _ = new()
    idx = list(range(30))
    random.Random(1).shuffle(idx)
    for i in idx:
        t = np.float64(T0 + 0.1 * i)
        lat, lon = xy_to_latlon(X0 + 200.0 + 0.5 * i, Y0)
        ini.add_fix('master', t, np.float64(lat), np.float64(lon), np.float32(Z_MAP + Z_OFF), np.int8(2), np.float64(0.5 * i))
        lat, lon = xy_to_latlon(X0 + 212.44 + 0.5 * i, Y0)
        ini.add_fix('rover', t, lat, lon, Z_MAP + Z_OFF, np.int8(2), 0.5 * i)
    assert ini.maybe_finalize(np.float64(T0 + 6.0))       # window opens at the first-arrived stamp (<= T0 + 2.9)
    r = ini.result()
    assert r.direction == 'T2S' and r.kind == 'onmap' and abs(r.s_offset - 200.0) < 0.05
    assert 14.0 <= r.diag['odo_ref'] <= 14.5          # reference = latest stamps, not latest arrivals


if __name__ == '__main__':
    n = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            n += 1
            print('ok', name)
    print('%d tests passed' % n)
