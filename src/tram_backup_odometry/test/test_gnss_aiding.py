"""GNSS aiding (bursts after the alignment), loop branches and base_link output."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tram_backup_odometry.core.config import Params                # noqa: E402
from tram_backup_odometry.core.frames import map_to_latlon         # noqa: E402
from tram_backup_odometry.core.pipeline import Pipeline            # noqa: E402
from tram_backup_odometry.core.track_map import RunPath, _branch, load_routes  # noqa: E402
from tram_backup_odometry.replay import default_maps_dir           # noqa: E402

MAPS = default_maps_dir()
ROUTE = load_routes(MAPS)['T2S']
S0 = 300.0
T0 = 1_700_000_000.0


def true_s(t, still=5.0, acc=1.0, vmax=10.0):
    if t <= still:
        return S0
    ta = vmax / acc
    d = t - still
    if d <= ta:
        return S0 + 0.5 * acc * d * d
    return S0 + 0.5 * acc * ta * ta + vmax * (d - ta)


def true_v(t, still=5.0, acc=1.0, vmax=10.0):
    return 0.0 if t <= still else min(vmax, acc * (t - still))


def drive(p, t_end=200.0, wheel_scale=1.01, bursts=(60.0, 120.0, 180.0), burst_s=1.5):
    """Wheels 1 % fast (a K error); RTK master fixes in the start window and in short bursts."""
    pipe = Pipeline(p, MAPS)
    k = p.k_for_vehicle()
    out = []
    n = int(t_end / 0.05)
    for i in range(n + 1):
        t = i * 0.05
        th = T0 + t
        notch = 0 if t < 5.0 else (5 if true_v(t) < 10.0 else 0)
        pipe.on_cmd(th, notch)
        if i % 2 == 0:
            gnss = t <= 4.0 or any(b <= t <= b + burst_s for b in bursts)
            if gnss:
                for ant, off in (('master', 0.0), ('rover', p.rover_ahead_m)):
                    x, y, z, _ = ROUTE.pose(true_s(t) + off)
                    lat, lon = map_to_latlon(x, y)
                    pipe.on_fix(ant, th, lat, lon, z + p.z_offset_m, 2)
            r = pipe.on_wheel('front', th + 0.007, true_v(t + 0.007) * k * wheel_scale)
            pipe.on_wheel('rear', th + 0.007, true_v(t + 0.007) * k * wheel_scale)   # same stamp: no output
            if r is not None and r.has_pose:
                out.append((t + 0.007, r))
    return pipe, out


def along_err(rows, p, t):
    """Output minus truth (base_link point) at the output closest to t, in metres along the route."""
    tt, r = min(rows, key=lambda q: abs(q[0] - t))
    x, y, _, yaw = ROUTE.pose(true_s(tt) + p.output_along_offset_m)
    return (r.x - x) * math.cos(yaw) + (r.y - y) * math.sin(yaw)


def params(**kw):
    p = Params()
    p.vehicle_id = '30618'
    p.output_velocity_delay_s = 0.0
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def test_bursts_correct_along_track_drift_and_never_touch_speed():
    pa, pn = params(), params(gnss_aiding=False)
    pipe_a, ra = drive(pa)
    pipe_n, rn = drive(pn)
    assert pipe_a.n_bursts >= 3 and pipe_a.gnss_active and not pipe_a.gnss_done
    assert pipe_n.gnss_done and pipe_n.n_bursts == 0
    # right after a burst the along-track error is small; the 1 % wheel error makes ~10 m
    # after 1.7 km without aiding
    for tb in (62.0, 122.0, 182.0):
        assert abs(along_err(ra, pa, tb)) < 1.0, (tb, along_err(ra, pa, tb))
    assert abs(along_err(ra, pa, 199.0)) < 3.0
    assert abs(along_err(rn, pn, 199.0)) > 5.0
    # speed never uses GNSS: identical velocity streams
    va = np.array([r.v for _, r in ra]); vn = np.array([r.v for _, r in rn])
    m = min(len(va), len(vn))
    assert np.allclose(va[:m], vn[:m], atol=1e-9)


def test_scale_is_learned_between_rtk_bursts():
    p = params()
    pipe, rows = drive(p, t_end=260.0, bursts=(60.0, 120.0, 180.0, 240.0))
    # wheels 1 % fast -> the position scale is ~ -1 %
    assert pipe.scale_known and -0.013 < pipe.pos_eps < -0.004, pipe.pos_eps
    assert abs(along_err(rows, p, 239.0)) < 3.0


def test_start_window_burst_keeps_trn_corrections():
    """Fixes only in the start window (before TRN runs): the anchor must still let the TRN
    corrections through afterwards (anchor_delta = the anchor's offset from the TRN path)."""
    p = params()
    pipe, _ = drive(p, t_end=4.8, bursts=())       # standing: the burst is flushed at ~4.55 s
    assert pipe.gnss_active and not pipe.trn_active and pipe.n_bursts >= 1
    assert pipe.anchor_delta is not None
    assert abs(pipe.anchor_delta - (pipe.anchor_s - pipe.s_offset - pipe.anchor_odo)) < 1e-9
    # a later TRN offset shows up in the published position (with the blend weight)
    pipe.trn_active, pipe.delta_std = True, 1.0
    so = pipe.anchor_odo + 1000.0
    s0 = pipe._s_master(so)
    pipe.delta_applied += 5.0
    assert 3.0 < pipe._s_master(so) - s0 <= 5.0


def test_inconsistent_burst_rejected():
    p = params()
    pipe = Pipeline(p, MAPS)
    k = p.k_for_vehicle()
    x, y, z, yaw = ROUTE.pose(S0)
    n0 = None
    for i in range(300):
        t = i * 0.05
        pipe.on_cmd(T0 + t, 0)
        if i % 2 == 0:
            if t <= 4.0:
                pipe.on_fix('master', T0 + t, *map_to_latlon(x, y), z + p.z_offset_m, 2)
                pipe.on_fix('rover', T0 + t, *map_to_latlon(*ROUTE.pose(S0 + p.rover_ahead_m)[:2]), z, 2)
            if 10.0 <= t < 11.0:
                if n0 is None:
                    assert pipe.init_done
                    n0 = pipe.n_bursts
                # a burst 30 m off the track (multipath / wrong street) is gated out
                lat, lon = map_to_latlon(x - 30.0 * math.sin(yaw), y + 30.0 * math.cos(yaw))
                pipe.on_fix('master', T0 + t, lat, lon, z, 2)
            pipe.on_wheel('front', T0 + t, 0.0 * k)
            pipe.on_wheel('rear', T0 + t, 0.0 * k)
    assert pipe.n_bursts == n0 and pipe.n_bursts_rejected >= 1


def test_run_path_loop_tail_and_project_near():
    L = ROUTE.L
    x1, y1, z1, yaw1 = ROUTE.pose(L)
    # a 90 deg left arc of radius 30 m leaving the route end
    th = np.linspace(0.0, math.pi / 2, 60)
    cx, cy = x1 - 30.0 * math.sin(yaw1), y1 + 30.0 * math.cos(yaw1)
    bx = cx + 30.0 * np.sin(yaw1 + th)
    by = cy - 30.0 * np.cos(yaw1 + th)
    br = _branch(dict(name='A', x=bx, y=by, z=np.full_like(bx, z1)))
    path = RunPath(ROUTE, None, tails=[br])
    assert abs(path.tail_L - br['L']) < 1e-9 and abs(br['L'] - 30.0 * math.pi / 2) < 0.1
    # pose on the branch follows the arc
    x, y, _, yaw = path.pose(L + br['L'])
    assert math.hypot(x - bx[-1], y - by[-1]) < 0.05
    assert abs(((yaw - (yaw1 + math.pi / 2)) + math.pi) % (2 * math.pi) - math.pi) < 0.05
    # projection of a point on the arc lands on the tail with the right arc length
    k = 30
    q = path.project_near(bx[k], by[k], L + br['u'][k] + 5.0, 40.0)
    assert q is not None and q[2] == 'tail' and abs(q[0] - (L + br['u'][k])) < 0.05 and q[1] < 0.05
    # on the map part
    x, y, _, _ = ROUTE.pose(1000.0)
    q = path.project_near(x, y, 1010.0, 80.0)
    assert q[2] == 'map' and abs(q[0] - 1000.0) < 0.05
    # the window limits the search: the true point is outside it, so only a far one is found
    q = path.project_near(x, y, 2000.0, 80.0)
    assert q is None or (q[1] > 100.0 and abs(q[0] - 2000.0) <= 80.0 + 0.5)
