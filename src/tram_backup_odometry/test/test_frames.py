"""Output frames: inverse UTM, local ENU and the OutputFrame modes."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tram_backup_odometry.core.config import Params  # noqa: E402
from tram_backup_odometry.core.frames import (LocalENU, OutputFrame, ecef, latlon_to_map,  # noqa: E402
                                              map_to_latlon, utm, utm_inverse)

# a point on the T2S route (map frame) and its surroundings
X0, Y0, Z0 = 101200.0, 85300.0, 170.0


def test_utm_inverse_round_trip_sub_mm():
    for dx in (-3000.0, 0.0, 2500.0):
        for dy in (-2000.0, 0.0, 1800.0):
            lat, lon = map_to_latlon(X0 + dx, Y0 + dy)
            x, y = latlon_to_map(lat, lon)
            assert abs(x - (X0 + dx)) < 1e-3 and abs(y - (Y0 + dy)) < 1e-3
    lat, lon = 55.8, 37.4
    assert all(abs(a - b) < 2e-8 for a, b in zip(utm_inverse(*utm(lat, lon)), (lat, lon)))   # 2e-8 deg ~ 2 mm


def test_local_enu_basics():
    lat0, lon0 = map_to_latlon(X0, Y0)
    enu = LocalENU(lat0, lon0, Z0)
    assert all(abs(c) < 1e-6 for c in enu.from_map(X0, Y0, Z0))
    # 100 m straight up in ECEF along the local normal is +100 m Up
    e, n, u = enu.from_geodetic(lat0, lon0, Z0 + 100.0)
    assert abs(e) < 1e-6 and abs(n) < 1e-6 and abs(u - 100.0) < 1e-6
    # a point due geodetic north has ~no East component and ~no Up at 1 km (curvature 0.08 m)
    e, n, u = enu.from_geodetic(lat0 + 0.009, lon0, Z0)
    assert abs(e) < 1e-6 and 990 < n < 1010 and -0.1 < u < 0.0
    # horizontal distances agree with the map up to the UTM point scale (0.9996..1.0000 here)
    e, n, u = enu.from_map(X0 + 3000.0, Y0 + 4000.0, Z0)
    r = math.hypot(e, n) / 5000.0
    assert 0.9995 < r < 1.0006
    # the map grid is rotated against true north by the meridian convergence (~ -1.2 deg at lon 37.5)
    conv = math.degrees(math.atan2(n, e) - math.atan2(4000.0, 3000.0))
    assert 0.8 < abs(conv) < 1.6


def test_ecef_known_value():
    X, Y, Z = ecef(0.0, 0.0, 0.0)
    assert abs(X - 6378137.0) < 1e-6 and abs(Y) < 1e-6 and abs(Z) < 1e-6


def test_output_frame_enu_auto_and_fixed_origin():
    p = Params()
    p.output_frame = 'enu'
    f = OutputFrame(p)
    assert f.frame_id == p.frame_id_enu
    f.set_origin(X0, Y0, Z0)
    assert all(abs(c) < 1e-6 for c in f.apply(X0, Y0, Z0))
    f.set_origin(X0 + 10.0, Y0, Z0)                          # a later (final) alignment moves the origin
    e, n, u = f.apply(X0 + 10.0, Y0, Z0)
    assert abs(e) < 1e-6 and abs(n) < 1e-6
    # heading: the grid convergence is added
    yaw_map = 0.3
    yaw = f.apply_yaw(X0, Y0, yaw_map)
    assert 0.8 < abs(math.degrees(yaw - yaw_map)) < 1.6
    # fixed origin: set_origin does not move it
    lat0, lon0 = map_to_latlon(X0, Y0)
    p.enu_origin_auto = False
    p.enu_origin_lat, p.enu_origin_lon, p.enu_origin_alt = lat0, lon0, Z0
    f = OutputFrame(p)
    f.set_origin(X0 + 50.0, Y0 + 50.0, Z0)
    assert all(abs(c) < 1e-6 for c in f.apply(X0, Y0, Z0))


def test_output_frame_other_modes_unchanged():
    p = Params()
    for mode, expect in (('map', (X0, Y0, Z0)), ('utm', (X0 + 300000.0, Y0 + 6100000.0, Z0)),
                         ('start', (0.0, 0.0, 0.0))):
        p.output_frame = mode
        f = OutputFrame(p)
        f.set_origin(X0, Y0, Z0)
        assert f.apply(X0, Y0, Z0) == expect
        assert f.apply_yaw(X0, Y0, 0.7) == 0.7


def test_params_validate_rejects_enum_typos():
    import pytest
    for k, bad in (('publish_on', 'wheel'), ('tail_mode', 'Decay'), ('output_frame', 'ENU')):
        p = Params()
        setattr(p, k, bad)
        with pytest.raises(ValueError):
            p.validate()
    Params().validate()
