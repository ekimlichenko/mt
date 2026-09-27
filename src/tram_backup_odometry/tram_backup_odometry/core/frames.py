"""Coordinate frames.

The pathgraph maps are in UTM zone 37N (WGS84) minus a constant offset
(300000, 6100000) — verified against RTK fixes to ~2 cm, no rotation/scale.
That frame is the default output frame ('map').  Alternatives: 'utm' (full
UTM 37N coordinates), 'start' (map frame shifted to the initial position) and
'enu' (local East-North-Up tangent plane of the WGS84 ellipsoid at the initial
position, or at a fixed geodetic origin).  'enu' differs from 'start' by the
UTM grid convergence (~1.33 deg here: ~100 m at 4.7 km), the UTM scale factor
and the Earth curvature in Up, so it matters when the reference trajectory is
built in a geodetic local frame.
"""
import math

MAP_OFFSET_E = 300000.0
MAP_OFFSET_N = 6100000.0

_A = 6378137.0
_F = 1 / 298.257223563


def utm(lat, lon, lon0=39.0, k0=0.9996, fe=500000.0):
    """WGS84 lat/lon (deg) -> UTM easting/northing (m), Krueger series (mm accuracy)."""
    n = _F / (2 - _F)
    A = _A / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64)
    al = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16,
          13 * n ** 2 / 48 - 3 * n ** 3 / 5,
          61 * n ** 3 / 240)
    e = math.sqrt(_F * (2 - _F))
    phi = math.radians(lat)
    lam = math.radians(lon - lon0)
    t = math.sinh(math.atanh(math.sin(phi)) - e * math.atanh(e * math.sin(phi)))
    xi = math.atan2(t, math.cos(lam))
    eta = math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    E = eta + sum(al[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta) for j in (1, 2, 3))
    N = xi + sum(al[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta) for j in (1, 2, 3))
    return fe + k0 * A * E, k0 * A * N


def utm_inverse(E, N, lon0=39.0, k0=0.9996, fe=500000.0):
    """UTM easting/northing (m) -> WGS84 lat/lon (deg), inverse Krueger series (sub-mm in the zone)."""
    n = _F / (2 - _F)
    A = _A / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64)
    be = (n / 2 - 2 * n ** 2 / 3 + 37 * n ** 3 / 96,
          n ** 2 / 48 + n ** 3 / 15,
          17 * n ** 3 / 480)
    de = (2 * n - 2 * n ** 2 / 3 - 2 * n ** 3,
          7 * n ** 2 / 3 - 8 * n ** 3 / 5,
          56 * n ** 3 / 15)
    xi = N / (k0 * A)
    eta = (E - fe) / (k0 * A)
    xp = xi - sum(be[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta) for j in (1, 2, 3))
    ep = eta - sum(be[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta) for j in (1, 2, 3))
    chi = math.asin(math.sin(xp) / math.cosh(ep))
    phi = chi + sum(de[j - 1] * math.sin(2 * j * chi) for j in (1, 2, 3))
    lam = math.atan2(math.sinh(ep), math.cos(xp))
    return math.degrees(phi), lon0 + math.degrees(lam)


def latlon_to_map(lat, lon):
    E, N = utm(lat, lon)
    return E - MAP_OFFSET_E, N - MAP_OFFSET_N


def map_to_latlon(x, y):
    return utm_inverse(x + MAP_OFFSET_E, y + MAP_OFFSET_N)


def ecef(lat, lon, h):
    """WGS84 geodetic (deg, deg, m) -> ECEF (m)."""
    e2 = _F * (2 - _F)
    p, l = math.radians(lat), math.radians(lon)
    sp, cp = math.sin(p), math.cos(p)
    nr = _A / math.sqrt(1 - e2 * sp * sp)
    return (nr + h) * cp * math.cos(l), (nr + h) * cp * math.sin(l), (nr * (1 - e2) + h) * sp


class LocalENU:
    """East-North-Up tangent plane at a geodetic origin."""

    def __init__(self, lat0, lon0, h0):
        self.origin = (lat0, lon0, h0)
        self.o = ecef(lat0, lon0, h0)
        p, l = math.radians(lat0), math.radians(lon0)
        sp, cp, sl, cl = math.sin(p), math.cos(p), math.sin(l), math.cos(l)
        self.r = ((-sl, cl, 0.0), (-sp * cl, -sp * sl, cp), (cp * cl, cp * sl, sp))

    def from_geodetic(self, lat, lon, h):
        X, Y, Z = ecef(lat, lon, h)
        d = (X - self.o[0], Y - self.o[1], Z - self.o[2])
        return tuple(r[0] * d[0] + r[1] * d[1] + r[2] * d[2] for r in self.r)

    def from_map(self, x, y, z):
        lat, lon = map_to_latlon(x, y)
        return self.from_geodetic(lat, lon, z)


def yaw_to_quat(yaw):
    """Rotation about z -> (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class OutputFrame:
    def __init__(self, params):
        self.mode = params.output_frame
        if self.mode not in ('map', 'utm', 'start', 'enu'):
            raise ValueError('output_frame must be map|utm|start|enu, got %r' % self.mode)
        self.frame_id = {'map': params.frame_id_map, 'utm': params.frame_id_utm,
                         'start': params.frame_id_start,
                         'enu': getattr(params, 'frame_id_enu', 'enu')}[self.mode]
        self.origin = (0.0, 0.0, 0.0)
        self.enu = None
        if self.mode == 'enu' and not getattr(params, 'enu_origin_auto', True):
            self.enu = LocalENU(params.enu_origin_lat, params.enu_origin_lon, params.enu_origin_alt)

    def set_origin(self, x, y, z):
        """Initial position (map frame, z comparable with the GNSS altitude)."""
        self.origin = (x, y, z)
        if self.mode == 'enu' and (self.enu is None or getattr(self, '_enu_auto', False)):
            lat, lon = map_to_latlon(x, y)
            self.enu = LocalENU(lat, lon, z)
            self._enu_auto = True

    def apply(self, x, y, z):
        if self.mode == 'map':
            return x, y, z
        if self.mode == 'utm':
            return x + MAP_OFFSET_E, y + MAP_OFFSET_N, z
        if self.mode == 'enu' and self.enu is not None:
            return self.enu.from_map(x, y, z)
        return x - self.origin[0], y - self.origin[1], z - self.origin[2]

    def apply_yaw(self, x, y, yaw):
        """Heading in the output frame (map/utm/start share the grid axes; enu adds the grid convergence)."""
        if self.mode != 'enu' or self.enu is None:
            return yaw
        e0, n0, _ = self.enu.from_map(x, y, 0.0)
        e1, n1, _ = self.enu.from_map(x + math.cos(yaw), y + math.sin(yaw), 0.0)
        return math.atan2(n1 - n0, e1 - e0)
