"""Pathgraph route geometry.

A route (one per travel direction) is resampled on a uniform ~1 m arc-length
grid: x, y, z, heading, curvature and smoothed grade dz/ds.  Arc length is the
cumulative segment length (map point spacing is not exactly 1 m).

Runs start and end off the map (terminus loops).  ``RunPath`` extends the
route for one run:
  * s < 0   : G1 cubic Hermite bridge from the initial pose (GNSS at start)
              to the map entry pose, parametrised by arc length;
  * s > L   : extrapolation past the map end.  'straight' follows the end
              tangent; 'decay' (default) continues the end curvature and lets
              it decay exponentially to zero over ``decay_m`` (a generic
              transition-curve model: heading(d) = th + k*lam*(1 - exp(-d/lam))).
  * terminus loops (maps/loops.json, built offline by tools/build_loops.py from
    RTK tracks of the training runs; the organisers allow restored loop
    geometry): when available the head is the loop branch leading to the map
    start and the tail follows the loop branch leaving the map end; past the
    branch end the pose continues straight.
Nothing is ever clamped to the map.
"""
import json
import math
import os

import numpy as np

ROUTE_FILES = {'T2S': 't2s.json', 'S2T': 's2t.json'}


def _moving_average(x, n):
    n = max(1, int(round(n)))
    if n <= 1:
        return x.copy()
    pad = n // 2
    xp = np.r_[np.full(pad, x[0]), x, np.full(n - 1 - pad, x[-1])]
    c = np.cumsum(np.r_[0.0, xp])
    return (c[n:] - c[:-n]) / n


class Route:
    def __init__(self, name, points, grade_window_m=31.0):
        self.name = name
        P = np.array([[p['x'], p['y'], p['z'], p.get('curv', 0.0)] for p in points], float)
        seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
        keep = np.r_[True, seg > 1e-6]
        P = P[keep]
        s = np.r_[0.0, np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))]
        self.L = float(s[-1])
        n = int(math.ceil(self.L)) + 1
        sg = np.linspace(0.0, self.L, n)
        self.ds = float(sg[1] - sg[0])       # python float: keeps the hot-path lookups numpy-free
        self.sg = sg
        self.x = np.interp(sg, s, P[:, 0])
        self.y = np.interp(sg, s, P[:, 1])
        self.z = np.interp(sg, s, P[:, 2])
        self.curv = np.interp(sg, s, P[:, 3])
        hd = np.unwrap(np.arctan2(np.gradient(self.y), np.gradient(self.x)))
        self.yaw = hd
        self.grade = _moving_average(np.gradient(self.z, self.ds), grade_window_m / self.ds)
        # python lists for fast scalar lookups in the hot path
        self._x = self.x.tolist(); self._y = self.y.tolist(); self._z = self.z.tolist()
        self._yaw = self.yaw.tolist(); self._g = self.grade.tolist(); self._k = self.curv.tolist()
        self._n = n
        # polyline segments for projection
        self._A = np.c_[self.x[:-1], self.y[:-1]]
        d = np.c_[np.diff(self.x), np.diff(self.y)]
        self._segL = np.hypot(d[:, 0], d[:, 1])
        self._U = d / self._segL[:, None]

    @classmethod
    def load(cls, name, maps_dir, grade_window_m=31.0):
        with open(os.path.join(maps_dir, ROUTE_FILES[name])) as f:
            j = json.load(f)
        return cls(name, j['points'], grade_window_m)

    # ------------------------------------------------------------ lookups
    def _lin(self, arr, s):
        f = s / self.ds
        if f <= 0:
            return arr[0]
        if f >= self._n - 1:
            return arr[-1]
        i = int(f)
        w = f - i
        return arr[i] * (1 - w) + arr[i + 1] * w

    def pose(self, s):
        """x, y, z, yaw for 0 <= s <= L (clamped)."""
        return (self._lin(self._x, s), self._lin(self._y, s), self._lin(self._z, s), self._lin(self._yaw, s))

    def grade_at(self, s):
        if s < 0 or s > self.L:
            return 0.0
        return self._lin(self._g, s)

    def curv_at(self, s):
        if s < 0 or s > self.L:
            return 0.0
        return self._lin(self._k, s)

    def grade_vec(self, s):
        """Vectorised grade lookup (0 outside the map)."""
        g = np.interp(s, self.sg, self.grade)
        return np.where((s < 0) | (s > self.L), 0.0, g)

    def project(self, x, y):
        """Closest point: (s, distance, signed lateral (left +), beyond_end flag).

        Before the start / after the end the tangent line is used, so s can be
        negative or larger than L (flagged as beyond)."""
        q = np.array([x, y], float)
        r = q - self._A
        t = np.einsum('ij,ij->i', r, self._U)
        tc = np.clip(t, 0.0, self._segL)
        c = self._A + self._U * tc[:, None]
        d = np.hypot(*(q - c).T)
        i = int(np.argmin(d))
        lat = float(self._U[i, 0] * r[i, 1] - self._U[i, 1] * r[i, 0])
        s = float(self.sg[i] + tc[i])
        dist = float(d[i])
        beyond = False
        if i == 0 and t[0] < 0:
            s = float(t[0]); dist = abs(lat); beyond = True
        elif i == len(self._segL) - 1 and t[i] > self._segL[i]:
            s = float(self.sg[i] + t[i]); dist = abs(lat); beyond = True
        return s, dist, lat, beyond


def load_routes(maps_dir, grade_window_m=31.0):
    return {k: Route.load(k, maps_dir, grade_window_m) for k in ROUTE_FILES}


LOOPS_FILE = 'loops.json'


def _branch(br):
    x = np.asarray(br['x'], float); y = np.asarray(br['y'], float); z = np.asarray(br['z'], float)
    u = np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    return dict(name=br.get('name', ''), L=float(u[-1]), u=u, x=x, y=y, z=z, yaw=yaw)


def load_loops(maps_dir):
    """Terminus loops: {loop: {'from', 'to', 'branches': [branch dicts]}} or None (no file)."""
    path = os.path.join(maps_dir, LOOPS_FILE)
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        j = json.load(f)
    out = {}
    for name, lp in j.get('loops', {}).items():
        brs = [_branch(b) for b in lp.get('branches', []) if len(b.get('x', [])) >= 2]
        if brs:
            out[name] = {'from': lp['from'], 'to': lp['to'], 'branches': brs}
    return out or None


def branch_as_head(br):
    """Loop branch ending at the route start as a RunPath head (s = u - L <= 0)."""
    return dict(L=br['L'], s=br['u'] - br['L'], x=br['x'], y=br['y'], z=br['z'], yaw=br['yaw'],
                name=br.get('name', ''))


def project_poly(x, y, px, py, ps, lo=None, hi=None):
    """Closest point of the polyline (px, py) with arc length ps, restricted to lo <= s <= hi.

    Returns (s, dist) or None when the window holds no segment."""
    n = len(ps)
    i0 = 0 if lo is None else max(0, int(np.searchsorted(ps, lo)) - 1)
    i1 = n if hi is None else min(n, int(np.searchsorted(ps, hi)) + 1)
    if i1 - i0 < 2:
        return None
    ax = px[i0:i1 - 1]; ay = py[i0:i1 - 1]
    dx = px[i0 + 1:i1] - ax; dy = py[i0 + 1:i1] - ay
    l2 = dx * dx + dy * dy
    l2 = np.where(l2 > 1e-12, l2, 1e-12)
    t = np.clip(((x - ax) * dx + (y - ay) * dy) / l2, 0.0, 1.0)
    cx = ax + t * dx; cy = ay + t * dy
    d = np.hypot(x - cx, y - cy)
    k = int(np.argmin(d))
    s = float(ps[i0 + k] + t[k] * (ps[i0 + k + 1] - ps[i0 + k]))
    return s, float(d[k])


def hermite_bridge(p0, th0, p1, th1, k=1.5, z0=None, z1=None, step=1.0):
    """G1 cubic Hermite curve p0 -> p1 with end headings th0, th1.

    Tangent magnitudes are k * chord (k calibrated offline per route,
    bridge_tangent_k_t2s 1.55 / bridge_tangent_k_s2t 1.25: median start error
    +0.11 / -0.42 m over 29 / 31 bridge starts with a known heading,
    tools/calibration/init_bridge.py)."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    chord = float(np.hypot(*(p1 - p0)))
    if chord < 1e-3:
        return None
    m0 = k * chord * np.array([math.cos(th0), math.sin(th0)])
    m1 = k * chord * np.array([math.cos(th1), math.sin(th1)])
    t = np.linspace(0.0, 1.0, 4001)[:, None]
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1; h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2; h11 = t ** 3 - t ** 2
    c = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
    cs = np.r_[0.0, np.cumsum(np.hypot(*np.diff(c, axis=0).T))]
    L = float(cs[-1])
    n = max(2, int(math.ceil(L / step)) + 1)
    sg = np.linspace(0.0, L, n)
    x = np.interp(sg, cs, c[:, 0]); y = np.interp(sg, cs, c[:, 1])
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    if z0 is None or z1 is None:
        z = np.zeros(n)
    else:
        z = z0 + (z1 - z0) * sg / L
    return dict(L=L, s=sg - L, x=x, y=y, z=z, yaw=yaw)


class RunPath:
    """Route of one run: bridge (s<0) + map (0..L) + tail (s>L)."""

    def __init__(self, route, head=None, tail_mode='decay', decay_m=40.0, curv_window_m=20.0,
                 tail_len_m=3000.0, tails=None, heads=None):
        self.route = route
        self.tail_mode = tail_mode
        self.tails = list(tails or [])          # loop branches leaving the map end (alternatives)
        self.tail_idx = 0
        self.heads = list(heads or ([head] if head is not None else []))   # alternatives of the head
        self.head = None
        self.head_idx = 0
        self.set_head(head)
        x0, y0, z0, yaw0 = route.pose(0.0)
        self._start = (x0, y0, z0, yaw0)
        # end tangent from the last ~10 m (robust to a noisy last point)
        xe, ye, ze, _ = route.pose(route.L)
        xb, yb, _, _ = route.pose(max(0.0, route.L - 10.0))
        self._end = (xe, ye, ze, math.atan2(ye - yb, xe - xb))
        self._tail = None
        if tail_mode == 'decay' and decay_m > 0:
            th = route.yaw[-1]
            w = min(curv_window_m, route.L)
            k = (route.yaw[-1] - route._lin(route._yaw, route.L - w)) / max(w, 1e-6)
            g = np.arange(0.0, tail_len_m + 1.0, 1.0)
            hd = th + k * decay_m * (1.0 - np.exp(-g / decay_m))
            tx = xe + np.r_[0.0, np.cumsum(np.cos(0.5 * (hd[1:] + hd[:-1])))]
            ty = ye + np.r_[0.0, np.cumsum(np.sin(0.5 * (hd[1:] + hd[:-1])))]
            self._tail = (tx.tolist(), ty.tolist(), hd.tolist(), len(g))
        elif tail_mode not in ('straight', 'decay'):
            raise ValueError('tail_mode must be straight|decay')

    def set_head(self, head):
        self.head = head
        if head is not None:
            for i, h in enumerate(self.heads):
                if h is head:
                    self.head_idx = i
            self._hs = list(map(float, head['s'])); self._hx = list(map(float, head['x']))
            self._hy = list(map(float, head['y'])); self._hz = list(map(float, head['z']))
            self._hyaw = list(map(float, head['yaw']))
            self._hds = float(head['s'][1] - head['s'][0]) if len(head['s']) > 1 else 1.0
            self._hu = np.asarray(head['s'], float)

    def set_tail(self, idx):
        if 0 <= idx < len(self.tails):
            self.tail_idx = idx

    @property
    def s_min(self):
        return -self.head['L'] if self.head is not None else 0.0

    @property
    def tail_L(self):
        """Length of the known (loop) track past the map end, 0 without loops."""
        return self.tails[self.tail_idx]['L'] if self.tails else 0.0

    def project_near(self, x, y, s_pred, win):
        """Closest point to (x, y) with |s - s_pred| <= win over map, head and tail alternatives.

        Returns (s, dist, part, branch_index) with part 'map' | 'head' | 'tail', or None."""
        r = self.route
        lo, hi = s_pred - win, s_pred + win
        best = None
        if hi >= 0.0 and lo <= r.L:
            q = project_poly(x, y, r.x, r.y, r.sg, lo, hi)
            if q is not None:
                best = (q[0], q[1], 'map', 0)
        if lo < 0.0:
            for i, h in enumerate(self.heads):
                q = project_poly(x, y, np.asarray(h['x']), np.asarray(h['y']), np.asarray(h['s'], float), lo, hi)
                if q is not None and (best is None or q[1] < best[1]):
                    best = (q[0], q[1], 'head', i)
        if hi > r.L:
            for i, b in enumerate(self.tails):
                q = project_poly(x, y, b['x'], b['y'], b['u'] + r.L, lo, hi)
                if q is not None and (best is None or q[1] < best[1]):
                    best = (q[0], q[1], 'tail', i)
        return best

    def on_map(self, s):
        return 0.0 <= s <= self.route.L

    def pose(self, s):
        r = self.route
        if 0.0 <= s <= r.L:
            return r.pose(s)
        if s > r.L:
            x, y, z, yaw = self._end
            d = s - r.L
            if self.tails:
                b = self.tails[self.tail_idx]
                u = b['u']
                if d <= b['L']:
                    i = min(int(np.searchsorted(u, d)) - 1, len(u) - 2)
                    i = max(i, 0)
                    w = (d - u[i]) / max(u[i + 1] - u[i], 1e-9)
                    lin = lambda a: float(a[i] * (1 - w) + a[i + 1] * w)
                    return lin(b['x']), lin(b['y']), lin(b['z']), lin(b['yaw'])
                x, y, z, yaw = float(b['x'][-1]), float(b['y'][-1]), float(b['z'][-1]), float(b['yaw'][-1])
                d -= b['L']
                return x + d * math.cos(yaw), y + d * math.sin(yaw), z, yaw
            if self._tail is not None:
                tx, ty, th, n = self._tail
                if d < n - 1:
                    i = int(d)
                    w = d - i
                    return (tx[i] * (1 - w) + tx[i + 1] * w, ty[i] * (1 - w) + ty[i + 1] * w, z,
                            th[i] * (1 - w) + th[i + 1] * w)
                x, y, yaw, d = tx[-1], ty[-1], th[-1], d - (n - 1)
            return x + d * math.cos(yaw), y + d * math.sin(yaw), z, yaw
        # s < 0
        h = self.head
        if h is not None and s >= h['s'][0]:
            f = (s - self._hs[0]) / self._hds
            i = min(int(f), len(self._hs) - 2)
            w = f - i
            lin = lambda a: a[i] * (1 - w) + a[i + 1] * w
            return lin(self._hx), lin(self._hy), lin(self._hz), lin(self._hyaw)
        if h is not None:
            # behind the bridge start (roll-back before departure): straight line
            x, y, z, yaw = h['x'][0], h['y'][0], h['z'][0], h['yaw'][0]
            d = s - h['s'][0]
            return x + d * math.cos(yaw), y + d * math.sin(yaw), z, yaw
        x, y, z, yaw = self._start
        return x + s * math.cos(yaw), y + s * math.sin(yaw), z, yaw

    def grade_at(self, s):
        return self.route.grade_at(s)

    def curv_at(self, s):
        return self.route.curv_at(s)
