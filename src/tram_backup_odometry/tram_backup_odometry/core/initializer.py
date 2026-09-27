"""Initial alignment on the track map from the first seconds of GNSS.

The estimator integrates an odometer (distance since start).  This module
turns the first GNSS fixes into the relation  s_route = s_offset + odometer
on one of the two routes (``T2S`` / ``S2T``) and, when the tram starts off the
map (terminus loops), the loop branch (or a G1 bridge) that carries the pose
until the map entry.  Only the fixes, the pathgraph maps, the restored terminus
loops (maps/loops.json: centrelines from the RTK tracks of the training runs,
tools/build_loops.py; see "Loops" below) and the caller's odometer are used;
there are no other offline GNSS priors (no stop lists).

Fixes
-----
(lat, lon) -> map frame with ``frames.latlon_to_map`` (UTM 37N minus a fixed
offset).  A fix is usable when its status is >= 0 and it is finite and within
``init_max_dist_m`` of the map bounding box.  Master fixes are preferred; if at
least ``init_rtk_min_fixes`` of them are RTK (status 2) only those are used,
because non-RTK fixes can be metres off.

Heading
-------
The rover antenna sits ``rover_ahead_m`` (12.44 m) ahead of the master on the
vehicle axis.  A master/rover pair with header stamps within ``init_pair_dt_s``
and a separation within ``rover_ahead_m +- init_baseline_tol_m`` gives
psi_i = atan2(y_r - y_m, x_r - x_m).  Pairs are ranked (both RTK) > (rover
RTK: the moving-baseline solution makes the vector precise even when the
absolute position is not) > (any status); the best non-empty class is
averaged on the circle, psi = atan2(sum sin psi_i, sum cos psi_i), after
dropping pairs more than ``init_heading_trim_deg`` from a first mean.
Motion heading psi_m: direction from the median of the first third to the
median of the last third of the selected fixes, when that displacement exceeds
``init_motion_min_m`` (RTK) / ``init_motion_min_nortk_m`` (non-RTK) and the
odometer advanced as well.  psi = psi_b when there are pairs, psi_m otherwise;
when both exist, the master fixes are RTK (psi_m then good to a degree) and
they disagree by more than ``init_heading_tol_deg``, the rover has a wrong fix
of the right length (moving-baseline false fix, seen on real data) and psi_m
wins -- the direction of travel is what defines the route.

Reference point
---------------
p = component-wise median of the selected fixes whose odometer reading is
within ``init_ref_odo_m`` of the latest one (all of them at standstill, the
last fix when moving); odo_ref = their median odometer, z_ref their median
altitude.

Route choice
------------
p is projected on both routes: (s_r, d_r, beyond_r).  A route is an on-map
candidate when not beyond its ends and d_r < ``init_onmap_dist_m``; it agrees
with the heading when cos(psi - yaw_r(s_r)) > cos(``init_heading_tol_deg``).
The two routes are antiparallel tracks 3.5 m apart, so the heading sign is
decisive whenever it is known.

1. Heading known: the nearest agreeing on-map candidate; else the nearest
   agreeing route within ``init_near_dist_m`` (parallel bypass tracks,
   non-RTK offsets); else the nearest agreeing route within
   ``init_far_dist_m`` (non-RTK fixes 10-20 m off in street canyons; the
   projection is interior, so the tram is along the route, not before its
   start); else off-map.
2. Heading unknown (no rover pair, standing): the only on-map candidate, or
   the nearer of two unless |d_A - d_B| < ``init_track_margin_m`` -- then
   the route whose start is nearer; with no candidate the nearer route if
   it is within ``init_near_dist_m``; else off-map.
(A moving tram without rover gets its heading from the motion, so rule 1
applies to it.)

On-map:  s_offset = median_i (s_r(fix_i) - odo_i)  (every fix projected).

Off-map (no loop branch within ``loop_max_dist_m``, see "Loops" below): the
route whose START (point 0) is nearest to p.  A tram past the
end of a route (e.g. a bag that starts where the previous run ended) is on
the terminus loop that leads to the start of the *other* route, so the
nearest start is also the right choice there; without a restored loop branch
the bridge below is only its G1 stand-in.  With the chord c = |P0 - p|, the
start tangent t0 = (cos psi, sin psi) and the route tangent t1 at s = 0,
track_map.hermite_bridge builds

    B(u) = h00(u) p + h10(u) k c t0 + h01(u) P0 + h11(u) k c t1,  u in [0, 1],

k = ``bridge_tangent_k_<route>`` (T2S 1.55, S2T 1.25: calibrated so the
bridge length matches the real approach, see BRIDGE_K).  Its arc length L_b
stands in for the unknown approach path, which occupies s in [-L_b, 0], so
s_offset = -L_b - odo_ref.
z along the bridge goes linearly from z_ref - ``z_offset_m`` to z_map(0)
(without any altitude, z_ref = z_map + ``z_offset_m`` at the start / projection).
Without any heading the chord direction is used for t0.
Limitation of the bridge (it applies only when no loop branch is used:
``loops_enable`` false, no maps/loops.json, or the start farther than
``loop_max_dist_m`` from every branch): a tram just past a route END heading
outbound (into the loop) has the next route's start BEHIND it; the bridge then
degenerates into a short out-and-back curve (L ~ 1.3 chord, yaw flips by pi)
while the real loop is hundreds of metres long, so such starts come out
90-460 m short at terminus T (more at S); bag starts at the layover (all
recorded ones) are not affected.

Rover only (no usable master fix): the rover fixes go through the same logic
and the master is placed ``rover_ahead_m`` behind them: s_offset is lowered by
rover_ahead_m, start_xyz is moved back along yaw0.

sigma0: ``sigma0_rtk_onmap_m`` (RTK, on the map), ``sigma0_nortk_m`` (non-RTK
or the near rule), hypot(``sigma0_nortk_m``, d) for the far rule,
max(``sigma0_bridge_{t2s,s2t}_m``, ``init_bridge_sigma_frac`` * L_b) for
bridges (the floor only acts on bridges longer than the calibrated terminus
approaches, e.g. a depot start), combined in quadrature with
``sigma0_nortk_m`` without RTK and with ``init_noheading_sigma_frac * chord``
without a heading.

Loops: with the restored terminus loops (``track_map.load_loops``, maps/loops.json)
an off-map start is first projected on every loop branch; the nearest branch
within ``loop_max_dist_m`` whose tangent agrees with the heading becomes the
head (kind 'loop'): s = u - L_branch at the projection, sigma0 =
``loop_sigma0_m`` (RTK; combined with ``sigma0_nortk_m`` otherwise), and the
route is the branch's target route.  The G1 bridge stays as the fallback.

Life cycle: ``add_fix`` stores the fix (master fixes are projected on both
routes at once, ~0.4 ms); ``result()`` recomputes lazily, ~1 ms (provisional
after the first usable fix); ``maybe_finalize(t_now)`` closes at first_fix_t +
``init_window_s`` (header time; later if fewer than ``init_min_fixes`` fixes),
and at the latest ``init_timeout_s`` after its first call (caller's clock, so a
glitched GNSS stamp cannot hold the window open) -- with kind 'none' (empty
direction, NaN s_offset, no pose) when no usable fix arrived.
"""
import bisect
import math
from dataclasses import dataclass, field
from typing import Optional

from .frames import latlon_to_map
from .track_map import branch_as_head, hermite_bridge, project_poly

_NAN = float('nan')

# Bridge tangent factor per route, calibrated offline on the recorded starts (zero median
# length error, stable under 2-fold and leave-vehicle-out CV).  Overridden by the params
# bridge_tangent_k_<route>; params.bridge_tangent_k is used for other route names.
BRIDGE_K = {'T2S': 1.55, 'S2T': 1.25}


@dataclass
class InitResult:
    direction: str          # 'T2S' | 'S2T' ('' when kind == 'none')
    s_offset: float         # route arc length s = s_offset + odometer (master antenna)
    sigma0: float           # std of s at init, m
    head: Optional[dict]    # track_map.hermite_bridge(...) output when the start is off-map, else None
    start_xyz: tuple        # master antenna map-frame (x, y, z_gnss) at the init reference
    yaw0: float             # heading at the init reference (route tangent on the map), rad
    kind: str               # 'onmap' | 'loop' | 'bridge' | 'none'
    final: bool
    rtk: bool
    diag: dict = field(default_factory=dict)   # rule, heading source, distances (diagnostics only)
    heads: list = field(default_factory=list)  # alternative heads (loop branches into the same route start)


def _median(v):
    n = len(v)
    if n == 0:
        return _NAN
    w = sorted(v)
    h = n // 2
    return w[h] if n % 2 else 0.5 * (w[h - 1] + w[h])


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _circ_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


class _Fix:
    __slots__ = ('t', 'x', 'y', 'z', 'status', 'odo', 'proj')

    def __init__(self, t, x, y, z, status, odo):
        self.t = t; self.x = x; self.y = y; self.z = z
        self.status = status; self.odo = odo
        self.proj = None


class Initializer:
    def __init__(self, params, routes: dict, loops=None):
        self.p = params
        self.routes = routes
        self.loops = loops or {}
        g = lambda name, default: getattr(params, name, default)
        self.window_s = float(g('init_window_s', 3.0))
        self.timeout_s = float(g('init_timeout_s', 10.0))
        self.min_fixes = int(g('init_min_fixes', 5))
        self.onmap_dist = float(g('init_onmap_dist_m', 3.0))
        self.near_dist = float(g('init_near_dist_m', 8.0))
        self.far_dist = float(g('init_far_dist_m', 30.0))
        self.track_margin = float(g('init_track_margin_m', 1.0))
        self.cos_tol = math.cos(math.radians(float(g('init_heading_tol_deg', 60.0))))
        self.trim = math.radians(float(g('init_heading_trim_deg', 20.0)))
        self.ahead = float(g('rover_ahead_m', 12.44))
        self.base_tol = float(g('init_baseline_tol_m', 2.5))
        self.pair_dt = float(g('init_pair_dt_s', 0.1))
        self.rtk_min = int(g('init_rtk_min_fixes', 3))
        self.motion_min = float(g('init_motion_min_m', 2.0))
        self.motion_min_nortk = float(g('init_motion_min_nortk_m', 10.0))
        self.ref_odo = float(g('init_ref_odo_m', 0.5))
        k_all = float(g('bridge_tangent_k', 1.5))
        self.k = {n: float(g('bridge_tangent_k_' + n.lower(), BRIDGE_K.get(n, k_all))) for n in routes}
        self.z_off = float(g('z_offset_m', 3.10))
        self.max_dist = float(g('init_max_dist_m', 20000.0))
        self.noheading_frac = float(g('init_noheading_sigma_frac', 0.2))
        self.bridge_frac = float(g('init_bridge_sigma_frac', 0.03))
        self.sig_onmap = float(g('sigma0_rtk_onmap_m', 3.0))
        self.sig_nortk = float(g('sigma0_nortk_m', 10.0))
        self.sig_bridge = {'T2S': float(g('sigma0_bridge_t2s_m', 15.0)),
                           'S2T': float(g('sigma0_bridge_s2t_m', 40.0))}
        self.sig_bridge_default = max(self.sig_bridge.values())
        self.loop_max_dist = float(g('loop_max_dist_m', 8.0))
        self.sig_loop = float(g('loop_sigma0_m', 1.5))
        xs = [float(r.x.min()) for r in routes.values()] + [float(r.x.max()) for r in routes.values()]
        ys = [float(r.y.min()) for r in routes.values()] + [float(r.y.max()) for r in routes.values()]
        self._bbox = (min(xs), max(xs), min(ys), max(ys))
        self._start = {n: (float(r.x[0]), float(r.y[0]), float(r.z[0]), float(r.yaw[0])) for n, r in routes.items()}
        self._master = []
        self._rover = []
        self._t0 = None
        self._t_first = None
        self._result = None
        self._dirty = False
        self._done = False

    # ------------------------------------------------------------ inputs
    def add_fix(self, antenna, t, lat, lon, alt, status, odo_at_t: float) -> None:
        """NavSatFix of 'master' or 'rover' (header time t, s); odo_at_t = odometer at t, m."""
        if self._done:
            return
        try:
            lat = float(lat); lon = float(lon)
        except (TypeError, ValueError):
            return
        if not (math.isfinite(lat) and math.isfinite(lon)) or abs(lat) > 84.0 or abs(lon) > 180.0:
            return
        if lat == 0.0 and lon == 0.0:
            return
        x, y = latlon_to_map(lat, lon)
        self.add_fix_xy(antenna, t, x, y, alt, status, odo_at_t)

    def add_fix_xy(self, antenna, t, x, y, alt, status, odo_at_t: float) -> None:
        """Same as add_fix with the position already in the map frame."""
        if self._done or antenna not in ('master', 'rover'):
            return
        try:
            t = float(t); x = float(x); y = float(y)
            status = int(status); odo = float(odo_at_t)
        except (TypeError, ValueError):
            return
        if status < 0 or not all(math.isfinite(v) for v in (t, x, y, odo)) or t <= 0.0:
            return
        try:        # a missing altitude does not make the horizontal fix unusable
            alt = float(alt)
        except (TypeError, ValueError):
            alt = _NAN
        if not math.isfinite(alt):
            alt = _NAN
        x0, x1, y0, y1 = self._bbox
        if x < x0 - self.max_dist or x > x1 + self.max_dist or y < y0 - self.max_dist or y > y1 + self.max_dist:
            return
        f = _Fix(t, x, y, alt, status, odo)
        if antenna == 'master':
            f.proj = {name: r.project(x, y) for name, r in self.routes.items()}
            self._master.append(f)
        else:
            self._rover.append(f)
        if self._t_first is None:
            self._t_first = t
        self._dirty = True

    # ------------------------------------------------------------ outputs
    def result(self):
        """Latest InitResult (provisional after the first usable fix, then final) or None."""
        if self._dirty and not self._done:
            self._result = self._estimate(final=False)
            self._dirty = False
        return self._result

    def maybe_finalize(self, t_now: float) -> bool:
        """Close the init window; True exactly once."""
        if self._done:
            return False
        try:
            t_now = float(t_now)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(t_now) or t_now <= 0.0:
            return False
        # The timeout runs on the caller's clock from its first call, so a glitched GNSS
        # header stamp can neither hold the window open nor close it early.
        if self._t0 is None:
            self._t0 = t_now
        timed_out = t_now - self._t0 >= self.timeout_s
        if self._t_first is None:
            if not timed_out:
                return False
            self._result = InitResult(direction='', s_offset=_NAN, sigma0=float('inf'), head=None,
                                      start_xyz=(_NAN, _NAN, _NAN), yaw0=_NAN, kind='none',
                                      final=True, rtk=False, diag={'rule': 'timeout'})
            self._done = True
            return True
        if t_now < self._t_first + self.window_s and not timed_out:
            return False
        n = max(len(self._master), len(self._rover))
        if n < self.min_fixes and not timed_out:
            return False
        res = self._estimate(final=True)
        if res is None:     # cannot happen with a stored fix; keep the contract anyway
            return False
        self._result = res
        self._done = True
        self._dirty = False
        return True

    @property
    def done(self) -> bool:
        return self._done

    # ------------------------------------------------------------ internals
    def _select(self, fixes):
        rtk = [f for f in fixes if f.status == 2]
        if len(rtk) >= min(self.rtk_min, len(fixes)):
            return rtk, True
        return fixes, False

    def _baseline_heading(self):
        """Circular mean of master->rover pair headings of the best pair class, or None."""
        if not self._master or not self._rover:
            return None, 'none', 0
        rov = sorted(self._rover, key=lambda f: f.t)
        rt = [f.t for f in rov]
        classes = {'rtk2': [], 'rover_rtk': [], 'any': []}
        for m in self._master:
            i = bisect.bisect_left(rt, m.t)
            best = None
            for j in (i - 1, i):
                if 0 <= j < len(rov) and abs(rov[j].t - m.t) <= self.pair_dt:
                    if best is None or abs(rov[j].t - m.t) < abs(best.t - m.t):
                        best = rov[j]
            if best is None:
                continue
            dx = best.x - m.x; dy = best.y - m.y
            if abs(math.hypot(dx, dy) - self.ahead) > self.base_tol:
                continue
            psi = math.atan2(dy, dx)
            classes['any'].append(psi)
            if best.status == 2:
                classes['rover_rtk'].append(psi)
                if m.status == 2:
                    classes['rtk2'].append(psi)
        for name in ('rtk2', 'rover_rtk', 'any'):
            a = classes[name]
            if a:
                mu = _circ_mean(a)
                kept = [x for x in a if abs(_wrap(x - mu)) <= self.trim] or a
                return _circ_mean(kept), 'baseline_' + name, len(kept)
        return None, 'none', 0

    def _motion_heading(self, fixes, rtk):
        """Direction of travel from the fixes when GNSS and odometer both show motion, else None."""
        min_disp = self.motion_min if rtk else max(self.motion_min, self.motion_min_nortk)
        if len(fixes) < 2:
            return None
        fs = sorted(fixes, key=lambda f: f.t)
        n = max(1, len(fs) // 3)
        a, b = fs[:n], fs[-n:]
        dx = _median([f.x for f in b]) - _median([f.x for f in a])
        dy = _median([f.y for f in b]) - _median([f.y for f in a])
        dodo = _median([f.odo for f in b]) - _median([f.odo for f in a])
        disp = math.hypot(dx, dy)
        if disp < min_disp or dodo < 0.5 * min_disp:
            return None
        return math.atan2(dy, dx)

    def _route_yaw(self, name, s):
        return self.routes[name].pose(min(max(s, 0.0), self.routes[name].L))[3]

    def _agrees(self, name, s, psi):
        return math.cos(psi - self._route_yaw(name, s)) > self.cos_tol

    def _start_dist(self, name, x, y):
        x0, y0, _, _ = self._start[name]
        return math.hypot(x - x0, y - y0)

    def _choose(self, proj, psi, x, y):
        """Returns (route name or None for off-map, rule)."""
        names = list(self.routes)
        onmap = [n for n in names if not proj[n][3] and proj[n][1] < self.onmap_dist]
        near = [n for n in names if not proj[n][3] and proj[n][1] < self.near_dist]
        far = [n for n in names if not proj[n][3] and proj[n][1] < self.far_dist]
        by_d = lambda n: proj[n][1]
        if psi is not None:
            c = [n for n in onmap if self._agrees(n, proj[n][0], psi)]
            if c:
                return min(c, key=by_d), 'onmap_heading'
            c = [n for n in near if self._agrees(n, proj[n][0], psi)]
            if c:
                return min(c, key=by_d), 'near_heading'
            c = [n for n in far if self._agrees(n, proj[n][0], psi)]
            if c:
                return min(c, key=by_d), 'far_heading'
            return None, 'offmap'
        if len(onmap) == 1:
            return onmap[0], 'onmap_single'
        if len(onmap) >= 2:
            onmap.sort(key=by_d)
            if proj[onmap[1]][1] - proj[onmap[0]][1] >= self.track_margin:
                return onmap[0], 'onmap_nearer'
            return min(onmap, key=lambda n: self._start_dist(n, x, y)), 'onmap_start'
        if near:
            return min(near, key=by_d), 'near_nearest'
        return None, 'offmap'

    def _loop_start(self, x, y, psi):
        """Nearest loop branch (heading-consistent) -> (route, branch, u, dist, other branches) or None."""
        best = None
        for lp in self.loops.values():
            if lp['to'] not in self.routes:
                continue
            for br in lp['branches']:
                q = project_poly(x, y, br['x'], br['y'], br['u'])
                if q is None or q[1] > self.loop_max_dist:
                    continue
                if psi is not None:
                    i = min(max(int(q[0]), 0), len(br['yaw']) - 1)
                    if math.cos(psi - float(br['yaw'][i])) <= self.cos_tol:
                        continue
                if best is None or q[1] < best[3]:
                    best = (lp['to'], br, q[0], q[1], [b for b in lp['branches'] if b is not br])
        return best

    def _estimate(self, final):
        rover_only = not self._master
        fixes = self._rover if rover_only else self._master
        if not fixes:
            return None
        sel, rtk = self._select(fixes)
        if rover_only:
            for f in sel:
                if f.proj is None:
                    f.proj = {name: r.project(f.x, f.y) for name, r in self.routes.items()}
        psi, hsrc, npairs = self._baseline_heading()
        psi_m = self._motion_heading(sel, rtk)
        if psi_m is not None and (psi is None or (rtk and math.cos(psi - psi_m) < self.cos_tol)):
            hsrc = 'motion' if psi is None else 'motion_override'
            psi = psi_m

        last_odo = max(sel, key=lambda f: f.t).odo
        ref = [f for f in sel if abs(f.odo - last_odo) <= self.ref_odo] or sel
        x = _median([f.x for f in ref]); y = _median([f.y for f in ref])
        zs = [f.z for f in ref if math.isfinite(f.z)] or [f.z for f in sel if math.isfinite(f.z)]
        z = _median(zs) if zs else _NAN      # no altitude at all: map z + z_offset_m below
        odo_ref = _median([f.odo for f in ref])
        proj = {name: r.project(x, y) for name, r in self.routes.items()}
        name, rule = self._choose(proj, psi, x, y)
        back = self.ahead if rover_only else 0.0
        diag = {'rule': rule, 'heading_src': hsrc, 'n_pairs': npairs, 'n_fixes': len(sel),
                'rover_only': rover_only, 'odo_ref': odo_ref,
                'd': {n: proj[n][1] for n in proj}, 's_proj': {n: proj[n][0] for n in proj},
                'heading': psi if psi is not None else _NAN,
                'heading_motion': psi_m if psi_m is not None else _NAN}

        if name is not None:
            r = self.routes[name]
            s_offset = _median([f.proj[name][0] - f.odo for f in sel]) - back
            xm, ym, zm, yaw0 = r.pose(min(max(s_offset + odo_ref, 0.0), r.L))
            if rover_only:
                x, y = xm, ym
            if not math.isfinite(z):
                z = zm + self.z_off
            if rtk and rule.startswith('onmap'):
                sigma0 = self.sig_onmap
            elif rule == 'far_heading':
                sigma0 = math.hypot(self.sig_nortk, proj[name][1])
            else:
                sigma0 = max(self.sig_onmap, self.sig_nortk)
            kind, head = 'onmap', None
        elif self._loop_start(x, y, psi) is not None:
            name, br, u, dist, alts = self._loop_start(x, y, psi)
            head = branch_as_head(br)
            s_offset = (u - br['L']) - odo_ref - back
            i = min(max(int(u), 0), len(br['yaw']) - 1)
            yaw0, kind = float(br['yaw'][i]), 'loop'
            sigma0 = self.sig_loop if rtk else math.hypot(self.sig_loop, self.sig_nortk)
            if not math.isfinite(z):
                z = float(br['z'][i]) + self.z_off
            if rover_only:
                x -= self.ahead * math.cos(yaw0)
                y -= self.ahead * math.sin(yaw0)
            diag.update(rule='loop', loop_branch=br.get('name', ''), loop_dist=dist)
            return InitResult(direction=name, s_offset=s_offset, sigma0=sigma0, head=head,
                              start_xyz=(x, y, z), yaw0=yaw0, kind=kind, final=final, rtk=rtk, diag=diag,
                              heads=[head] + [branch_as_head(a) for a in alts])
        else:
            name = min(self.routes, key=lambda n: self._start_dist(n, x, y))
            xs, ys, zs0, yaws = self._start[name]
            chord = math.hypot(xs - x, ys - y)
            th0 = psi if psi is not None else math.atan2(ys - y, xs - x)
            if not math.isfinite(z):
                z = zs0 + self.z_off
            head = hermite_bridge((x, y), th0, (xs, ys), yaws, k=self.k[name], z0=z - self.z_off, z1=zs0)
            if head is None:        # standing exactly on the route start
                s_offset, yaw0, kind = -odo_ref - back, yaws, 'onmap'
                sigma0 = self.sig_onmap if rtk else self.sig_nortk
            else:
                s_offset = -head['L'] - odo_ref - back
                yaw0, kind = th0, 'bridge'
                # calibrated for the terminus approaches (L 130-215 m T2S, ~600 m S2T); a longer
                # bridge is an extrapolation, so its std grows with its length
                sigma0 = max(self.sig_bridge.get(name, self.sig_bridge_default), self.bridge_frac * head['L'])
                if not rtk:
                    sigma0 = math.hypot(sigma0, self.sig_nortk)
                if psi is None:
                    sigma0 = math.hypot(sigma0, self.noheading_frac * chord)
            if rover_only:
                x -= self.ahead * math.cos(yaw0)
                y -= self.ahead * math.sin(yaw0)
            diag['chord'] = chord
        return InitResult(direction=name, s_offset=s_offset, sigma0=sigma0, head=head,
                          start_xyz=(x, y, z), yaw0=yaw0, kind=kind, final=final, rtk=rtk, diag=diag)
