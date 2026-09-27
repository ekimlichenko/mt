"""Estimator pipeline: the single entry point of the ROS node and the offline replay.

Per input message (bag / bus arrival order, original header stamps):

  front/rear wheel  -> TimeGuard -> WheelFusion.add_sample -> SpeedEstimator.predict_to(t)
                       -> WheelFusion.measure -> SpeedEstimator.update -> GradeMatcher.add
  driver cmd        -> TimeGuard -> TractionModel.add_cmd -> SpeedEstimator.predict_to(t)
  GNSS fix          -> Initializer (initial alignment window), then GNSS aiding:
                       fixes are grouped into bursts; a consistent burst corrects the
                       along-track position (anchor) and, over >= gnss_scale_min_m,
                       the position scale.  The speed never uses GNSS.

One Output is produced per wheel/cmd message, stamped with that message's
original header stamp (each stamp once); the filter state is extrapolated to
the stamp (wheel stamps trail the command stamps by ~0.05 s, glitched stamps
are corrected internally but the output keeps the original stamp).

Position: s_route = s_offset + delta_applied + s_odo (+ output_along_offset_m)
  s_odo          odometer of the speed filter (integrated speed since start),
  s_offset       from the initial alignment (projection or G1 bridge length),
  delta_applied  along-track correction from grade matching (TRN), introduced
                 at most trn_apply_rate_mps once the TRN posterior std is below
                 trn_apply_sigma_m.
After the first accepted GNSS burst the published position is
  s_route = s_anchor + (1 + eps_pos) * (s_odo - odo_anchor) (+ output_along_offset_m);
TRN keeps feeding only the speed model's grade lookup, so GNSS never reaches the speed.
The pose is the run path (loop/bridge + pathgraph + loop/tail) at s_route; the
published point is base_link (output_along_offset_m ahead of the master
antenna), z = map height + output_z_offset_m.  The published speed is the
estimate at stamp - output_velocity_delay_s.
Without any usable GNSS fix the pose is the odometer along x of a start frame.
"""
import math
from collections import deque
from dataclasses import dataclass, field

from .config import Params
from .estimator import SpeedEstimator
from .frames import OutputFrame
from .grade_matcher import GradeMatcher
from .initializer import Initializer
from .preprocess import TimeGuard, FLAG_INVALID, FLAG_LATE
from .frames import latlon_to_map
from .track_map import RunPath, load_loops, load_routes
from .traction import TractionModel
from .wheel_fusion import WheelFusion

SQRT2 = math.sqrt(2.0)
STAMP_MEMORY = 64          # recent output stamps remembered for de-duplication (start bursts re-send ~40)


@dataclass
class Output:
    stamp: float
    v: float
    a: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    has_pose: bool = False
    var_v: float = 0.0
    var_along: float = 0.0
    var_cross: float = float('nan')     # nan: the node uses pose_sigma_cross_m
    var_yaw: float = float('nan')       # nan: the node uses pose_sigma_yaw_rad
    mode: str = 'init'
    slip: bool = False
    frame_id: str = ''
    diag: dict = field(default_factory=dict)


class Pipeline:
    def __init__(self, params: Params, maps_dir: str):
        p = params.validate()
        self.p = p
        self.routes = load_routes(maps_dir, p.grade_window_m)
        self.frame = OutputFrame(p)
        self.k = p.k_for_vehicle()
        self.tg = TimeGuard(p)
        self.wheels = WheelFusion(p, self.k)
        self.model = TractionModel(p)
        self.est = SpeedEstimator(p, self.model, self._grade, self._curv, 0.0)
        self.loops = load_loops(maps_dir) if p.loops_enable else None
        self.init = Initializer(p, self.routes, self.loops)
        self.started = False
        self.init_res = None
        self.route = None
        self.path = None
        self.direction = ''
        self.s_offset = 0.0
        self.sigma0 = 0.0
        self.odo_init = 0.0
        self.trn = None
        self.trn_active = False
        self.delta_target = 0.0
        self.delta_applied = 0.0
        self.delta_std = float('nan')
        self.eps = 0.0
        self.t_trn = None
        self.fallback = False
        self.last_meas = None
        self.last_innov = None
        self._stamps = deque(maxlen=STAMP_MEMORY)
        self._stamp_set = set()
        self.n_late = 0
        self.n_out = 0
        # GNSS aiding
        self._burst = []
        self._odo_hist = deque()
        self._v_hist = deque()
        self.gnss_active = False
        self.anchor_s = 0.0
        self.anchor_odo = 0.0
        self.anchor_var = 0.0
        self.anchor_part = ''
        self.anchor_delta = None
        self.pos_eps = 0.0
        self.scale_known = False
        self.n_bursts = 0
        self.n_bursts_rejected = 0
        self.last_nu = float('nan')

    # ------------------------------------------------------------ properties
    @property
    def init_done(self):
        return self.init.done or self.fallback

    @property
    def gnss_done(self):
        """True when the node may drop the GNSS subscriptions (no aiding)."""
        return self.init_done and (self.fallback or not self.p.gnss_aiding)

    # ------------------------------------------------------------ inputs
    def on_wheel(self, bogie, t_hdr, raw_kmh):
        t, flag = self.tg.correct(bogie, t_hdr)
        if flag == FLAG_INVALID:
            return None
        if flag == FLAG_LATE:
            # older than what the filter already processed: not fed, but still answered
            self.n_late += 1
        else:
            accepted = self.wheels.add_sample(bogie, t, raw_kmh)
            if not self.started:
                if not accepted:
                    return None
                self.est.reset(t, max(float(raw_kmh), 0.0) / self.k)
                self.started = True
            self.est.predict_to(t)
            tm = self.est.t if self.est.t > t else t
            m = self.wheels.measure(tm, self.est.v)
            if m is not None:
                if self.p.wheel_pair_sigma_inflate and m.front_ok and m.rear_ok:
                    # front and rear share a stamp and arrive < 1 ms apart: the two updates per
                    # stamp are one piece of evidence, so each carries half of its information
                    m.sigma *= SQRT2
                self.last_innov = self.est.update(m)
            self.last_meas = m
            self._trn_step(tm, m)
            self._hist_push()
        if not self.started:
            return None
        self._check_init(t)
        self._gnss_flush(t)
        if self.p.publish_on == 'cmd':
            return None
        return self._output(t_hdr)

    def on_cmd(self, t_hdr, notch):
        t, flag = self.tg.correct('cmd', t_hdr)
        if flag == FLAG_INVALID:
            return None
        try:
            n = int(notch)
        except (TypeError, ValueError):
            return None
        if flag == FLAG_LATE:
            self.n_late += 1
        self.model.add_cmd(t, n)        # late commands are inserted by time (history only)
        if not self.started:
            return None
        if flag != FLAG_LATE:
            self.est.predict_to(t)
            self._hist_push()
        self._check_init(t)
        self._gnss_flush(t)
        if self.p.publish_on == 'wheels':
            return None
        return self._output(t_hdr)

    def on_fix(self, antenna, t_hdr, lat, lon, alt, status):
        if self.gnss_done:
            return None
        try:
            t_hdr = float(t_hdr)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(t_hdr):
            return None
        if self.init_done:
            self._aid_fix(antenna, t_hdr, lat, lon, status)
            return None
        self.init.add_fix(antenna, t_hdr, lat, lon, alt, status, self._odo_at(t_hdr))
        res = self.init.result()
        if res is not None and res is not self.init_res and res.kind != 'none':
            self._apply_init(res)
        return None

    # ------------------------------------------------------------ GNSS aiding
    def _hist_push(self):
        est = self.est
        h = self._odo_hist
        if h and est.t <= h[-1][0]:
            return
        h.append((est.t, est.s))
        self._v_hist.append((est.t, est.v))
        horizon = est.t - self.p.gnss_hist_s
        while h and h[0][0] < horizon:
            h.popleft()
        while len(self._v_hist) > 2 and self._v_hist[1][0] < est.t - 2.0:
            self._v_hist.popleft()

    @staticmethod
    def _interp(h, t):
        """Linear interpolation in a (t, value) deque; None outside its span."""
        if not h or t < h[0][0]:
            return None
        if t >= h[-1][0]:
            return None
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, a = h[lo]; t1, b = h[hi]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return a + (b - a) * w

    def _odo_at_fix(self, t):
        v = self._interp(self._odo_hist, t)
        if v is not None:
            return v
        if self._odo_hist and t >= self._odo_hist[-1][0]:
            return self.est.extrapolate(t)[1]
        return None

    def _trn_blend(self, s_odo):
        """(correction, variance) since the last GNSS anchor.

        Odometer drift since the anchor (variance (frac*D)^2) is blended with the TRN
        view of it (the change of the TRN offset since the anchor, variance delta_std^2)."""
        p = self.p
        dist = s_odo - self.anchor_odo
        frac = p.gnss_drift_frac_scaled if self.scale_known else p.gnss_drift_frac
        var_odo = (frac * dist) ** 2
        if not (p.gnss_trn_blend and self.trn_active and self.anchor_delta is not None
                and math.isfinite(self.delta_std)):
            return 0.0, var_odo
        var_trn = self.delta_std ** 2
        if var_odo <= 0.0:
            return 0.0, 0.0
        innov = (self.delta_applied - self.anchor_delta) - self.pos_eps * dist
        lim = 3.0 * math.sqrt(var_odo + var_trn)
        innov = max(-lim, min(lim, innov))
        w = var_odo / (var_odo + var_trn)
        return w * innov, (1.0 - w) * var_odo

    def _s_master(self, s_odo):
        """Route arc length of the master antenna for an odometer reading."""
        if self.gnss_active:
            return (self.anchor_s + (1.0 + self.pos_eps) * (s_odo - self.anchor_odo)
                    + self._trn_blend(s_odo)[0])
        return self.s_offset + self.delta_applied + s_odo

    def _var_along_base(self, s_odo):
        p = self.p
        if self.gnss_active:
            return self.anchor_var + self._trn_blend(s_odo)[1]
        if self.trn_active:
            lag = self.delta_target - self.delta_applied
            return (self.delta_std ** 2 + lag * lag + self.est.P[4][4]) * p.pose_trn_sigma_scale ** 2
        return self.sigma0 ** 2 + self.est.var_s

    def _aid_fix(self, antenna, t, lat, lon, status):
        if self.path is None or antenna not in ('master', 'rover'):
            return
        try:
            lat = float(lat); lon = float(lon); status = int(status)
        except (TypeError, ValueError):
            return
        if status < 0 or not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0.0 and lon == 0.0):
            return
        x, y = latlon_to_map(lat, lon)
        if self._burst and (t - self._burst[-1][1] > self.p.gnss_burst_gap_s):
            self._process_burst()
        self._burst.append((antenna, t, x, y, status))
        if len(self._burst) >= self.p.gnss_burst_max_fixes:
            self._process_burst()

    def _gnss_flush(self, t):
        if self._burst and t - self._burst[-1][1] > self.p.gnss_burst_gap_s:
            self._process_burst()

    def _process_burst(self):
        p = self.p
        burst, self._burst = self._burst, []
        if self.path is None or not self.started:
            return
        rows = []
        for ant, t, x, y, st in burst:
            so = self._odo_at_fix(t)
            if so is None:
                continue
            back = p.rover_ahead_m if ant == 'rover' else 0.0
            s_pred = self._s_master(so) + back
            sig = math.sqrt(max(self._var_along_base(so), 0.0))
            win = max(p.gnss_window_min_m, p.gnss_window_sigma_k * sig)
            q = self.path.project_near(x, y, s_pred, win)
            if q is None:
                continue
            gate = p.gnss_gate_rtk_m if st == 2 else p.gnss_gate_nortk_m
            if q[1] > gate:
                continue
            rows.append((q[0] - s_pred, st == 2, q[2], q[3], so))
        rtk = [r for r in rows if r[1]]
        use = rtk if len(rtk) >= p.gnss_min_fixes else rows
        if len(use) < p.gnss_min_fixes:
            if burst:
                self.n_bursts_rejected += 1
            return
        is_rtk = use is rtk
        nus = sorted(r[0] for r in use)
        nu = nus[len(nus) // 2] if len(nus) % 2 else 0.5 * (nus[len(nus) // 2 - 1] + nus[len(nus) // 2])
        spread = sorted(abs(r[0] - nu) for r in use)[len(use) // 2]
        if spread > (p.gnss_spread_rtk_m if is_rtk else p.gnss_spread_nortk_m):
            self.n_bursts_rejected += 1
            return
        parts = [r[2] for r in use]
        part_now = max(set(parts), key=parts.count)
        # loop branch actually driven (majority of the used fixes)
        for part in ('head', 'tail'):
            idx = [r[3] for r in use if r[2] == part]
            if len(idx) * 2 > len(use):
                best = max(set(idx), key=idx.count)
                if part == 'tail' and best != self.path.tail_idx:
                    self.path.set_tail(best)
                elif part == 'head' and best != self.path.head_idx and best < len(self.path.heads):
                    self.path.set_head(self.path.heads[best])
        so = sorted(r[4] for r in use)[len(use) // 2]
        sigma = p.gnss_sigma_rtk_m if is_rtk else p.gnss_sigma_nortk_m
        var_prior = max(self._var_along_base(so), 1e-4)
        k = var_prior / (var_prior + sigma * sigma)
        s_now = self._s_master(so)
        # scale only from an RTK-to-RTK baseline on the pathgraph (loop geometry is a median of runs)
        if (self.gnss_active and is_rtk and self.anchor_var <= (3.0 * p.gnss_sigma_rtk_m) ** 2
                and self.anchor_part == 'map' and part_now == 'map'):
            base = so - self.anchor_odo
            if base > p.gnss_scale_min_m:
                e = self.pos_eps + p.gnss_scale_gain * nu / base
                self.pos_eps = max(-p.gnss_scale_max, min(p.gnss_scale_max, e))
                self.scale_known = True
        self.anchor_s = s_now + k * nu
        self.anchor_odo = so
        self.anchor_var = (1.0 - k) * var_prior
        self.anchor_part = part_now
        # TRN offset matching the anchor: with TRN running, its current value (the blend then
        # uses only the TRN change since the anchor); before TRN has started (start-window
        # bursts), the anchor's own offset from the TRN reference path s_offset + s_odo, so
        # the TRN corrections still reach the output after the last burst
        self.anchor_delta = (self.delta_applied if self.trn_active
                             else self.anchor_s - self.s_offset - so)
        self.gnss_active = True
        self.last_nu = nu
        self.n_bursts += 1
        # The anchor replaces TRN for the published position only.  TRN keeps running
        # for the speed model's grade lookup (est.route_offset), so the speed stream is
        # bit-identical with and without GNSS aiding.

    # ------------------------------------------------------------ internals
    def _odo_at(self, t):
        if not self.started:
            return 0.0
        return self.est.extrapolate(t)[1]

    def _grade(self, s):
        return self.path.grade_at(s) if self.path is not None else 0.0

    def _curv(self, s):
        return self.path.curv_at(s) if self.path is not None else 0.0

    def _check_init(self, t):
        if self.init_done:
            return
        if self.init.maybe_finalize(t):
            res = self.init.result()
            if res is None or res.kind == 'none':
                self._start_fallback()
            else:
                self._apply_init(res)

    def _apply_init(self, res):
        p = self.p
        self.init_res = res
        self.direction = res.direction
        self.route = self.routes[res.direction]
        tails = []
        for lp in (self.loops or {}).values():
            if lp['from'] == res.direction:
                tails = lp['branches']
        self.path = RunPath(self.route, res.head, p.tail_mode, p.tail_curv_decay_m, p.tail_curv_window_m,
                            tails=tails, heads=getattr(res, 'heads', None) or None)
        if self.gnss_active:        # a late init result must not undo the aiding anchor
            self.gnss_active = False
        look = p.trn_lookahead_t2s_m if res.direction == 'T2S' else p.trn_lookahead_s2t_m
        self.est.lookahead = look if p.model_lookahead else 0.0
        self.s_offset = res.s_offset
        self.sigma0 = res.sigma0
        self.odo_init = self.est.s if self.started else 0.0
        self.est.route_offset = self.s_offset + self.delta_applied
        x0, y0, z0 = res.start_xyz
        if not math.isfinite(z0):
            z0 = 0.0
        self.frame.set_origin(x0, y0, z0)
        if res.final and p.trn_enable and self.trn is None:
            # the grid covers +-trn_delta_range_m; a wider prior would be silently truncated
            sig = min(res.sigma0, p.trn_delta_range_m / 3.0)
            self.trn = GradeMatcher(p, self.route, res.direction, sig)

    def _start_fallback(self):
        """No usable GNSS: dead reckoning in a start-relative frame (x along the initial heading)."""
        if self.fallback:
            return
        self.fallback = True
        self.path = None
        self.frame.mode = 'start'
        self.frame.frame_id = self.p.frame_id_start
        self.frame.set_origin(0.0, 0.0, 0.0)
        self.odo_init = self.est.s if self.started else 0.0

    def _trn_step(self, t, m):
        trn = self.trn
        if trn is None:
            return
        mo = self.est.last_out
        ok = (m is not None and mo is not None and m.state == 'both' and not m.slip and not mo.latch
              and mo.has_cmd and mo.mode not in ('hold8', 'standstill') and abs(mo.notch) <= 10)
        s_route = self.s_offset + self.est.s
        v = m.v if m is not None else self.est.v
        trn.add(t, s_route, v, mo.a_ng if mo is not None else 0.0, mo.c_grade if mo is not None else 0.0, ok)
        if trn.n_updates == 0:
            return
        dmean, dstd, eps = trn.correction(s_route)
        self.delta_std = dstd
        self.eps = eps
        if dstd < self.p.trn_apply_sigma_m:
            self.delta_target = dmean
            self.trn_active = True
        if self.t_trn is not None and t > self.t_trn:
            step = self.p.trn_apply_rate_mps * (t - self.t_trn)
            d = self.delta_target - self.delta_applied
            self.delta_applied += step if d > step else (-step if d < -step else d)
            self.est.route_offset = self.s_offset + self.delta_applied
        self.t_trn = t

    def _mode(self):
        if self.fallback:
            base = 'no_gnss'
        elif self.init_res is None:
            base = 'init'
        elif not self.init.done:
            base = 'init_provisional'
        else:
            base = 'nav_gnss' if self.gnss_active else ('nav_trn' if self.trn_active else 'nav')
        m = self.last_meas
        return base + ':' + (m.state if m is not None else 'model_only')

    def _output(self, stamp):
        if stamp in self._stamp_set:
            return None
        if len(self._stamps) == self._stamps.maxlen:
            self._stamp_set.discard(self._stamps[0])
        self._stamps.append(stamp)
        self._stamp_set.add(stamp)
        p = self.p
        est = self.est
        v, s_odo = est.extrapolate(stamp)
        if p.output_velocity_delay_s > 0.0:
            vd = self._interp(self._v_hist, stamp - p.output_velocity_delay_s)
            if vd is None and self._v_hist and stamp - p.output_velocity_delay_s >= self._v_hist[-1][0]:
                vd = est.extrapolate(stamp - p.output_velocity_delay_s)[0]
            if vd is not None:
                v = vd
        if p.trn_scale_feedback and self.trn_active:
            v *= 1.0 + self.eps
        if not v > 0.0:
            v = 0.0
        m = self.last_meas
        out = Output(stamp=stamp, v=v, a=est.last_a, var_v=est.var_v, mode=self._mode(),
                     slip=bool(m is not None and m.slip), frame_id=self.frame.frame_id)
        if self.path is not None:
            s_route = self._s_master(s_odo) + p.output_along_offset_m
            x, y, z, yaw = self.path.pose(s_route)
            out.x, out.y, out.z = self.frame.apply(x, y, z + p.output_z_offset_m)
            yaw = self.frame.apply_yaw(x, y, yaw)
            out.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            out.has_pose = True
            var = self._var_along_base(s_odo)
            # off the pathgraph the track is interpolated (bridge, s < 0) or extrapolated (tail,
            # s > L): the along-track odometry error grows with tail_sigma_frac, the geometric error
            # of the assumed track with bridge_cross_frac (cross-track only: the bridge length is
            # calibrated) or tail_cross_frac (isotropic: after the map end the track may turn)
            L = self.route.L
            off = s_route - L if s_route > L else (-s_route if s_route < 0.0 else 0.0)
            var_c = p.pose_sigma_cross_m ** 2
            head_loop = self.path.head is not None and self.init_res is not None and self.init_res.kind == 'loop'
            if s_route > L and s_route <= L + self.path.tail_L:
                var_c += p.loop_cross_sigma_m ** 2          # restored loop branch
                off = 0.0
            elif s_route > L:
                off = s_route - L - self.path.tail_L
                geo = (p.tail_cross_frac * off) ** 2
                var += (p.tail_sigma_frac * off) ** 2 + geo
                var_c += geo
            elif s_route < 0.0 and head_loop and s_route >= self.path.s_min:
                var_c += p.loop_cross_sigma_m ** 2
                off = 0.0
            elif s_route < 0.0:
                var_c += (p.bridge_cross_frac * off) ** 2
            out.var_along = var
            out.var_cross = var_c
            sy = min(math.pi, math.hypot(p.pose_sigma_yaw_rad, p.offmap_yaw_rad_per_m * off))
            out.var_yaw = sy * sy
            out.diag['s_route'] = s_route
            out.diag['off_map_m'] = off
        elif self.fallback:
            d = s_odo - self.odo_init
            out.x, out.y, out.z = self.frame.apply(d, 0.0, 0.0)
            out.has_pose = True
            out.var_along = est.var_s
        vf = m.v_front if m is not None else float('nan')
        vr = m.v_rear if m is not None else float('nan')
        vref = v if v > 1.0 else 1.0
        out.diag.update(s_odo=s_odo, delta=self.delta_applied, delta_std=self.delta_std, eps=self.eps,
                        gnss_bursts=self.n_bursts, gnss_rejected=self.n_bursts_rejected, pos_eps=self.pos_eps,
                        gnss_nu=self.last_nu,
                        gain_tr=est.g_tr, gain_br=est.g_br, bias=est.b,
                        v_front=vf, v_rear=vr,
                        slip_ratio_front=(vf - v) / vref if vf == vf else float('nan'),
                        slip_ratio_rear=(vr - v) / vref if vr == vr else float('nan'),
                        adhesion_used=abs(est.last_a) / 9.81, a=est.last_a)
        self.n_out += 1
        return out
