"""Estimator parameters.

All tunables live in one flat dataclass so that the very same names are used as
ROS 2 parameters (``config/params.yaml``), in the offline replay and in the docs
(``docs/PARAMETERS.md``).  Values are calibrated offline on the training bags
(see ``tools/calibration/`` and ``docs/PARAMETERS.md``); units are SI unless the name says
otherwise.  ``validate()`` rejects unknown enum values at start-up.
"""
from dataclasses import dataclass, field, fields, asdict
import yaml


@dataclass
class Params:
    # ---------------------------------------------------------------- wheels
    # Wheel topics carry km/h (the dataset README says m/s, the data says km/h).
    # v [m/s] = raw / wheel_k.  Calibrated per vehicle against RTK GNSS.
    wheel_k: float = 3.595                  # used when vehicle_id is unknown
    wheel_k_30618: float = 3.597
    wheel_k_30639: float = 3.59              # RTK median 3.585; CV sweep 3.585-3.605 on 11 runs: best mean score at 3.59
    vehicle_id: str = ''                    # '30618' / '30639' / '' (unknown)
    wheel_stale_s: float = 0.5              # bogie invalid if its last sample is older
    wheel_extrap_max_s: float = 0.2         # linear extrapolation horizon of a bogie
    wheel_max_kmh: float = 100.0            # plausibility bound of a raw sample
    wheel_min_kmh: float = -5.0
    stuck_zero_other_mps: float = 0.2       # bogie == 0 while other > this -> ignore the zero (= agree_abs)
    frozen_s: float = 1.5                   # identical non-zero value this long ...
    frozen_other_dv_mps: float = 0.3        # ... while the other bogie changed this much -> frozen
    spike_kmh: float = 4.0                  # single-sample jump beyond physics + this -> spike
    agree_abs_mps: float = 0.2              # |F-R| <= max(abs, rel*v) -> average
    agree_rel: float = 0.03
    wheel_align_other: bool = True          # agreement test also on the time-aligned pair
    wheel_pair_sigma_inflate: bool = True   # both bogies valid: each of the two updates per stamp gets sigma*sqrt(2)
    # ------------------------------------------------------ rate limiter / slip
    acc_max: float = 1.8                    # m/s^2, above traction envelope (clean max 1.55)
    dec_max_consistent: float = 6.0         # bogies agree: genuine emergency braking reaches -5.15 (wheel, 0.2 s)
    dec_max_slip: float = 2.4               # bogies disagree: slide suspected
    slip_hold_s: float = 1.0                # slip state persists after the bogies re-agree
    slip_acc_flag: float = 2.5              # a > this over >= slip_acc_win_s -> slip flag (spin)
    slip_dec_flag: float = 6.0              # a < -this -> slip flag (slide); genuine emergency braking reaches -5.15
    slip_acc_win_s: float = 0.3
    slip_anchor_s: float = 3.0              # disagreement: bogie nearer the last agreed speed, for this long
    lock_zero_dec: float = 1.0              # fused 0 during slip: keep slip for v0/this seconds (wheel lock)
    # ---------------------------------------------------------------- filter
    meas_sigma: float = 0.03                # m/s, both bogies agree
    meas_sigma_single: float = 0.045        # one bogie
    meas_sigma_slip: float = 0.35           # disagreement / clipped measurement
    sigma_a_traction: float = 0.11          # process noise of the dynamic model per mode
    sigma_a_coast: float = 0.16
    sigma_a_brake: float = 0.085
    sigma_a_hold8: float = 0.6
    sigma_a_brake_high: float = 0.43
    bias_enable: bool = True
    bias_tau_s: float = 10.0                # Gauss-Markov model-bias state
    bias_sigma: float = 0.05                # steady-state std of the model bias, m/s^2
    sigma_a_nocmd: float = 0.5              # process noise without a (fresh) driver command
    sigma_a_unlatched: float = 0.6          # model latched but the wheels move
    accel_corr_s: float = 1.0               # correlation time of the model acceleration error
    gap_sigma_a: float = 0.5                # accel noise over gaps > max_gap_s
    meas_sigma_clip: float = 0.0            # extra sigma of a rate-clipped measurement
    clip_resync_s: float = 3.0              # clipped this long -> accept the raw measurement
    clip_min_dt_s: float = 0.1              # dt floor of the rate-limit clip
    standstill_sigma_v: float = 0.01        # v std at a wheel-confirmed standstill
    meas_fresh_s: float = 0.5               # wheel sample age that confirms latch / motion
    s_correction: bool = True               # correct s through the v-s covariance (dropouts)
    hold8_rest_grow: bool = True            # -8 at rest without wheels: P grows (departure possible)
    latch_needs_wheels: bool = True         # collapse P at standstill only with fresh wheel zero
    odo_scale_sigma: float = 0.003          # relative wheel-scale uncertainty in var_s
    extrap_max_s: float = 1.0               # output extrapolation horizon to the stamp
    init_sigma_v: float = 1.0               # v std after reset
    max_step_s: float = 0.05                # integration sub-step
    max_gap_s: float = 5.0                  # longer input gaps are not integrated in one go
    # ------------------------------------------------------- traction model
    # a_tr(n,v) = min(b0 + a0*n/(1+max(v-v1,0)/va), A0*min(1,vb/v)^pw) - r0
    tr_b0: float = 0.0968
    tr_a0: float = 0.1131
    tr_v1: float = 3.49
    tr_va: float = 6.90
    tr_A0: float = 1.048
    tr_vb: float = 5.90
    tr_pw: float = 0.571
    # a_br(n,v) = max(-(d0 + d1|n|)*(1 - f*exp(-v/vf)), cap) - r0
    br_d0: float = 0.193
    br_d1: float = 0.1123
    br_f: float = 0.327
    br_vf: float = 2.98
    br_cap: float = -1.7
    res_r0: float = 0.04                    # running resistance, m/s^2
    curve_k: float = 3.0                    # curve resistance k*|curv|, m^2/s^2
    grade_c_traction: float = -0.886        # a += c * g * grade
    grade_c_brake: float = -0.812
    grade_c_coast: float = -0.978
    grade_c_hold8: float = -0.812
    lag_tr_dead_s: float = 0.05
    lag_tr_tau_s: float = 0.25
    lag_br_dead_s: float = 0.15
    lag_br_tau_s: float = 0.30
    hold8_notch: int = -8                   # closed-loop hold position of the controller
    hold8_v: float = 2.5
    hold8_low_acc: float = -0.85
    start_delay_s: float = 1.2              # traction must be held this long to leave standstill
    standstill_v: float = 0.05
    gain_tr: float = 1.0
    gain_br: float = 1.0
    gain_adapt: bool = True
    gain_sigma: float = 0.002               # random walk, 1/sqrt(s)
    gain_sigma0: float = 0.07               # initial gain std
    gain_gate_sigma: float = 3.0            # gains adapt only for innovations within this gate
    cmd_stale_s: float = 1.0                # command older than this -> no command
    cmd_history_s: float = 3.0              # command history kept behind the model time
    gain_min: float = 0.8
    gain_max: float = 1.3
    # ---------------------------------------------------------------- map
    grade_window_m: float = 31.0
    z_offset_m: float = 3.10                # GNSS (master antenna) altitude above the pathgraph z (start / bridge z;
                                            # the published z uses output_z_offset_m)
    # published point: base_link (organisers: the reference /localization/kinematic_state is base_link,
    # tf master antenna at (-9.873, 0, 3.0), and its z equals the pathgraph z)
    output_along_offset_m: float = 9.873    # along-track offset from the master antenna: 0 master, 9.873 base_link,
                                            # 12.44 rover
    output_z_offset_m: float = 0.0          # published z above the pathgraph z (3.10 = master antenna height)
    output_velocity_delay_s: float = 0.09   # published speed = estimate at stamp - delay (the reference twist of
                                            # the organisers' test bag lags the wheel stamps by ~0.09 s)
    # ------------------------------------------- terminus loops (maps/loops.json)
    loops_enable: bool = True               # use the restored loop geometry off the map (head / tail)
    loop_max_dist_m: float = 8.0            # start fix farther from every branch -> G1 bridge instead
    loop_sigma0_m: float = 1.5              # along-track std of an RTK start on a loop branch
    loop_cross_sigma_m: float = 1.0         # cross-track std on a loop branch (branch geometry, chord offset)
    # ----------------------------------------------------------- GNSS aiding
    # sparse mid-route fixes correct the position only (organisers: allowed, no penalty)
    gnss_aiding: bool = True                # keep the GNSS subscriptions after the initial alignment and use
                                            # every later burst of fixes to correct the along-track position
                                            # (the speed never uses GNSS)
    gnss_burst_gap_s: float = 0.5           # a burst is closed after this silence (or gnss_burst_max_fixes)
    gnss_burst_max_fixes: int = 20
    gnss_min_fixes: int = 2                 # usable fixes needed to accept a burst
    gnss_gate_rtk_m: float = 2.0            # max distance of a fix to the run path (RTK / other)
    gnss_gate_nortk_m: float = 6.0
    gnss_sigma_rtk_m: float = 1.2           # along-track std of a burst correction (RTK / other); CV-calibrated
                                            # together with gnss_drift_frac* for 95 % coverage (tools/run_cv.py --gnss-burst)
    gnss_sigma_nortk_m: float = 3.0
    gnss_spread_rtk_m: float = 1.0          # burst rejected when its fixes disagree more (median abs dev)
    gnss_spread_nortk_m: float = 4.0
    gnss_window_min_m: float = 80.0         # fixes are projected within +-max(this, k*sigma) of the prediction
    gnss_window_sigma_k: float = 5.0
    gnss_scale_min_m: float = 300.0         # odometer baseline needed to update the position scale
    gnss_scale_gain: float = 0.3
    gnss_scale_max: float = 0.02            # |position scale correction| bound
    gnss_drift_frac: float = 0.006          # along-track std growth per metre after a correction (scale unknown)
    gnss_drift_frac_scaled: float = 0.004   # same once the scale was estimated from two corrections
    gnss_hist_s: float = 30.0               # odometer history kept to place late fixes
    gnss_trn_blend: bool = True             # between GNSS bursts blend odometer drift with the TRN offset change
    # ------------------------------------------------ off-map: bridge and tail
    # G1 bridge before the map entry (no loop branch at the start) and the extrapolated
    # tail past the map end (without loops) or past the end of the loop branch
    bridge_tangent_k: float = 1.5           # G1 Hermite bridge tangent length / chord (other routes)
    bridge_tangent_k_t2s: float = 1.55      # calibrated per route (median start error +0.1 m)
    bridge_tangent_k_s2t: float = 1.25      # (median -0.4 m; 1.5 would bias S2T by -35 m)
    tail_mode: str = 'decay'                # extrapolation after the map end: decay | straight
    tail_curv_decay_m: float = 40.0         # end curvature decays over this length (transition curve)
    tail_curv_window_m: float = 20.0        # end curvature estimated over the last metres of the map
    tail_sigma_frac: float = 0.03           # along-track std growth per metre past the map end (tail)
    bridge_cross_frac: float = 0.06         # cross-track std per metre before the map entry (interpolated bridge)
    tail_cross_frac: float = 0.3            # isotropic std per metre after the map end (track unknown)
    offmap_yaw_rad_per_m: float = 0.005     # yaw std growth per metre off the map (capped at pi)
    # --------------------------------------------------- grade matching (TRN)
    trn_enable: bool = True
    trn_delta_range_m: float = 120.0
    trn_delta_step_m: float = 1.0
    trn_eps_range: float = 0.015
    trn_eps_step: float = 0.001
    trn_eps_prior: float = 0.004
    trn_update_every_m: float = 2.0
    trn_win_s: float = 1.0                  # causal window for the wheel acceleration
    trn_sigma: float = 0.1                  # residual std, m/s^2 (measured rms 0.061-0.070; 0.1 absorbs model error, CV-tuned)
    trn_sigma_t2s: float = 0.0              # per-direction override (0 = trn_sigma)
    trn_sigma_s2t: float = 0.0
    trn_trunc: float = 0.3                  # residual truncation (robust), m/s^2
    trn_corr_len_m: float = 20.0            # likelihood tempering (correlated samples)
    trn_grade_window_m: float = 9.0         # matcher's own grade smoothing (0 = grade_window_m)
    trn_use_curv: bool = True
    trn_min_samples: int = 5
    trn_lookahead_t2s_m: float = 12.0       # grade point ahead of the master antenna
    trn_lookahead_s2t_m: float = 10.0
    model_lookahead: bool = True            # the speed model uses the same grade point
    trn_min_v: float = 2.0
    trn_apply_sigma_m: float = 15.0         # apply the correction once posterior std is below
    trn_apply_rate_mps: float = 1.0         # max speed of the applied correction
    trn_forget: float = 1.0                 # log-likelihood forgetting per update
    trn_scale_feedback: bool = False        # use the TRN scale estimate for speed output
    # ------------------------------------------------------- initialisation
    init_window_s: float = 3.0
    init_min_fixes: int = 5
    init_timeout_s: float = 10.0
    init_onmap_dist_m: float = 3.0
    rover_ahead_m: float = 12.44
    sigma0_rtk_onmap_m: float = 3.0
    sigma0_bridge_t2s_m: float = 8.0
    sigma0_bridge_s2t_m: float = 12.0
    sigma0_nortk_m: float = 10.0
    init_bridge_sigma_frac: float = 0.03    # bridge sigma0 floor, fraction of the bridge length
    init_near_dist_m: float = 8.0
    init_far_dist_m: float = 30.0
    init_track_margin_m: float = 1.0
    init_heading_tol_deg: float = 60.0
    init_heading_trim_deg: float = 20.0
    init_baseline_tol_m: float = 2.5
    init_pair_dt_s: float = 0.1
    init_rtk_min_fixes: int = 3
    init_motion_min_m: float = 2.0
    init_motion_min_nortk_m: float = 10.0
    init_ref_odo_m: float = 0.5
    init_max_dist_m: float = 20000.0
    init_noheading_sigma_frac: float = 0.2
    # --------------------------------------------------------------- timing
    time_glitch_s: float = 0.3              # header jump beyond the other streams -> glitch
    time_jump_margin_s: float = 0.15
    time_late_tol_s: float = 0.15           # older than the filter clock by more -> late
    time_consensus_n: int = 30
    time_resync_n: int = 20
    # --------------------------------------------------------------- output
    output_frame: str = 'map'               # map | utm | start | enu
    frame_id_map: str = 'map'
    frame_id_utm: str = 'utm_37n'
    frame_id_start: str = 'odom'
    frame_id_enu: str = 'enu'
    enu_origin_auto: bool = True            # enu origin = initial position; false: the fixed origin below
    enu_origin_lat: float = 0.0             # deg WGS84 (used when enu_origin_auto is false)
    enu_origin_lon: float = 0.0             # deg WGS84
    enu_origin_alt: float = 0.0             # m, same altitude datum as the GNSS fixes
    child_frame_id: str = 'base_link'
    velocity_frame_id: str = 'base_link'
    publish_on: str = 'all'                 # all | cmd | wheels
    pose_sigma_cross_m: float = 0.8         # Odometry covariance: cross-track std on the map (CV: 95 % coverage;
                                            # S2T runs leave the pathgraph for a parallel track 4.2 m off at s~500-1200)
    pose_trn_sigma_scale: float = 0.6       # scale of the along-track std while TRN is active (the tempered TRN
                                            # posterior is ~2x wider than the error; CV: 95 % coverage, NEES ~1)
    pose_sigma_z_m: float = 0.5
    pose_sigma_tilt_rad: float = 0.02
    pose_sigma_yaw_rad: float = 0.03
    input_qos_depth: int = 50
    diag_period_s: float = 1.0

    def k_for_vehicle(self):
        if self.vehicle_id == '30618':
            return self.wheel_k_30618
        if self.vehicle_id == '30639':
            return self.wheel_k_30639
        return self.wheel_k

    ENUMS = {'publish_on': ('all', 'cmd', 'wheels'), 'tail_mode': ('decay', 'straight'),
             'output_frame': ('map', 'utm', 'start', 'enu')}

    def validate(self):
        """Raise ValueError on an unknown enum value (a typo would otherwise degrade silently)."""
        for k, allowed in self.ENUMS.items():
            if getattr(self, k) not in allowed:
                raise ValueError('%s must be one of %s, got %r' % (k, '|'.join(allowed), getattr(self, k)))
        return self

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        names = {f.name: f.type for f in fields(cls)}
        p = cls()
        for k, v in (d or {}).items():
            if k not in names:
                raise KeyError('unknown parameter %s' % k)
            cur = getattr(p, k)
            if isinstance(cur, bool):
                v = v.strip().lower() in ('1', 'true', 'yes', 'on') if isinstance(v, str) else bool(v)
            elif isinstance(cur, int):
                v = int(float(v))
            elif isinstance(cur, float):
                v = float(v)
            elif isinstance(cur, str):
                v = str(v)
            setattr(p, k, v)
        return p

    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        # accept both a plain mapping and the ROS 2 "<node>: ros__parameters:" layout
        if len(d) == 1:
            only = next(iter(d.values()))
            if isinstance(only, dict) and 'ros__parameters' in only:
                d = only['ros__parameters']
        return cls.from_dict(d)
