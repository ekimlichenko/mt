#################### timing_units
SUMMARY: Units, scaling, timing and stream properties across 72 unique bags (30618: 55, 30639: 17; duplicates removed by header-stamp hash).
- Wheel speed unit: the raw wheel VelocitySensor.v is signed km/h. v_mps = k * raw / 3.6.
- Scale factor k: k is not 1.000 everywhere. On one run it is constant (±0.3% over 150 s windows) and does not depend on the traction, coast or brake command, so it is a calibration or diameter offset, not slip. Across runs and dates it moves by up to ±1.6%:
  - 30618: 0.9997 / 1.0007 / 1.0004 on 07-27 / 08-10 / 08-26, and 1.006-1.016 on 09-03.
  - 30639: 0.984-0.994 on 05-05, and 1.003 on 08-26.
  This error alone gives 0.5-1.6% end drift, so a fixed k is the main risk to the drift metric.
- Front vs rear wheel: the two agree to 0.02% and carry identical header stamps. Vehicle 30639 has rear-only outages of up to 73.5 s, plus one front gap of 19.8 s.
- Wheel rate: 9.36 Hz median, with about 7% of 0.1 s slots missing. A wheel-triggered output would miss GNSS reference stamps under the ±0.05 s matching rule. Publish from the 20.05 Hz cmd stream, which has a single phase-locked publisher and no duplicates, stamped with the vehicle clock.
- Header-clock alignment:
  - Wheel speed matches GNSS velocity to within about 0.005 s (median; 95.5% of 60 s windows within ±0.05 s).
  - In the record clock, wheel lags by about 43 ms (30618) or 22 ms (30639). So always work in header stamps.
  - Wheel and GNSS velocity both describe the motion about 49 ms before their stamp, relative to GNSS fix position. Integrated position therefore trails the fix by v*0.049 (about 0.7 m at 14 m/s) unless a lead is applied.
- GNSS velocity: ENU, true north. vz is a real grade signal. Speed noise for RTK is ±0.06 m/s p5-p95 while moving and p99 0.04 m/s at standstill. Status-0 velocity is still usable (Doppler).
- Antenna offset: the rover antenna sits 12.44 m ahead of the master along the track. Which antenna the reference uses changes the position error by 12 m.
- Clock anomalies:
  - ±1 s header-stamp glitches hit 0.71% of GNSS velocity messages (14 bags; worst is 30639_3b3d9eb8 with 242 s) and 0.005% of vehicle messages (2 bags).
  - Host clock slews in 2 bags.
  - A startup burst replays 2-6 s old messages in the first second of recording.
- Bag start: every bag starts at standstill. The first motion comes after a median of 11.3 s, and RTK is available in the first 10 s in about 71 bags. The start window can initialize position and heading, but not k.

MCP connectors that need authorization (amplitude, atlassian, bigquery, definite, hex) were not needed and were not used.
- [A1_wheel_unit_kmh] (high, CRIT) The raw front/rear VelocitySensor.v is in km/h and is signed. Convert with v_mps = k*raw/3.6.
    EVID: Max raw is about 53.3. In steady cruise, k = v_gnss_RTK/(raw/3.6) has a median of 1.0012 (front) and 1.0014 (rear) for 30618 over 51 bags, and 0.9956 for 30639 over 17 bags. Negative values appear only in 30618_3e012faf (-0.16..-0.39 km/h) and 30639_9c362687 (-0.22..-0.26 km/h); both are short roll-backs.
    IMPL: Apply the 1/3.6 conversion. Treating raw as m/s would give a 260% error. Keep the sign, or clamp small negative values to 0; roll-back distance is negligible.
- [A2_scale_varies_per_run] (high, CRIT) The effective wheel scale k is stable within a run but differs between dates and runs by up to ±1.6%.
    EVID: Map along-track regression s = a + k*D_wheel, fitted per bag with trimming. Median k [range] by date:
- 30618 07-27: 0.99970 [0.99926-1.00036]
- 30618 08-10: 1.00069 [1.00041-1.00100]
- 30618 08-26: 1.00042 [1.00008-1.00056]
- 30618 09-03: 1.0064-1.0160 (0686195f 1.0138, 27e994fc 1.0062, 88548b02 1.0064, defd0170 1.0160)
- 30639 05-05: 0.9910 (253671cc 0.9942, 92226df0 0.9904, c31df386 0.9898, d601d28f 0.9843, dce52be4 0.9915)
- 30639 08-26: 1.0029 [1.0024-1.0031]
Velocity-based steady-state k runs 0.05-0.1% higher than the map-based k; UTM scale explains 0.03% of that.
    IMPL: A fixed k costs up to 1.6% of distance, which is enough to dominate end-drift %. Options: use a per-vehicle default near 1.000; look up k by date from the absolute header stamp (the dataset allows vehicle and date knowledge only if the rules permit); or calibrate k online against map landmarks (station stops, terminus, known segment lengths between curvature features). Report the k uncertainty (prior sigma about 0.8%) in covariance.
- [A3_scale_not_slip] (high) k does not depend on the command (traction, coast or brake), barely depends on speed, and does not drift during a run, so it behaves as a constant calibration offset rather than slip.
    EVID: - By command, bag 0686195f: k = 1.0157 (traction), 1.0153 (coast), 1.0141 (brake).
- By speed: the 3-6 m/s bin is 0.1-0.5% higher than 6-9 m/s; 12-15 m/s is about 0.1% lower.
- Last third vs first third of a run: median +0.03%.
- Stability: ±0.3% over 150 s windows.
    IMPL: Model k as a slowly varying random-walk state (or a constant per run). A slip model tied to the command is not justified by the data.
- [A4_front_rear_identical] (high) Front and rear wheel speeds agree to 0.02%, and their header stamps are identical in 99.9% of messages (one source frame).
    EVID: Median steady front/rear ratio: 1.00012 (std 0.00018) for 30618, 0.99976 for 30639.
    IMPL: Average front and rear when both are fresh; the second wheel mainly adds redundancy. The front-rear difference is a weak slip/skid detector: flag if |F-R|/max(F,R) > 2% at >2 m/s.
- [A5_wheel_outages] (high, CRIT) Long single-wheel outages occur on 30639, plus one long outage of the front wheel.
    EVID: Rear-only gaps on 30639:
- 3b3d9eb8: 73.5 s + 30 s (1017 front-only stamps)
- 4285f2bc: 47.1 s
- 44226bde: 16.7 s
- 584b6e32: 16.3 s
- 927002c2: 20.5 s + 8.7 s
- d927f360: 25.6 s + 12 s
- 9f0b519f: 5.8 s
Front gaps longer than 1 s: 30639_c31df386 19.8 s, 30618_2255aade 1.2 s, 30618_40ffd323 1.1 s.
    IMPL: The per-wheel freshness check is mandatory. Use the single wheel when the other is stale for more than 0.3 s. When both are stale, predict with the longitudinal model from cmd and inflate the covariance. Never average with a stale value.
- [A6_near_zero_deadzone] (high) Wheel speed has a dead zone near zero: it drops to exactly 0 below about 0.1-0.16 m/s. Readings of 0 while moving are rare, but there are occasional 0 -> ~42 km/h jumps.
    EVID: From 3111 transitions:
- Last nonzero value before 0: median 0.34 km/h (0.093 m/s), p95 0.585 km/h (0.16 m/s).
- First nonzero value after 0: median 0.39 km/h.
- Minimum nonzero value: 0.153 km/h.
- Zero dropouts while moving: 1 front and 7 rear events across all bags.
    IMPL: The distance lost in the dead zone is under 0.1 m per stop, so it is negligible. Use raw == 0 on both wheels (or < 0.6 km/h) as the standstill flag (ZUPT, speed = 0, no position growth). Reject physically impossible jumps with an acceleration gate of about 3 m/s^2, and require agreement with the other wheel.
- [A7_quantization] (medium) Quantization of the wheel signal is negligible: values are continuous float32 with steps of about 0.005 km/h near zero.
    EVID: There is no uniform grid in the raw values. Repeated identical nonzero consecutive values occur 70-100 times per bag (about 0.7%), consistent with a stale-hold.
    IMPL: Quantization noise needs no special handling. A repeated identical value may be a stale sample, so the measurement noise could be inflated slightly.
- [B1_gnss_vel_frame] (high) GNSS TwistStamped vx, vy, vz are ENU relative to true north. Horizontal speed is hypot(vx, vy). vz is a real vertical velocity.
    EVID: - corr(vx, dE/dt) = 0.98 and corr(vy, dN/dt) = 0.997 (master).
- The heading difference versus the UTM grid is 1.25 deg (grid convergence at lon 37.4).
- Speed ratio to position-derived speed: 1.0002.
- corr(vz, map grade * along-track speed) = 0.92-0.93, slope about 1.01, residual std 0.053 m/s. Map grade ranges from -4% to +4%.
    IMPL: The speed reference should be hypot(vx, vy); the 3D norm differs by at most 0.08%. If heading is derived from GNSS velocity, rotate by the 1.25 deg grid convergence before comparing with the map/UTM frame. vz can be ignored for speed but confirms the map grade.
- [B2_gnss_vel_noise_by_status] (high) GNSS velocity accuracy is about ±0.06 m/s p5-p95 for RTK and about ±0.14 m/s for status 0. Standstill p99 is about 0.04 m/s. Status-0 velocity is usable; status-0 position is not.
    EVID: Standstill speed (p50 / p99):
- master status 2: 0.005 / 0.038
- rover status 2: 0.006 / 0.044 (outliers to 15-17 m/s)
- master status 0: 0.007 / 0.052
Moving (wheel > 3 m/s), GNSS - wheel (median, p5/p95):
- master status 2: 0.002, -0.055/0.061
- rover status 2: 0.000, -0.087/0.074
- master status 0: -0.018, -0.13/0.14
Master - rover speed, both RTK: p5/p95 ±0.04, p99 0.078.
    IMPL: The reference speed carries about 0.03-0.04 m/s of noise, which sets a floor on speed RMSE. Offline, use RTK samples (status 2) from either antenna to calibrate k and lag; gate out rover outliers larger than 1 m/s vs master or wheel.
- [B3_master_rover_offset] (high, CRIT) The rover antenna is 12.44 m ahead of the master along the direction of travel, in both driving directions and on both vehicles.
    EVID: Offset is 12.44 m on 30618 and 12.42 m on 30639. Message counts: master velocity has 629774 status-2 and 160769 status-0 messages; rover velocity has 771576 status-2 and 29073 status-0. The master stream on 30639 has many gaps, so the rover is often the better RTK source there and on 30618 09-03.
    IMPL: Output position must refer to the same antenna or point as the judge's reference; a mismatch is a constant 12.4 m error. Keep an internal reference point and apply a configurable along-track lever arm. Initializing from the rover without correcting for the offset puts the start 12.4 m off.
- [C1_header_vs_record] (high, CRIT) Header stamps precede record time by a latency that depends on the topic. Cmd is nearly real-time; GNSS arrives 45-90 ms late.
    EVID: Median header - record offset:
- wheel: -0.047 s (per bag -0.072..-0.021)
- cmd: -0.001 s
- master fix: -0.045 s
- master vel: -0.085 s
- rover fix: -0.081 s
- rover vel: -0.089 s
Wheel records arrive on a fixed 10 Hz phase while wheel header stamps jitter (dt_hdr p5 0.076, p95 0.19). GNSS stamps lie exactly on a 0.1 s grid.
    IMPL: Do not stamp outputs with node now() or bag time. Use the vehicle header clock, from cmd or wheel stamps.
- [C2_wheel_gnss_lag] (high, CRIT) In the header clock, wheel speed and GNSS velocity are aligned (lag about 0). In the record clock, wheel lags GNSS velocity by 43 ms (30618) or 22 ms (30639).
    EVID: Lag estimated by MSE minimization of k*W(t) vs G(t - tau) over transients (|a| > 0.2 m/s^2), with parabolic refinement. Header clock: 30618 median +0.001 s (IQR -0.001..0.003), 30639 -0.004 s; 92.5% of 60 s windows have |lag| <= 0.03 s and 95.5% <= 0.05 s. RMSE in transients is about 0.025 m/s.
    IMPL: Output speed stamped with a vehicle header stamp needs no lag compensation. Compensation matters only if record or arrival time is used.
- [C3_measurement_delay_vs_fix] (medium) Wheel speed and GNSS velocity both represent the motion about 49 ms before their header stamp, relative to the instantaneous GNSS fix position.
    EVID: On an exact 0.1 s grid with a 1 s trapezoid window:
- velocity(t + 0.048) ≈ position-derived speed(t) for the master
- wheel(t + 0.049) ≈ position-derived speed(t), IQR 0.037-0.054
    IMPL: When integrating position, place each wheel sample at t_hdr - 0.049, or add a lead of v*0.049 m to the published position (about 0.7 m at 14 m/s). Do not shift the published speed, because the speed reference has the same delay.
- [C4_stamp_glitches] (high) ±1 s header-stamp glitches and host-clock slewing affect about 14 bags. Non-monotonic stamps occur only in these bags.
    EVID: Messages with |hdr - rec - median| > 0.5 s:
- GNSS vel master: 5631 (0.71%); rover: 5611 (0.69%). Worst bags: 30639_3b3d9eb8 (+1 s for about 242 s), 30618_40ffd323 (118 s), 27e994fc (45 s), af7496f0 (36 s), 28538acf (32 s).
- Vehicle front: 46 (0.005%). Cmd: 126. Bags 2255aade and 40ffd323, where wheel and cmd jump together by +1 s for 2-3 s.
- Host clock drifts about 0.1 s/s in 28538acf and 1cc230fa.
- Duplicate stamps: 0 on every topic.
    IMPL: The node must tolerate dt < 0 and dt around 1 s: skip the integration step when dt <= 0 or dt > 0.3 s after a jump, and do not integrate across a jump. A GNSS-stamped reference with a +1 s error will be matched against our output 1 s off, which is unavoidable noise. Stamping from the vehicle clock is still the safest choice.
- [C5_startup_burst] (high) The first second of each recording replays stale messages whose stamps are 2-6 s old: about 34 wheel, about 73 cmd and about 35 GNSS messages.
    EVID: Observed at the start of each recording.
    IMPL: Process messages by header stamp, not arrival order. With best-effort QoS on replay, start the filter at the first wheel stamp and ignore messages older than the current filter time.
- [D1_wheel_rate] (high, CRIT) The wheel stream runs below 10 Hz: median 9.36 Hz, with about 7% of 0.1 s slots missing.
    EVID: Record periods: 92.8% at 0.1 s, 6.9% at 0.2 s, 0.06% at 0.3 s. Per-bag rate is 8.9-10.6 Hz. GNSS reference stamps lie on an exact 0.1 s grid, and the judge matches by nearest stamp with ±0.05 s tolerance.
    IMPL: If output is triggered by the wheel callback, about 7% of reference samples have no output within ±0.05 s, and wheel stamp jitter up to ±0.09 s adds more misses. Trigger publishing on the 20 Hz cmd callback, or on a 20 Hz timer in the vehicle clock. Predict the state forward from the last wheel stamp: the horizon is typically 47 ms and at most 150 ms.
- [D2_cmd_stream] (high) The cmd stream comes from a single phase-locked publisher at 20.05 Hz with no duplicates. Command changes are mostly single notches, with rare one-sample A-B-A flips.
    EVID: - dt p5-p95: 0.0499-0.0501 s. Header stamp mod 0.05 is constant per bag, so there are no interleaved publishers despite the two QoS profiles in metadata.
- Of 2.07M messages, 150491 are changes: 96% are ±1 notch, the largest step is 12, 19285 are single-sample runs, and 1220 are A-B-A flips (0.06%).
- Gaps > 0.2 s occur only at clock events.
- Cmd shares the vehicle clock with the wheels (they jump together) but its stamps are not synchronized with wheel stamps.
    IMPL: Cmd is the best output clock: 20 Hz, low latency (hdr - rec about -1 ms), and gives full coverage of the 0.1 s reference grid. Use a zero-order hold of the latest cmd in the model. Filtering A-B-A flips (one-sample hold) is optional.
- [D3_bag_start] (high) Every bag starts at standstill. First motion (> 0.5 m/s) comes after a median of 11.3 s (p10 7.5 s, p90 44.9 s), and RTK is available in the first 10 s in about 71 of 72 bags.
    EVID: Checked across all unique bags.
    IMPL: The initial window can give position (average of the RTK fix) and heading/direction (master-rover baseline, or first motion projected on the map). It cannot give k, because no motion happens before the GNSS cut-off, if one applies. Initialize in ZUPT mode.
- [E1_utm_scale] (medium) At lon 37.4, the UTM zone 37N point scale is about 0.9997, so UTM/pathgraph distances are about 0.03% shorter than ground distances measured by the wheels.
    EVID: From the map-based vs velocity-based k comparison and the geo.py projection.
    IMPL: If the reference frame is UTM minus (300000, 6100000), fold the 0.9997 factor into k. The effect is small compared with the ±1.6% run-to-run spread.
PARAMS:
  * wheel_raw_to_mps = 1/3.6 = 0.277778 (times k)  (Steady-cruise ratio of RTK GNSS speed to raw wheel value, median about 1.001 x (1/3.6).)
  * k_default_30618 = 1.0005 (normal dates); 1.010 if the date is 09-03  (Median of the map along-track fit over dates 07-27, 08-10 and 08-26; 09-03 bags gave 1.006-1.016.)
  * k_default_30639 = 0.997 (neutral across dates); 0.991 on 05-05; 1.0029 on 08-26  (Map along-track fit per bag, median by date.)
  * k_prior_sigma = 0.008 (relative)  (Run-to-run spread 0.984-1.016 about per-vehicle defaults.)
  * k_process_noise = about 1e-4 per sqrt(100 s), or treat k as constant per run  (Within-run stability ±0.3% over 150 s windows; first-to-last third drift +0.03%.)
  * wheel_fusion = mean(F, R) if both ages < 0.3 s; otherwise use the fresh wheel; if neither is fresh, model-only prediction  (F/R ratio 1.0001 with std 0.0002; outages up to 73.5 s on 30639.)
  * wheel_stale_timeout_s = 0.3  (99.94% of wheel periods are <= 0.2 s; 0.3 s periods occur 0.06% of the time.)
  * standstill_threshold = both wheels raw == 0, or |raw| < 0.6 km/h (0.16 m/s)  (p95 of the last nonzero value before a stop is 0.585 km/h; minimum nonzero is 0.153 km/h.)
  * wheel_accel_gate_mps2 = 3.0 (reject a sample if the implied |dv/dt| is above this and it disagrees with the other wheel)  (Observed rare 0 -> 42 km/h spikes; tram acceleration is at most about 1.5 m/s^2.)
  * output_trigger = cmd callback at 20 Hz (or a 20 Hz timer in the vehicle clock); header.stamp = latest cmd header stamp  (Cmd runs 20.05 Hz with hdr - rec about -1 ms and full coverage of the 0.1 s reference grid; wheel runs only 9.36 Hz.)
  * prediction_horizon_s = t_out - t_last_wheel, typically 0.047 s, clamp to 0.3 s  (Wheel hdr - rec median -0.047 s; cmd hdr - rec -0.001 s.)
  * wheel_vs_gnssvel_lag_header_s = 0.0  (Whole-bag MSE lag: 30618 +0.001 s, 30639 -0.004 s.)
  * wheel_vs_gnssvel_lag_record_s = -0.043 (30618), -0.022 (30639); use only if the record clock is used  (The same lag fit in the record clock.)
  * wheel_position_delay_s = 0.049 (position lead = v * 0.049 m)  (wheel(t + 0.049) ≈ fix-position-derived speed(t), IQR 0.037-0.054.)
  * master_rover_along_track_offset_m = 12.44 (rover ahead of master)  (Mean along-track baseline of RTK fixes, both directions: 30618 12.44 m, 30639 12.42 m.)
  * gnss_vel_sigma_mps = 0.035 for RTK moving; 0.07 for status 0; standstill p99 0.04  (GNSS - wheel p5/p95 by status and antenna.)
  * dt_guard = skip the integration step if dt <= 0 or dt > 0.3 s; do not integrate across a jump of about ±1 s  (±1 s vehicle-stamp glitches in 2255aade and 40ffd323; non-monotonic stamps up to -1.0 s.)
  * cmd_debounce_samples = 1 (optional; 50 ms hold to suppress A-B-A flips)  (1220 A-B-A flips out of 2.07M cmd messages.)
  * utm_scale_factor = 0.9997 (fold into k if the reference frame is UTM)  (Transverse Mercator point scale at lon 37.4 in zone 37.)
OPEN Q: Does the judge's reference use GNSS header stamps (subject to the ±1 s glitches in about 0.7% of samples) or bag record time? A record-time reference would shift the optimal output stamp by about 0.04-0.09 s. | Which antenna (master or rover) and which status filter does the judge's position reference use? This is a 12.44 m difference. | Is the reference speed hypot(vx, vy) or the 3D norm, and from which antenna? The effect is under 0.1%, but it matters for the rover's outliers of 15-17 m/s at standstill. | Is k-by-date lookup allowed (using the absolute stamp or vehicle id), or must k be learned online? The ±1.6% spread across runs dominates drift. | What causes the scale shifts on 30618 09-03 (+0.6..+1.6%) and 30639 05-05 (-0.6..-1.6%)? Candidates are wheel wear or reprofiling between dates, or a different calibration constant. | How should the ±1 s GNSS stamp glitch periods be scored, and can the vehicle-clock glitches (2255aade, 40ffd323) be detected online with the record clock or cmd period as a cross-check? | Is there a GNSS cut-off time after which GNSS is unavailable online? If so, k cannot be calibrated from the stationary start window, and map landmarks (stops, terminus, curvature features) would be the only online source of scale.
SCRIPTS: # unique bags by header-stamp hash
seen=set(); U=[]
for b in bags:
    d=pickle.load(open(f'cache/{b}.pkl','rb'))
    h=hashlib.md5(b''.join(d[k]['t_hdr'].tobytes() for k in sorted(d))).hexdigest()
    if h in seen: continue
    seen.add(h); U.append(b) # gap-aware interpolation
def iv(tq,t,v,maxgap):
    y=np.interp(tq,t,v); j=np.searchsorted(t,tq)
    j0=np.clip(j-1,0,len(t)-1); j1=np.clip(j,0,len(t)-1)
    return y,(tq>=t[0])&(tq<=t[-1])&((t[j1]-t[j0])<=maxgap) # lag: for tau in grid, G=iv(t-tau); k=lstsq(W,G); mse over |a|>0.2 m/s^2; parabolic refine around the minimum. Ref = GNSS vel samples whose same-stamp fix status==2 (key=round(t_hdr*100)), master preferred over rover; speed=hypot(vx,vy) # scale: project RTK fixes (UTM37 - (300000,6100000)) onto the pathgraph via cKDTree + segment projection -> arc length s; regress s = a + k*cumsum(raw/3.6*dt) with iterative 3-sigma trimming, per bag # stamp glitches: off=t_hdr-t_rec after t0+5 s; bad=|off-median(off)|>0.5 s; count per topic and per bag
VERDICTS:
  ~ [A1_wheel_unit_kmh] confirmed: Raw VelocitySensor.v is in km/h, so v_mps = k*raw/3.6. There is no single k: it varies by run (see A2) between 0.984 and 1.016. The value is signed, but negative readings are only tiny standstill roll-backs. Below about 0.15 km/h the sensor reports exactly 0.
      EVID: Method: my own steady-cruise ratio. G is the Doppler speed from /gnss/rover/vel (header clock, status-2 samples, median-filter outliers removed), taken where G>5 m/s and |a|<0.05 m/s^2 over 1 s and 2 s windows. It is compared with wheel/3.6 interpolated at the same stamps. Units of the GNSS vel topic were checked separately against speed differenced from positions: ratio 0.9991-0.9999, and vx/vy is ENU. Result on 51 unique 30618 bags and 17 unique 30639 bags (25 duplicate pairs removed; all 08-10 30618 bags are duplicated), per-date medians of k: 30618 07-27 1.0007, 08-10 1.0011, 08-26 1.0011, 09-03 1.0060-1.0153; 30639 05-05 0.9922 (0.9838-0.9957), 08-26 1.0039 (1.0033-1.0050). An independent distance method (sum of RTK position steps at >3 m/s divided by integrated wheel distance) agrees to within about 0.05%. Front and rear k differ by at most 0.0007 per bag. The analyst's pooled 30639 median of 0.9956 hides a bimodal split (about 0.992 vs about 1.004). Max raw is 53.33 on 30618 and 53.73 on 30639. Sign: negatives occur only in 30618_3e012faf (and its duplicate bcc9e7a2: 6 front and 5 rear samples, -0.16 to -0.39) and 30639_9c362687 (3 front and 3 rear, -0.22 to -0.26). They appear on both bogies at once, at standstill, between brake and light-traction toggles, and imply about 2-5 cm of roll-back. GNSS never shows real reversing: only 7 isolated status-2 samples have the rover behind the master, and they are glitches at up to 14 m/s. So 'signed' is plausible but cannot be tested for larger reverse motion. 30.1% of samples are exactly 0; the smallest positive value is 0.153 km/h. No files were written: plan mode was read-only, so all code ran inline via stdin.
  ~ [A2_scale_varies_per_run] partially_confirmed: CRITICAL. Within a run, k is stable to about ±0.1%. Between runs it varies by up to about 1.2% even on the same date, so it is a per-run (or per-shift) quantity, not a per-date one. There is also a systematic direction effect: S2T k is about 0.05-0.1% higher than T2S. The overall range is 0.984-1.016. k cannot be hard-coded offline and cannot be learned from a few seconds of GNSS at rest, so about ±1% distance-scale error must be handled another way (for example map or stop constraints).
      EVID: Velocity-based k per bag (rover Doppler). Within-date spread: 30639 05-05 rises through the day: d601d28f 07:34 0.9838, 3b3d9eb8 07:51 0.9892, c31df386 10:22 0.9901, dce52be4 10:45 0.9921, 92226df0 15:22 0.9907, 9c362687 16:29 0.9930, 253671cc 16:53 0.9949, 44226bde 17:30 0.9940, d3c43d69 17:49 0.9954 (spread 1.2%). 30618 09-03 steps between morning and afternoon: defd0170 07:45 1.0153, 0686195f 08:06 1.0144, then 88548b02 15:55 1.0060, 27e994fc 16:17 1.0063. Normal dates are tight: 30618 07-27 0.9997-1.0015, 08-10 1.0010-1.0024, 08-26 1.0007-1.0015; 30639 08-26 1.0033-1.0049. Direction medians (S2T vs T2S): 07-27 1.0011 vs 1.0000; 08-10 1.0014 vs 1.0011; 08-26 1.0013 vs 1.0008; 30639 08-26 1.0044 vs 1.0035; 30639 05-05 0.9935 vs 0.9907. This explains the alternating pattern in consecutive 07-27 runs. Within-run first half vs second half: difference at most 0.15%, typically 0.05%; on 08-10 and 08-26 the second half is consistently about 0.1% higher, a likely grade or direction effect. The distance-based method (RTK step sums) gives the same per-date picture: master 07-27 1.0003, 08-10 1.0011, 08-26 1.0010, 09-03 1.0063; 30639 05-05 0.9924 (rover range 0.9843-0.9960), 08-26 1.0035. My velocity-based k is about 0.05-0.1% above the analyst's map-based k, consistent with their note. Caveat for the analyst's map-based 30639 05-05 values: rover status-2 positions there are poor (see B3), but that affects k by less than 0.1%.
  ~ [A5_wheel_outages] partially_confirmed: Long single-channel wheel outages occur only on vehicle 30639: 7 bags with rear outages and one with a front outage (c31df386, 19.8 s). Every outage starts at standstill (v=0) and ends with the tram already moving at 4-42 km/h, having covered 18-550 m. They are real dropouts under motion, so the node must fall back to the remaining channel. The 30618 'front gaps' (2255aade 1.2 s, 40ffd323 1.1 s) are not wheel outages: they are record-time stalls of all vehicle topics with continuous header stamps and no data loss.
      EVID: Gap scan (>1 s) on both clocks, all 97 unique bags. Rear gaps: 3b3d9eb8 30.4 s + 73.5 s (1017 front-only stamps; other channel covers 137 m + 550 m, up to 42.6 km/h); 4285f2bc 47.1 s (143 m); 44226bde 16.7 s (44 m); 584b6e32 16.3 s (39 m); 927002c2 20.5 s + 8.7 s (131 m + 34 m, up to 36 km/h); d927f360 25.6 s + 12.1 s (190 m + 31 m, up to 44 km/h; the 12.1 s gap is coasting with cmd=0); 9f0b519f 5.8 s (18 m). Front gap: c31df386 19.8 s (57 m, 194 rear-only stamps). No single-channel gap over 1 s exists in any 30618 bag. In 2255aade and 40ffd323 the front and rear gaps are simultaneous and cmd has only 4 and 2 messages in the gap (max cmd dt 1.05 s), and there is no header gap over 1 s. Other facts: front and rear share identical header stamps in 97.5-100% of messages. Single dropped samples (dt of about 0.2 s) make up 7.0% of wheel intervals in the median bag (p5 0.9%, p95 11.9%).
  ~ [B3_master_rover_offset] partially_confirmed: The geometry is confirmed: the rover is 12.43 m ahead of the master along the direction of travel in both travel directions, on both vehicles. It also sits laterally about 0.32 m to the right on 30618 and about 0.10 m on 30639. But the claim that the rover is the better RTK source is refuted: status==2 does not mean a reliable fixed solution, and rover status-2 positions are much worse than master ones, especially on 30639 05-05. Ground truth and alignment need a quality gate, such as a baseline-length check or map distance.
      EVID: Used simultaneous status-2 pairs (same 0.1 s stamp), projected onto rover-vel direction at >2 m/s. Baseline: 30618 median 12.438 m (per-bag 12.433-12.444), along-track p5/50/p95 12.306/12.431/12.453; 30639 12.425 m (12.417-12.430), along-track 12.329/12.420/12.438. Eastbound and westbound along-track medians are 12.430/12.432 on 30618 and 12.418/12.424 on 30639. Rover-behind fraction is at most 0.9% per bag, all glitches. Lateral offset: 30618 +0.33 (E) / +0.31 (W); 30639 +0.12 / +0.08. Each bag is one one-way trip (T2S or S2T) and termini are loops. Quality: 12.2% of 629k status-2 pairs have |L - nominal| > 0.2 m, independent of speed (not a timing effect); per-bag median 9%, worst 30618_87afe526 51% and 30639_0be558e2 46%. Where attributable via map distance, the rover is the bad antenna in 25% of bad pairs vs 1.4% for the master. On-map cross-track distance to the pathgraph, share of status-2 fixes above 1 m / 3 m: 30618 master 5.5%/3.0%, rover 9.3%/3.4%; 30639 master 4.3%/0.5%, rover 29.0%/12.6%. Rover-only 30639 05-05 bags have median distance to map of 3.10 m (44226bde), 14.6 m (9c362687), 2.84 m (e4379d7f) and 0.81 m (3b3d9eb8, d3c43d69). The NavSatFix covariance is all zeros, so it cannot be used for gating. 30618 09-03 has median map distance 0.54 m for both antennas vs about 0.28 m on other dates.
  ~ [C1_header_vs_record] partially_confirmed: The median latencies are confirmed. Additions: every bag starts with a backlog burst of 20-60 wheel messages whose header stamps are 2-6 s older than record time. GNSS header stamps have integer-second (about ±1.0 s) error episodes lasting 18-243 s in 10 bags. Wheel header stamps have rare short jumps. Header stamps are therefore the right clock for the wheel, but GNSS header stamps must be sanity-checked against record time.
      EVID: Median header minus record (all unique bags): wheel -0.048 s (per-bag 30618 -0.079..-0.013, 30639 -0.069..-0.024); cmd -0.001 (one short bag, 30618_1551d0a9, at -0.84); master fix -0.0445; master vel -0.084 (30618 -0.087, 30639 -0.062); rover fix -0.078 (-0.083 / -0.061); rover vel -0.088 (-0.092 / -0.072). Vehicle 30639 GNSS latency is about 20-25 ms lower. GNSS stamps: 100% lie within 1 ms of the 0.1 s grid. Wheel record phase is fixed per bag (circular concentration R median 0.996, p5 0.904). Wheel dt_hdr p1/5/50/95/99 = 0.060/0.0765/0.102/0.190/0.210; dt_rec p5/p95 = 0.099/0.1997. Cmd is exactly 20 Hz (dt_hdr p1-p99 0.0498-0.0502). Startup backlog: the first record offset is -2.1 to -3.0 s on the front channel and -3.3 to -4.4 s on the rear (30618), and -4.0 to -6.25 s on 30639 05-05; about 0.29% of wheel messages have offset below -0.2 s. GNSS vel offset episodes: 30639_3b3d9eb8 +1.0 s for 243 s; 30618_40ffd323 +1.0 s for 60 s and -1.0 s for 60 s; 27e994fc -1.0 for 45 s; af7496f0 -0.99 for 36 s; 28538acf -0.95 for 32 s; 748832b9 -0.93 for 21 s; 2366c74a -0.92 for 20 s; 4d487b0d -0.93 for 18 s; 9c362687 -0.97 for 20 s. In these episodes the wheel-vs-GNSS lag is ±0.76 to 1.0 s in the header clock but -0.12 to +0.02 s in the record clock, so the GNSS header stamp is the wrong one. Some bags also have GNSS stamp offsets at the very start (0686195f -0.88 s for 6.4 s; e4379d7f -2.73 s for 3.7 s), which matters for GNSS initial alignment. Wheel header jumps: 28538acf -0.23 s for 6 s; 40ffd323 -0.95 s for 2.3 s and +0.25 s for 6.3 s; 1cc230fa +0.16 s for 5.7 s.
  ~ [C2_wheel_gnss_lag] partially_confirmed: In the header clock, wheel and GNSS Doppler speed are aligned (lag about 0 ±0.01 s) outside GNSS header-glitch episodes. In the record clock the magnitude is confirmed (about 45 ms on 30618, about 30 ms on 30639 rover, about 20 ms on 30639 master), but the sign is reversed from the claim: the wheel LEADS GNSS, because GNSS vel is recorded about 88 ms after its stamp vs about 48 ms for the wheel. Header-clock residual is lower (about 0.029 vs about 0.034 m/s), so header stamps are the better alignment basis, except in the roughly 1-s GNSS stamp glitches.
      EVID: Lag was fitted by grid search (5 ms steps with parabolic refinement, or 10 ms per 60 s window), minimising the MSE of G(t) vs k*W(t+tau) over transients (|a|>0.2 m/s^2). tau>0 means the wheel lags. Sign convention verified: artificially delaying the wheel record stamps by +0.2 s moved tau from -0.040 to +0.160 (bag b15eafc3). Per-bag header-clock medians (rover): 30618 07-27 +0.0015, 08-10 +0.0028, 08-26 -0.0001, 09-03 +0.0040; 30639 05-05 -0.0001, 08-26 -0.0032. Record clock: 30618 -0.045/-0.040/-0.051/-0.053; 30639 -0.031/-0.030 (master -0.023/-0.018). 60-s windows (1001 on 30618, 339 on 30639), header clock: median 0.000; |lag|≤0.03 in 95.6% / 94.7% of windows, ≤0.05 in 97.5% / 96.8%, >0.5 s in 0.6% / 1.5% (the GNSS 1-s glitches). Record clock: median -0.050 / -0.030 s. Transient RMSE (front, rover) median 0.029 m/s in the header clock vs 0.035 in the record clock. The header-clock RMSE blows up to 0.11-0.27 m/s in bags with GNSS stamp episodes (28538acf, 2366c74a, 4d487b0d, 40ffd323, 3b3d9eb8), where record-clock RMSE stays at 0.03-0.05. It is also elevated in 2050d396, 33bec73f (rear) and 50956d6e, where both clocks are high (0.11-0.21), pointing to a sensor or slip issue rather than timing.
#################### map_geometry
SUMMARY: Coordinate frames, map geometry, antennas, direction, off-map segments and stops, checked on 97 unique bags (25 duplicate pairs were dropped: 122 bags minus 25 = 97). 69 of the unique bags have GNSS; 58 of those are full runs with a known label.

1. **Frame.** The pathgraph frame is exactly UTM 37N (WGS84) minus (300000, 6100000). A least-squares similarity fit leaves a translation of about 2 cm, a rotation below 2e-6 rad and a scale error below 31 ppm. Cross-track residual RMS is 0.07 m. Use the identity transform.
2. **Height.** GNSS altitude minus map z is 3.10 m, with a spread of only 0.008 m across 38 clean bag/antenna series.
3. **Antennas.** The master antenna is always 12.44 m behind the rover along the track: 12.439 m on vehicle 30618 and 12.423 m on vehicle 30639. This holds in both directions, so the tram always runs forward and turns on loops at both termini. The two antennas are time-synchronised. The master antenna sits closest to the centerline. Picking the wrong reference antenna would add a constant 12.4 m along-track error.
4. **Direction.** Every bag covers one direction only; runs come in pairs (Tallinskaya to Shchukinskaya, then back). Direction can be told from the first 3 s of GNSS with 100% accuracy, using start terminus, lateral offset (the two tracks are 3.48 m apart) and the heading of the master-to-rover line.
5. **Off-map time.** Roughly 20–30% of each run is off the map (terminus loops). The Shchukinskaya loop follows one path and is 1110 m from the end of one map direction to the start of the other. The Tallinskaya terminus has at least two branches about 25–39 m apart plus a local variant about 5 m apart. So the off-map path there can only be predicted from the initial GNSS fix for run starts, and cannot be predicted for run ends.
6. **Stops.** There are 7 regular stops per direction. Each is used by 85–97% of runs, and the first stop at each lands within 0.07–0.34 m (median absolute deviation) of the stop's usual position. Some runs stop a second time 5–12 m further on, so only the first stop should be used to correct position.
7. **Grade and curves.** Grades reach ±40‰, with about 820 m per direction steeper than ±20‰. This is the largest resistance term in the dynamics model. There is one sharp 81° curve with a radius of about 30–35 m, near position 1685 m on the Tallinskaya→Shchukinskaya track. The terminus loops have radii of 17–27 m.
8. **Side findings.** The wheel speed is in km/h and is very accurate. Over the mapped section, wheel distance divided by map length is 0.9996 for vehicle 30618 and 0.997 for vehicle 30639, so vehicle 30639 needs a 0.3% scale correction. The GNSS reference is only centimetre-level (quality class A) in 27 of 69 bags, and in only 3 of the 19 bags from vehicle 30639.

The analysis ran inline in Python without writing any files, because the session was in read-only plan mode. Nothing under the project directory was changed.
- [F1_frame] (high, CRIT) Pathgraph frame = UTM37N(WGS84, lon0=39, k0=0.9996) minus (300000, 6100000) with no further rotation/scale; the provided geo.utm() + offset is exact to ~2 cm.
    EVID: Point-to-line least-squares similarity fit on 38 clean bag-antenna series (67k points, both directions, per-antenna lateral bias as nuisance): t=(-0.019,+0.008) m, rot=4.8e-7 rad, scale=-20 ppm (T2S only: t=(-0.023,-0.002), rot 4.7e-7, scale -31 ppm; S2T: t=(-0.012,+0.055), rot -1.8e-6, scale +10 ppm). Residual cross-track RMS 0.067 m (T2S) / 0.12 m (S2T), |res| p50 0.025 m, p95 0.095 m; RMS with bias-only model 0.0683 vs 0.0670 with similarity => transform adds nothing. Map: T2S 4709 pts, L=4708.0 m; S2T 4710 pts, L=4708.3 m; spacing 1.000 m; 'tang' = heading atan2(dy,dx) in rad (median diff 0.0004 rad); 'curv' in 1/m, + = left turn (d(tang)/ds / curv median 0.998, corr 0.997).
    IMPL: Node output frame: E-300000, N-6100000 of UTM37N; use map polyline directly; no extra transform. Use map 'tang'/'curv' fields as-is.
- [F2_tracks_parallel] (high, CRIT) The two directions are separate parallel tracks 3.48 m apart; each direction's track has the other on its left (right-hand running). Every bag follows exactly one direction on-map.
    EVID: NN distance T2S->S2T polyline: p5 3.41, p50 3.48, p95 3.55, max 4.26 m; signed side +3.3..+4.2 m (left) for both. In all 69 GNSS runs, on-track points (|d|<1.2 m) belong to one track (e.g. 30618_073f08d1: 9506 T2S vs 0 S2T); mixed counts only in bags with meter-level GNSS.
    IMPL: Choose the track once at initialization (threshold |d|<1.74 m = half separation); after that, do 1-D along-track estimation (s) on a fixed polyline. No track switching is needed on-map.
- [F3_antenna_layout] (high, CRIT) The master antenna is always BEHIND the rover along the direction of travel, by 12.439 m (vehicle 30618) or 12.423 m (vehicle 30639), in both directions. The tram never reverses and turns on loops at both termini. The two antennas are time-synchronised. The master antenna is closest to the centerline.
    EVID: s_master - s_rover, median per bag: 30618 = -12.439 m (std across 49 bags 0.002), 30639 = -12.423 m (std 0.002, 10 bags). This is dir=+1 in all 59 bags, 25 T2S and 34 S2T. Stationary vs moving (>5 m/s) medians agree within 3 mm, so the header-stamp offset between antennas is below 0.5 ms. Baseline length equals the along-track distance (12.438 vs 12.439 m). Lateral offset d_master - d_rover is +0.025 m on 30618 and -0.175 m on 30639. On straight track the median cross-track is: master -0.032 m (30618) and -0.026 m (30639); rover -0.058 m (30618) and +0.149 m (30639).
    IMPL: Estimate the position of one reference point, most likely the master antenna, because the judge presumably uses /sensing/gnss/master/fix. Rover-based measurements are converted with s_master = s_rover - 12.44 m. Using the rover or the midpoint as the reference would add a constant 12.4 m or 6.2 m along-track bias. Confirm with the organisers which point the reference uses.
- [F4_curve_lateral] (high) Both antennas sit to the outside of curves in proportion to curvature, by about 8.7 m² × curvature. At R≈35 m this is ~0.25 m outward.
    EVID: Pooled clean bags, regression of cross-track d on map curvature k. Master 30618: d = -0.026 - 8.7k. Master 30639: -0.021 - 9.0k. Rover 30618: -0.051 - 8.7k. Rover 30639: +0.140 - 8.6k. In curves with R<50 m, median d·sign(k) = -0.25 m (master 30618, n=2067).
    IMPL: If the reference is the raw master fix, the output could add a lateral offset of -0.03 - 8.7·curv metres (left-positive). If cross-track is scored against the map centerline, output the centerline itself (0 by construction). Either way the effect is at most 0.3 m.
- [F5_altitude] (high) GNSS altitude minus map z at the projected point is a constant 3.10 m (antenna height plus datum). Map z is very smooth.
    EVID: 38 class-A bag-antenna series. Per-bag medians: T2S master 30618 3.095±0.008 (n=14), T2S rover 30618 3.097±0.007 (n=10), S2T master 30618 3.098±0.008 (n=11), 30639 3.097–3.114. Trend along track below 0.02 m/km; within-bag p5–p95 width 0.30–0.37 m. Map z second difference at 1 m has σ = 3.8 mm; z range 144.8–173.6 m. Bags without cm-level fixes show offsets of -3.3 to +19.7 m, i.e. a wrong altitude reference.
    IMPL: z_out = map_z(s), or map_z(s) + 3.10 m if the reference z is the raw GNSS altitude. Off-map, use z from the offline extended centerline (GNSS altitude - 3.10). The |alt - z_map - 3.10| < 0.3 m test is also a good quality filter for GNSS fixes.
- [F6_gnss_quality] (high, CRIT) NavSatFix status==2 does NOT mean a cm-level RTK fix. Covariance is always zero (cov_type=0), so quality can only be judged from the map residual. About a third of GNSS runs have a meter-level reference.
    EVID: Quality metric: fraction of on-map samples within 0.15 m of the antenna's lateral bias and within 0.3 m of z+3.10. Class A (≥0.9): 27 runs (24 on 30618, 3 on 30639: 927002c2, 9f0b519f, d927f360). Class B (0.7–0.9): 18 runs. Class C (<0.7): 24 runs, including 13 of 19 bags on 30639. Examples: 30618_27e994fc has status 2 but |d| p50 0.49 m; 30618_88548b02 status 2 with p95 |d-med| 1.2 m; the 30639 master is status 0 for the whole run in 9 bags (e.g. 3b3d9eb8, c31df386, d601d28f, dce52be4). On 30639 the rover is status 2 more often than the master. cov[:,0] is 0.000 in all bags checked.
    IMPL: Calibrate position-related parameters (wheel scale, stop map, off-map centerline) on class-A runs only, plus class-B runs with outlier rejection. When evaluating offline, expect reference noise of 0.5–2 m in class-C bags. At initialization, prefer the antenna whose fixes agree with the map (|d - bias| < 0.15 m) and fall back to the other one using the 12.44 m offset.
- [F7_direction_init] (high, CRIT) Direction and track can be identified from the first 3 s of GNSS with 100% accuracy on the 69 GNSS runs. Every bag starts stationary.
    EVID: All 97 unique bags have wheel v=0.0 km/h in the first 20 samples. GNSS begins 0 to 1.6 s after the first wheel message, giving 17–33 fixes per antenna in 3 s. S2T runs start 0–14 m from the Shchukinskaya stop (103632, 86048); 30618_a869780d starts 67 m away, inside the loop. T2S runs start 0–52 m from the Tallinskaya stop (99008, 84942). Mid-route starts: 30618_2366c74a is on T2S at s=664, d=-0.1 m, heading-vs-tangent 0° (the S2T hypothesis gives d=+3.5 m and 180°). 30618_2cb9ce37 is on T2S at s=1257, d=-0.1, 0°. Both restarts happened at regular stops (664.5 and 1256.1). The master-to-rover baseline heading gives the travel direction, since the rover is ahead: baseline 12.43–12.45 m with status 2, and 10.5–14.6 m with status 0 (heading error up to about 10°). Consecutive bags are paired T2S then S2T with a 10–25 s gap and a 0.0 m jump at Shchukinskaya.
    IMPL: Initialization over the first N seconds (2–5 s): take the median of master and rover fixes and project both onto the EXTENDED centerlines of both directions. Choose the direction/track that (a) is within 1.74 m laterally and (b) has baseline heading within 30° of the tangent. Off-map, use the terminus rule instead: within 100 m of Shchukinskaya means S2T; within 100 m of Tallinskaya means T2S. Initial s = s_master (or s_rover - 12.44). Also fit an initial wheel-speed bias only if the tram moves during the window; normally it does not.
- [F8_offmap] (high, CRIT) About 20–30% of each run's time is off-map (terminus loops). The Shchukinskaya loop is a single repeatable path. The Tallinskaya terminus has branches that the node cannot tell apart without GNSS.
    EVID: Median distance (from front-wheel odometry) and time per segment, 30618 / 30639. T2S head (Tallinskaya stop to T2S s=0): 187 m (135–222) / 182 m, 103 s / 76 s. T2S tail (T2S map end to Shchukinskaya stop): 511 m (484–512) / 510 m, 156 s. S2T head (Shchukinskaya stop to S2T s=0): 599 m (452–617) / 598 m, 227 s / 186 s. S2T tail: 149 m (22–324) / 165 m, 63 s / 103 s. S2T recordings usually end while still moving at 9–15 km/h, before the Tallinskaya stop. Shchukinskaya: the T2S tail ends where the S2T head starts, at (103629, 86042) vs (103627, 86040). The loop from T2S map end to S2T map start is 1110 m, turns 160° with Rmin≈22 m, and has grades of -17 to +18‰. Spread of individual runs around the median template (p50/p95 cross-track): T2S tail 0.01–0.03 / 0.06–0.6 m; S2T head 0.01–0.05 / 0.06–0.2 m in 14 of 21 runs. Larger deviations (1–5 m) come from GNSS multipath, since master and rover disagree about where they occur (e.g. 30618_9c09b081, 2dbce472). Tallinskaya: the main loop from S2T map end to stop A (99008, 84942) is 324 m, turns 247° with Rmin 16.6 m (30618_0652866c). Branch B has starting positions (99026, 84924), (99022, 84921) and (98999, 84907), about 25–39 m from branch A; it joins the main path about 80 m before T2S s=0. S2T tails split onto branch B 115 m after S2T map end in 3 of 23 runs (8158f0b0, 49fe4c54, 30639_0be558e2). Another variant with a constant 5 m offset appears 107 m before T2S s=0 in 6 of 22 T2S heads.
    IMPL: Offline, build extended centerlines by taking the median of class-A master tracks, parameterised by wheel distance, and anchor them to map s. T2S covers s = -222..5219 (head variant A, plus B when the start fix is on B); S2T covers s = -617..5032 (main-loop tail A). Online, compute s from wheel odometry and look up x, y, z on the extended line. At a Tallinskaya start, choose branch A or B from the initial GNSS fix. At the Tallinskaya end (S2T tail) always use the main branch: this accepts up to 39 m error in about 13% of run endings. Keep s monotonic, since the tram never reverses (wheel v ≥ 0 in 100% of samples).
- [F9_wheel_scale] (high, CRIT) Wheel 'v' is in km/h with an almost exact scale on vehicle 30618 (0.04%). Vehicle 30639 reads about 0.3% low. Dead-reckoning error at stops grows to about 3 m (p90) after 2.5 km on 30618 and to about 15 m on 30639 without calibration.
    EVID: On-map wheel distance (front bogie, v/3.6) divided by map length: median 0.9996 (T2S, 30618, n=23), 0.9995 (S2T, n=25), 0.9971–0.9973 (30639, n=14). Offset between wheel-integrated s and map s at the start vs the end of the map (map s minus wheel distance, class-A runs): -2.8..+6.7 m on 30618 and +11.9..+16.2 m on 30639. Error of pure wheel odometry at regular-stop visits, starting from the first on-map GNSS fix: 30618 p90 0.63 m (<1 km), 1.77 m (1–2.5 km), 3.0 m (>2.5 km), max 6.75 m; 30639 median -0.35 / -5.96 / -12.27 m. Stationary samples are exactly 0.0 on both bogies (22–34% of samples); the smallest non-zero value is 0.15–0.20 km/h; there are no negative values.
    IMPL: Use a per-vehicle wheel scale: v_mps = v_kmh/3.6 × 1.0004 (30618) or × 1.003 (30639), refined by the speed/slip analysis. If the node cannot know the vehicle ID, use a mid value (about 1.0015) and rely on stop anchoring. Treat v==0 on both bogies as an exact 'stationary' flag for zero-velocity updates.
- [F10_stops] (high, CRIT) There are 7 regular stops per direction. Each is used by 85–97% of runs, and the first stop lands within 0.07–0.34 m (median absolute deviation) of its usual position. Several optional stop clusters (traffic lights and queues, 9–56% of runs) are also tight. Second, creeping stops fall 5–12 m beyond the first.
    EVID: Stops defined as v==0 on both bogies for ≥3 s; 1039 stops, 662 of them on-map. Positions (s, map coordinates) are master-equivalent. Regular T2S stops: 38.7 (90%, dwell median 17 s), 251.7 (93%, 18 s), 664.5 (97%, 18 s), 1256.1 (88%, 35 s), 1817.8 (88%, 25 s), 2248.6 (88%, 20 s), 4370.6 (91%, 19 s). Regular S2T stops: 282.1 (94%, 20 s), 2400.5 (89%, 31 s), 2833.5 (89%, 27 s), 3499.5 (85%, 32 s), 3976.0 (94%, 17 s), 4396.2 (91%, 18 s), 4616.5 (~100%, 16 s). First-stop deviation in class A/B runs: MAD 0.07–0.34 m, p90 |dev| 0.17–0.90 m, max 0.22–2.3 m. Second stops within ±15 m happen in 0–6 runs per cluster, at median +5.3 to +12.3 m. Optional T2S stops: 931.1 (31%, MAD 0.06), 1164.1 (19%), 1315.5 (15%), 1986.1 (42%, MAD 0.17), 2345.5 (15%), 4278.2 (48%, MAD 0.11), 4663.0 (56%, MAD 0.15). Optional S2T stops: 239.4 (15%), 315.9 (18%), 364.9 (15%), 1799.0 (15%), 2288.5 (20%), 2621.7 (40%, MAD 0.33), 2979.2 (11%), 3320.3 (54%, MAD 0.21), 3720.5 (41%, MAD 0.13). The closest pair of clusters is 34 m apart (S2T 282→316); otherwise ≥43 m. Terminus stops: Shchukinskaya at T2S s≈5219 / S2T s≈-599; Tallinskaya stop A at T2S s≈-187 (starting positions range -222..-135).
    IMPL: Optional anchoring, which counts as offline calibration and should be confirmed as allowed. When a stop starts (v==0 on both bogies for ≥3 s, or earlier after 1–2 s), find the nearest cluster centre c. If |s_pred - c| < min(4 m, 3·σ_pred) and this is the first stop within c±15 m in this run, apply a Kalman update s=c with σ=0.3 m (regular stops) or 0.5 m (optional clusters with MAD<0.35 m). Do not anchor on later stops in the same window, because creep stops are 5–12 m ahead. Skip anchoring if σ_pred > 8 m, which would be ambiguous given the 34–43 m spacing. On vehicle 30618 this should cap along-track error at about 1–3 m between stops.
- [F11_stop_pattern_labels] (high) The sequence of stops alone, in wheel distance, identifies the direction and the along-track offset of runs without GNSS.
    EVID: Stop-pattern matching (align detected stops to regular clusters, count matches within 8 m) classified 58 of 58 GNSS-labelled full runs correctly. For 16 no-GNSS bags on vehicle 30618 it matched 7–10 regular stops with median residual 0.2–1.2 m. Labels alternate T2S/S2T consistently with the chronological pairs (e.g. 79b204dc T2S, then 2161b58b S2T; 9bbe6faa T2S, then 0259fe53 S2T). Starting positions came out at -204..-155 (T2S) and -597..-447 (S2T), matching the GNSS runs.
    IMPL: The 16 no-GNSS bags can be given pseudo-position labels offline, adding dynamics and slip training data and validation of the wheel scale. The same logic could serve as a fallback for re-localising online if initialization fails.
- [F12_grade] (high, CRIT) Grade reaches ±40‰ with long ramps of about 30–35‰. About 820 m per direction is steeper than 20‰. Grade is the dominant non-inertial longitudinal force.
    EVID: Map z smoothed over 25 m. T2S: min -39.5‰ at s=1493 (ramp s≈1400–1600 at -31 to -33‰), max +35.5‰ at s=3638 (ramp s≈3400–3600 at +28 to +34‰); p5/p50/p95 = -29.5/-2.7/+32.5‰; 822 m steeper than 20‰; never steeper than 40‰. S2T is the mirror image: min -35.5‰ at s=1072, max +39.7‰ at s=3223. Results for 10/25/50 m windows differ by <0.3‰ because map z is already smooth. Profile every 200 m for T2S: 0:+2, 400:-8, 1200:-4, 1400:-33, 1600:-31, 1800:-14, 2400:-14, 2600:+2, 3400:+28, 3600:+34, 3800:+16, 4400:+16. Off-map loops: -17..+18‰. At 40‰, g·sinθ ≈ 0.39 m/s², versus about 0.02–0.05 m/s² for rolling resistance.
    IMPL: In the dynamics model, use a_grade = -g·dz/ds evaluated at the current s estimate. Precompute a 1 m lookup of grade from 25 m-smoothed map z for each extended centerline (GNSS altitude - 3.10 off-map). An s error of a few metres causes negligible grade error. Grade must be included when calibrating traction and brake maps, otherwise notch-to-force fits pick up a ±0.35 m/s² bias.
- [F13_curvature] (high) The only sharp on-map curve is an 81° turn with Rmin≈30–35 m. The other curves have radii of 105–200 m. The terminus loops have radii of 17–27 m.
    EVID: Map curvature: T2S s=1641–1716 (right turn, 81°, Rmin 34 m; 3-point circle over 5 m gives 35.2 m at s=1685). S2T s=2994–3070 (left turn, 81°, Rmin 30 m, 32.1 m at s=3026); this is the same intersection. Track length with R<100 m: 66 m (T2S) and 69 m (S2T); with R<300 m: 487 and 484 m. Other curves on T2S: s 888–936 (R 137), 1015–1053 (R 105). On S2T: 3659–3706 (R 119), 3756–3824 (R 137). Off-map loops: Shchukinskaya S2T head Rmin 21.9 m (105 m of track with R<50 m); T2S tail Rmin 24.4 m; Tallinskaya S2T tail Rmin 16.6 m (120 m with R<50 m); T2S head Rmin 27.2 m.
    IMPL: Add a curve-resistance term a_c = c_curve/R, looked up from map curvature (for example c_curve ≈ 0.5–0.8 m²/s² per metre of 1/R, to be fitted). It only matters at the sharp curve and in the loops. Slip/slide flags are more likely in the sharp curve, since front and rear bogies differ geometrically there. Speed is naturally low there, which the model can use as a prior.
- [F14_duplicates] (high) There are 25 exact duplicate pairs, leaving 97 unique bags. Of these, 69 have GNSS (including 11 short or partial bags) and 19 have none.
    EVID: MD5 of front-bogie v plus t_hdr gives 25 pairs, for example 0259fe53=cfd9fd5a, 0a83c933=e392e5bd, 117c2d02=46e21b9b, 1cc230fa=a395846d, 21dd3af3=f3b8c99b, 2366c74a=93dc866e, 2cb9ce37=5eb8d2c9, 3b36e5cd=ae4eb346, 40ffd323=efb92709, 4d487b0d=b3042f78, 6cb3280a=f3a0694d, 748832b9=7849303f, and others. Each group of topics is identical in message count.
    IMPL: Deduplicate before any train/validation split, otherwise duplicates leak across splits. Split cross-validation by day and vehicle (days 05-05, 07-27, 08-10, 08-26, 09-03).
- [F15_timestamps] (medium) Record time is later than header stamp by a topic-dependent latency of 1–90 ms.
    EVID: Median t_rec - t_hdr: master fix 0.043–0.048 s; rover fix 0.068–0.081 s; GNSS vel topics 0.067–0.089 s; wheels 0.028–0.064 s (p95 up to 0.097 s); driver cmd 0.001–0.005 s. At 14.8 m/s, 50 ms equals 0.74 m of along-track error.
    IMPL: Estimate state at the wheel header stamp and set the output header.stamp to the stamp the judge expects, either the header of the triggering input or its record time. Compare against the GNSS reference by header stamp. An inconsistency here gives a speed-proportional along-track bias of up to about 1 m.
PARAMS:
  * FRAME = x = UTM37N_E(WGS84, lon0=39, k0=0.9996) - 300000; y = UTM37N_N - 6100000; no rotation or scale  (Similarity fit on 67k class-A RTK points gave residual t=(-0.019, 0.008) m, rot 5e-7 rad, scale -20 ppm)
  * ANT_ALONG_OFFSET (rover ahead of master) = 30618: 12.439 m; 30639: 12.423 m; s_master = s_rover - offset  (Median of s_master - s_rover over 49 / 10 bags, spread across bags 0.002 m)
  * ANT_LATERAL (left +, straight track) = master: -0.03 m (both vehicles); rover: -0.058 m (30618), +0.149 m (30639); plus a curvature term -8.7*curv m  (Median cross-track on straight track plus regression on map curvature)
  * ALT_MINUS_MAPZ = 3.10 m (sigma 0.008 m)  (Per-bag median of GNSS alt - map z over 38 class-A series)
  * TRACK_SEPARATION / assignment threshold = 3.48 m / |d| < 1.74 m  (Nearest-neighbour distance between the two map polylines, p50 3.48 m (p5 3.41, p95 3.55))
  * TERMINUS_STOPS (pathgraph xy) = Shchukinskaya (103632, 86048) = T2S s≈5219 = S2T s≈-599; Tallinskaya stop A (99008, 84942) = T2S s≈-187; branch B starts about (99022, 84921)  (Start/end positions of the 69 GNSS runs mapped to wheel-distance extended s)
  * INIT_DIRECTION_RULE = On-map: track with |d|<1.74 m and baseline heading within 30° of the tangent. Off-map: within 100 m of Shchukinskaya gives S2T; within 100 m of Tallinskaya gives T2S. Window 2-5 s, median of fixes  (100% accurate on 69 GNSS runs, including mid-route starts (2366c74a, 2cb9ce37) and the in-loop start of a869780d)
  * EXTENDED_S_RANGE = T2S: [-222, 5219]; S2T: [-617, 5032] (map core [0, 4708])  (Off-map head and tail lengths measured with wheel odometry on class-A runs)
  * WHEEL_SCALE = v_mps = v_kmh/3.6 × 1.0004 (30618), × 1.003 (30639)  (Median on-map wheel distance / map length = 0.9996 and 0.9971-0.9973)
  * STOP_CLUSTERS_REGULAR (s, m) = T2S: 38.7, 251.7, 664.5, 1256.1, 1817.8, 2248.6, 4370.6. S2T: 282.1, 2400.5, 2833.5, 3499.5, 3976.0, 4396.2, 4616.5  (1-D gap clustering (12 m) of first-stop positions from GNSS; 85-97% of runs; MAD 0.07-0.34 m)
  * STOP_CLUSTERS_OPTIONAL (s, m) = T2S: 931.1, 1986.1, 4278.2, 4663.0 (plus 1164.1, 1315.5, 2345.5 as weaker). S2T: 2621.7, 3320.3, 3720.5, 2288.5 (plus 239.4, 315.9, 364.9, 1799.0, 2979.2 as weaker)  (Same clustering; 9-56% of runs; the listed primary ones have MAD ≤ 0.33 m)
  * STOP_ANCHOR = trigger: v==0 on both bogies for ≥3 s; gate |s_pred - c| < min(4 m, 3·sigma_pred); first stop per cluster only (ignore stops up to +15 m after it); measurement sigma 0.3 m (regular) / 0.5 m (optional); disable if sigma_pred > 8 m  (First-stop p90 |dev| 0.17-0.9 m; creep stops +5..+12 m; minimum cluster spacing 34 m; DR p90 error 0.6-3 m)
  * GRADE_LOOKUP = dz/ds from map z smoothed over 25 m, 1 m lookup; range ±40‰; off-map from GNSS alt - 3.10  (Grade statistics on the map; results insensitive to a 10-50 m window)
  * CURVATURE_LOOKUP = Map 'curv' (1/m, + = left); sharp curve T2S s 1641-1716 / S2T s 2994-3070 with Rmin 30-35 m; loops Rmin 17-27 m  (Map field checked against d(tang)/ds (ratio 0.998) and against 3-point circle fits)
  * CALIBRATION_BAG_SET (class A) = 30618: 01f73500, 073f08d1, 0f120b35, 2050d396, 22c1c589, 2f104a1d, 33bec73f, 3e9f4952, 40ffd323, 49fe4c54, 4d487b0d, 67b89902, 68847170, 748832b9, 76e1f9c7, 9c09b081, a53d5f6f, ab5921a4, b8044aa0, b83d854d, d4ba7005, e2dcf65f, e3d94878, e9a34502. 30639: 927002c2, 9f0b519f, d927f360  (≥90% of on-map samples within 0.15 m laterally and 0.3 m vertically of the map)
OPEN Q: Which point is the judge's position reference: master fix, rover fix, or a vehicle-centre point? A wrong choice costs a constant 6.2-12.4 m along-track error. | Does the judge score off-map samples, which are about 20-30% of each run? If so, is the reference the raw GNSS in the terminus loops, where GNSS itself is meter-level in about 30% of runs? | Is the reference z the raw GNSS altitude (map z + 3.10) or map z, and is z scored at all? | Output timestamp: the task says 'input bag time'. Is that the record timestamp or the header stamp of the input message? They differ by 30-90 ms, about 0.5-1.3 m at cruising speed. | Is it allowed to use offline-built stop locations and extended off-map centerlines taken from training GNSS? This probably counts as 'offline calibration', but confirm. | Vehicle identity is not in the 3 input topics. Is the vehicle known at test time (bag prefix 30618/30639)? If not, the 0.3% wheel-scale difference needs online estimation or a compromise value. | At Tallinskaya the S2T tail takes branch B (25-39 m away) in about 13% of runs, and there is a 5 m variant in about 27% of T2S heads. This is not observable without GNSS. Accept the error or model it as the most likely branch? | Will test bags include mid-route starts (seen twice in training, both at regular stops) or starts with only status-0 GNSS (the 30639 master)? The initializer should then use the rover, or both antennas with the 12.44 m offset.
SCRIPTS: NOTE: the session was in read-only plan mode, so all analysis ran inline (python via stdin) and no script files were written. Core projection used everywhere (pathgraph polyline, signed cross-track left+, extrapolates beyond the ends):
class PL:
    def __init__(s,P):
        s.P=np.asarray(P,float); seg=np.diff(s.P,axis=0); s.L=np.hypot(*seg.T); s.cs=np.r_[0,np.cumsum(s.L)]
        s.u=seg/s.L[:,None]; s.tree=cKDTree(s.P); s.n=len(s.P)
    def project(s,xy):
        _,i=s.tree.query(xy); best=None
        for j in (np.clip(i-1,0,s.n-2),np.clip(i,0,s.n-2)):
            r=xy-s.P[j]; t=(r*s.u[j]).sum(1)
            t=np.clip(t,np.where(j==0,-np.inf,0),np.where(j==s.n-2,np.inf,s.L[j]))
            e=xy-(s.P[j]+t[:,None]*s.u[j]); dist=np.hypot(*e.T)
            d=s.u[j][:,0]*e[:,1]-s.u[j][:,1]*e[:,0]; sv=s.cs[j]+t
            if best is None: best=[sv,d,dist]
            else:
                m=dist<best[2]; best=[np.where(m,a,b) for a,b in zip([sv,d,dist],best)]
        return best  # s, signed d (left +), |d|
# xy = np.c_[utm(lat,lon)[0]-3e5, utm(lat,lon)[1]-6.1e6]; map files: 'таллинская - щукинская.json' (T2S), 'щукинская - таллинская.json' (S2T)
VERDICTS:
  ~ [F1_frame] partially_confirmed: The pathgraph frame is UTM zone 37N (WGS84, lon0=39°, k0=0.9996) minus a false origin of (300000, 6100000). The residual translation is only 1–4 cm and there is no measurable rotation. The ENU/local-tangent alternative (about -280 ppm scale and 1.3° rotation) is excluded. Three corrections apply. (1) Antenna cross-track residuals are not flat: they depend on curvature, d ≈ bias − 8.6·curv (m), which is outward and about 0.25 m at R≈30–40 m. This is expected for roof antennas on a bogie vehicle and must be modelled or tolerated (±0.25 m), not read as frame error. (2) Frame scale is not resolved by cross-track data: T2S fits -61 ppm and S2T -5.5 ppm, which disagree. Map distances are also UTM grid distances, and the point scale here is 0.99972, so grid distances are 280 ppm shorter than ground distances (about 1.3 m over 4.7 km). Wheel odometry measures ground distance, so this 280 ppm is the relevant along-track factor. (3) Point spacing is not exactly 1 m: the mean is 0.999993 m on T2S but 0.999861 m on S2T. On S2T, index ≠ metres by about 139 ppm (0.65 m over the route), so s must come from cumulative segment length, not point index.
      EVID: Method: I re-derived UTM with independent Krueger n^4 and Snyder series. geo.utm agrees within 0.03 mm and 0.65 mm. I then ran a least-squares fit of signed cross-track residuals of both antennas, for 31–42 clean bags (about 0.5M samples, 49–61 series), against the map polylines. The model had a per-series bias, a per-(vehicle, antenna) curvature term and a global translation. Standard errors are from a bag bootstrap.

Translation (dx, dy) in m:
- pooled: (-0.007, +0.029)
- T2S: (-0.010, +0.038)
- S2T: (-0.004, +0.015)
- SD 3–8 mm

Adding rotation and scale to the fit:
- rotation ≤6e-6 rad (≤1.4 cm at 2.3 km)
- scale T2S -61 ppm, S2T -5.5 ppm, pooled -42 ppm; inconsistent between tracks, so poorly constrained

Residuals after the fit:
- |res| p50 0.023–0.030 m, p95 0.095 m
- 200 m binned mean residual range: T2S 0.132 m (+0.08 at both ends), S2T 0.066 m

Curvature term: k = -8.4..-8.8 m² for both antennas on both vehicles. Pooled residual is +0.25 m at curv -0.02..-0.04 and -0.23 m at curv +0.02..+0.04.

Vertical: alt − z_map = 3.097–3.100 m median (p5–p95 3.085–3.116 over 46 series) and |dz| p50 0.025 m. So z_map is ground/rail level and the antennas are about 3.10 m above it.

Map field checks:
- The curv field matches d(tang)/ds at 1-point differencing: median ratio 0.998 (T2S) and 0.999 (S2T); LS slope 0.993 / 0.989; corr 0.997 / 0.989. My earlier S2T ratio of 0.93 came from a wider smoothing window and is withdrawn.
- Along-track registration could only be checked through heading diversity (-22..68° on T2S). A grade/z-based along-track check was inconclusive (±4 m per 600 m segment).
  ~ [F2_tracks_parallel] partially_confirmed: The two mapped tracks are antiparallel double track. Centre-line separation is 3.46 m median (p5 3.40, p95 3.53, min 3.31), with a local widening to 4.24 m around T2S s≈1653–1706 m. The opposite-direction track is always on the LEFT of the direction of travel. However, the map does not contain every track actually used. On S2T there is an unmapped parallel bypass or siding about 4.2 m to the RIGHT of the mapped track. It runs between S2T s≈530 and s≈1140 m, with turnouts at s≈520–550 and s≈1140–1150. 13 of 26 S2T runs of vehicle 30618 use it. The bypass is about 610 m long, the same as the mapped section, so along-track distance is nearly unaffected. But GNSS map-matching must allow a -4.2 m lateral branch there, and it must not be read as a GNSS error or a wrong-track event. No equivalent exists on T2S. Other brief minority-track or lateral-offset episodes of 1–3.8 m are GNSS common-mode errors, not real track changes.
      EVID: I projected every map point of one track onto the other: separation p5 3.397, p50 3.461, p95 3.533, min 3.306, max 4.24 m (at T2S s 1653–1706). Median heading difference is 179.9°, and the other track is on the left in both directions.

Bypass evidence, from per-run cross-track e of both antennas along S2T:
- e profile at s=520/530/550…1120/1140/1150: -0.3, -2.4, -4.2 (steady), -1.7, -0.1 m
- Master and rover agree, status 2
- Runs that use it (13): 0652866c, 095a115b, 0e41eac3, 28538acf, 2dbce472, 49fe4c54, 68d1748a, 76e1f9c7, 8158f0b0, 87afe526, 9c09b081, ab5921a4, b83d854d
- Runs that do not use it: 01f73500, 1cc230fa, 3b36e5cd, 3e9f4952, 40ffd323, 437e855c, 4d487b0d, a53d5f6f, a869780d, dd8e6395, dd8d0741, and all clean 30639 S2T runs (0be558e2, 50956d6e, 584b6e32, 4285f2bc)
- Transit time s530–1140 is 105–107 s on the bypass vs 76–78 s on the mapped track, which suggests a speed restriction or a stop
- Wheel-integrated distance over the section is 608–609 m vs 601–607 m
- Clean T2S bags have 0% of samples with |e|>0.5 m

GNSS-error episodes (not real track changes):
- 30639_d3c43d69 t 201–259: both antennas about 3 m left; rover status 2, master status 0
- 30639_253671cc: simultaneous -2.3 m jump at t=476
- 30618_87afe526: rover-only 3.5 m lateral plus 20–64 m along-track error at t 580–610, while status=2
  ~ [F3_antenna_layout] confirmed: The two GNSS antennas are mounted on the vehicle axis with a fixed longitudinal baseline. The master is always BEHIND the rover (s_master − s_rover ≈ -12.44 m on 30618, -12.42 m on 30639), and the baseline length equals the along-track separation. Both receivers share epoch timestamps. Tram motion heading is master→rover, and it equals the map tangent. None of the data shows reversing. Lateral mounting offsets on straight track, left-positive:
- 30618: master ≈-0.03 m, rover ≈-0.06 m
- 30639: master ≈-0.03 m, rover ≈+0.15 m
The master is the better antenna for centreline position. Caveat: the rover can fail independently, e.g. an along-track jump of 20–64 m in 87afe526 while it reports status 2. Use the baseline length (12.43±0.05 m) as a consistency check before trusting either antenna.
      EVID: Per-bag median s_master − s_rover from map projection:
- 30618: -12.431..-12.443 (median about -12.438, SD about 0.002, 50 bags)
- 30639: -12.419..-12.437 (median about -12.423, 14 bags)
Stationary and moving estimates agree within a few mm. The master is behind in all 64 bags with both antennas, on both tracks. Median of motion heading − map tangent is 0.0°.

Timing:
- Header stamps of master and rover are identical epoch-for-epoch: min 96.9%, median 100% of epochs, 69 bags
- Receive − header latency: master about 0.045 s, rover about 0.080 s

Reversing check:
- GNSS-inferred 'reverse' fractions (0686195f 3.3%, defd0170 4.8%, 27e994fc, 88548b02) are common-mode GNSS artifacts. At those times wheel speed was 0 or forward. Example: defd0170 t 19–64 s, wheels 0 while GNSS showed 15 m/s, master status 0.
- Negative wheel speeds occur only in 30618_3e012faf (-0.38/-0.39 km/h at about 592 s) and 30639_9c362687 (-0.26 km/h at 96 s). Both are standstill noise.

Lateral bias per bag on straight segments (|curv|<0.002):
- 30618: master -0.015..-0.05, rover -0.034..-0.075
- 30639: master ≈-0.03, rover +0.134..+0.163
  ~ [F6_gnss_quality] partially_confirmed: GNSS quality metadata cannot be trusted as an accuracy indicator:
- position_covariance is identically 0, and covariance_type is 0 (unknown) in every bag for both antennas.
- status=2 does not guarantee cm-level absolute accuracy. For the rover, status 2 reflects the moving-baseline (heading) fix relative to the master. The absolute solution can still have common-mode errors of 0.5–4 m and occasional along-track jumps of tens of metres.
- The master is status 0 for the entire run in 6 of 16 vehicle-30639 bags (3b3d9eb8, 44226bde, c31df386, d3c43d69, d601d28f, dce52be4), and for most of 9c362687 (94%), 2b4a6347 (81%) and 253671cc (71%). e4379d7f has no master messages at all. On 30618 the master is fully status 0 in 0686195f and defd0170.
Only about a third of runs have high-quality master GNSS; about a third are degraded. So GNSS-derived ground truth or initialisation must be validated against the map (cross-track residual, alt−z≈3.10 m, baseline≈12.43 m), not against the status flags.
      EVID: Quality metric per run: fraction of on-map samples with |cross-track residual|<0.15 m after the curvature correction and |alt − z_map − 3.10|<0.3 m.
- Master, 67 runs: ≥0.9 → 24, 0.7–0.9 → 21, <0.7 → 22. Degraded (<0.7): 12/51 on 30618, 10/16 on 30639.
- Rover: 14 / 21 / 32.

Fraction of status-2 samples with |e|>0.5 m:
- 27e994fc 0.50 (p50 |e| 0.45)
- 88548b02 0.44
- 095a115b 0.45 (bypass samples excluded)
- f19a4ac3 0.37
- 30639 rovers: 253671cc 0.77, 44226bde 0.91, 3b3d9eb8 0.65, c31df386 0.48, d601d28f 0.43, dce52be4 0.56

In these runs the baseline stays about 12.44 m and both antennas shift together, so the error is common-mode. That is consistent with rover status 2 meaning a moving-baseline RTK fix.
  ~ [F7_direction_init] partially_confirmed: Direction and track can be initialised reliably from the first seconds of GNSS, but the recipe must be robust:

(1) Every bag starts at standstill. Both bogie speeds read exactly 0 for at least the first 20 samples. First motion (front >0.5 km/h) occurs at 5.6–113 s, median 11.2 s. So stationary GNSS gives position but no velocity heading.

(2) Track and direction:
- The primary rule is the nearest-terminus rule (start near Shchukinskaya → S2T; near Tallinskaya → T2S). It is correct in 68/68 long runs.
- For mid-route starts (2366c74a at T2S s=664, 2cb9ce37 at s=1257), use the master→rover baseline heading compared with the map tangent. The two tracks are 3.46 m apart and 180° apart, so the choice is unambiguous even with degraded heading.

(3) The initial baseline is NOT consistently 12.43–12.45 m during the first 3 s, even with status 2 on both antennas: it ranges 10.49–12.67 m. Heading error vs the later path:
- status 2: p50 0.33°, p90 2.3°, max 9.9°
- other status: p50 0.8°, p90 7.1°, max 16.9°
Use the baseline sign and direction only, or map-snap, rather than the raw heading.

(4) Initial s should come from the master projected on the map, averaged over the standstill window. Stationary drift is p50 0.03 m, p90 0.39 m, max about 1.1 m (27e994fc).

(5) Do not chain consecutive bags by assuming continuity. Handover jumps are 0–0.2 m when the tram was stopped at the terminus. They reached 4.9–90 m when the previous bag ended in motion or the gap was 59–92 s.

(6) The front wheel sensor can be stuck at 0 early. In 30639_c31df386 the front read 0 until about 26 s while the rear and GNSS showed motion from about 14 s. Initialisation must not rely on the front wheel alone.
      EVID: Checks over all 97 unique bags (duplicates removed by md5 of the front wheel series):
- Both bogies read exactly 0.0 for the first 20 samples in every bag.
- GNSS velocity in the first 3 s is at most 0.06 m/s (p95 0.04).
- The first GNSS message arrives -0.98..+1.55 s relative to the first wheel message, with 14–30 fixes in the first 3 s.

Terminus rule:
- S2T starts are 0–14 m from Shchukinskaya (a869780d 67 m).
- T2S starts are 0–51 m from Tallinskaya.
- Mid-route start 2366c74a: T2S d=-0.09 m and dh 0°, vs S2T d=+3.51 m and dh 180°. 2cb9ce37 at T2S s=1257, d=-0.06.

Initial baseline:
- Status 2 on both antennas, 53 bags: 10.49–12.67 m (2f104a1d 10.85, ab5921a4 10.49, 49fe4c54 11.03, 87afe526 12.10, a869780d 12.67).
- Other status, 16 bags: 10.63–14.59 m.
- Heading error: 4d487b0d max 9.9°; 30639_44226bde 16.9°.

Initialisation anomalies:
- c31df386: first front message had t_rec − t_hdr = 5.1 s. The front then stayed stuck at 10.69 km/h for about 8 s.
- Stationary GNSS drift: p50 0.03 m, p90 0.39 m; 27e994fc 1.06 m range.

Bag pairing T2S→S2T at Shchukinskaya:
- Gaps are typically 10.2–27 s, but 59–92 s in 4 pairs.
- Jumps: 0.0–0.2 m when the previous bag ended stopped; 12.3 m (e9a34502 ended at 10.9 km/h), 16.4 m (67b89902), 4.9 m, 75 m, 90 m (e2dcf65f→28538acf, gap 70 s).
  ~ [F8_offmap] could_not_check: I did not verify this claim independently. The only related results I checked are these. (a) At the Shchukinskaya terminus, the end of a T2S bag and the start of the next S2T bag are geometrically consistent (0.0–0.2 m jump) when the tram was stopped. So the terminal loop or turnaround between the two polylines is short and off-map, but it does not break continuity at the endpoints. (b) Separately, there IS a significant unmapped on-route track: the S2T bypass 4.2 m to the right, s≈530–1140, used by 13/26 runs of 30618 (see F2). Any off-map detector based on a cross-track threshold <4.5 m would falsely flag these runs.
      EVID: I did not run a dedicated off-map analysis (samples outside the polylines at termini/depot, or excluded sections). The supporting data comes from the pairing check in F7:
- consecutive T2S→S2T bags at Shchukinskaya jump 0.0–0.2 m when stopped
- larger jumps of 4.9–90 m occur only when the previous bag ended moving or the gap was long
The bypass evidence is under F2: per-run e profile -4.2 m for s 550–1120 on S2T.
#################### dynamics
SUMMARY: Identified the notch-to-acceleration dynamics on 68 unique bags (122 raw bags; the rest were exact duplicates removed by md5 of the front wheel array): 51 bags from 30618 and 17 from 30639, about 820k samples at 10 Hz, with 20 Hz ZOH notch. The target was wheel-derived acceleration using header stamps; wheel vs GNSS-vel lag is 0.00 s.

**Model.** A Hammerstein model fitted by linear least squares, with in-sample RMSE 0.163 m/s^2 against a static-map RMSE of 0.205. It has four parts:
- a(n,v) as a hat-basis table with knots at v = 0,1,2,3,4,6,8,10,12,14,16 m/s;
- a first-order lag plus dead time per mode: traction d=0.05 s, tau=0.25 s; brake d=0.15 s, tau=0.30 s;
- a grade term with coefficients traction -0.886, brake -0.812, coast -0.978 (times g*grade);
- a curvature term of -2.2 m^2/s^2 times |curv|.

**Traction** has a low-speed plateau per notch, about 0.10 + 0.113*n. The total is capped at A0≈1.05 m/s^2 up to vb≈5.9 m/s, then decays as (vb/v)^0.57. This is not constant power.

**Braking** is linear in |n|: -(0.19 + 0.112|n|)*(1 - 0.33*exp(-v/3.0)), which is roughly flat above 4 m/s with a 20-30% fade toward standstill. Coast resistance is nearly constant at -0.03 to -0.05 m/s^2 for v = 4-16 m/s.

**Three special behaviours.** Getting these wrong breaks model-only odometry:
- Notch -8 is a closed-loop hold mode. It is entered from and exited to -9, lasts 10-40 s, the speed can rise during it, and it is uncorrelated with grade; residual std is 0.61.
- There are unannounced strong braking events at notch 0 or low traction (13 events, down to -2.5 m/s^2, at one map location). They are not wheel slide, so an external brake source exists.
- Start from rest has a 1.25 s delay after the first traction notch, and standstill needs hold logic.

**Open-loop speed error** (train even bags / test odd bags):
| Horizon | Speed MAE (m/s) | Distance MAE | Distance error |
|---|---|---|---|
| 2 s | 0.11 | 0.13 m | 1.4% |
| 5 s | 0.23 | 0.64 m | 2.8% |
| 10 s | 0.36 | 2.0 m | 4.5% |
| 30 s | 0.58 | 10.7 m | 7.9% |

Removing the grade term roughly doubles these errors.

**Vehicle and per-run differences.** Vehicle 30639 has about 8-11% higher traction and brake gains than 30618. However, oracle per-run gains barely help open-loop (30 s MAE 0.569 to 0.566).

**Wheel scale (outside my area but critical).** The wheel speed scale varies per run by 1-2%: 3.587 up to 3.62-3.66 km/h per m/s on 30639, and 3.54-3.55 on status-0 runs of 30618. This dominates long-horizon distance drift more than model gains do.

**Files not written.** No files were written because the session is READ-ONLY and in plan mode. All tables are given in recommended_parameters, and the fit script is in scripts; run it with OUT=<dir> to produce the JSON/CSV.
- [D1_traction_shape] (high, CRIT) Traction accel is a per-notch plateau at low speed, capped at a common maximum, and decays above about 6 m/s as (vb/v)^0.57. This is slower than constant power (1/v). Max accel per notch at low v is about 0.10+0.113*n, saturating at about 1.05 m/s^2 for n>=9-10.
    EVID: Hammerstein table a_level(n,v), for example n=+10: 1.08, 1.00, 0.99, 1.00, 0.88, 0.73, 0.65, 0.50, 0.45 at v=1,2,3,4,6,8,10,12,14. For n=15, a*v rises from 6.0 at 6 m/s to 8.4 at 12 m/s, so it is not constant power. Parametric fit a_tr = min(b0 + a0*n/(1+max(v-v1,0)/va), A0*min(1,vb/v)^pw) - 0.04 with b0=0.0968, a0=0.1131, v1=3.49, va=6.90, A0=1.048, vb=5.90, pw=0.571. Weighted cell RMSE 0.045 m/s^2, max 0.19.
    IMPL: Use a 2D lookup table (notch x speed, linear interpolation) or the 7-parameter parametric form as the propagation model. Do not assume a constant-force or constant-power traction curve.
- [D2_brake_linear] (high) Service braking is linear in |notch| and nearly speed-independent above about 4 m/s. There is a 20-33% fade toward v→0.
    EVID: Table rows: -1 about -0.30, -2 about -0.44, -3 about -0.55, -4 about -0.65, -5 about -0.74, -6 about -0.83, -7 about -0.95 m/s^2, roughly flat over v=2-14. Fit a_br = -(d0 + d1|n|)*(1 - f*exp(-v/vf)) - 0.04. For -1..-7 alone: d0=0.184, d1=0.1157, f=0.269, vf=3.62, wRMSE 0.033. For all brake notches except -8: d0=0.193, d1=0.1123, f=0.327, vf=2.98, wRMSE 0.075. Notches -9..-15 are well supported only near v≈1 (-0.97, -1.10, -1.41, -1.18, -1.35, -1.61); in the raw table they are -1.1 to -1.66 at 4-10 m/s.
    IMPL: Model brake as a linear-in-|n| gain times a low-speed fade factor. Only one brake gain needs online adaptation. For -9..-15 at speed, extrapolate linearly and cap at about -1.7.
- [D3_transient_lag] (high) Notch changes act through a short dead time plus a first-order lag. The optimum is traction d=0.05-0.10 s, tau=0.25 s, and brake d=0.10-0.15 s, tau=0.30-0.35 s. Brake engagement from coast (on/off) is slower, with d≈0.2 s and tau≈0.33 s. There is about 5-10% overshoot.
    EVID: 20 Hz FIR step-response identification. Traction per-notch step: d=0.02 s, tau=0.24-0.26 s. Brake per-notch: d=0.04-0.06 s, tau=0.17-0.24 s. Brake on/off indicator: d=0.17-0.21 s, tau=0.32-0.34 s, DC -0.22 m/s^2. Lag grid on train/test: static test RMSE 0.2048 vs 0.1826 with lags (-11%). The result is flat around the optimum (tau_tr 0.15→0.1836, 0.40→0.1835). Open-loop speed and distance error is almost unchanged with or without the lag (10 s MAE 0.384 static vs 0.416 lag in an early run).
    IMPL: Implement the lag, because it helps innovation gating and short-term accel prediction, but it is not decisive for distance drift. Driver notch changes happen about 1.5 times per second in ±1 steps, so the lag mostly smooths the command.
- [D4_grade_essential] (high, CRIT) Track grade from pathgraph z is essential. The fitted grade coefficient is close to the physical value (-g*grade): traction -0.886, brake -0.812, coast -0.978 (free Davis fit 0.874).
    EVID: Map z equals GNSS alt minus 3.10 m (std 0.08 on RTK), so the z coordinate is consistent. Raw grade is up to ±4%. The smoothing window is flat for 11-81 m (RMSE 0.1635-0.1633); 1 m gives 0.1662, so use about 31 m. Open-loop without grade: 10 s MAE 0.77 m/s and distance MAE 4.2 m; 30 s MAE 1.50 m/s and distance 27 m. With grade these are 0.36, 2.0 m, 0.58 and 10.7 m.
    IMPL: The model needs the map position (arc length s) and direction of travel to look up grade(s). Grade should be signed by travel direction and smoothed over about 30 m. Coupling odometry position to grade lookup is fine because grade varies slowly.
- [D5_resistance_curve] (medium) Coasting running resistance is small and almost constant, -0.03 to -0.05 m/s^2 for v=4-16 m/s, with no clear v^2 term. Curve resistance is measurable at about kc*|curv| with kc≈2.2-4.4 m^2/s^2.
    EVID: Robust soft_l1 Davis fit on steady coast (age>=3 s, v>2, N=11361): a0=0.0069, a1=0.0077, a2=-0.00043, kc=4.40, residual MAD 0.014. Coast accel by |curv| bin: <0.002: -0.055; 0.002-0.005: -0.066; 0.005-0.01: -0.074; 0.01-0.02: -0.084; 0.02-0.05: -0.119. Hammerstein curv coefficient -2.23. Coast R(v) knots at v=1..16: -0.20, -0.11, -0.08, -0.06, -0.01, -0.06, -0.02, -0.05, -0.02, -0.08. The v<2 values are contaminated by stop and brake transitions.
    IMPL: Use a constant coast resistance of about -0.04 m/s^2 plus -3*|curv|. A Davis polynomial is not justified. The curve term only matters in the few sharp curves (|curv|>0.01 on 1.4% of the track).
- [D6_notch_minus8_closed_loop] (high, CRIT) Notch -8 is not a fixed brake level. It behaves as an automatic speed-hold or closed-loop mode, so the model cannot predict acceleration during -8 at speed.
    EVID: In steady -8 (age>2 s, v>1), a = -0.218 - 0.023*g*grade with corr 0.00 to grade; other notches have coefficients of -0.38 to -0.91. Long -8 intervals (>5 s, N=24) are entered from -9 in 23/24 cases and exited to -9 in 23/24. Exit speed is 1.70-2.00 m/s (median 1.86). Duration p10/50/90 = 10.8/17.2/39.6 s. Speed can rise during -8 (e3d94878: 94 s, v 8.7→10→4.8→6.5→2.1). Table row at v=1..10: -1.35, -0.86, -0.85, -0.30, -0.16, -0.10, +0.03. Residual std 0.61 (robust 0.35). Excluding windows with -8 at v>2.5 improves 10 s MAE from 0.36 to 0.28 (4.5%→3.4% distance).
    IMPL: Treat -8 as its own mode. Either (a) use a constant-speed hold with large process noise (sigma_a about 0.6), or (b) model it as target-speed tracking until v<2 m/s, then as a brake about -0.85. Inflate filter covariance and trust wheels while in -8.
- [D7_external_brake_notch0] (high, CRIT) Strong braking occurs that the notch does not show: the notch reads 0 or +1/+2 while the tram decelerates at 0.7-2.5 m/s^2 to near-stop. This happens at a recurring location, and the wheels agree with GNSS, so it is not slide.
    EVID: 13 events, 58 s total, a_level -0.66 to -2.5 m/s^2, from up to 11.6 m/s down to about 1 m/s. 2.4% of steady-coast samples have a<-0.6. The 5 events examined are all at x≈101250-101417, y≈85360 (map frame). The pattern is a brake sweep to -7, then 0 or +1/+2 during about 1.2 m/s^2 decel. There is no cmd message gap (max dt 0.05 s), and vF=vR=vG.
    IMPL: The notch-driven model has unmodelled brake inputs (ATP/AEB/track brake). Fault and slide detection must not flag wheels just because they disagree with the model. Require front/rear wheel inconsistency or a physically impossible jerk or accel. In GNSS-denied bridging, keep wheels as the primary source and the model as a secondary check. Consider a location-based prior for this spot.
- [D8_standstill_start] (high, CRIT) There is a start delay and a stop behaviour that must be handled explicitly. After the first traction notch from rest, motion (v>0.1 m/s) starts after a median 1.25 s (p10 1.15, p90 1.35, N=958). At standstill the notch is 0 in 947/960 stops. The final stop happens at notch -15/-14 with a median decel of -0.96 m/s^2 in the last second.
    EVID: Open-loop simulation with hold logic (keep v=0 if v<0.05 and (n<=0 or traction held <1.2 s); clamp v>=0) vs without it: 30 s bias +0.14 vs +0.30 m/s, MAE 0.58 vs 0.66.
    IMPL: Implement a standstill latch: v=0 until traction has been held for 1.2 s, then release. Clamp v>=0 so braking never produces negative speed. This fixes positive speed bias at stops and false creeping.
- [D9_vehicle_run_gains] (medium) 30639 has about 8-11% higher effective traction and brake gains than 30618. Per-run gains vary ±5-8%, and traction and brake gains are correlated (r=0.64). Online gain adaptation gives only marginal open-loop benefit.
    EVID: Per-run regression. 30618 (N=51): Gtr 0.985±0.047 [0.887, 1.151], Gbr 0.977±0.067 [0.860, 1.254]. 30639 (N=17): Gtr 1.076±0.078 [0.867, 1.200], Gbr 1.114±0.047 [1.035, 1.214]. One-step RMSE 0.157→0.154 (30618) and 0.143→0.130 (30639). Oracle per-run gains in open-loop: 30 s MAE 0.566 vs 0.569. At 10 s, 30618 MAE is 0.318 (4.1%) and 30639 is 0.477 (5.9%). Part of the 30639 excess may come from its different wheel scale (D11), since the target accel is wheel-derived.
    IMPL: Use one scalar gain per mode (Gtr, Gbr), initialised at 1.0 (or 1.08/1.11 if the vehicle ID is 30639). Adapt them slowly (random walk sigma about 0.002 per s, bounded [0.8, 1.3]) while GNSS or wheels are healthy. Do not expect large gains from it.
- [D10_openloop_budget] (high, CRIT) Model-only dead reckoning from notch, speed-table and grade alone is usable for bridging about 5-10 s. Distance error is about 3-5% of distance travelled at 10 s and about 8% at 30 s, which is worse than wheel odometry unless the wheels are faulty.
    EVID: Train on even bags, test on odd bags; all test windows are initialised at the true speed. Results are speed MAE / RMSE / p90 in m/s, then distance MAE and relative error:
- 2 s: 0.114 / 0.314 / 0.228; 0.13 m (1.4%).
- 5 s: 0.232 / 0.584 / 0.478; 0.64 m (p90 1.24 m, 2.8%).
- 10 s: 0.356 / 0.809 / 0.829 (bias +0.034); 2.02 m (p90 4.29 m, 4.5%).
- 30 s: 0.581 / 1.199 / 1.535 (bias +0.125); 10.7 m (p90 27.7 m, 7.9%).
- 60 s: 0.688; 26.8 m (9.7%).
The parametric model is equivalent: 10 s MAE 0.362, 30 s MAE 0.563.
    IMPL: Architecture: the wheel speed from both bogies is the primary odometry. The model is a predictor or prior in a Kalman filter that detects slip or slide and bridges short wheel dropouts (<10 s) with inflated covariance. For longer outages, speed uncertainty grows about 0.02-0.04 m/s per s.
- [D11_wheel_scale_per_run] (high, CRIT) The wheel speed scale (km/h reported per true m/s) is not constant. It is 3.595-3.601 on most 30618 RTK runs, 3.586-3.591 on some 30639 runs, and 3.62-3.66 on other 30639 runs. Several 30618 runs with fix status 0 show 3.54-3.58. This is a 1-2% per-run variation, likely wheel wear or diameter config.
    EVID: Median ratio of wheel speed to GNSS speed, confirmed on RTK runs by distance ratios (wheel distance / position-path distance vs / vel-integrated distance): 0e41eac3 3.601/3.598; 927002c2 3.587/3.588; 92226df0 3.623/3.633; d601d28f 3.658/3.665; c31df386 3.618/3.613; 253671cc 3.611/3.622; 88548b02 3.580/3.575. Status-0 runs 0686195f (vel 3.548) and defd0170 (vel 3.542) cannot be verified by position.
    IMPL: This is outside the dynamics area but dominates long-range drift. A 2% scale error means 20 m per km. Estimate the wheel scale online as a filter state against GNSS or map landmarks, or at least from an initial GNSS segment; do not hard-code 3.6. Dynamics fitting should also normalise with the per-run scale.
- [D12_timing] (high) Use the ROS header stamps (t_hdr). With header stamps, wheel speed vs GNSS velocity lag is 0.00 s, and the notch command is effectively instantaneous (header minus receive time -0.9 ms).
    EVID: Cross-correlation at 0.01 s resolution on 4 bags, RMSE 0.024-0.04 m/s at zero lag. With receive time, wheels appear 0.02-0.07 s early. Header minus receive time: wheel -0.035 s, GNSS -0.088 s, cmd -0.0009 s. Wheel topics come at about 10 Hz with jittered dt 0.06-0.2 s and include duplicate samples, which must be deduplicated with dt>0.02.
    IMPL: Time-align on header stamps and deduplicate repeated wheel samples. No extra latency compensation is needed between wheels and GNSS.
- [D13_process_noise] (medium) The residual acceleration of the model against the 1 s smoothed GNSS acceleration depends strongly on mode. Coast residuals have heavy tails because of D7.
    EVID: Std (robust MAD*1.48) in m/s^2: traction 0.109 (0.064); coast 0.162 (0.044), p1 -0.66; brake -1..-7 0.085 (0.066); n=-8 0.608 (0.354); n<=-9 0.433 (0.128).
    IMPL: Use mode-dependent process noise Q_a (values in recommended_parameters). For coast, use a heavy-tailed or robust innovation (Student-t / Huber) instead of Gaussian inflation.
- [D14_slip_rare] (medium) Wheel slip or slide against GNSS is rare in this data: |dv|>0.3 m/s in 0.6-1.1% of moving samples, and |dv|>1.0 in 0.16-0.43%, by notch group.
    EVID: Comparison of the wheel mean against GNSS speed across 68 bags. The clean mask (|vF-vR|<0.15, |vW-vG|<0.3, |aF-aR|<0.3) keeps 96.2% of samples.
    IMPL: Front/rear bogie disagreement is the main slip detector. The model check is secondary. Few real slip events exist to validate against, so synthetic slip injection may be needed for scoring.
- [D15_dataset_duplicates] (high) 54 of the 122 bags are byte-identical duplicates of others (same wheel array md5), leaving 68 unique long bags with GNSS fix. Also, 20-30% of each run lies off the provided pathgraph (off-map, dist>6 m).
    EVID: md5 of the front_bogie v array. Example duplicate pairs: 2cb9ce37=5eb8d2c9, 748832b9=7849303f, 46e21b9b=117c2d02 and 22 more. The on-map fraction is 0.754.
    IMPL: Deduplicate before any train/test split to avoid leakage. Off-map segments (depot, other tracks) have no grade, so fall back to grade 0 with inflated noise there.
PARAMS:
  * lag_traction = dead_time 0.05 s, tau 0.25 s (first-order lag applied to per-notch indicator or accel command)  (Grid search of test RMSE on train/test split (best 0.1826). FIR step-response identification: d 0.02 s, tau 0.24-0.26 s.)
  * lag_brake = dead_time 0.15 s, tau 0.30 s; brake engagement from coast: d≈0.2 s, tau≈0.33 s  (Grid search, plus FIR on-off indicator (d 0.17-0.21 s, tau 0.32-0.34 s).)
  * accel_table_level_track = a_level(n,v) in m/s^2, level track, includes running resistance. Columns v=1,2,3,4,6,8,10,12,14 m/s; '-' means under 20 s support, so use the parametric form there.
+1: .19 .11 .06 .09 .17 .11 .14 .11 .08
+2: .37 .14 .25 .27 .31 .22 .20 .18 .15
+3: .48 .31 .33 .42 .34 .26 .27 .22 .18
+4: .58 .52 .44 .49 .47 .34 .31 .27 .22
+5: .68 .69 .59 .58 .43 .36 .36 .31 .25
+6: .79 .79 .65 .64 .58 .48 .45 .35 .28
+7: .95 .84 .81 .84 .74 .53 .51 .37 .33
+8: .99 .92 .88 .89 .81 .64 .52 .44 .39
+9: 1.03 .97 .92 .95 .85 .68 .58 .46 .43
+10: 1.08 1.00 .99 1.00 .88 .73 .65 .50 .45
+11: - 1.03 1.00 1.04 .94 .79 .68 .53 .46
+12: - - 1.05 1.07 .96 .83 .70 .58 -
+13: - - - 1.16 .96 .88 .73 .58 -
+14: - - - 1.20 1.00 .90 .77 .62 -
+15: - - - 1.08 1.00 .88 .76 .65 -
-1: -.25 -.31 -.30 -.29 -.34 -.28 -.29 -.32 -.32
-2: -.40 -.39 -.43 -.46 -.45 -.46 -.44 -.52 -.43
-3: -.50 -.49 -.53 -.51 -.60 -.62 -.60 -.57 -.55
-4: -.54 -.58 -.63 -.60 -.67 -.69 -.69 -.69 -.71
-5: -.69 -.69 -.74 -.69 -.75 -.79 -.77 -.76 -.72
-6: -.76 -.76 -.82 -.76 -.87 -.86 -.87 -.85 -
-7: -.79 -.89 -.92 -.91 -.99 -1.04 -.98 - -
-8: -1.35 -.86 -.85 -.30 -.16 -.10 +.03 (v=1..10; closed-loop, see D6)
-9..-14 at v≈1: -0.97, -1.10, -1.41, -1.18, -1.35, -1.61; at speed, use the parametric brake form  (Hammerstein linear LS over 68 unique bags at 20 Hz. Features are lagged one-hot notch times hat-basis speed functions, plus grade and |curv| regressors, ridge 1e-2. Mask: clean, v>0.3, on-map, and excluding the start-from-rest delay. In-sample RMSE 0.163.)
  * traction_parametric = a_tr(n,v) = min(b0 + a0*n/(1+max(v-v1,0)/va), A0*min(1, vb/v)^pw) - 0.04; b0=0.0968, a0=0.1131, v1=3.49 m/s, va=6.90 m/s, A0=1.048 m/s^2, vb=5.90 m/s, pw=0.571  (Weighted (by support) LS on table cells; wRMSE 0.045, max 0.19 m/s^2.)
  * brake_parametric = a_br(n,v) = -(d0 + d1*|n|)*(1 - f*exp(-v/vf)) - 0.04 for n in -1..-7 and -9..-15; d0=0.193, d1=0.1123, f=0.327, vf=2.98 m/s. Fit on -1..-7 only: d0=0.184, d1=0.1157, f=0.269, vf=3.62. Cap at -1.7 m/s^2.  (Weighted LS on table cells; wRMSE 0.075 (0.033 on -1..-7).)
  * notch_minus8_mode = If n==-8 and v>2.5: a = 0 (speed hold) with sigma_a 0.6; if v<=2.5 (or v<3): a = -0.85. Alternatively, a first-order speed tracking toward the -8 entry speed.  (Steady -8 statistics (D6). Exit speed median 1.86 m/s. Grade coefficient about 0.)
  * notch0_coast = a_coast = -0.04 - kc*|curv| - g*grade*0.98; kc ≈ 3 m^2/s^2 (range 2.2-4.4)  (Robust Davis fit (soft_l1) and Hammerstein coast knots. Resistance is constant at -0.03..-0.05 over 4-16 m/s.)
  * grade_coefficients = a += c_mode * 9.81 * grade(s)*dir, with c_tr=-0.886, c_br=-0.812, c_coast=-0.978. Using -1.0 for all loses little. Grade = dz/ds from pathgraph z, smoothed over 31 m, signed by travel direction.  (Hammerstein joint fit; window scan 11-81 m flat.)
  * curve_coefficient = -2.2 m^2/s^2 * |curv| (Hammerstein), up to -4.4 (coast-only Davis); use about -3  (Joint LS and coast bins by |curv|.)
  * standstill_logic = If v<0.05 and (n<=0 or traction held <1.2 s), then v=0 and a=0. Release after traction has been held 1.2 s (observed start delay median 1.25 s, p10 1.15, p90 1.35). Always clamp v>=0. Typical final stop: notch -14/-15, decel -0.96 m/s^2.  (958 starts and 960 stops. Open-loop 30 s bias +0.14 with the logic vs +0.30 without.)
  * per_vehicle_gain_priors = 30618: Gtr 0.985 (sd 0.047), Gbr 0.977 (sd 0.067); 30639: Gtr 1.076 (sd 0.078), Gbr 1.114 (sd 0.047); corr(Gtr,Gbr)=0.64  (Per-run regression of the accel residual on the traction and brake model components (68 runs).)
  * online_gain_adaptation = States Gtr, Gbr multiply the traction and brake parts. Init 1.0 (or the vehicle prior). Random walk sigma about 0.002/sqrt(s). Clamp to [0.8, 1.3]. Update only when wheels are consistent and |jerk| is small; freeze during n=-8 and unannounced-brake events.  (Engineering choice from the observed per-run spread. Oracle per-run gains give only marginal improvement, so adapt slowly.)
  * process_noise_sigma_a = Traction 0.11 (robust 0.064); coast 0.16 with a heavy tail (robust 0.044; use Huber/Student-t); brake -1..-7 0.085; n=-8 0.61; n<=-9 0.43 m/s^2  (Residual of the table model against 1 s Savitzky-Golay GNSS accel.)
  * wheel_scale = k = 3.597 km/h per m/s nominal (30618 RTK). Per-run values range 3.54-3.66 (30639 has 3.587-3.658). Estimate online with prior sd about 0.02 (0.6%).  (Median wheel speed divided by GNSS speed, and distance ratios on RTK runs.)
  * openloop_error_budget = Speed MAE at 2/5/10/30 s: 0.11/0.23/0.36/0.58 m/s. Distance MAE: 0.13/0.64/2.0/10.7 m (1.4/2.8/4.5/7.9% of distance travelled). p90 distance: 1.24 m at 5 s, 4.3 m at 10 s, 27.7 m at 30 s.  (Vectorized 20 Hz simulation. Train on even bags, test on odd bags, all windows initialised at the true speed.)
  * signal_conventions = Use header stamps; deduplicate wheel samples with dt>0.02 s. Speed = mean of front and rear bogie km/h divided by k. GNSS vel is in ENU (vx=East, vy=North). Map frame = UTM37N - (300000, 6100000). Map z = GNSS alt - 3.10 m.  (Frame and timing checks (D12).)
OPEN Q: The requested JSON/CSV files in scratchpad/analysis/dynamics/ were NOT written, because this session is READ-ONLY and in plan mode. The script in `scripts` regenerates notch_accel_table.json and .csv when run with OUT=<dir> from the scratchpad directory (it was test-run in print-only mode and reproduces the table and coefficients). | What is notch -8 physically: cruise or speed-hold (ATO), or a driver 'hold' position? Can the organizers confirm? The current model treats it as a speed hold with high noise. | What is the source of the strong braking at notch 0 near x≈101250-101417, y≈85360: ATP, AEB, track magnetic brake, or a signal stop? Is it logged on another topic (brake pressure, safety state)? If yes, add it as a model input. | Notches -9..-15 at speeds above 2 m/s are almost unobserved in a steady state; they are mostly transient at stops. The brake extrapolation there (-1.2 to -1.7 m/s^2) is uncertain, and emergency brake behaviour is unknown. | Is the 30639 gain excess (+8-11%) a real vehicle difference (mass or load, motor config), or a partial artifact of its different per-run wheel scale? Refit with per-run wheel scale normalisation to separate the two. | Will the evaluation or test data provide GNSS at the start of each run? That would allow calibrating the wheel scale and gains before the outage. If not, the 1-2% wheel scale uncertainty dominates distance error. | Is passenger load or vehicle mass available? The per-run gain spread (±5-8%) is consistent with load variation, but this is not verifiable without it. | The coast resistance at v<2 m/s (knot value +0.23 at v=0) is an artifact of stop and start transitions. How does the model behave in creep at 0-1 m/s? This matters for platform-stop odometry precision.
SCRIPTS: # fit_dynamics.py -- run from the scratchpad dir with /opt/miniconda3/envs/ml/bin/python; set env OUT=<dir> to write notch_accel_table.json/.csv (otherwise print-only)
# Needs: cache/<bag>.pkl (topic -> dict of numpy arrays incl. t_hdr), geo.py (utm), pathgraph JSONs
import pickle, numpy as np, json, glob, os, hashlib
from scipy.spatial import cKDTree
from scipy.ndimage import uniform_filter1d
from scipy.signal import lfilter
G_=dict(); exec(open('geo.py').read(),G_); utm=G_['utm']   # keep geo globals isolated (geo uses global f=flattening)
FT='/vehicle/front_bogie_velocity'; RT='/vehicle/rear_bogie_velocity'; CT='/vehicle/driver_position_cmd'
MAPD='/Users/egor/Documents/sideprojects/приколы/ХакатонМосТранспорт/'
PTS=[];GR=[];TG=[];CV=[]
for fn in ['таллинская - щукинская.json','щукинская - таллинская.json']:
    j=json.load(open(MAPD+fn)); p=np.array([[q['x'],q['y'],q['z'],q['curv']] for q in j['points']])
    PTS.append(p[:,:2]); GR.append(uniform_filter1d(np.gradient(p[:,2]),31)); CV.append(uniform_filter1d(p[:,3],5))
    tg=np.gradient(p[:,:2],axis=0); TG.append(tg/np.linalg.norm(tg,axis=1,keepdims=True))
allxy=np.vstack(PTS); allgr=np.r_[GR[0],GR[1]]; alltg=np.vstack(TG); allcv=np.r_[CV[0],CV[1]]; tree=cKDTree(allxy)
DT=0.05; KSCALE=3.597
def build(b):
    d=pickle.load(open(f'cache/{b}.pkl','rb')); fr=d[FT]; rr_=d[RT]; c=d[CT]; mv=d['/sensing/gnss/master/vel']; mf=d['/sensing/gnss/master/fix']
    T0=max(fr['t_hdr'][0],rr_['t_hdr'][0],c['t_hdr'][0],mv['t_hdr'][0])+1; T1=min(fr['t_hdr'][-1],rr_['t_hdr'][-1],c['t_hdr'][-1],mv['t_hdr'][-1])-1
    t=np.arange(T0,T1,DT)
    vG=np.interp(t,mv['t_hdr'],np.hypot(mv['vx'],mv['vy']))
    ff=np.r_[True,np.diff(fr['t_hdr'])>0.02]; rr=np.r_[True,np.diff(rr_['t_hdr'])>0.02]
    vF=np.interp(t,fr['t_hdr'][ff],fr['v'][ff])/KSCALE; vR=np.interp(t,rr_['t_hdr'][rr],rr_['v'][rr])/KSCALE
    aF=np.interp(t,fr['t_hdr'][ff],np.gradient(fr['v'][ff]/KSCALE,fr['t_hdr'][ff])); aR=np.interp(t,rr_['t_hdr'][rr],np.gradient(rr_['v'][rr]/KSCALE,rr_['t_hdr'][rr]))
    a=uniform_filter1d(0.5*(aF+aR),3)
    n=c['pos'][np.clip(np.searchsorted(c['t_hdr'],t,'right')-1,0,None)].astype(int)
    E,N=utm(mf['lat'],mf['lon']); x=np.interp(t,mf['t_hdr'],E-300000); y=np.interp(t,mf['t_hdr'],N-6100000)
    vx=np.interp(t,mv['t_hdr'],mv['vx']); vy=np.interp(t,mv['t_hdr'],mv['vy'])
    dist,k=tree.query(np.c_[x,y]); dot=alltg[k,0]*vx+alltg[k,1]*vy
    sgn=np.where(np.abs(dot)>0.5,np.sign(dot),np.nan); idx=np.where(~np.isnan(sgn),np.arange(len(sgn)),0); np.maximum.accumulate(idx,out=idx); sgn=sgn[idx]; sgn[np.isnan(sgn)]=1
    on=dist<6; grade=np.where(on,allgr[k]*sgn,np.nan); curv=np.where(on,np.abs(allcv[k]),np.nan)
    clean=(np.abs(vF-vR)<0.15)&(np.abs(0.5*(vF+vR)-vG)<0.3)&(np.abs(aF-aR)<0.3)
    pos_=(n>0); last0=np.maximum.accumulate(np.where(~pos_,np.arange(len(n)),-1)); trun=(np.arange(len(n))-last0)*DT*pos_
    return dict(v=0.5*(vF+vR),a=a,n=n,grade=grade,curv=curv,clean=clean,trun=trun)
seen=set(); sel=[]
for b in sorted(os.path.basename(p)[:-4] for p in glob.glob('cache/*.pkl')):
    d=pickle.load(open(f'cache/{b}.pkl','rb')); fv=d.get(FT,{}).get('v',np.array([])); h=hashlib.md5(fv.tobytes()).hexdigest()
    if h in seen: continue
    seen.add(h)
    if len(fv)>=3000 and len(d.get('/sensing/gnss/master/fix',{}).get('status',[]))>0: sel.append(b)
D={b:build(b) for b in sel}
KN=np.array([0,1,2,3,4,6,8,10,12,14,16.]); NOT=[k for k in range(-15,16) if k!=0]; nk=len(KN)
def hats(v):
    v=np.clip(v,KN[0],KN[-1]); j=np.clip(np.searchsorted(KN,v,'right')-1,0,nk-2); w=(v-KN[j])/(KN[j+1]-KN[j])
    H=np.zeros((len(v),nk)); H[np.arange(len(v)),j]=1-w; H[np.arange(len(v)),j+1]=w; return H
def fo(u,d,tau):
    D_=int(round(d/DT)); u=np.r_[np.full(D_,u[0]),u[:len(u)-D_]] if D_>0 else u
    al=1-np.exp(-DT/tau); return lfilter([al],[1,-(1-al)],u,zi=[u[0]*(1-al)])[0]
PT=(0.05,0.25); PB=(0.15,0.30)
P=nk*(1+len(NOT))+4; A=np.zeros((P,P)); B=np.zeros(P); W=np.zeros(P)
for b in sel:
    x=D[b]; H=hats(x['v']); Z=np.stack([fo((x['n']==k).astype(float),*(PT if k>0 else PB)) for k in NOT],1)
    ztr=Z[:,np.array(NOT)>0].sum(1); zbr=Z[:,np.array(NOT)<0].sum(1); gg=9.81*np.nan_to_num(x['grade'])
    X=np.hstack([H]+[Z[:,i:i+1]*H for i in range(len(NOT))]+[(gg*ztr)[:,None],(gg*zbr)[:,None],(gg*(1-ztr-zbr))[:,None],np.nan_to_num(x['curv'])[:,None]])
    m=x['clean']&(x['v']>0.3)&~np.isnan(x['grade'])&~((x['n']>0)&(x['trun']<1.3)&(x['v']<0.5)); m[1::2]=False
    A+=X[m].T@X[m]; B+=X[m].T@x['a'][m]; W+=np.abs(X[m]).sum(0)*2*DT
cf=np.linalg.solve(A+1e-2*np.eye(P),B)
R=cf[:nk]; TAB={k:(R+cf[nk*(1+i):nk*(2+i)]) for i,k in enumerate(NOT)}; SUP={k:W[nk*(1+i):nk*(2+i)] for i,k in enumerate(NOT)}
out={'units':'m/s^2 level-track accel a(notch,v) incl. running resistance; v in m/s','knots_v':KN.tolist(),'lag':{'traction':{'dead_s':PT[0],'tau_s':PT[1]},'brake':{'dead_s':PB[0],'tau_s':PB[1]}},
     'coast_R':R.tolist(),'table':{str(k):[None if SUP[k][j]<20 else round(float(TAB[k][j]),3) for j in range(nk)] for k in NOT},'support_s':{str(k):np.round(SUP[k],1).tolist() for k in NOT},
     'grade_coef':{'traction':float(cf[-4]),'brake':float(cf[-3]),'coast':float(cf[-2])},'curv_coef':float(cf[-1]),'bags':sel}
print('fit ok, bags',len(sel),'grade coefs',np.round(cf[-4:-1],3),'curv',round(cf[-1],2))
o=os.environ.get('OUT')
if o:
    os.makedirs(o,exist_ok=True); json.dump(out,open(os.path.join(o,'notch_accel_table.json'),'w'),indent=1)
    with open(os.path.join(o,'notch_accel_table.csv'),'w') as fh:
        fh.write('notch,'+','.join('v%g'%v for v in KN)+'\n')
        for k in NOT: fh.write('%d,'%k+','.join('' if SUP[k][j]<20 else '%.3f'%TAB[k][j] for j in range(nk))+'\n') # openloop_step.py -- reference propagation step (20 Hz) used for the open-loop evaluation
# state: v [m/s], zt (lagged traction level), zb (lagged brake level), hold_t (s traction held while stopped)
import numpy as np
DT=0.05; G=9.81
def a_tr(n,v):
    b0,a0,v1,va,A0,vb,pw=0.0968,0.1131,3.49,6.90,1.048,5.90,0.571
    return min(b0+a0*n/(1+max(v-v1,0)/va), A0*min(1.0,vb/max(v,1e-3))**pw)-0.04
def a_br(n,v):
    if n==-8: return 0.0 if v>2.5 else -0.85          # closed-loop hold mode (D6)
    d0,d1,f,vf=0.193,0.1123,0.327,2.98
    return max(-(d0+d1*abs(n))*(1-f*np.exp(-v/vf)),-1.7)-0.04
def step(v,n_delayed_tr,n_delayed_br,zt,zb,hold_t,grade,curv,Gtr=1.0,Gbr=1.0):
    # n_delayed_*: notch delayed by 0.05 s (traction) / 0.15 s (brake)
    ut=Gtr*a_tr(n_delayed_tr,v) if n_delayed_tr>0 else 0.0
    ub=Gbr*a_br(n_delayed_br,v) if n_delayed_br<0 else 0.0
    zt+= (1-np.exp(-DT/0.25))*(ut-zt); zb+=(1-np.exp(-DT/0.30))*(ub-zb)
    mode_c = -0.886 if n_delayed_tr>0 else (-0.812 if n_delayed_br<0 else -0.978)
    coast = -0.04 if (n_delayed_tr<=0 and n_delayed_br>=0) else 0.0
    a = zt+zb+coast + mode_c*G*grade - 3.0*abs(curv)
    hold_t = hold_t+DT if (n_delayed_tr>0 and v<0.05) else 0.0
    if v<0.05 and (n_delayed_tr<=0 or hold_t<1.2): return 0.0,zt,zb,hold_t,0.0   # standstill latch
    v=max(v+a*DT,0.0)
    return v,zt,zb,hold_t,a
VERDICTS:
  ~ [D1_traction_shape] confirmed: Confirmed, with caveats. The steady traction table is (a) about flat below about 4 m/s for n>=7 and (b) softly capped near 1.0 m/s^2 (0.96-1.15 for n=10-15 at 4 m/s). Above about 6 m/s it decays roughly as v^-0.6 (log-log slope over 6-12 m/s is 0.58-0.61 for n=15 and 0.65-0.70 for n=13/14), which is clearly not constant power. For n=15, a*v rises 5.8 -> 6.8 -> 7.3 -> 7.55 at v=6/8/10/12 (claimed 6.0 -> 8.4). Best notch->accel lag is a pure delay of about 0.3 s. Caveats: (1) The parametric formula overestimates low notches (n=1-6) at v<=4 by about 0.1 m/s^2. (2) Low-speed cells for n<15 come mostly from transients, because notches sweep with a median dwell of 0.05-0.5 s; only n=0 and n=15 have long dwells (p50 0.5 s and 2.05 s). (3) The table is vehicle-dependent. Vehicle 30639 traction cells are on average +0.04 m/s^2 higher (RMS diff 0.069; up to +0.10 to +0.18 for n=8-9 at 2-6 m/s), and its brake cells are 0.067 stronger. Calibrate per vehicle, or add a per-run gain estimated in the GNSS window.
      EVID: My own Hammerstein fit: hat basis over v nodes 0..16 x 31 notches, 3 regime grade terms, target dv over 1 s from GNSS master vel. Data: 69 deduplicated GNSS bags, 2-fold by bag. Lag grid: delay 0.3/tau 0 gives test RMSE 0.1708 (R2 0.911); delay 0/tau 0.3 gives 0.1711; no lag gives 0.1839; delay 0.9/tau 1.0 gives 0.3087. My n=10 row at v=1,2,3,4,6,8,10,12,14: 1.06, 0.96, 0.95, 0.98, 0.85, 0.72, 0.64, 0.47, 0.45 (analyst: 1.08, 1.00, 0.99, 1.00, 0.88, 0.73, 0.65, 0.50, 0.45). My n=15 row at v=4..12: 1.05, 0.96, 0.85, 0.73, 0.63. Analyst formula vs my table over 110 traction cells with weight>100: weighted RMS 0.055, mean (table - formula) -0.023, max abs 0.197 (claimed 0.045/0.19). Low-v rows, table vs formula at v=2-4: n=1 0.04-0.12 vs 0.17; n=2 0.09-0.25 vs 0.28; n=5 0.53-0.69 vs 0.62. Per-vehicle fit (52 vs 17 bags): traction diff mean +0.042, RMS 0.069 over 30 cells; brake diff -0.067, RMS 0.078 over 29 cells. Wheel units confirmed as km/h: wheel/3.6 vs GNSS speed RMS 0.024 m/s at zero lag in clean bags; per-bag ratio median 0.9991. All code ran inline via python stdin. No script files were written because the session was in read-only plan mode; only an empty analysis/verify_dynamics dir was created.
  ~ [D4_grade_essential] partially_confirmed: Map z is a valid grade source: alt - z = 3.10 m, and grade from GNSS altitude agrees with map grade (slope 1.007). Grade matters for model-only prediction and should be smoothed over 11-81 m (31 m is fine). The fitted coefficient is NOT consistently close to the physical value g*grade. It is 0.84-0.88 in traction, 0.68-0.85 in braking (0.32-0.52 for steady notches -3..-6, which suggests partly deceleration-regulated brakes) and 0.71-0.97 in coast. The coast value is unstable across folds and methods and is contaminated by hidden-brake events. The open-loop benefit of grade is about 1.4-1.7x, not 2-2.5x: 10 s MAE 0.56 -> 0.39 m/s and distance 3.05 -> 2.17 m; 30 s MAE 1.20 -> 0.68 m/s and distance 20.7 -> 12.4 m. Grade is irrelevant when wheel odometry is healthy.
      EVID: RTK master alt - map z, 59 bags (RTK status 2, within 0.8 m of the map): median offset 3.100 m (p10 3.088, p90 3.121), robust std 0.041, plain std 0.197 (RTK vertical outliers >0.5 m in 1.2% of samples, 31% in the p90 bag). GNSS-alt-derived 31 m grade vs map 31 m grade: N=183k, corr 0.912, RMS diff 0.0072, slope 1.007. The two tracks agree: z diff std 0.032 m and grade corr 0.999. Map grade p0.5/p99.5: -0.040/+0.036. Accel-fit test RMSE (2-fold): no grade 0.1892; W=1 m 0.1580; 11 m 0.1572; 31 m 0.1572; 81 m 0.1574; 201 m 0.1583. Coefficients (trac/coast/brake): fold1 0.863/0.972/0.802, fold2 0.842/0.758/0.778; vehicle 1 0.878/0.859/0.846, vehicle 2 0.825/0.857/0.677. Steady per-notch k: -1 -0.83, -2 -0.93, -3 -0.32, -4 -0.33, -5 -0.42, -6 -0.52, -7 -0.92, +1 -0.65. Steady coast Davis fit (N=8563): trimmed k=-0.71, OLS k=-0.19. Open loop (both folds, table refit without grade): 10 s 0.564 m/s, 3.05 m (6.7%); 30 s 1.202 m/s, 20.7 m (15.1%). With grade: 0.386, 2.17 m (4.8%); 0.682, 12.4 m (9.0%). Map-matching confounder: 98.5% of moving samples align with the map tangent. The rest were non-RTK fixes snapping to the parallel track 3.5 m away, which flips the grade sign. I used a per-bag majority map (39 T2S / 30 S2T).
  ~ [D6_notch_minus8_closed_loop] partially_confirmed: The statistics replicate, and -8 is indeed unpredictable from the notch. The 'speed-hold' interpretation is wrong. In long -8 intervals the tram performs whole stop-to-stop hops while the notch stream reads -8. It accelerates at up to about +1.2 m/s^2 from standstill to 6-12 m/s, then brakes at up to about -1.8 m/s^2. Entry follows a 0 -> -13..-15 -> -9 -> -8 sweep while the tram is already accelerating. Exit is at about 1.8 m/s, followed by a -9..-15 sweep to stop. Treat -8, and the preceding negative sweep that begins at standstill, as 'control unobserved': use wheel odometry, or inflate the model process noise to about ±1.5 m/s^2. Do not freeze v=0 at standstill with n<=0 in this mode.
      EVID: Steady -8 (age>2 s, v>1, N=4099): a = -0.254 + 0.023*g*grade, corr 0.004, residual std 0.620 (robust 0.546), 35% of samples with a>0. Other notches: residual std 0.047-0.062, grade k -0.32..-0.93. At v>2.5 (N=9977) a percentiles p5/25/50/75/95: -1.2/-0.8/-0.1/+0.08/+0.86. Long -8 (>5 s) in 69 GNSS bags: 43 intervals. Entered from -9 in 41 (from -7 in 2); exited to -9 in 40. Exit speed p10/50/90: 1.68/1.83/2.37. Duration 7.6/16.7/29.1 s. 38/43 rise by >0.5 m/s. Over 85 unique long bags (wheel speed): 58 intervals. 45/58 have both an accel phase (>+0.3; amax median 0.82, p90 1.17) and a brake phase (<-0.5; amin median -1.29). 25/58 start from standstill within 3 s before entry. In 45/58 speed gains >1 m/s in the 3 s before entry. Pre-entry minimum notch is -13..-15 in 53/58. vmax p10/50/90: 3.6/6.5/10.2. Example 30618_4d487b0d t=237-270: v 0 -> 11.9 -> 4.0 -> 6.1 -> 0 entirely at -8, with vF, vR and vG within 0.05 m/s. 30618_87afe526 t=537-566: 0 -> 10.8 -> 0 at -8. Exposure: 1307 s in -8 vs about 58,200 s moving (2.2%); 998 s at v>2.5; 41% of that off-map (di>5 m); 25/69 bags have >10 s. Excluding windows with -8 at v>2.5: 10 s MAE 0.386 -> 0.324, distance 4.8% -> 3.9% (claimed 0.36 -> 0.28, 4.5% -> 3.4%).
  ~ [D7_external_brake_notch0] partially_confirmed: Confirmed: strong deceleration (-0.7 to -2.6 m/s^2 mean, peaks to -4.3) occurs to near-stop (about 0.5-0.6 m/s) while the notch reads 0/+1/+2. It is not brake-release lag, not a cmd gap, and the wheels agree with GNSS (except one real slide). There is more than one location. There are about 14 on-map events at 3 recurring spots (both tracks): A x≈101260-101417, y≈85357-85364, which holds the long high-speed events (8-11.5 -> 0.6 m/s over 7.5-9.5 s, after a brake sweep to -7/-5); B x≈99800-99843, y≈84882-84890; C x≈103062-103096, y≈85526-85530, including a -4.3 m/s^2 emergency-level event. Another 5 are off-map near the Shchukinskaya terminus. A location prior (known stop zones) or reliance on wheels is needed in model-only mode.
      EVID: Detector: notch (0.3 s delayed) in {0,1,2}, aG<-0.6 for >=1 s, v>0.5. 69 GNSS bags: 20 events, 75.5 s. Of these, 5 are off-map (di 240-500 m) and 1 is a slide (30618_27e994fc, |vF-vG| 1.8 m/s), leaving 14 on-map, wheel-consistent events (claimed 13 / 58 s). Steady coast samples with a<-0.6: 500/20650 = 2.42% (claimed 2.4%). Brake-release control: 23 transitions (brake <0 for 1.5 s with min<=-3 -> 0 held >=4.5 s, v>3). Median a: -0.32 at 0 s, -0.11 at +0.5 s, +0.01 at +1 s, +0.02 at +2..4 s; only 2/23 stay < -0.6 at >=1.5 s. Cluster A events: e2dcf65f (11.48 -> 0.57, 9.5 s, amean -1.15, prior -7 at 1.43 s); 21dd3af3 (11.43 -> 0.57, 9.0 s, -1.21, -7); 0f120b35 (11.40 -> 0.52, 8.0 s, -1.37, -7); 88548b02 (9.72 -> 0.62, 7.5 s, -1.23, -5); b83d854d on S2T at (101417, 85357) (8.11 -> 0.60, 8.8 s, -0.86). Max |vF-vG| in these 0.06-0.11. cmd max dt 0.050-0.051 s in all events. Cluster B: 2dbce472, 88548b02, defd0170 (1.6-4.3 m/s, 1.7-2.0 s). Cluster C: 2366c74a, 927002c2, 616ec56b (amin -4.27, 6.03 -> 0.51 in 2.2 s, wheels within 0.16).
  ~ [D8_standstill_start] partially_confirmed: The start delay and stop pattern are confirmed. After the first traction notch from rest there is a dead time of about 1.0 s (GNSS) before motion, then a jerk-limited ramp: v≈0.1 m/s at 1.2 s, 0.2 at 1.5 s, 0.37 at 2 s, 0.95 at 3 s. The v>0.1 crossing is at median 1.26 s by wheel (1.18 s by GNSS). At standstill the notch is 0 in 936/941 stops. Stops end at -15/-14 in 95% of cases, with dv over the last 1 s of median -0.88 m/s. Caveats: (1) 3.6% of motion onsets (37/1027) have no positive notch in the prior 5 s (the -8-mode starts). A hard 'v=0 while n<=0' hold will pin the model at 0 through whole hops, so gate the hold on wheel speed, not only on the notch. (2) The benefit of the hold is mainly bias: in my simulation the 30 s bias is +0.107 vs +0.172 without the hold, but MAE is 0.682 vs 0.687 (claimed 0.58 vs 0.66).
      EVID: Motion onsets (vF crosses 0.1 m/s after >=2.5 s still), 69 GNSS bags: 1027; 990 with a positive notch in the prior 5 s. Delay from first traction notch: p10/50/90 1.13/1.26/1.38 s; <0.5 s 0.1%, >3 s 0.1%. Clean starts (traction held >=3 s after >=3 s still; N=1070, 840 with GNSS), median profile at t=0, 0.5, 0.8, 1.0, 1.1, 1.2, 1.3, 1.5, 2.0, 3.0 s. Wheel: 0, 0, 0, 0, 0.019, 0.103, 0.144, 0.207, 0.371, 0.945. GNSS: 0.005, 0.005, 0.006, 0.028, 0.068, 0.116, 0.154, 0.213, 0.377, 0.947. GNSS-based delay (vG>0.1): 1.06/1.18/1.28 s. Smallest nonzero wheel |v| is 0.043 m/s, so the threshold is not a sensor artefact. Stops (>=3 s, preceded by v>1): 941. Dominant standstill notch: 0 in 936, 3 in 3, 1 in 2; median share of standstill time at 0 is 0.877. Minimum notch in the last 1 s: -15 in 490, -14 in 407, -13 in 15. dv over the last 1 s: p10/50/90 -1.02/-0.88/-0.70 m/s. Open-loop sim with hold (v=0 if v<0.05 and (n<=0 or traction age<1.2 s)) vs clamp only, 30 s: bias +0.107 vs +0.172, MAE 0.682 vs 0.687; 60 s: bias +0.186 vs +0.280.
  ~ [D10_openloop_budget] confirmed: Confirmed: model-only dead reckoning (notch x speed table + 0.3 s delay + grade) is usable for about 5-10 s. Relative distance error is about 3% at 5 s, about 5% at 10 s, about 9% at 30 s and about 12% at 60 s. That is 8-25x worse than wheel odometry, which stays at 0.46-0.61% of distance at every horizon. Use the model only to bridge wheel faults, slip or slide, and as a consistency check. My 30/60 s errors are slightly larger than the analyst's (9.0% vs 7.9%, 12.0% vs 9.7%). One possible reason is duplicate-bag leakage in their split: there are 25 duplicate pairs, and several fall on different parities of the sorted 122-bag list.
      EVID: My own model and simulator: 69 deduplicated GNSS bags, 2 folds by bag, windows every 5 s, initialised at the true speed, 10 Hz, table + 0.3 s delay + regime grade terms, hold logic on. Results (both folds, N=15647 windows), given as speed MAE/RMSE/p90/bias in m/s, then distance MAE and relative error: 2 s 0.117/0.279/0.252/-0.001, 0.13 m (1.4%). 5 s 0.240/0.551/0.528, 0.66 m (p90 1.42 m, 2.9%). 10 s 0.386/0.853/0.917/+0.022, 2.17 m (p90 4.85 m, 4.8%). 30 s 0.682/1.494/1.787/+0.107, 12.4 m (p90 30.7 m, 9.0%). 60 s 0.868, 33.1 m (p90 80.8 m, 12.0%). Fold train-even/test-odd alone: 10 s 0.394, 2.21 m (4.7%); 30 s 0.718, 12.9 m (9.2%). Moving windows only (mean v>0.5): 10 s 4.7%, 30 s 8.9%. Wheel odometry ((vF+vR)/2) over the same windows: 0.055/0.133/0.254/0.684/1.27 m, i.e. 0.61/0.58/0.56/0.50/0.46%. Per-bag wheel/GNSS speed ratio: median 0.9991 (p10 0.9964, p90 1.0056), a residual scale bias of up to ±0.5% that the initial GNSS window can calibrate. Duplicate pairs that fall on different parities of the sorted 122-bag list: 2366c74a/93dc866e, 40ffd323/efb92709, 4d487b0d/b3042f78, 21dd3af3/f3b8c99b.
#################### anomalies
SUMMARY: Wheel and command stream anomaly analysis. Data: 122 bags, 25 exact-duplicate pairs, 97 unique bags, about 1.03M wheel samples at 10 Hz. The wheel speed topics (front/rear bogie) are in km/h. Divided by 3.6 they match GNSS RTK speed with zero lag and 0.031 m/s residual std, so wheels are near-truth. There are NO NaN/inf values, NO spikes (max deviation from a 5-sample median is 0.83 km/h), NO noise bursts, and practically no stuck values. The real anomalies are:
(1) Per-vehicle-day wheel scale differs by ±0.6–1.0% (30618 Sep-03 0.994, 30639 May-05 1.006–1.009). This is the dominant source of distance drift.
(2) 30639 single-bogie dropouts of 5.8–73.5 s in 8 bags. The dropped bogie keeps its last value of 0 at departure. Naive averaging then loses 1.8–5.3% of distance; the robust/staleness rule fixes it.
(3) 10 real slip/slide episodes of 1–3 s: single-bogie in 6 bags, common-mode in 3 (0.2 events/h). Robust bogie selection plus a rate limiter of +1.6/−2.4 m/s² cuts the worst per-event distance error from −7.65 m to −0.05 m and max speed error from 2.59 to 1.15 m/s.
(4) Header timestamp glitches of ±0.4–1 s in 5 unique bags, with interleaved/backward stamps.
(5) The command stream is clean (20 Hz, ±15, 97% ±1 steps), but in 2.5% of departures its sign is inconsistent with the wheel acceleration (cmd ≤ −9 while accelerating 0.6–1.3 m/s²). Cmd therefore must never override wheels.
(6) GNSS reference anomalies: ±1 s header offsets in 12 bags, master vel dropouts/errors up to 7–9 m/s, 19 unique bags with no GNSS, 8 status-0 only. These must be cleaned before calibration and evaluation.

A naive physics-model fallback on implausible jumps (follow the notch model until the measurement comes back in bounds) FAILED in testing: lock-out gave whole-bag RMSE 0.67–2.46 m/s. Use clip-toward-measurement rate limiting instead, never model hold. Unrelated note: several MCP data connectors (amplitude, atlassian, bigquery, definite, hex) require authorization via claude.ai connector settings or /mcp; they were not needed here.
- [A1_units_truth] (high, CRIT) Wheel velocity v (front and rear bogie) is in km/h. v/3.6 matches GNSS speed with zero lag, so wheel speed is essentially ground truth apart from a per-day scale.
    EVID: Lag scan over ±0.6 s: best lag 0.00 s in 58/60 RTK bags. Pooled residual (wheel/3.6 − hypot(vx,vy) of master vel, after GNSS time-glitch correction) over 631k samples: std 0.031 m/s, robust 0.017. Stops: 0.0097. Cruise (|a|<0.1, v>5): 0.026, p0.01/p99.99 −0.15/+0.14. Residual-vs-accel slope ≈ 0. Per-bag RMSE 0.026–0.035 m/s.
    IMPL: The predicted speed should be the (robust-fused) wheel speed/3.6 with a scale correction. Any learned or model component should only fill gaps. Do not smooth heavily: the noise is 0.012–0.03 m/s and smoothing adds lag.
- [A2_scale_by_day] (high, CRIT) The wheel/GNSS scale depends on vehicle and day by up to ±1% but is stable within a bag to ±0.3%. It dominates integrated distance error, far more than slip.
    EVID: Wheel/GNSS ratio: 30618 Jul-27 0.9989–1.0004; Aug-10 0.9988–0.9992; Aug-26 0.9987–0.9996; Sep-03 0.9938–0.9940 (RTK), 0.9845/0.9860 (non-RTK). 30639 May-05 1.0064–1.0092 RTK, 1.0048–1.0162 non-RTK; Aug-26 0.9961–0.9975. Front/rear ratio kR/kF 0.9989–1.0009 (median 0.9999). With fixed K=0.999 the per-bag distance error is −0.95% (27e994fc), −0.65% (88548b02), +0.97% (92226df0), +0.53% (253671cc), and +1.03% on 30639_3b3d9eb8 (non-RTK reference). Slip events contribute at most 2–8 m per event on 5–6 km bags (≤0.16%).
    IMPL: Use a per-vehicle scale (30618: 0.999, 30639: 0.997–1.0) as the default. If the evaluation integrates position, consider online scale adaptation from map-matched distance (known inter-stop lengths on the pathgraph). It cannot come from the first seconds of GNSS: in the first 10 s the median max speed is 0 and distance is at most 6 m. Treat scale uncertainty (±0.7%) as the main error budget.
- [A3_start_bias] (medium) In the first 1–40 s of many bags the wheel/GNSS ratio reads 1.0–1.7% low versus the bag median. Mid-run there is no gradual wet-rail creep.
    EVID: 20 s rolling (wheel/GNSS)/bag-median − 1 over 3300 windows: p0.1/1/5/50/95/99/99.9 = −5.48/−1.01/−0.31/0.012/0.143/0.244/0.906%. 37 windows exceed 1%: mostly in the first 1–41 s (01f73500, 0652866c, 0e41eac3, 28538acf, 49fe4c54, 9c09b081, ab5921a4, 50956d6e at −1.0 to −1.6%), otherwise at slip events (2050d396 −3%, 33bec73f +13.4%/+6%).
    IMPL: Do not calibrate the scale on bag starts. This is likely terminus-loop curvature with the antenna offset from the wheel path, i.e. a GNSS-reference artifact rather than a wheel fault. Wheel speed needs no special start-of-bag handling.
- [A4_rear_dropouts_30639] (high, CRIT) Vehicle 30639 has single-bogie message dropouts of 5.8–73.5 s. They always begin while stopped, at departure, with the dropped bogie's last value 0.0, while the other bogie keeps streaming.
    EVID: 3b3d9eb8 R 372.2 s/30.4 s and 417.2/73.5 (count F 11559 vs R 10542); 4285f2bc R 6.9/47.1; 44226bde R 4.8/16.7; 584b6e32 R 1085.0/16.3; 927002c2 R 244.4/20.5 and 916.4/8.7; 9f0b519f R 506.2/5.8; c31df386 F 9.8/19.8; d927f360 R 276.6/25.6 and 1059.2/12.1. Before its dropout, 3b3d9eb8 R is frozen at 4.81 km/h for 19 samples (369.9–371.7 s). Distance error with naive mean of last values: 3b3d9eb8 −287.7 m (−5.30%), d927f360 −121.9 m (−2.51%), 927002c2 −92.5 m (−1.80%), 584b6e32 −37.9 m (−0.80%). With staleness 0.35 s + stuck-zero rule + robust selection: +56.5 m (+1.04%, scale), −11.3 m (−0.23%), −17.4 m (−0.34%), −18.1 m (−0.38%). RMSE drops from 1.015/0.675/0.497/0.194 to 0.084/0.060/0.052/0.100 m/s.
    IMPL: Mandatory: per-bogie staleness gating (message age > 0.3–0.35 s means invalid) plus a stuck-zero rule (one bogie = 0 while the other > 0.5 m/s means ignore the zero). Never interpolate or hold the last value of a silent bogie. The hidden test set likely contains similar or injected single-sensor dropouts.
- [A5_single_bogie_slip] (high, CRIT) Real single-bogie wheelspin and slide episodes exist. In every case with GNSS, the bogie closer to the other bogie's trajectory and to physical acceleration limits was correct.
    EVID: 18 episodes with |F−R| > max(1.5 km/h, 8%) lasting at least 0.3 s. Clean mismatch |F−R| moving: p50 0.027, p99 0.228, p99.9 0.457 km/h. Relative at v > 10 km/h: p99 1.8%, p99.9 3.4%. Clean F−R std 0.0153 m/s (robust 0.0114). Real events: 2050d396 397.6/400.5 R slide (−4.28 m/s, 56%, 1.5–1.9 s, cmd −5..−9); 33bec73f 100–109 R spin 3× (+2.4..+3.0 m/s, up to +80%); 50956d6e 77.9 R spin +1.55, 94.8 F spin +3.22, 1025.8 F spin +0.86; 117c2d02 97 R slide (F 9.94 vs R 3.46 km/h, no GNSS); e4379d7f 1147.5 R slide −0.8. Per-event distance error, mean vs robust selection: 2050d396@396 −3.61 m → +0.07 m (max err 2.14 → 0.12); 50956d6e@76.5 1.09 → −0.26; @93.5 1.85 → −0.13; @1025 0.36 → −0.02.
    IMPL: Fusion: average the two bogies when |F−R| ≤ max(0.2 m/s, 3%). Otherwise pick the bogie closer to the previous fused estimate, or to the cmd-model prediction, which is equivalent in practice. min/max strategies each fail on half the cases (min is wrong on spin, max is wrong on slide).
- [A6_common_mode_slip] (high, CRIT) Common-mode slides/locks (both bogies wrong) are rare, short (1–3 s) and physically detectable by deceleration beyond the train's braking capability.
    EVID: After GNSS time correction, 24 episodes with |wheel − GNSS| > 0.3 m/s lasting at least 0.3 s. Real common-mode events: 2050d396@440.7 (F −3.19, R −2.06 m/s, 1.3 s, cmd −14); 68d1748a@991.5 (F −1.2, R −2.3, cmd −6); 50956d6e@1036.8 lock at cmd −15 (both 0 while GNSS 1.4–1.7 m/s for about 2 s, −3.2 m); 33bec73f@100.9 partial. That is 4 of about 60 RTK bags, ≈ 0.2 events/h. Slide decel −2.6 to −4.75 m/s² (−6.7 in 2050d396@440.8), spin +2.75 to +4.3 m/s² over 0.2 s, versus a clean-data p0.01 of −2.02 m/s² over 0.2 s. Rate limiter +1.6/−2.4 m/s² (clip toward measurement): 2050d396 max err 2.59 → 1.15 m/s, RMSE 0.090 → 0.045, dist −8.4 → −3.7 m; 33bec73f max 2.27 → 1.26, dist +3.6 → 0.0 m.
    IMPL: Add a clip-toward-measurement rate limiter on the fused speed, with bounds just outside the physical envelope. Do NOT switch to open-loop model hold when the measurement is implausible: tested variant (hold notch-model prediction while |dv/dt| out of bounds) locked out and gave whole-bag RMSE 0.67 (2050d396), 1.23 (33bec73f), 2.46 m/s (68d1748a). If a model fallback is used it needs forced re-acquisition after 1–2 s or once both bogies agree and are smooth for 0.5 s.
- [A7_physical_bounds] (high) The train's physical acceleration/jerk envelope is tight and cmd-dependent. Raw per-sample differences are jitter-dominated and must not be gated directly.
    EVID: Clean data, both bogies agree, 694k moving samples. a over 1 s, traction: p0.01/p1/p50/p99/p99.99 = −0.64/−0.19/0.47/1.04/1.23, max 1.55. Coast: −1.83/−0.62/−0.01/0.39/1.24, min −2.03. Brake: −1.88/−1.31/−0.60/0.41/1.32, min −2.09. a over 0.2 s, all phases: p0.01 −2.02, p99.99 1.45. Jerk over 1 s: traction p0.01/p99.99 −1.49/+1.65, brake −1.98/+1.30 m/s³. Single raw-step acceleration p0.001/p0.01/p99.99/p99.999 = −4.54/−2.71/2.19/2.83 (header dt jitter 0.038–0.314 s, median 0.102). Noise of F around a 2 s quadratic SG fit: std 0.0124 m/s (robust 0.0103, p99 0.038). Per-notch median a (m/s²): −1 −0.108, −2 −0.393, −3 −0.59, −5 −0.674, −7 −0.767, −9 −0.869, −12 −0.961, −14 −1.105, 0 −0.012, +1 0.055, +3 0.217, +5 0.442, +7 0.666, +9/+10 0.89, +12 0.825, +15 0.673. Cmd→accel lag ≈ 0–0.1 s (coarse).
    IMPL: Rate-limit with dt from header stamps (not a fixed 0.1 s). Use +1.6/−2.4 m/s² as hard bounds and ±2.5 m/s² over ≥0.5 s as a slip flag. The notch→accel table works as a KF process-model input only while wheels are unavailable (both bogies stale), with velocity clamped ≥ 0.
- [A8_no_injected_noise] (high) The provided data has no synthetic value corruption: no NaN/inf, no spikes, no noise bursts, no scale steps within a bag, and negative speeds are only tiny real rollbacks.
    EVID: NaN/inf count 0 in all bags. Residual vs 5-sample median at v > 5 km/h: p99.99 0.41, max 0.83 km/h, 0 samples > 1 km/h. Max 50-sample rolling RMS residual 0.17 km/h. Stuck: only 3b3d9eb8 R 19 samples at 4.81 km/h and 6cb3280a R 5 identical samples at 367.9 s. Negatives, both bogies agreeing: 3e012faf (=bcc9e7a2) 588.6–589.1 s down to −0.389 km/h; 30639_9c362687 91.8–92.0 s down to −0.264 km/h. Float32 with low-speed cutoff: min positive ≈ 0.15 km/h; near-zero step ≈ 0.0048 km/h.
    IMPL: Still add cheap defensive guards (non-finite → invalid, |v| > 100 km/h → invalid, a Hampel spike filter over 5 samples with a 1 km/h threshold, stuck detection: identical nonzero value for ≥ 1.5 s while the other bogie varies). They cost nothing on clean data and protect against hidden-test injections. Clamp output ≥ 0, or allow small negatives only if the scorer uses signed speed.
- [A9_input_timestamp_glitches] (high, CRIT) Wheel and cmd header stamps occasionally jump ±0.4–1 s, with interleaved or backward-going stamps. Sorting by header stamp restores a consistent curve, and header stamps carry real sampling time (better than record time).
    EVID: 30618_2255aade (=7bfbb5ed) 195.8–199 s: interleaved +1 s stamps on F, R, C, with 6 backward steps. 30618_40ffd323 (=efb92709): 84–86 s +1 s, 504–506.5 s −0.4 s. 30639_9c362687 1331.7 s: one message +1 s. 30618_28538acf 167 s +0.4 s. 30618_af7496f0 141.6 s −0.41 s. Accel-phase RMSE vs GNSS: 0.027 with header stamps, 0.030–0.043 with median-corrected record time, 1.5–4.8 with a uniform grid. Wheel dt_rec: 93.0% < 0.15 s, 6.9% 0.15–0.25, 0.06% 0.25–0.35, only 39 gaps > 0.35 s. Simultaneous both-bogie gaps of 0.5–1.3 s in 11 bags (e.g. 0652866c 1372.4, 27e994fc 1606.9, 40ffd323 82.9/506.5, 92226df0 571.6).
    IMPL: Per stream: detect a header-offset jump (|(t_rec − t_hdr) − running median| > 0.3 s) and replace the stamp with t_rec − median offset. Enforce a monotonic time per stream: drop, or buffer ~0.15 s and reorder, stamps going backwards. Compute dt from corrected stamps and clamp dt to [0.02, 0.5] s in the rate limiter.
- [A10_cmd_inconsistency] (high, CRIT) The driver command stream is well-formed, but in about 2.5% of departures and ~1% of moving time its sign contradicts the actual (wheel = GNSS) acceleration, at recurring map locations.
    EVID: 20 Hz, range −15..+15, max gap 0.15 s. Steps: ±1 in 145k of ~150k changes. Jumps ≥ 4 about 2 per bag (mostly traction→0). 42 single-sample returning glitches. 32 of 1282 departures accelerate at 0.6–1.3 m/s² with cmd −9..−15. Windows with notch ≤ −3 and a > 0.3 total ~737 s (1.0% of moving time), recurring at Tallinskaya terminus +1520..1840 m, Shchukinskaya terminus +2155..2185 m, and the terminus loops. Worst: 30618_4d487b0d 200–330 s, cmd at −8/−15 through several accel/decel cycles while wheels match GNSS. Direction forward (cos ≈ 1). Reverse case (cmd ≥ 5 and a < −0.5): only 2.4 s total. Notches −8 and −15 median a are contaminated (−0.221, −0.125).
    IMPL: Cmd must be a weak prior only. Gate the model when sign(cmd) disagrees with the wheel-derived acceleration over the last 1 s, and never let cmd veto consistent wheel readings. When training a notch→accel model, exclude these windows or fit robustly. For a pure-model (no wheels) mode, expect large errors at these locations.
- [A11_gnss_reference_hygiene] (high, CRIT) The GNSS reference itself has ±1 s header glitches and master-vel outages/errors that create fake wheel-vs-GNSS residuals of a×1 s and speed errors up to 9 m/s.
    EVID: Header offsets (master and rover, fix and vel together): 1cc230fa 122.4 s/2.7 s/0.48; 2366c74a 1041.6/22/0.93; 27e994fc 194/46/1.0; 28538acf 133.8/34/0.95; 40ffd323 86–146 −1.0 and 445–507 +1.0; 49fe4c54 122.9/6.3/0.56; 4d487b0d 141/20/0.9; 748832b9 1059/23/0.93; af7496f0 105.6/37.8/0.99; 30639_3b3d9eb8 580/243/−1.0; 9c362687 1311/21/0.96; spikes 0e41eac3 571.7 (2.3 s) and 7b4d83f4 56.2 (2.2 s). Master vel wrong: 4d487b0d 33 s gap at 64.4 then wrong 67–128 s up to 7.16 m/s; 2dbce472 153–161 s up to 4.5 m/s; 584b6e32 at 175/955/986 s; 50956d6e 310 s; 0e41eac3 568.3 s gap. Rover-vs-master RMSE 0.6–0.7 m/s in 49fe4c54, 4d487b0d. Residual max errors of 7.7–9.0 m/s in 68d1748a, 50956d6e, 584b6e32, 27e994fc come from these reference errors, not wheels.
    IMPL: Before calibrating scale or computing validation metrics: fix GNSS stamps (same median-offset rule), require RTK status, cross-check master vs rover (|Δv| < 0.3 m/s), and mask vel gaps > 0.5 s. Otherwise validation RMSE is dominated by reference artifacts and model selection will be wrong.
- [A12_duplicates_split] (high, CRIT) 25 exact-duplicate bag pairs (all 30618 Aug-10) inflate the set to 122; unique bags = 97 across 6 vehicle-days. Parallel 30618/30639 bags are different trams, not duplicates.
    EVID: Full per-topic hash identity: 0259fe53=cfd9fd5a, 0a83c933=e392e5bd, 117c2d02=46e21b9b, 1cc230fa=a395846d, 20096314=e7dcdab1, 2161b58b=f28179bb, 21dd3af3=f3b8c99b, 2255aade=7bfbb5ed, 2366c74a=93dc866e, 2cb9ce37=5eb8d2c9, 2d2fa7be=8eb8615c, 2f2f1175=ddad08d6, 3b36e5cd=ae4eb346, 3ba2326f=b1098bdb, 3e012faf=bcc9e7a2, 40ffd323=efb92709, 4d487b0d=b3042f78, 4e1e3181=95c49c30, 6236f680=7d88fa79, 6cb3280a=f3a0694d, 748832b9=7849303f, 79b204dc=887a2b9a, 7a152380=8f08df08, 81c22fee=92b1b453, 9bbe6faa=9c10cd0d. Time-overlapping 30618/30639 bags: GNSS 10–4700 m apart, wheel correlation ≈ 0. Unique bags per vehicle-day: 30618 Jul-27 29, Aug-10 32, Aug-26 13, Sep-03 4; 30639 May-05 10, Aug-26 9. No GNSS: 19 unique (0259fe53, 0a83c933, 117c2d02, 20096314, 2161b58b, 2255aade, 2d2fa7be, 2f2f1175, 3ba2326f, 3e012faf, 4e1e3181, 5036aa78, 6236f680, 74559c73, 79b204dc, 7a152380, 81c22fee, 9bbe6faa, e151d6e4). Status-0 only: 30618_0686195f, defd0170; 30639_3b3d9eb8, 44226bde, c31df386, d3c43d69, d601d28f, dce52be4. RTK < 50%: 30639_253671cc, 2b4a6347, 4285f2bc, 9c362687, e4379d7f (rover only). RTK 50–90%: 30618_2366c74a, 27e994fc, 2dbce472, 68d1748a, 88548b02; 30639_584b6e32, 92226df0.
    IMPL: Deduplicate by hash first. Use leave-one-vehicle-day-out CV, because scale is per-day and a random split leaks it. No-GNSS bags are usable only for self-consistency tests (F/R agreement, dropouts, timestamp handling), not for metrics. Evaluate on RTK-only, time-corrected, master/rover-consistent samples.
PARAMS:
  * wheel_units_conversion = v_mps = v_kmh / 3.6 / K  (Lag scan 0.00 s in 58/60 RTK bags; residual std 0.031 m/s over 631k samples.)
  * scale_K = 30618: 0.999 (Sep-03 0.994); 30639: 0.997 default (May-05 ~1.008); unknown vehicle: 0.999  (Median per-bag wheel/GNSS ratio per vehicle-day on RTK, time-corrected GNSS; within-bag stability ±0.3%.)
  * staleness_timeout = 0.35 s (range 0.30–0.35)  (Wheel dt_rec 99.94% < 0.25 s, 0.06% at 0.25–0.35 s, only 39 gaps > 0.35 s; header dt p95 0.13, max 0.314 s.)
  * stuck_zero_rule = ignore bogie if v == 0 while other bogie > 0.5 m/s (also if identical nonzero value >= 1.5 s while other varies > 0.3 m/s)  (All 30639 dropouts start with last value 0.0 at departure; 3b3d9eb8 R frozen 19 samples at 4.81 km/h. Removes −1.8..−5.3% distance errors.)
  * bogie_agreement_gate = |F−R| <= max(0.2 m/s, 3% of max(F,R)) -> average; else pick bogie closer to previous fused estimate  (Clean |F−R| p99.9 0.457 km/h (0.127 m/s); relative p99.9 3.4% at v > 10 km/h; clean std 0.0153 m/s (robust 0.0114).)
  * rate_limit_accel_bounds = +1.6 m/s^2 / −2.4 m/s^2, applied as clip toward measurement using header-stamp dt clamped to [0.02, 0.5] s  (Clean a over 1 s max +1.55, min −2.09; over 0.2 s p0.01 −2.02, p99.99 1.45. Validated: 2050d396 max err 2.59 -> 1.15 m/s, 33bec73f 2.27 -> 1.26; no degradation on clean bags.)
  * slip_flag_thresholds = spin: dv/dt > +2.5 m/s^2 over >= 0.2–0.5 s; slide: dv/dt < −2.5 m/s^2; or |v_bogie − v_fused| > 0.3–0.5 m/s  (Observed spin +2.75..+4.3, slide −2.6..−6.7 m/s^2 vs clean envelope p0.01 −2.02 / p99.99 1.45 over 0.2 s.)
  * slip_recovery_hysteresis = 0.5–1.0 s after bogies re-agree  (Recovery to GNSS speed after spin/slide takes 0.5–1 s in all 10 events.)
  * model_fallback_max_duration = <= 1–2 s open-loop, then forced re-acquisition to wheels  (Hold-model-on-implausible variant locked out: whole-bag RMSE 0.67/1.23/2.46 m/s on 2050d396/33bec73f/68d1748a vs 0.045–0.147 for clip rate limiter.)
  * measurement_noise_sigma = 0.012–0.02 m/s per bogie (process-level 0.03 m/s vs GNSS)  (SG-2s residual std 0.0124 m/s (robust 0.0103); wheel-vs-GNSS cruise std 0.026, stop std 0.0097.)
  * notch_accel_table = −14:-1.105, −12:-0.961, −9:-0.869, −7:-0.767, −5:-0.674, −3:-0.59, −2:-0.393, −1:-0.108, 0:-0.012, +1:0.055, +3:0.217, +5:0.442, +7:0.666, +9/+10:0.89, +12:0.825, +15:0.673 m/s^2 (interpolate the others; −8/−15 contaminated)  (Median a over 1 s per notch on clean data; use only when both bogies stale; clamp v >= 0.)
  * cmd_trust_gate = disable cmd-model if sign(cmd) disagrees with 1 s wheel accel (cmd <= −3 and a > 0.3, or cmd >= 5 and a < −0.5)  (32/1282 departures (2.5%) and ~737 s (1.0% of moving time) with notch <= −3 while a > 0.3.)
  * timestamp_glitch_fix = offset = t_rec − t_hdr; if |median_filter(offset, 11) − median(offset)| > 0.3 s then t = t_rec − median(offset); drop non-monotonic stamps or reorder buffer ~0.15 s  (Glitches of ±0.4–1.0 s in 2255aade, 40ffd323, 9c362687, 28538acf, af7496f0 (input) and 12 GNSS bags; header stamps give RMSE 0.027 vs 0.030–0.043 for t_rec.)
  * defensive_guards = non-finite -> invalid; |v| > 100 km/h -> invalid; Hampel(5 samples, 1 km/h) spike rejection; output clamp >= 0  (No such anomalies in the data (max median-residual 0.83 km/h), so zero cost on clean data; protects against hidden-test injections.)
  * cv_split = dedupe (97 unique) -> leave-one-vehicle-day-out; example holdout: 30618 Aug-26 (13) + 30618 Sep-03 (4) + 30639 May-05 (10); train 30618 Jul-27 (29) + Aug-10 (32) + 30639 Aug-26 (9)  (Scale varies by vehicle-day (0.994–1.009); duplicates all in 30618 Aug-10; holdout covers both vehicles and the extreme scales.)
  * robustness_test_bags = slip: 30618_2050d396, 30618_33bec73f, 30639_50956d6e, 30618_68d1748a, 30618_117c2d02 (no GNSS), 30639_e4379d7f; dropouts: 30639_3b3d9eb8, 4285f2bc, 927002c2, d927f360, c31df386 (front), 44226bde, 584b6e32, 9f0b519f; timestamps: 30618_2255aade, 30618_40ffd323, 30639_9c362687, 30618_28538acf, 30618_af7496f0; cmd inconsistency: 30618_4d487b0d, 30639_4285f2bc, 30618_e3d94878; negative speed: 30618_3e012faf; GNSS reference faults: 30618_4d487b0d, 2dbce472, 27e994fc, 30639_584b6e32  (Episode detection in sections a–c plus the fusion evaluation above.)
OPEN Q: Does the judge's reference speed come from GNSS vel (master or rover) or from differentiated RTK position, and are the ±1 s GNSS header glitches cleaned in the reference? Uncleaned glitches add apparent errors of a×1 s (up to ~1 m/s) that no wheel-based method can match. | Does the hidden test set inject synthetic spikes, noise, stuck values, scale steps or dropouts that are absent here (0 NaN, 0 spikes > 1 km/h)? If so, the defensive guards and the slip/rate-limit thresholds become score-critical. | Is the metric speed-only, or integrated distance/position? If position, the ±0.6–1% per-day scale dominates; consider online scale estimation from map-matched stop-to-stop distances on the pathgraph. | What causes the cmd-sign inconsistency (cmd −9..−15 while accelerating) at recurring locations near the termini: a different control mode, a signal mapping issue, or a deliberate injection? | Why is the wheel/GNSS ratio 1.0–1.7% low in the first 1–41 s of many bags: terminus-loop geometry with the antenna offset, or a real wheel effect? | Will test bags be from the same vehicles (30618/30639) and days, so per-vehicle-day scale priors can be used, or from unseen vehicles (need K ≈ 0.999 default)? | Should the output allow small negative speeds (real rollbacks down to −0.39 km/h exist) or be clamped at 0? | Unrelated to the analysis: MCP connectors amplitude, amplitude-eu, atlassian, bigquery, definite and hex need authorization via claude.ai connector settings or /mcp before they can be used.
SCRIPTS: # Causal robust fusion validated on 2050d396, 33bec73f, 30639 dropout bags. Pseudocode-level Python; inputs sorted by corrected header time.
def fixt(x):
    off = x['t_rec'] - x['t_hdr']; med = np.median(off)
    rm = median_filter(off, size=11, mode='nearest')
    t = np.where(np.abs(rm - med) > 0.3, x['t_rec'] - med, x['t_hdr'])
    o = np.argsort(t, kind='stable'); return t[o], o

def fuse_step(vp, f, age_f, r, age_r, dt, AMAX=1.6, AMIN=-2.4, STALE=0.35):
    fo, ro = age_f < STALE, age_r < STALE
    if fo and ro:
        if f == 0 and r > 0.5: fo = False
        if r == 0 and f > 0.5: ro = False
    c = [x for x, ok in ((f, fo), (r, ro)) if ok]
    if not c: m = vp  # or notch-model prediction for <= 1-2 s, then hold
    elif len(c) == 2 and abs(c[0]-c[1]) <= max(0.2, 0.03*max(c)): m = 0.5*(c[0]+c[1])
    elif len(c) == 2: m = min(c, key=lambda x: abs(x - vp))
    else: m = c[0]
    dt = min(max(dt, 0.02), 0.5)
    m = min(max(m, vp + AMIN*dt), vp + AMAX*dt)  # clip toward measurement, never open-loop hold
    return max(m, 0.0)
# f, r in m/s = v_kmh/3.6/K ; initialize vp from the first valid measurement (not 0)
VERDICTS:
  ~ [A1_units_truth] partially_confirmed: Wheel v is in km/h and v/3.6 matches the GNSS velocity topic with zero lag. Per-bag RMSE is 0.022–0.030 m/s after a per-(vehicle, day) scale. It is not ground truth in five places. (1) Slides, spins and 30639 dropouts can cause errors of 1–4.8 m/s. (2) The wheel speed is unsigned and cuts off near 0.15 km/h: 92% of samples where GNSS reads 0.02–0.10 m/s have wheel exactly 0. (3) At standstill GNSS reads about +0.009 m/s higher than the wheel. (4) The wheel and GNSS velocity stamps lag speed derived from GNSS position (fix) by about 0.05 s in most bags, and by about −0.05 to +0.02 s on 09-03 and in some 30639 05-05 bags. That is about 0.7 m along-track at 15 m/s if the position reference uses fix stamps. (5) The master-velocity reference itself has glitches: it reads zero while moving, and its header stamps are shifted by ±1 s.
      EVID: All analysis was run inline through python stdin; no files were written because the session was read-only/plan mode. Data: 58–60 unique RTK bags (duplicates removed by md5 of F.v; short bags excluded). Lag scan of mean(F,R)/3.6 against hypot(master vx,vy) on header stamps, ±0.6 s in 0.02 s steps: best lag 0.00 in 51/58 bags. Parabolic-refined lag in clean bags is −0.006..+0.009 s. Nonzero lags appear only in anomaly bags: 2050d396 −0.02, 33bec73f −0.02, 50956d6e −0.04, 927002c2 +0.02, d927f360 +0.06, and 4d487b0d −0.08, which is caused by master vel =0 while the rover and wheel read 4.3 m/s at 100–132 s. Pooled over 624,529 samples with GNSS time-glitch, master/rover mismatch >0.3 and stale samples excluded, k=1: std 0.0291, robust 0.0179, mean −0.0073. Stops: std 0.0086, mean −0.0086. Cruise (|a|<0.1, v>5): std 0.033, robust 0.028, p0.01%/p99.99% −0.16/+0.19. Braking tail p0.01% −1.22 (slides). Slope of residual vs acceleration 0.0011 s, i.e. about 1 ms lag. With per-bag scale: cruise std 0.027. Per-bag RMSE with per-bag scale: 0.022–0.030 in clean bags; 0.08–0.21 in bags with slip or dropouts; 0.64 in 4d487b0d (GNSS artefact). Independent check against speed from RTK position central differences (0.4 s window): the wheel lags by +0.05 s in 55/58 bags; exceptions are 27e994fc −0.05, 88548b02 −0.03, 253671cc +0.01, 92226df0 +0.02. The GNSS velocity topic shows the same +0.05 s lag against the fix. Rover velocity against rover fix: +0.05 s median over 67 bags, 6 bags in −0.03..0. Wheel values: min 0.0, no negatives; smallest positive 0.153 km/h.
  ~ [A2_scale_by_day] partially_confirmed: The wheel/GNSS scale is set per vehicle AND day. Ranges: 30618 07-27 0.9996–1.0006, 08-10 0.9990–1.0000, 08-26 0.9983–0.9999, 09-03 0.9934–0.9954; 30639 08-26 0.9966–0.9975, 05-05 about 1.0095. The total spread is about 1.6% (−0.66% to +0.95% around 1.0). Within a bag the scale is stable: 1 km chunks differ by 0.09% median and 0.26% at p90. Once 30639 dropouts are handled, the scale is the dominant distance error: ±0.5–1% (25–55 m per 5.4 km bag), against at most about 6.6 m per bag from slip. If dropouts are NOT handled, they dominate instead, with errors up to −344 m. The scale cannot be calibrated in the initial GNSS window: all 86 unique bags start stationary. Some per-bag numbers differ from the analyst's; for 27e994fc the error with fixed K=0.999 is −0.56% (position-based) to −0.69% (velocity-based), not −0.95%.
      EVID: Method 1, independent of GNSS velocity: wheel distance divided by map arc length between the first and last RTK master fixes projected onto the pathgraph (NN distance <1 m, one leg of ≥3 km per bag). 30618 07-27: n=24, median 1.0001 (0.9996–1.0006). 08-10: n=7, median 0.9991 (0.9990–1.0000). 08-26: n=12, median 0.9994 (0.9983–0.9999); the 0.9983 is 2050d396, lowered by slides. 09-03 RTK: 0.9934 (27e994fc), 0.9954 (88548b02); non-RTK 0.9860–0.9936. 30639 08-26: 0.9966–0.9975 (n=5). 05-05 RTK: 1.0095 (92226df0); non-RTK 0.9889–1.0162. Method 2 (GNSS velocity, RTK samples): 253671cc 1.0064, 92226df0 1.0094, 27e994fc 0.9935, 88548b02 0.9938, which agrees with the analyst. Whole-bag velocity-integrated error with K=0.999: 27e994fc −0.69%, 88548b02 −0.64%, 92226df0 +0.90%, 253671cc +0.61%, 3b3d9eb8 +1.06% (F only, non-RTK). Within-bag spread over 1 km chunks, 57 bags: min 0.017%, median 0.086%, p90 0.26%. Outliers are 3e9f4952 and 616ec56b (paired +1.07%/−1.1% chunks, i.e. a GNSS fix error at a chunk boundary) and 2050d396 (one chunk at 0.9932, i.e. −6.8 m from slides). The grade effect of 3D vs 2D arc length is only 0.012%. Slip impact with a naive F/R mean over event windows: 2050d396 −6.56 m, 33bec73f +5.27, 50956d6e +2.92 and −3.23, 68d1748a −2.29 (≤0.12% of a bag). Start of bags (86 unique): speed at t0 is 0 in all; first motion (>0.5 m/s) at 6.7–113 s, median 11.4 s; distance in the first 10 s ≤6.2 m, median 0.
  ~ [A4_rear_dropouts_30639] partially_confirmed: Only vehicle 30639 has single-bogie dropouts: 11 of them, in 8 of 19 bags, and none in 30618. They are R in 10 cases and F in 1, and last 5.8–73.5 s. The last value before each gap is 0.0 in 11/11 cases. 10/11 begin at departure after a 7–68 s stop: the dropped bogie keeps reporting 0.0 for 0.3–0.6 s while the other bogie is already at 1.2–2.1 km/h, then goes silent. 1/11 does NOT begin at a stop (3b3d9eb8, the first 30.4 s gap). There, R froze at 4.808 km/h for 19 samples (1.79 s) while rolling at about 5 km/h with cmd 0, then sent 0.0 while F read 5.2 km/h, then went silent. The other bogie travels 18–550 m during a gap. On resume the dropped bogie matches the other within 0.1 km/h. A naive ZOH mean loses 20–344 m per bag; a staleness rule plus a stuck-zero rule removes this.
      EVID: Header-gap scan (>2 s) over all unique bags, with times relative to the first wheel stamp (the analyst's t0 is offset by 2–5 s; durations match exactly): 3b3d9eb8 R 30.4 s and 73.5 s; 4285f2bc R 47.1; 44226bde R 16.7; 584b6e32 R 16.3; 927002c2 R 20.5 and 8.7; 9f0b519f R 5.8; c31df386 F 19.8; d927f360 R 25.6 and 12.1. Other bogie at gap start: 1.15–2.07 km/h, except 3b3d9eb8#1 at 5.18. Stop duration before: 7.4–68.4 s, except 3b3d9eb8#1 which had no stop in the previous 100 s. d927f360@1061 started rolling with cmd 0 throughout. Stuck-zero episodes (v==0 while the other bogie >1 km/h for ≥0.25 s): exactly 10, all immediately before these dropouts, all 30639. Frozen nonzero runs ≥5 samples: 3b3d9eb8 R (19 samples, 1.79 s, 4.808) and one benign run at cruise in 6cb3280a (5 samples). Naive ZOH mean vs a robust rule (staleness 0.35 s, stuck-zero rejection, fall back to the fresh bogie), against master+rover-consistent GNSS velocity. 3b3d9eb8: naive −4.56%, robust +1.82%, difference −344 m. d927f360: −2.36% vs −0.30% (−111 m). 927002c2: −1.95% vs −0.39% (−84 m). 584b6e32: −0.55% vs −0.18% (−20 m). 4285f2bc: −72 m. c31df386: −29 m. 44226bde: −22 m. 9f0b519f: −9 m. RMSE naive→robust: 1.136→0.083, 0.706→0.046, 0.462→0.050, 0.178→0.047 m/s. The residual robust error of −0.2..−0.4% matches the 30639 08-26 scale of 0.997. 3b3d9eb8's +1.8% combines the 05-05 scale of about +0.95% with a non-RTK reference.
  ~ [A5_single_bogie_slip] partially_confirmed: Single-bogie spin and slide episodes are real but rare: 12 episodes with |F−R|>max(1.5 km/h, 8%) for ≥0.3 s across 86 unique bags. The bogie with the lower peak |accel| was the one closer to GNSS in 10/10 GNSS-covered episodes. It was actually correct (within 0.3 km/h of GNSS) in only 7/10. In the other 3 (2050d396@441.6, 33bec73f@102, 68d1748a@994.6) BOTH bogies were wrong by 1.5–3.1 m/s, i.e. common-mode, and picking the better bogie still leaves errors of 1.4–2.3 m/s. Without dropouts or slips, F−R is tiny: p50 0.027, p99 0.226, p99.9 0.458 km/h; relative at v>10 km/h p99 1.7%, p99.9 3.3%.
      EVID: Episodes (my time base; GNSS in km/h), listed as F / R / GNSS, with peak acceleration aF/aR in m/s². 2050d396@398.7: 32.57/24.13/32.64 (1.08/8.23), F correct. @401.4: 27.44/12.16/27.57, F correct. @441.6: 6.95/11.22/18.32, both wrong. 33bec73f@102.0: 18.42/23.11/12.81, both wrong (spin, cmd +6..+8). @104.9: 14.70/22.87/14.48, F correct. @108.5: 15.63/26.44/15.70, F correct. 68d1748a@994.6: 24.99/20.77/27.47, both wrong. 50956d6e@78.7: 20.70/28.46/20.73, F correct. @96.0: 41.81/30.21/30.16, R correct. @1026.5: 10.53/7.56/7.55, R correct. No GNSS: 117c2d02@100.2 (F 9.94, R 3.46, cmd −8) and e4379d7f@1153.9 (F 30.0, R 26.97, cmd −12). Pooled clean F−R over 721,803 moving samples: robust std 0.0109 m/s (analyst 0.0114). A simple robust fusion (pick the bogie closer to the previous estimate when mismatched) gave, over the window: 50956d6e 70–100 s max error 1.58→0.27 m/s, distance +2.92→−0.32 m. 2050d396 380–460 s −6.56→−2.50 m; the remaining −2.2 m is the common-mode slide at 441. 33bec73f 95–115 s max 2.26→1.62 m/s, distance +5.27→+1.77 m; not fully fixed because F also spins.
  ~ [A6_common_mode_slip] partially_confirmed: Common-mode slides, locks and spins are rare and short: about 5 events in 4 bags over about 21.7 h of valid GNSS (~0.2–0.3 per hour), lasting 0.7–3 s and each costing 1.4–3.3 m. Their onset does exceed normal wheel acceleration (−2.97 to −7.1 or +2.1 to +3.4 m/s²). BUT a deceleration threshold is not specific. Real GNSS-confirmed decelerations beyond −2.4 m/s² exist (4 episodes). The worst is 616ec56b@1020.9: −5.15 m/s² on the wheel, −4.72 on GNSS, with cmd 15→0 and never negative, so it is an uncommanded emergency or safety brake. A +1.6/−2.4 rate limiter makes that event worse: max error 0.17→2.02 m/s, RMSE 0.031→0.687, distance +0.02→+4.31 m, about as much as it gains on the slides. Slides and spins are separable by signature. Slides/spins show a rebound (accel +2.1..+6.8 m/s² after the dip during braking), and slides/spins show |F−R| of 4.2–8.2 km/h. Real braking stays monotonic with |F−R|≤0.49 km/h, |aF−aR|≤0.65 and rebound ≤0.49. A lock (both bogies at exactly 0 with no rebound) looks similar to a real hard final stop (88548b02@1230: −2.7 m/s² at 5.9 km/h).
      EVID: Two-antenna GNSS reference (master and rover agree within 0.25 m/s, both fresh, time-glitch samples removed), 68 bags. Episodes with both bogies off by >0.3 m/s in the same direction for ≥0.3 s. 2050d396@440.9: 1.1 s, F −3.16, R −2.05 m/s, cmd −14, wheel acc −7.1/+4.85, −2.2 m. 33bec73f@101.4: 1.1 s spin, +1.65/+2.91, cmd +8, +3.39, +1.85 m. 33bec73f@104.1: 0.7 s, +0.5/+2.3. 50956d6e@1037: 2.6 s lock to 0 at cmd −15, −1.68/−1.83, acc −3.68, −3.24 m. 68d1748a@994–996.6: master only (rover frozen); F −1.7, R −2.5 m/s, cmd −5/−6, acc −2.97 then +6.78. Two false episodes: 27e994fc@239.6 is a GNSS header shift of 1.07 s, and 40ffd323@85.5 (3.3 s, −1.05 m/s, −4.3 m) is a wheel stamp glitch, not slip. Clean wheel acceleration (0.2 s central difference, 546k samples, windows around events removed): p0.001% −4.02, p0.01% −2.07, p0.1% −1.70, p99.99% +1.44, max +1.80, min −5.15. Real decelerations beyond −2.4 with GNSS agreeing within 0.3 m/s: 616ec56b@1020.9 (−5.15; GNSS −4.72; confirmed by fix-derived speed; 29.6→1.4 km/h in 3 s), 616ec56b@1022.6 (−2.61), 88548b02@1230.4 (−2.71, GNSS −2.50), f19a4ac3@851.6 (−2.45, GNSS −2.08). Also 2050d396@407.1 at −2.44 (GNSS −1.83).
  ~ [A9_input_timestamp_glitches] partially_confirmed: Wheel and cmd header stamps do sometimes jump ±0.4–1 s, with interleaved or backward steps. In those interleave episodes, sorting by header restores a smooth curve. In normal operation header stamps are slightly better than record time. However, 'header = true sampling time' does NOT hold in every glitch window. In clock-slew windows (28538acf 167–174 s, 1cc230fa 127–133, 40ffd323 505–512, 9c362687 1336) wheel and GNSS agree only in the recorder clock (0.12–0.21 km/h). In header clocks they disagree (0.84–2.49 km/h), because GNSS headers carry ~1 s offsets that slew back at about 0.1 s/s. GNSS header glitches affect 0.77% of master-velocity samples in 17/68 bags, so the reference itself is misaligned there; this is not fixable on the output side. For integration, clamp dt to [0, ~0.3 s] and sort or deduplicate by header. Simultaneous both-bogie header gaps >0.5 s occur 4 times, not in 11 bags; the analyst's count probably used record time. The first 2–6 s of every bag arrive as a startup burst with rec−hdr up to 2–6 s, overlapping the GNSS initialisation window.
      EVID: Offset scan (rec−hdr against a rolling median): wheel/cmd glitches in 2255aade/7bfbb5ed (F 8, R 8, C 20 samples; backsteps F 6, R 7, C 17); 40ffd323/efb92709 (8/8/10; backsteps 6/6/20); 9c362687 (1 each); 28538acf (dev −0.39..+0.52 over 6 s); af7496f0 (−0.46..+0.43 over 6 s); 1cc230fa and 49fe4c54 (−0.2 over 6–7 s). Interleave episodes, sum|d2v| in record order → header order: 2255aade 197.5–202.5 s 70.1→3.2 (max|dv| 4.02→0.72 km/h); 40ffd323 86–89.5 s 103.6→6.1. Wheel-vs-GNSS RMSE (km/h) in glitch windows, in the order Whdr/Ghdr, Whdr/Grec, Wrec/Ghdr, Wrec/Grec. 28538acf: 1.81, 0.62, 1.72, 0.14. 1cc230fa: 0.84, 0.36, 0.56, 0.13. 40ffd323@505: 1.44, 0.74, 0.95, 0.12. 40ffd323@86: 2.49, 1.43, 2.21, 0.76. 9c362687@1336: 1.18, 0.33, 1.19, 0.21. In 28538acf, MV rec−hdr drifts 0.97→0.11 and F drifts 0.01→−0.31 (about −0.1 s/s). Normal operation, clean RTK bags, |a|>0.4, n=202,799: RMSE with wheel header 0.0473 (robust 0.0256) vs record time minus median 0.0499 (robust 0.0312). Sample intervals over 1.90M: dt_rec <0.15 s 92.99%, 0.15–0.25 6.94%, 0.25–0.35 0.063%, 35 gaps >0.35 s (13 >1 s); dt_hdr 92.69/7.23/0.079%, 28 >0.35 s. Simultaneous header gaps >0.5 s: 0652866c@1375.4 (0.91 s), 117c2d02@101.8 (0.87), 27e994fc@1609.4 (0.96), 3b3d9eb8@405.7 (0.89). GNSS master-velocity header glitches >0.3 s: 6,037 of 787,350 samples; largest 3b3d9eb8 244 s at −1.0 s, 40ffd323 125 s, 27e994fc 47 s, af7496f0 40 s, 28538acf 36 s.
#################### baseline_eval
SUMMARY: I built a judge-like evaluator and cross-validated baselines B1..B6 on 58 unique bags with good GNSS (RTK status 2). It uses a 2-fold split that keeps duplicate bags together and is stratified by vehicle and direction.

The evaluator was NOT saved to disk because this session is read-only / plan mode. Its source is in scripts[0], and the best estimator (B6) is in scripts[1].

**What the reference looks like**
- Positions: master GNSS fixes with status==2, converted to UTM37N minus (300000, 6100000), z = altitude.
- Speed: hypot(vx, vy) from master/vel, kept only where a status-2 fix is within 0.05 s.
- Matching: nearest stamp within 0.05 s.

**Main results**
- **Off-map handling dominates position error.** Clamping the arc length to the map (the literal B1/B2 spec) gives a median along-track error of 438–472 m and a median final drift of 6.3%. Every run starts 130–640 m before the map and ends up to 530 m after it.
- **Extending the route with training GNSS tracks (B2x)** brings the median along-track |error| down to 3.1 m (mean 6.2).
- **Adding stop re-anchoring (B4)** gives 1.3 m (mean 4.1).
- **B6 is the best version.** It adds: a robust off-map start estimate, uncertainty-based gating, finer stop clusters (rare stops used as neighbours), online wheel-scale correction from stop-to-stop distances, a 0.5 s stale timeout per bogie, and short linear extrapolation of wheel speed. Results:
  - along-track |error|: median 0.64 m (mean 1.87, max-per-bag median 7.8);
  - 3D error: median 1.46 m (mean 2.86);
  - cross-track RMSE: median 0.18 m;
  - speed RMSE: median 0.032 m/s (mean 0.055);
  - speed coverage within 0.05 s: 100% when publishing on every input message.

**Error budget (B6, on-map along-track)**
- Start error: median 0.46 m (p90 2.0).
- Scale trend: median 0.59 m (p90 5.9).
- Other residual (slip, reference jumps), p95: median 1.6 m (p90 5.6).
- Without stop anchoring the scale trend is much larger: median 3.75 m, p90 8.4 m, max 67 m.

**Worst bags (B6)**
- 92226df0 and 253671cc: poor-quality GNSS at start plus a 1–1.3% wheel-scale outlier and an unusual off-map path, so no stop can be matched.
- 27e994fc: the tram takes an alternative branch after the map, and its altitude offset is abnormal.
- 4d487b0d: the reference GNSS speed freezes at 0 while the tram moves.
- 40ffd323 (slip), 50956d6e (front bogie fault), 28538acf (lag-like wheel deviation plus a ~37 m jump in the reference position).

**Antenna choice matters a lot.** The rover antenna is 12.44 m ahead of the master along the track. The map matches the master. A master-based estimate scores e3d 13.3 m against a rover reference and 7.1 m against the midpoint.

**Timing matters.** Speed error is lowest at zero time shift. A 0.1 s stamp error raises speed RMSE from 0.032 to about 0.057, and 0.3 s raises it to about 0.15.
- [F01] (high, CRIT) Off-map track segments dominate position error. Clamping the arc length s to the map, as the literal B1/B2 spec does, is catastrophic. The route must be extended with centerlines built from training GNSS tracks.
    EVID: B1/B2, 2-fold CV, 58 bags: median along-track |error| 438/472 m (mean 359/364 m), median e2d 342/356 m, median final drift 6.33%. Reasons:
- T2S runs start 130-220 m before the map (b8044aa0: 404 m) and end 415-530 m after it.
- S2T runs start 450-640 m before the map and end 0-375 m after it.
B2x (same wheel speed, route extended by off-map templates, no stops): median along-track |error| 3.14 m (mean 6.16), median e2d 3.15 m, median drift 0.14%.
    IMPL: Represent position as arc length s on one extended polyline per direction: pre-map template + map + post-map template. Allow s < 0 and s > L. Never clamp to the map.
- [F02] (high, CRIT) Re-anchoring at learned stop clusters is the main drift correction. It only works together with the extended route.
    EVID: Median along-track |error| [mean]:
- B2x (extended, no stops): 3.14 [6.16].
- B3 (stops on the clamped map): 473.8 [365.3]. No gain, because the clamp dominates.
- B4 (extended + stops, fixed 15 m gate): 1.30 [4.10].
- B6: 0.64 [1.87].
Without stops (B5_nostop), the end-of-map along-track error is median 5.2 m. With B6 it is 0.41 m.
    IMPL: Detect a stop as fused v < 0.03 m/s for at least 3 s. At the 3 s mark, snap s to the nearest learned cluster if it passes the gate, then reset the position uncertainty sigma to the cluster spread plus 0.5 m.
- [F03] (high, CRIT) Picking the start arc length off-map from the single nearest template is fragile at the Tallinskaya end (T2S). There are parallel tracks about 5 m apart, and templates at the same lateral distance imply start positions (s0) up to 45 m apart.
    EVID: Nearest-template errors:
- 2050d396: +45.1 m (candidates at d=5.0 m give s0 = -126 / -169 / -169; true -171).
- 9f0b519f: +44.9 m.
- 21dd3af3: +23.2 m (22 m from every template).
- 92226df0: -44 m (status-0 start, 20 m off all templates).
Taking the median s0 over templates with d <= dmin + 1 m cut B5 along-track |error| from 40.5 to 0.88 m (2050d396) and from 47.3 to 0.66 m (9f0b519f). Over all 53 off-map starts, the median s0 error was 3.0-3.6 m.
    IMPL: Estimate the off-map start as the median s0 over all training templates whose closest-point distance is within dmin + 1 m. Set the initial sigma to max(3, std(candidates) + 3, dmin) so the first on-map stop can correct it.
- [F04] (high, CRIT) A fixed 15 m gate with coarse (8 m) clusters snaps wrongly. An uncapped adaptive gate is dangerous next to rare stops. What works: finer clusters, rare clusters counted as neighbours, and a capped uncertainty-based gate.
    EVID: - Fixed 15 m gate, 8 m merge: 31 of 404 snaps wrong by more than 5 m. Most come from two-position stops (S2T 2400/2406, T2S 1818/1824 and 2249/2257). In 2366c74a a wrong snap (4381 to 4371) drove the end error to -11.2 m.
- Adaptive gate without a cap: 10 of 391 wrong, but 76e1f9c7 and ab5921a4 were pulled 42 m from a rare stop at s=241 to the cluster at 282 (along-track |error| 0.4 -> 42 m).
- B6 (3 m merge; snap targets need >=20% support; neighbours >=10%; gate = min(max(3 sigma, 6), 0.45 x gap to neighbour, 25 m)): no catastrophic snaps; along-track |error| > 5 m in only 3 bags.
- Cluster spread (sd) with 3 m merge: 0.05-0.8 m.
    IMPL: Use a 1D Kalman-style arc-length state. Sigma grows about 0.4% of distance travelled. Gate = min(max(3 sigma, 6 m), 0.45 x distance to the nearest cluster including rare ones, 25 m). Snap only to clusters seen in >=20% of training runs.
- [F05] (high, CRIT) Output stamps must equal the input stamps, and output should be published on every input message. A stamp shift of 0.1 s roughly doubles the speed error, and publishing on front-bogie messages only loses up to 17% of matches.
    EVID: Speed RMSE versus stamp shift (median over bags):
- B6 with extrapolation: -0.3 s 0.146, -0.1 s 0.060, 0 s 0.032, +0.1 s 0.057, +0.3 s 0.148.
- Zero-order hold: -0.1 s 0.044, 0 s 0.039, +0.1 s 0.076.
Best shift at 0.05 s resolution is 0. Coverage within 0.05 s: 83-97.5% when publishing on front-bogie stamps only; 100% when publishing on the union of front, rear and cmd stamps (~40 Hz).
    IMPL: Publish /result/velocity and /result/position in the callback of every input message (front, rear and cmd), with header.stamp = that message's header.stamp. No timer-based publishing with now().
- [F06] (high) Holding the last wheel sample constant (zero-order hold) biases speed by regime. Short linear extrapolation of each bogie's last slope removes the bias.
    EVID: - Zero-order hold (B4): bias +/-0.02-0.03 m/s by regime (accel -0.021, brake +0.028), v_rmse median 0.039 [mean 0.061].
- Linear extrapolation <= 0.2 s (B4p/B6): bias 0.000-0.007 by regime (accel +0.007, brake 0.000), v_rmse median 0.032 [mean 0.055].
Position barely changes (along-track median 0.77 -> 0.64 m).
    IMPL: Per bogie, v_hat(t) = v_last + clip(t - t_last, 0, 0.2) x slope of the last two samples, clamped at >= 0.
- [F07] (high, CRIT) Rear-bogie dropouts (up to 73.5 s on vehicle 30639) look like single-bogie faults if the last value is held. A 0.5 s stale timeout per bogie fixes them.
    EVID: v_rmse, naive hold with no timeout -> 0.5 s timeout:
- 927002c2: 0.461 -> 0.033
- d927f360: 0.706 -> 0.037
- 584b6e32: 0.205 -> 0.035
- 9f0b519f: 0.147 -> 0.034
Overall: median 0.040 -> 0.032, mean 0.085 -> 0.056.
    IMPL: Fuse only bogies with a sample younger than 0.5 s. If both are stale, hold the last fused value. Log when this happens.
- [F08] (medium) Real single-bogie faults (steady offsets of about 3 m/s) are handled by picking, when the bogies disagree, the one closer to the prediction. A 3 m/s^2 rate limit does not help.
    EVID: Rule: if |front - rear| > 0.3 m/s, take the bogie closer to v_prev + a_prev x dt.
- 33bec73f (rear +3 m/s): 0.086 -> 0.048.
- 2050d396: 0.090 -> 0.053.
- Mean over bags: 0.0555 -> 0.0541.
Adding a 3 m/s^2 rate limit hurt 616ec56b (0.030 -> 0.062).
Still unsolved: 50956d6e 0.140 (front +3.2 and a spike to 9.65), 40ffd323 0.172 (slip/slide on both bogies), 28538acf 0.124 (both bogies about 1 s lag-like in braking).
    IMPL: Add the disagreement-pick rule plus a spike check against a controller-command-based acceleration model. A rate limit, if used, should sit above 3 m/s^2.
- [F09] (high, CRIT) The wheel scale is vehicle-specific: about 1/3.597 for 30618 and 1/3.585 for 30639 (not 1/3.6). Some individual runs deviate by 1-1.3%.
    EVID: - GNSS/wheel ratio per bag: 30618 median 3.5970 (sd 0.0044, i.e. 0.12%); 30639 median 3.5854 (sd 0.018).
- CV fold estimates: 3.5968/3.5974 and 3.5838/3.5869.
- Outliers: 92226df0 3.634, 253671cc 3.622. For these two the naive 1/3.6 scores better (v_rmse 0.069/0.060 vs 0.089/0.074).
- Front/rear ratio: 1.0000.
- Without stops, the scale trend over the map is median 3.75 m, p90 8.4 m, max 67 m.
- Online stop-to-stop scale correction (prior 800 m, clip +/-3%): median along-track 0.748 -> 0.636 m, but it cannot fix outliers that never snap.
    IMPL: Use a scale table per vehicle prefix (unknown vehicle: 1/3.595). Update scale online from anchored stop-to-stop distances, weighted by distance and bounded to +/-3%.
- [F10] (high, CRIT) The master antenna defines the map. The rover sits 12.44 m ahead along the track, so the jury's choice of antenna shifts the along-track reference by 6-12 m.
    EVID: - Rover minus master: 12.438 m median, always along the direction of motion; cross 0.01 m (30618) / 0.17-0.18 m (30639); dz about 0.
- Distance to map: master 0.03-0.04 m on its own direction, rover 0.05-0.16 m.
- B6 (master frame) scored against each reference: e3d median 1.46 (master) / 13.30 (rover) / 7.08 (midpoint); along-track mean -0.08 / -12.70 / -6.36.
- With a matching shift (+12.44 m for rover, +6.22 m for midpoint): e3d 2.18 / 1.70.
    IMPL: Default to the master point. Add one parameter for the along-track offset of the output point (0 / 6.22 / 12.44 m), applied as s + offset before converting to x, y, z.
- [F11] (medium) The post-map extension should be a consensus path (per-arc median over training runs), not a single template.
    EVID: Leave-one-out test:
- T2S, longest template: mean error 4.28 m, end error 17.3 m. Consensus: mean 0.30 m, end 0.5 m.
- S2T: mean 6.15 -> 0.22 m. The p90 stays 12-34 m because some runs take alternative branches at the Tallinskaya end (8158f0b0, 49fe4c54, 0be558e2, 27e994fc).
B6 used the longest template, which explains its end_e2d of about 15 m on T2S runs (median end_e2d 7.6 m, about 0.28% drift). Consensus should roughly halve the final-drift metric.
    IMPL: Build pre- and post-map extensions as the per-arc-length median of training off-map tracks (needs >=3 runs), aligned at the map boundary. Accept a bounded lateral error on the S2T post branches.
- [F12] (high) B6 error budget: after anchoring, the start error, the scale trend and the slip/other residual are all small. Errors above 5 m come from a few specific failures.
    EVID: On-map along-track, per bag, linear fit of error versus s:
- start error: median 0.46 m, p90 2.0;
- scale trend: median 0.59 m, p90 5.9;
- residual p95: median 1.6 m, p90 5.6.
Across bags: along-track |error| median 0.64 (mean 1.87); RMSE median 1.06 (mean 2.84); per-bag max median 7.8 (mean 12.5); cross-track RMSE median 0.18 (mean 0.47; S2T up to 1.65); e2d on-map median 1.26; e2d off-map median 2.65; e3d median 1.46.
    IMPL: The remaining effort should go to: the rare catastrophic cases (start with poor GNSS, unusual off-map paths, scale outliers), slip detection, and the consensus off-map geometry.
- [F13] (high) The worst B6 bags have identifiable causes.
    EVID: - 92226df0 (T2S, 30639): along-track |error| 27.8, e3d 30.5, end +63. Status-0 GNSS during the start window, 20 m from every template, scale 1.3% off; no snap passes the gate.
- 253671cc (S2T, 30639): along-track 24.6, end +45.9. Unusual pre-map path plus scale 1% off.
- 27e994fc (S2T): e3d 23.0. Post-map alternative branch 31 m off; altitude offset 4.85 m instead of 3.10.
- 4d487b0d: v_rmse 0.636. Reference speed frozen at 0 for up to 30 s (119 samples) while moving; about 0.03 without that.
- 40ffd323: 0.172 (slip/slide).
- 50956d6e: 0.140 (front bogie fault).
- 28538acf: 0.124, along-track max 37.6. Wheels about 1 s lag-like in braking, and the reference fix is about 37 m wrong at t~690-730 s even though status is 2.
    IMPL: When the start GNSS is not status 2, or the start is more than 10 m from any template, set sigma0 to 40-60 m and widen the first-stop gate for the terminus stop (T2S cluster at 38 m; next one is 213 m away). Flag reference artifacts when reporting.
- [F14] (medium) The reference itself contains artifacts: status-2 position jumps and frozen speed.
    EVID: - Master status-2 fixes that disagree with integrated speed by more than 1.5 m over 1 s: up to 3.5% per bag (0be558e2 3.5%, ab5921a4 3.0%).
- Removing them barely moves the aggregate: along-track median 0.636 -> 0.618 m, max-per-bag mean 12.47 -> 11.99.
- Single events can be large: 28538acf, about 37 m.
    IMPL: Report metrics both raw and with the reference cleaned. Do not tune parameters on per-bag max errors without checking the reference first.
- [F15] (high) Output z should be map z + 3.10 m. z adds little to the 3D error.
    EVID: - Altitude minus map z: 3.10 m (sd 0.08-0.5).
- Outliers: 27e994fc 4.85, 88548b02 4.26, f19a4ac3 4.60.
- B6 median e2d 1.26 vs e3d 1.46; mean z error about 0 +/- 2 m.
    IMPL: z_out = map z(s) + 3.10, and interpolate z along the off-map templates the same way.
- [F16] (high) Direction and starting route can be found from the first 3 s of GNSS with a simple rule. Every run goes one way, terminus to terminus.
    EVID: Rule: if the start point is within 2.5 m of a route and projects inside it, take that route; otherwise take the route whose first point is nearest. Correct on 58/58 bags.
- 4 runs start mid-route (s = 664, 1256, 1662 and partial-RTK runs).
- In 4 runs the start window has only status-0 GNSS (92226df0, 68d1748a, 253671cc, 4285f2bc).
- In 30639 bags GNSS starts 1.1-1.4 s after the wheels.
    IMPL: Start: use the median of status-2 fixes in the first 3 s after the first GNSS message (fall back to any status). Carry the odometer forward from the fix stamp to the current time. Freeze direction after the start.
- [F17] (high) Duplicate bags must be grouped in train/validation splits. Only 58 unique bags have enough good GNSS for scoring.
    EVID: - 122 bags: 97 unique (25 identical pairs by wheel-message count and first stamp).
- 58 unique bags with >=3000 status-2 fixes: 49 for 30618, 9 for 30639.
- 8 bags have status 0 only; 30639_e4379d7f has no master receiver.
    IMPL: Fit clusters, templates and scales with grouped CV (by duplicate group, stratified by vehicle and direction). Report per-vehicle results, because 30639 has only 9 scorable bags.
PARAMS:
  * k_wheel_30618 = 1/3.597 (0.27801 m/s per unit)  (Least-squares GNSS speed / wheel ratio for v > 3 m/s and |a| < 0.1 m/s^2 over 49 bags. CV folds gave 3.5968 and 3.5974.)
  * k_wheel_30639 = 1/3.5854  (Same method over 9 bags. Folds gave 3.5838 and 3.5869. Some runs reach 3.62-3.63.)
  * k_wheel_unknown_vehicle = 1/3.595  (Between the two vehicles. Online stop-based correction should absorb the rest.)
  * online_scale_update = scale = (800 + sum of anchored distances) / (800 + sum of odometer distances), only for stop pairs > 300 m apart, clipped to [0.97, 1.03]; also applied to the published speed  (B6 vs B6_noscale: median along-track 0.748 -> 0.636 m.)
  * publish_trigger = Every input message (front, rear, cmd; ~40 Hz); header.stamp = input stamp; frame_id = map  (100% coverage within 0.05 s versus 83-97.5% for front-only. The lag scan minimum is at 0 s.)
  * bogie_stale_timeout = 0.5 s  (Fixes rear dropouts in 927002c2, d927f360, 584b6e32, 9f0b519f (v_rmse 0.15-0.71 -> 0.033-0.037).)
  * speed_extrapolation = Linear from the last two samples, horizon <= 0.2 s, clamp v >= 0  (v_rmse median 0.039 -> 0.032; removes the ZOH accel/brake bias.)
  * bogie_disagreement_threshold = 0.3 m/s: pick the bogie closer to v_prev + a_prev x dt  (33bec73f 0.086 -> 0.048, 2050d396 0.090 -> 0.053; no regressions.)
  * z_offset = 3.10 m above map z  (Median altitude minus map z over good runs (sd 0.08-0.5).)
  * stop_detection = fused v < 0.03 m/s for >= 3.0 s; snap at t_start + 3 s  (Stop-cluster learning with 3 m merge gives cluster sd of 0.05-0.8 m.)
  * stop_cluster_merge_gap = 3 m; snap targets need >= 20% of training runs; neighbour list needs >= 10%  (An 8 m merge produced 31/404 wrong snaps from two-position stops (2400/2406, 1818/1824, 2249/2257).)
  * stop_gate = min(max(3 sigma, 6 m), 0.45 x gap to neighbouring cluster, 25 m); sigma grows by 0.4% of distance; sigma after snap = cluster sd + 0.5  (B6: no catastrophic snaps. An uncapped gate caused 42 m errors (76e1f9c7, ab5921a4).)
  * sigma0 = 3 m for an RTK start on the map; 10 m for a non-RTK start; off-map: max(std of candidate s0 + 3, dmin)  (Start-error diagnostics over 53 off-map starts.)
  * offmap_start = median s0 over templates with closest distance <= dmin + 1 m  (2050d396 40.5 -> 0.9 m, 9f0b519f 47.3 -> 0.7 m along-track |error|.)
  * stop_clusters_T2S_m = frequent: 39, 252, 665, 1256, 1818(+1824), 2249(+2257), 4371; occasional: 931, 1986, 2641, 4278, 4663; rare (neighbours only): 1164, 1315, 1775, 2211, 2346  (Median projected stop positions on the T2S map across training runs.)
  * stop_clusters_S2T_m = frequent: 282, 2400(+2406), 2833, 3499, 3976, 4396, 4616; occasional: 1798, 2288, 2621, 3320, 3720; rare: 241, 316, 364  (Same method on the S2T map.)
  * output_antenna_along_offset = 0 m (master) by default; +12.44 m if the jury uses rover; +6.22 m for the midpoint  (Median master-to-rover baseline along track 12.438 m; e3d versus rover drops 13.3 -> 2.18 m with the shift.)
  * evaluator_tolerances = match tol 0.05 s; on-map gate 5 m for along/cross-track; direction detection by projection onto T2S with d < 6 m  (From the README (judge uses ~0.05 s) and the 3.3-4.3 m spacing between tracks.)
OPEN Q: Which antenna, or which tram point, does the jury use as the reference: master, rover, midpoint, or tram centre? The choice moves the answer by 12.44 m along the track. Is the reference z the antenna altitude? | Does the jury keep only status==2 fixes? How does it handle frozen GNSS speed (4d487b0d: 0 for up to 30 s while moving) and status-2 position jumps (28538acf, ~37 m)? | Is the jury's speed reference hypot(vx, vy) of master/vel, or something else (rover, 3D norm, differentiated position)? How does it match when several outputs fall within 0.05 s of one reference sample? | Is the jury's frame exactly UTM37N minus (300000, 6100000), i.e. the pathgraph frame? The README only says 'local metric frame consistent with the reference'. | Will hidden runs start off-map and/or without status-2 GNSS (like 92226df0 or 253671cc)? How many seconds of GNSS are guaranteed? | Will hidden runs include other vehicles (unknown wheel scale) or unseen branches at the terminus loops? | evaluator.py was NOT saved to .../scratchpad/analysis/baseline_eval/evaluator.py because the session was read-only / plan mode. Its source is scripts[0]; save it before use. It reads the scratchpad cache pickles (cache/<bag>.pkl) made by to_cache.py. | The consensus post-map path (F11) and the disagreement-pick rule (F08) were tested separately (leave-one-out and speed-only). They are not yet combined into B6's position metrics.
SCRIPTS: # evaluator.py  (intended path: /private/tmp/claude-501/-Users-egor-Documents-sideprojects----------------------------/2010eef1-6052-45eb-85da-8f36b4a3a3e6/scratchpad/analysis/baseline_eval/evaluator.py ; NOT saved - read-only session)
# Judge-like evaluator. Reference = GNSS fix with status==2 -> UTM37N (lon0=39) minus (300000, 6100000); z = altitude.
# Speed ref = hypot(vx, vy) of /sensing/gnss/master/vel, kept only if a status-2 master fix is within 0.05 s.
# Matching: for each reference sample, nearest estimate stamp within tol (0.05 s).
# Input cache: SCR/cache/<bag>.pkl = {topic: {t_hdr, lat, lon, alt, status} or {t_hdr, vx, vy} or {t_hdr, v} ...}
import json, pickle, numpy as np
from scipy.spatial import cKDTree
SCR = '/private/tmp/claude-501/-Users-egor-Documents-sideprojects----------------------------/2010eef1-6052-45eb-85da-8f36b4a3a3e6/scratchpad'
MAPDIR = '/Users/egor/Documents/sideprojects/приколы/ХакатонМосТранспорт/'
OFF = np.array([300000.0, 6100000.0])
A_, F_ = 6378137.0, 1 / 298.257223563

def utm(lat, lon, lon0=39.0, k0=0.9996, fe=500000.0):
    n = F_ / (2 - F_); A = A_ / (1 + n) * (1 + n**2 / 4 + n**4 / 64)
    al = [None, n/2 - 2*n**2/3 + 5*n**3/16, 13*n**2/48 - 3*n**3/5, 61*n**3/240]
    phi = np.radians(lat); lam = np.radians(np.asarray(lon) - lon0); e = np.sqrt(F_ * (2 - F_))
    t = np.sinh(np.arctanh(np.sin(phi)) - e * np.arctanh(e * np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam)); eta = np.arctanh(np.sin(lam) / np.sqrt(1 + t * t))
    E = eta + sum(al[j] * np.cos(2*j*xi) * np.sinh(2*j*eta) for j in (1, 2, 3))
    N = xi + sum(al[j] * np.sin(2*j*xi) * np.cosh(2*j*eta) for j in (1, 2, 3))
    return fe + k0 * A * E, k0 * A * N

class Route:
    def __init__(s, P):
        s.P = np.asarray(P, float); d = np.diff(s.P[:, :2], axis=0); s.L = np.hypot(d[:, 0], d[:, 1]); s.u = d / s.L[:, None]
        s.s = np.r_[0, np.cumsum(s.L)]; s.len = s.s[-1]; s.tree = cKDTree(s.P[:, :2])
    def project(s, xy):
        # returns arc length (extrapolated past ends), distance, signed lateral offset, beyond-end flag
        xy = np.atleast_2d(xy); _, i = s.tree.query(xy); n = len(s.P)
        bs = np.zeros(len(xy)); bd = np.full(len(xy), np.inf); bl = np.zeros(len(xy)); bo = np.zeros(len(xy), bool)
        for k in (i - 1, i):
            k = np.clip(k, 0, n - 2); a = s.P[k, :2]; r = xy - a; u = s.u[k]
            t = np.einsum('ij,ij->i', r, u); lat = u[:, 0] * r[:, 1] - u[:, 1] * r[:, 0]
            lo = (k == 0) & (t < 0); hi = (k == n - 2) & (t > s.L[k]); tc = np.where(lo | hi, t, np.clip(t, 0, s.L[k]))
            q = a + tc[:, None] * u; dd = np.hypot(*(xy - q).T); dd = np.where(lo | hi, np.abs(lat), dd)
            m = dd < bd; bd[m] = dd[m]; bs[m] = (s.s[k] + tc)[m]; bl[m] = lat[m]; bo[m] = (lo | hi)[m]
        return bs, bd, bl, bo
    def point(s, sq):
        sq = np.clip(sq, 0, s.len)
        return np.c_[np.interp(sq, s.s, s.P[:, 0]), np.interp(sq, s.s, s.P[:, 1]), np.interp(sq, s.s, s.P[:, 2])]

ROUTES = {}
for nm, fn in [('T2S', 'таллинская - щукинская.json'), ('S2T', 'щукинская - таллинская.json')]:
    d = json.load(open(MAPDIR + fn)); ROUTES[nm] = Route([[p['x'], p['y'], p['z']] for p in d['points']])

def nearest(tq, ts):
    j = np.clip(np.searchsorted(ts, tq), 1, len(ts) - 1); jl = j - 1
    pick = np.where(np.abs(ts[jl] - tq) <= np.abs(ts[j] - tq), jl, j); return pick, np.abs(ts[pick] - tq)

_C = {}
def load(b):
    if b not in _C: _C[b] = pickle.load(open(SCR + '/cache/%s.pkl' % b, 'rb'))
    return _C[b]

def fixes(D, ant):
    g = D.get('/sensing/gnss/%s/fix' % ant, {})
    if len(g.get('lat', [])) == 0: return None
    E, N = utm(g['lat'], g['lon']); return dict(t=g['t_hdr'], xyz=np.c_[E - OFF[0], N - OFF[1], g['alt']], st=g['status'])

def glitch_mask(t, xyz, tv, v, win=1.0, thr=1.5):
    # flags status-2 fixes whose 1 s displacement disagrees with integrated GNSS speed by > thr metres
    vi = np.r_[0, np.cumsum(np.diff(tv) * (v[1:] + v[:-1]) / 2)]; cum = np.interp(t, tv, vi)
    ib = np.clip(np.searchsorted(t, t - win), 0, len(t) - 1)
    return np.abs(np.hypot(*(xyz[:, :2] - xyz[ib, :2]).T) - (cum - cum[ib])) > thr

def load_ref(b, antenna='master', clean=False):
    D = load(b); m = fixes(D, 'master')
    if antenna == 'mid':
        r = fixes(D, 'rover'); j, dt = nearest(m['t'], r['t']); ok = (dt < 0.05) & (m['st'] == 2) & (r['st'][j] == 2)
        t = m['t'][ok]; xyz = (m['xyz'][ok] + r['xyz'][j[ok]]) / 2
    else:
        g = fixes(D, antenna); ok = g['st'] == 2; t = g['t'][ok]; xyz = g['xyz'][ok]
    gv = D['/sensing/gnss/master/vel']; tv0 = gv['t_hdr']; v0 = np.hypot(gv['vx'], gv['vy'])
    j, dt = nearest(tv0, m['t']); okv = (dt < 0.05) & (m['st'][j] == 2); tv = tv0[okv]; v = v0[okv]
    if clean:
        gm = glitch_mask(t, xyz, tv0, v0); t = t[~gm]; xyz = xyz[~gm]
    mm = m['st'] == 2; s, d, _, _ = ROUTES['T2S'].project(m['xyz'][mm][:, :2]); on = d < 6; ds = np.diff(s)[on[1:] & on[:-1]]
    dirn = 'T2S' if (ds > 0.05).sum() >= (ds < -0.05).sum() else 'S2T'
    return dict(tp=t, xyz=xyz, tv=tv, v=v, dir=dirn)

def regimes(tv, v):
    vs = np.convolve(v, np.ones(11) / 11, mode='same'); a = np.gradient(vs, tv)
    r = np.full(len(v), 'cruise', dtype=object); r[a > 0.15] = 'accel'; r[a < -0.15] = 'brake'; r[vs < 0.1] = 'stopped'; return r

def evaluate(bag_id, est_t, est_v, est_xyz, antenna='master', tol=0.05, gate=5.0, clean_ref=False, ref=None):
    ref = ref or load_ref(bag_id, antenna, clean_ref)
    o = np.argsort(np.asarray(est_t), kind='stable'); est_t = np.asarray(est_t, float)[o]; est_v = np.asarray(est_v, float)[o]; est_xyz = np.asarray(est_xyz, float)[o]
    R = {'bag': bag_id, 'dir': ref['dir'], 'antenna': antenna}
    # speed
    j, dt = nearest(ref['tv'], est_t); ok = dt <= tol; e = est_v[j[ok]] - ref['v'][ok]
    R.update(v_cov=ok.mean(), v_rmse=np.sqrt(np.mean(e**2)), v_mae=np.mean(np.abs(e)), v_bias=e.mean(), v_p99=np.percentile(np.abs(e), 99), v_max=np.abs(e).max())
    rg = regimes(ref['tv'], ref['v'])[ok]
    for k in ('accel', 'brake', 'stopped', 'cruise'):
        m = rg == k; R['v_bias_' + k] = e[m].mean() if m.any() else np.nan; R['v_rmse_' + k] = np.sqrt(np.mean(e[m]**2)) if m.any() else np.nan; R['n_' + k] = int(m.sum())
    # position
    j, dt = nearest(ref['tp'], est_t); ok = dt <= tol; E = est_xyz[j[ok]]; F = ref['xyz'][ok]; tp = ref['tp'][ok]
    d2 = np.hypot(*(E[:, :2] - F[:, :2]).T); d3 = np.linalg.norm(E - F, axis=1)
    R.update(p_cov=ok.mean(), e2d_mean=d2.mean(), e2d_rmse=np.sqrt(np.mean(d2**2)), e2d_p95=np.percentile(d2, 95), e2d_max=d2.max(), e3d_mean=d3.mean(), e3d_rmse=np.sqrt(np.mean(d3**2)), e3d_max=d3.max(), ez_mean=np.mean(E[:, 2] - F[:, 2]))
    # along/cross-track on the map polyline of the run direction; reference points off-map (dist >= gate or beyond ends) are excluded
    rt = ROUTES[ref['dir']]; sr, dr, lr, outr = rt.project(F[:, :2]); se, de, le, oute = rt.project(E[:, :2])
    on = (dr < gate) & ~outr; R['onmap_frac'] = on.mean(); al = (se - sr)[on]; cr = (le - lr)[on]
    R.update(al_mean=al.mean(), al_mean_abs=np.abs(al).mean(), al_rmse=np.sqrt(np.mean(al**2)), al_max=np.abs(al).max(), ct_rmse=np.sqrt(np.mean(cr**2)), ct_max=np.abs(cr).max())
    R['e2d_onmap_mean'] = d2[on].mean(); R['e2d_offmap_mean'] = d2[~on].mean() if (~on).any() else np.nan; R['e2d_offmap_max'] = d2[~on].max() if (~on).any() else np.nan
    # distance travelled (1 s decimated reference path, jump-filtered) and final drift
    g = np.arange(tp[0], tp[-1], 1.0); ii, _ = nearest(g, ref['tp']); P = ref['xyz'][ii, :2]; st = np.hypot(*np.diff(P, axis=0).T); dtt = np.diff(ref['tp'][ii])
    dist = st[(dtt > 0) & (st < 25 * np.maximum(dtt, 1e-3))].sum()
    R.update(dist=dist, end_e2d=d2[-1], end_e3d=d3[-1], drift_pct=100 * d2[-1] / max(dist, 1.0))
    lo = np.where(on)[0]
    if len(lo):
        R['end_onmap_al'] = (se - sr)[lo[-1]]; R['drift_onmap_pct'] = 100 * abs(R['end_onmap_al']) / max(sr[lo[-1]] - sr[lo[0]], 1.0)
    return R

def aggregate(results, keys=('v_rmse', 'v_bias', 'al_mean_abs', 'al_rmse', 'al_max', 'ct_rmse', 'e2d_mean', 'e3d_mean', 'e3d_rmse', 'end_e2d', 'drift_pct')):
    return {k: (float(np.nanmedian([abs(r[k]) for r in results])), float(np.nanmean([abs(r[k]) for r in results]))) for k in keys}
 # b6_baseline.py  (reference causal baseline used for the reported B6 numbers; replays inputs offline; not saved)
# from evaluator import Route, ROUTES, nearest, load, fixes, load_ref, evaluate
import numpy as np
from scipy.spatial import cKDTree
ZOFF = 3.10

def inputs(b, extrap=True, stale=0.5):
    # union of front/rear/cmd stamps; per-bogie stale timeout; linear extrapolation <= 0.2 s; mean of valid bogies
    D = load(b); F = D['/vehicle/front_bogie_velocity']; Rr = D['/vehicle/rear_bogie_velocity']; C = D['/vehicle/driver_position_cmd']
    t = np.sort(np.r_[F['t_hdr'], Rr['t_hdr'], C['t_hdr']], kind='stable'); vs = []
    for S in (F, Rr):
        T = S['t_hdr']; V = S['v']; i = np.searchsorted(T, t, side='right') - 1; ic = np.clip(i, 0, None); v = V[ic].astype(float)
        if extrap:
            ip = np.clip(ic - 1, 0, None); dtp = T[ic] - T[ip]; sl = np.where(dtp > 0.02, (V[ic] - V[ip]) / np.maximum(dtp, 0.02), 0)
            v = np.maximum(v + np.clip(t - T[ic], 0, 0.2) * sl, 0)
        v[(i < 0) | ((t - T[ic]) > stale)] = np.nan; vs.append(v)
    vw = np.nanmean(np.c_[vs[0], vs[1]], axis=1); bad = np.isnan(vw)
    if bad.any():
        idx = np.where(~bad, np.arange(len(vw)), 0); np.maximum.accumulate(idx, out=idx); vw = np.nan_to_num(vw[idx])
    return t, vw

def resample(T, step=1.0):
    seg = np.hypot(*np.diff(T[:, :2], axis=0).T); T = T[np.r_[True, seg > 0.05]]
    cs = np.r_[0, np.cumsum(np.hypot(*np.diff(T[:, :2], axis=0).T))]; g = np.arange(0, cs[-1] + 1e-9, step)
    return np.c_[[np.interp(g, cs, T[:, k]) for k in range(3)]].T

def train(bags, REF, INFO, K_ALL, merge=3.0, minsup=0.2):
    # INFO[b] = dict(pre=xyz before first on-map fix, post=xyz after last on-map fix, xyz, t) from status-2 master fixes
    M = {}
    for veh in ('30618', '30639'): M['k' + veh] = np.median([K_ALL[b] for b in bags if b.startswith(veh)])
    for dirn in ('T2S', 'S2T'):
        runs = [b for b in bags if REF[b]['dir'] == dirn]; pres, posts, stops = [], [], []
        for b in runs:
            I = INFO[b]
            if len(I['pre']) > 20: pres.append(resample(I['pre']))
            if len(I['post']) > 20: posts.append(resample(I['post']))
            t, vw = inputs(b); z = vw * M['k' + b[:5]] < 0.03; idx = np.where(np.diff(np.r_[0, z.astype(int), 0]))[0]
            for a_, e_ in zip(idx[::2], idx[1::2]):
                if t[e_ - 1] - t[a_] < 3: continue
                mm = (I['t'] >= t[a_]) & (I['t'] <= t[e_ - 1])
                if mm.sum() < 5: continue
                sc, dc, _, oc = ROUTES[dirn].project(np.median(I['xyz'][mm][:, :2], axis=0)[None])
                if dc[0] < 5 and not oc[0]: stops.append((b, sc[0]))
        S = np.array([x[1] for x in stops]); Bn = np.array([x[0] for x in stops]); o = np.argsort(S); S = S[o]; Bn = Bn[o]
        br = np.where(np.diff(S) > merge)[0]; cl, sd, sup = [], [], []
        for a_, e_ in zip(np.r_[0, br + 1], np.r_[br + 1, len(S)]):
            su = len(set(Bn[a_:e_])) / len(runs)
            if su >= 0.1:
                c = np.median(S[a_:e_]); cl.append(c); sd.append(1.4826 * np.median(np.abs(S[a_:e_] - c))); sup.append(su)
        # NOTE: recommended upgrade (tested separately by leave-one-out): post = per-arc median consensus of posts instead of the longest
        M[dirn] = dict(pres=pres, post=max(posts, key=len) if posts else None, all=np.array(cl), csd=np.array(sd), sup=np.array(sup), minsup=minsup)
    return M

def estimate(b, M, extrap=True, adapt_scale=True, gmax=25.0, along_shift=0.0):
    t, vw = inputs(b, extrap=extrap); v = vw * M['k' + b[:5]]
    g = fixes(load(b), 'master'); w = g['t'] <= t[0] + 3.0
    if not w.any(): w = np.arange(len(g['t'])) < 30
    w2 = w & (g['st'] == 2); rtk = w2.any(); x0 = np.median(g['xyz'][w2 if rtk else w], axis=0)
    dirn, best = None, 9e9
    for nm, rt in ROUTES.items():
        s_, d_, _, o_ = rt.project(x0[None, :2])
        if d_[0] < 2.5 and not o_[0] and d_[0] < best: dirn, best = nm, d_[0]
    if dirn is None: dirn = min(ROUTES, key=lambda nm: np.hypot(*(ROUTES[nm].P[0, :2] - x0[:2])))
    rt = ROUTES[dirn]; Md = M[dirn]; s0, d0, _, o0 = rt.project(x0[None, :2]); s0 = s0[0]; sig = 3.0 if rtk else 10.0; pre = None
    if d0[0] > 5 or o0[0]:
        cand = []
        for p in Md['pres']:
            dd, ii = cKDTree(p[:, :2]).query(x0[:2]); cand.append((dd, ii - (len(p) - 1), len(cand)))
        cand.sort(); dm = cand[0][0]; near = [c for c in cand if c[0] <= dm + 1.0]
        s0 = np.median([c[1] for c in near]); sig = max(sig, np.std([c[1] for c in near]) + 3.0, dm)
        pre = Md['pres'][min(near, key=lambda c: abs(c[1] - s0))[2]]
    dt = np.r_[0, np.diff(t)]; odo = np.cumsum(np.r_[0, v[:-1]] * dt)
    C = Md['all']; tgt = Md['sup'] >= Md['minsup']; gaps = np.diff(C); ng = np.minimum(np.r_[np.inf, gaps], np.r_[gaps, np.inf])
    z = v < 0.03; idx = np.where(np.diff(np.r_[0, z.astype(int), 0]))[0]; ev = []
    for a_, e_ in zip(idx[::2], idx[1::2]):
        if t[e_ - 1] >= t[a_] + 3.0: ev.append(np.searchsorted(t, t[a_] + 3.0))
    s = np.empty(len(t)); scale = np.ones(len(t)); sa, ja, sc, last, numc, numo, sl = s0, 0, 1.0, None, 0.0, 0.0, s0
    for j in ev + [len(t)]:
        s[ja:j] = sa + sc * (odo[ja:j] - odo[ja]); scale[ja:j] = sc
        if j == len(t): break
        cur = sa + sc * (odo[j] - odo[ja]); sig = np.hypot(sig, 0.004 * abs(cur - sl)); sl = cur
        ci = np.argmin(np.abs(C - cur)); c = C[ci]; gt = min(max(3 * sig, 6.0), 0.45 * ng[ci], gmax)
        if tgt[ci] and abs(c - cur) < gt:
            if last is not None and adapt_scale and odo[j] - last[1] > 300:
                numc += c - last[0]; numo += odo[j] - last[1]; sc = float(np.clip((800 + numc) / (800 + numo), 0.97, 1.03))
            last = (c, odo[j]); sa, ja = c, j; sig = max(Md['csd'][ci], 0.5) + 0.5
        else:
            sa, ja = cur, j
    vout = v * scale; s = s + along_shift
    P = rt.point(s); P[:, 2] += ZOFF; m = s < 0
    if m.any():
        if pre is None: pre = min(Md['pres'], key=lambda p: cKDTree(p[:, :2]).query(x0[:2])[0])
        pl = np.r_[0, np.cumsum(np.hypot(*np.diff(pre[:, :2], axis=0).T))]; q = np.clip(pl[-1] + s[m], 0, pl[-1])
        P[m] = np.c_[[np.interp(q, pl, pre[:, k]) for k in range(3)]].T
    m = s > rt.len
    if m.any() and Md['post'] is not None:
        post = Md['post']; pl = np.r_[0, np.cumsum(np.hypot(*np.diff(post[:, :2], axis=0).T))]; q = np.clip(s[m] - rt.len, 0, pl[-1])
        P[m] = np.c_[[np.interp(q, pl, post[:, k]) for k in range(3)]].T
    return t, vout, P, dirn, s
# CV: fold[b] = index parity within sorted groups keyed by (vehicle, direction) after removing duplicates;
# MODELS[f] = train(bags not in fold f); for each bag: evaluate(b, *estimate(b, MODELS[fold[b]])[:3])

VERDICTS:
  ~ [F01] partially_confirmed: Clamping s to the map is catastrophic, but mostly for a different reason than 'off-map segments'. Clamping s0 to 0 shifts every on-map sample by the full pre-map distance: T2S 135-222 m (median 186 m), S2T 453-628 m (median 599 m). Just unclamping s and extending the map straight along its end tangent still leaves about 100 m of error, because the terminus paths curve: tangent s0 is -85 to -133 m for T2S against a true -135 to -222 m, and about -496 m for S2T against about -600 m. So the off-map arc length and geometry must come from training GNSS tracks (templates, or a lookup from start position to s0). With that fix, wheel-only along-track error on the map is about 1.3-1.5 m median. Off-map samples are also a large share of evaluated time (T2S 22%, S2T 28%), so extended 2D centerlines are needed for the x/y output there as well. The 'b8044aa0: 404 m' figure is an artifact: its true pre-map distance is 186.6 m.
      EVID: Own code, run inline with python -B -c because the session was read-only, so no script files were saved. Setup: 59 unique RTK bags (29 T2S, 30 S2T; 49 from 30618, 10 from 30639), leave-one-out (LOO).

Wheel scale: wheel v is in km/h. The scale k = map arc / (integral of v/3.6) is 1.0003 for vehicle 30618 (p10-p90 0.9995-1.0010, n=49) and 1.0027 for 30639 (range 0.9907-1.0035).

On-map along-track error. Each figure is the median over bags of the per-bag median |e| (the mean over bags is in brackets):
- clamp: 452.8 m [387.2]; per-bag mean |e| has a median of 420.9. Analyst reported 438/472.
- tangent extrapolation, unclamped: 99.7 m [88.9]. s0 errors 19-132 m.
- nearest-template s0: 1.32 m [3.26].
- median over near templates: 1.49 m [3.37].
- oracle s0: 1.43 m [2.50].

Off-map share of GNSS samples (median, p10-p90):
- T2S: 0.216 (0.165-0.276)
- S2T: 0.279 (0.212-0.348)

Post-map distance:
- T2S: 419-512 m (median 510)
- S2T: -63 to 346 m (median 150)

b8044aa0: the tram enters the map at t=85 s (wheel distance 187 m), inside a status-0 stretch from 77.7 to 323.6 s. The first RTK on-map fix is at t=136.9 s, at wheel distance 404 m and s=218. So 404 m is 'distance to the first RTK on-map fix', not the pre-map distance.
  ~ [F02] partially_confirmed: Snapping to learned stop clusters is the largest remaining drift correction once s0 is right. It cuts on-map along-track error about 6x and map-exit error about 12x. It only works with the extended route: clamp offsets of 135-628 m are far beyond any gate (25 m or less) and larger than the spacing between stations. The analyst's absolute numbers are about 2x worse than mine, but the ratios agree. Two caveats: wheel odometry alone (km/h/3.6, k≈1.000) already drifts only about 2.3 m over 4.7 km, and after snapping the mean error is dominated by s0 initialisation errors of 10-35 m, which capped gates cannot remove.
      EVID: LOO, 59 bags. Template s0 = median over templates with d ≤ dmin+1. Per-vehicle k from LOO. Stops = v < 0.05 m/s for at least 3 s.

On-map along-track error, median over bags of per-bag median |e| (mean in brackets):
- no stops: 1.49 m [3.53]. Analyst's B2x: 3.14 [6.16].
- stops, fixed 15 m gate, 8 m merge: 0.29 [1.77].
- B6-like gate, non-chained 3 m clusters: 0.24 [1.71]. Analyst's B6: 0.64 [1.87].

Error at map exit, |e| median (mean):
- no stops: 2.26 m (5.42). Analyst: 5.2.
- with stops: 0.18 m (1.63). Analyst: 0.41.

Bags with median |e| > 5 m after B6-like snapping: 21dd3af3 21.7 m, 92226df0 34.6 m, 253671cc 14.3 m, 4d487b0d 9.2 m. All are caused by s0 or pre-existing wheel error, not by snaps.

Adding a wide gate on the first snap (40 m cap, and at most 0.45 × gap to the neighbour cluster):
- mean per-bag median |e|: 1.71 → 1.02 m
- mean exit |e|: 1.63 → 1.20 m
- 21dd3af3 fixed; 92226df0: 34.6 → 14.8 m

Clamped variant (B3): the true pre-map offsets (T2S 135-222 m, S2T 453-628 m) exceed every gate, and T2S stop clusters at 39/252/665 are about 210+ m apart, so stops cannot recover the offset. Consistent with the analyst's 473.8 m.
  ~ [F03] refuted: I could not reproduce the cited ±45 m failures. For 2050d396 and 9f0b519f the nearest-template s0 is off by less than 1 m in both LOO and 2-fold CV. A parallel track about 4.9-5.0 m away does exist at the Tallinskaya start, but templates on it give the same s0 (-169/-170), not -126. My large s0 errors (10-25 m) come from two other mechanisms, at both termini:
(a) the start lies beyond the coverage of all templates: 21dd3af3 starts about 20 m further back than any template;
(b) the pre-map wheel path length varies for the same start spot: at the S2T/Shchukinskaya end, 4d487b0d and dd8e6395 start within 1 m of each other on the same path, yet their wheel distances to map entry are 628.0 m and 606.5 m.
The median over near templates barely helps with either. Over all off-map starts the median |s0 error| is 0.27-0.42 m, not 3.0-3.6 m. Fix: extrapolate along the template, and use the first on-map stop with a wide gate.
      EVID: 57 off-map starts. |s0 error| summary:
- nearest, LOO: median 0.42 m, mean 2.43, p90 5.2, max 23.5, 4 bags > 10 m.
- median-of-near, LOO: median 0.27, mean 1.89, p90 2.6, max 23.5, 3 bags > 10 m.
- 2-fold (alternating split): median 0.29-0.35, mean 1.85-1.91, max 25.1.
- Averaging the first 3 s of fixes changes nothing.

Cited bags:
- 2050d396: true s0 -170.6; nearest candidates (d, s0) = (0.0, -170), (0.1, -170), (4.9, -170), (5.0, -169). Error +0.6 m.
- 9f0b519f: true -166.6; candidates (0.0, -167), (4.9, -167). Error about -0.4 m.
- 21dd3af3: true -222.2; nearest d = 22.4 m at s0 -198.7. The along-template offset is -17 to -21 m and the lateral offset 12-18 m, so the start is behind template coverage. Error +23.5 m, or 4-6 m with along-track correction. Matches the analyst's +23.2.
- 92226df0: status-0 first fix, 19.8 m from every template. Error +9.7 m (analyst: -44 m).

S2T outliers:
- 4d487b0d +21.6 m, dd8e6395 -20.4 m, 253671cc -18.8 m.
- 4d487b0d vs dd8e6395: start points differ by 1.1 m; their tracks are 0.7 m apart at the median (p90 2.9 m).
- Wheel distance to map entry: 628.0 vs 606.5 m. GNSS path: 614.0 vs 609.3 m.

Possible source of the analyst's -126 candidate: S2T run 27e994fc passes 0.9 m from the 2050d396 start at t=1839 s. If opposite-direction tails are in the template pool, a wrong s0 results.
  ~ [F04] partially_confirmed: The fixed 15 m gate with 8 m chained clusters does produce about 7% wrong snaps (over 5 m), and almost all of them are at real two-position stops 5.5-12 m apart. The 2366c74a 4381→4371 case is confirmed. B6-style gating is safe only if the 3 m clustering does not chain. With single-linkage chaining, S2T 2400.4 and 2405.8 merge through a stop at 2402.8 (cluster sd 2.1 m), and B6 still makes 10 wrong snaps, 7 of them at 2400/2406. The 'sd 0.05-0.8 m' claim holds only for non-chained clusters. I could not reproduce the 42 m pull from 241 to 282 for 76e1f9c7/ab5921a4: whether an uncapped gate fails depends on how fast the σ model grows. The rare 241 cluster itself is real.
      EVID: Stop fine structure, as stop count at the main mode vs the secondary mode (all bags with stops):
- S2T: 2400.0-2401.1 (26) vs 2405.8-2405.9 (6), bridged by 2402.8; 2832.8-2835.5 (28) vs 2838.5-2839.6 (6); 279.8-284.2 (28) vs 288.3-288.7 (3) vs 239.4-241.6 (3).
- T2S: 1816.6-1818.1 (27) vs 1824.0-1824.7 (5); 2247.9-2249.9 (27) vs 2256.8-2257.0 (5); 4370.4-4372.7 (28) vs 4381.1-4383.0 (3); 1256 (24 bags) vs 1264 (3).

Wrong snaps (> 5 m) by variant:
- fixed 15 m gate + 8 m chain merge: 37 of 505 snaps (analyst: 31/404). Includes 2366c74a (target 4371, true 4381.1) and 927002c2 (4371 vs 4382.7/4383.0).
- uncapped adaptive gate (max(3σ, 6), σ = hypot(1, 0.003·d)): 10 of 491 (analyst: 10/391). 76e1f9c7 and ab5921a4 were NOT pulled.
- B6-like with chained 3 m clusters: 10 of 483.
- B6-like with non-chained 3 m clusters: 2 of 472 (28538acf, 4285f2bc; both bags already had large drift).

Cluster sd with 3 m chain merge: T2S major clusters 0.18-0.48 m; S2T 2400 sd 2.08 (range 6.0), 2833 sd 2.31 (range 6.8), 3499 sd 1.03, 365 sd 1.11.

Bags with median |e| > 5 m after B6: 3-5, all caused by s0 errors. The analyst also reported 3.
  ~ [F05] confirmed: Confirmed: stamps must be exactly the input stamps (best shift is 0), and output should be published on the union of input stamps. Corrections:
- Rear-bogie stamps equal front stamps in 99.9% of messages, so the union of unique stamps runs at about 29 Hz (9.3 Hz wheel + 20 Hz cmd), not about 40 Hz.
- Front-only publishing gives only 9.29 Hz median, which by itself breaks the ≥ 10 Hz requirement.
- On cmd stamps between wheel samples, use extrapolation rather than zero-order hold (ZOH).
      EVID: 56 unique RTK bags. Reference is the master/vel horizontal norm at RTK times, matched to the nearest output stamp within 0.05 s.

Median speed RMSE (m/s) vs output stamp shift, publishing on front-bogie stamps:
- -0.3 s: 0.1439
- -0.2 s: 0.0993
- -0.1 s: 0.0576
- -0.05 s: 0.0380
- 0: 0.0305
- +0.05 s: 0.0396
- +0.1 s: 0.0571
- +0.2 s: 0.0991
- +0.3 s: 0.1442

Best shift per bag: 0 in 50 of 56 bags, ±0.05 s in the other 6. A fine lag search at 0.02 s steps gives 0.000 s at p10, p50 and p90.

Publishing on union stamps, RMSE at shifts -0.1 / 0 / +0.1 s:
- ZOH: 0.045 / 0.0398 / 0.0761 (analyst: 0.044 / 0.039 / 0.076)
- linear extrapolation: 0.0599 / 0.0316 / 0.0582

Match coverage within 0.05 s:
- front only: min 0.831, median 0.889, max 0.980
- front+rear: identical to front only
- front+rear+cmd: 0.9996-1.0000

Rates: front median 9.29 Hz (dt 0.1000-0.1020 s), cmd 20.0 Hz, union 29.29 Hz. Cmd has gaps of up to 1.05 s in some 30618 bags, so do not publish on cmd alone.
  ~ [F07] confirmed: Confirmed: long dropouts on vehicle 30639 need a per-bogie stale timeout of about 0.5 s. Additions:
- The front bogie can also drop out: 19.8 s on 30639_c31df386. The timeout must apply to both bogies, and when both are stale the output must fall back to the model.
- Several 30618 bags have short simultaneous front+rear gaps of 0.9-1.3 s (up to 8 per bag).
- Over all bags the median RMSE does not change (0.0305 → 0.0305); the gain is in the mean and in the 4-5 affected bags.
      EVID: Largest rear-bogie gap per bag (s):
- 3b3d9eb8: 73.50 (2 gaps, 103.9 s total, no RTK)
- 4285f2bc: 47.11
- d927f360: 25.58 (37.7 s total)
- 927002c2: 20.50 (29.2 s total)
- 44226bde: 16.70
- 584b6e32: 16.30
- 9f0b519f: 5.80

Front-bogie gap: c31df386 19.81 s.

On vehicle 30618, 2255aade has 8 gaps of more than 0.5 s (max 1.19 s) and 40ffd323 has 7 (max 1.29 s). These gaps hit both bogies at once, and cmd gaps reach up to 1.05 s.

Speed RMSE (m/s), naive hold → 0.5 s timeout → front only:
- 927002c2: 0.430 → 0.039 → 0.039 (analyst: 0.461 → 0.033)
- d927f360: 0.719 → 0.037 → 0.036 (analyst: 0.706 → 0.037)
- 584b6e32: 0.195 → 0.037 → 0.038 (analyst: 0.205 → 0.035)
- 9f0b519f: 0.154 → 0.037 → 0.038 (analyst: 0.147 → 0.034)

All 56 RTK bags: median 0.0305 / 0.0305 / 0.0312; mean 0.0780 / 0.0539 / 0.0538 (analyst: median 0.040 → 0.032, mean 0.085 → 0.056).

Wheel speed is never negative (min 0.00), so reversing cannot be detected from the wheels.