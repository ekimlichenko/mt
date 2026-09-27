# _deliv_novid

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {'vehicle_id': 'unknown'}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 1.29 | 1.51 | 2.71 | 7.16 |
| v_rmse | 0.0305 | 0.0513 | 0.0754 | 0.638 |
| v_mae | 0.0212 | 0.0253 | 0.0301 | 0.114 |
| v_bias | 0.00319 (-0.00061) | 0.00705 | 0.0164 | 0.0746 |
| v_bias_trans | 0.0143 | 0.0246 | 0.036 | 0.422 |
| v_p99 | 0.0944 | 0.204 | 0.161 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 4.84 | 6.7 | 10.2 | 72.3 |
| e3d_mean | 5.05 | 6.79 | 10.3 | 72.5 |
| e3d_max | 35 | 67.5 | 116 | 268 |
| al_mean | 1.33 (0.437) | 2.38 | 6.29 | 12.6 |
| al_mean_abs | 2.61 | 3.34 | 6.32 | 12.7 |
| al_rmse | 3.19 | 4.08 | 7.89 | 14.1 |
| al_max | 9.13 | 12.9 | 31 | 72.6 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 11.2 | 14.6 | 25 | 104 |
| drift_pct | 0.313 | 1.1 | 2.17 | 4.75 |
| drift_onmap_pct | 0.0594 | 0.136 | 0.166 | 3.51 |
| al_growth_pct | 0.0648 (0.0147) | 0.112 | 0.24 | 0.82 |
| p_in95 | 0.997 | 0.958 | 1 | 1 |
| p_nees | 0.281 | 1.04 | 1.58 | 23.1 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 1.9 | 0.0309 | 0.0145 | 2.91 | 3.67 | 13.2 | 6.36 | 1.89 |
| T2S | 30 | 0.75 | 0.0295 | 0.0139 | 2.07 | 2.58 | 6.38 | 3.4 | 0.265 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 1.26 | 0.0294 | 0.0131 | 2.61 | 3.18 | 9.61 | 4.92 | 0.318 |
| 30639 | 11 | 1.42 | 0.0328 | 0.0191 | 4.53 | 5.42 | 8.29 | 8.91 | 0.286 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 1.29 | 0.0292 | 0.0131 | 2.64 | 2.97 | 8.09 | 5.19 | 0.297 |
| 30618_2026-08-10 | 9 | 1.26 | 0.0401 | 0.0165 | 1.92 | 2.22 | 7.87 | 4.86 | 0.342 |
| 30618_2026-08-26 | 12 | 0.918 | 0.0285 | 0.0113 | 2.55 | 3.22 | 11.6 | 4.59 | 0.315 |
| 30618_2026-09-03 | 2 | 2.38 | 0.0497 | 0.0471 | 6.82 | 7.66 | 15.2 | 39.5 | 2.49 |
| 30639_2026-05-05 | 3 | 1.93 | 0.0558 | 0.0515 | 8.05 | 8.71 | 15.8 | 10.9 | 0.262 |
| 30639_2026-08-26 | 8 | 0.993 | 0.0304 | 0.0161 | 2.23 | 2.75 | 7.56 | 4.07 | 0.693 |

## Worst bags

- **score_loss**: 30618_4d487b0d 7.16, 30618_27e994fc 3.32, 30618_28538acf 3.05, 30639_0be558e2 2.77, 30639_92226df0 2.77, 30639_4285f2bc 2.75, 30639_50956d6e 2.71, 30618_40ffd323 2.68
- **al_rmse**: 30639_92226df0 14.1, 30618_4d487b0d 13.9, 30618_28538acf 9.27, 30639_253671cc 8.71, 30618_f19a4ac3 8.57, 30618_27e994fc 8.01, 30639_4285f2bc 7.88, 30618_88548b02 7.31
- **v_rmse**: 30618_4d487b0d 0.638, 30618_40ffd323 0.18, 30639_50956d6e 0.127, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0953, 30639_92226df0 0.0732, 30618_748832b9 0.0657
- **drift_pct**: 30618_27e994fc 4.75, 30618_0652866c 4.53, 30639_0be558e2 3.8, 30618_49fe4c54 3.54, 30618_8158f0b0 3.42, 30618_a869780d 2.18, 30618_4d487b0d 2.17, 30639_50956d6e 2.09
- **e2d_offmap_mean**: 30618_27e994fc 104, 30618_0652866c 41.7, 30618_4d487b0d 33.1, 30618_40ffd323 27.7, 30639_4285f2bc 27.6, 30618_8158f0b0 27, 30639_50956d6e 24.5, 30639_92226df0 20.7

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 26.7, min 12.3, max 435
- lat_ms_p99: median 0.25, min 0.0242, max 1.69
- lat_ms_max: median 5.18, min 0.316, max 54.1
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.886, min 0, max 25.3
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 140
- total_msgs: 5129019
