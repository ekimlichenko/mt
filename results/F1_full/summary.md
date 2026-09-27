# F1_full

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 1.33 | 1.56 | 2.53 | 6.99 |
| v_rmse | 0.03 | 0.0516 | 0.0899 | 0.637 |
| v_mae | 0.0207 | 0.0255 | 0.0319 | 0.114 |
| v_bias | 0.00279 (-0.00176) | 0.00664 | 0.00838 | 0.0716 |
| v_bias_trans | 0.0121 | 0.0234 | 0.0355 | 0.422 |
| v_p99 | 0.0931 | 0.204 | 0.179 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 5.48 | 7.09 | 8.55 | 72.8 |
| e3d_mean | 5.63 | 7.19 | 8.73 | 73 |
| e3d_max | 35.6 | 67.8 | 115 | 272 |
| al_mean | 1.93 (0.508) | 2.41 | 4.65 | 14.2 |
| al_mean_abs | 3.3 | 3.75 | 5.63 | 14.2 |
| al_rmse | 4.02 | 4.63 | 6.72 | 15.1 |
| al_max | 10.9 | 13.8 | 31 | 67.9 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 11.3 | 14.8 | 24.9 | 106 |
| drift_pct | 0.365 | 1.11 | 2.17 | 4.82 |
| drift_onmap_pct | 0.0731 | 0.123 | 0.19 | 2.13 |
| al_growth_pct | 0.0757 (0.0378) | 0.121 | 0.231 | 0.677 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 1.88 | 0.03 | 0.0123 | 3.22 | 4.02 | 13.5 | 6.3 | 1.88 |
| T2S | 30 | 0.913 | 0.0298 | 0.011 | 3.56 | 3.98 | 8.16 | 4.56 | 0.273 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 1.34 | 0.0292 | 0.0121 | 3.16 | 3.92 | 10.7 | 5.5 | 0.334 |
| 30639 | 11 | 1.28 | 0.0308 | 0.016 | 3.46 | 4.21 | 11.4 | 6.89 | 0.425 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 1.33 | 0.0283 | 0.0121 | 3.07 | 4.02 | 10.9 | 5.54 | 0.297 |
| 30618_2026-08-10 | 9 | 1.45 | 0.0401 | 0.0151 | 3.13 | 3.63 | 9.53 | 5.63 | 0.363 |
| 30618_2026-08-26 | 12 | 0.913 | 0.0282 | 0.0106 | 3.21 | 3.7 | 10.5 | 4.96 | 0.328 |
| 30618_2026-09-03 | 2 | 2.34 | 0.0513 | 0.0501 | 5.78 | 6.61 | 16.9 | 39.4 | 2.51 |
| 30639_2026-05-05 | 3 | 1.81 | 0.0722 | 0.0692 | 4.52 | 5.53 | 9.71 | 8.65 | 0.283 |
| 30639_2026-08-26 | 8 | 1.2 | 0.0303 | 0.0115 | 3.36 | 4.17 | 12.5 | 6.37 | 0.79 |

## Against rover reference (medians)

e3d_mean 14; al_mean 11.9; al_mean_abs 12.3; al_rmse 12.9;

## Against mid reference (medians)

e3d_mean 7.84; al_mean 6.07; al_mean_abs 6.52; al_rmse 7.38;

## Worst bags

- **score_loss**: 30618_4d487b0d 6.99, 30618_27e994fc 3.27, 30618_28538acf 3.08, 30639_92226df0 3.06, 30618_40ffd323 2.76, 30618_49fe4c54 2.55, 30618_0652866c 2.53, 30639_50956d6e 2.52
- **al_rmse**: 30639_92226df0 15.1, 30618_4d487b0d 12.4, 30618_28538acf 9.6, 30618_f19a4ac3 8.66, 30618_616ec56b 7.47, 30639_927002c2 6.74, 30618_27e994fc 6.72, 30618_68d1748a 6.64
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.126, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0952, 30639_92226df0 0.0893, 30639_253671cc 0.0722
- **drift_pct**: 30618_27e994fc 4.82, 30618_0652866c 4.52, 30639_0be558e2 3.88, 30618_49fe4c54 3.53, 30618_8158f0b0 3.39, 30618_4d487b0d 2.17, 30618_a869780d 2.17, 30639_50956d6e 2.15
- **e2d_offmap_mean**: 30618_27e994fc 106, 30618_0652866c 41.7, 30618_4d487b0d 33, 30618_40ffd323 27.7, 30618_8158f0b0 27, 30639_50956d6e 25.1, 30639_4285f2bc 24.8, 30618_49fe4c54 19.8

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 20.6, min 9.52, max 331
- lat_ms_p99: median 0.224, min 0.0202, max 1.69
- lat_ms_max: median 3.11, min 0.176, max 27.7
- pub_rate_hz: median 29.4, min 28.1, max 31.9
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.743, max 1
- max_step_m: median 0.907, min 0, max 25.3
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 110
- total_msgs: 5129019
