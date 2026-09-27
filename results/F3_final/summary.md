# F3_final

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 1.25 | 1.49 | 2.51 | 7.09 |
| v_rmse | 0.0295 | 0.0511 | 0.0826 | 0.637 |
| v_mae | 0.0206 | 0.0251 | 0.0315 | 0.114 |
| v_bias | 0.00367 (-0.00277) | 0.00692 | 0.0109 | 0.0716 |
| v_bias_trans | 0.0121 | 0.0229 | 0.0355 | 0.422 |
| v_p99 | 0.0929 | 0.203 | 0.164 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 4.67 | 6.64 | 10.1 | 72.3 |
| e3d_mean | 4.74 | 6.73 | 10.1 | 72.5 |
| e3d_max | 34.6 | 67.3 | 115 | 267 |
| al_mean | 1.32 (0.0106) | 2.24 | 6.22 | 14.4 |
| al_mean_abs | 2.42 | 3.28 | 6.71 | 14.4 |
| al_rmse | 3.03 | 4.03 | 8.13 | 15.9 |
| al_max | 9.51 | 12.7 | 30.1 | 70.3 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 11.2 | 14.5 | 24.6 | 104 |
| drift_pct | 0.317 | 1.1 | 2.16 | 4.73 |
| drift_onmap_pct | 0.0525 | 0.146 | 0.143 | 4.56 |
| al_growth_pct | 0.0704 (-0.000803) | 0.113 | 0.222 | 0.93 |
| p_in95 | 0.998 | 0.956 | 1 | 1 |
| p_nees | 0.245 | 1.05 | 1.6 | 23.6 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 1.85 | 0.0298 | 0.0124 | 2.82 | 3.44 | 12.9 | 5.89 | 1.87 |
| T2S | 30 | 0.746 | 0.0286 | 0.01 | 2.02 | 2.51 | 5.97 | 3.53 | 0.255 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 1.24 | 0.0292 | 0.0121 | 2.41 | 2.97 | 8.95 | 4.73 | 0.301 |
| 30639 | 11 | 1.76 | 0.0305 | 0.0119 | 2.96 | 3.81 | 9.79 | 7.86 | 0.33 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 1.24 | 0.0283 | 0.0121 | 2.4 | 2.91 | 8.06 | 4.8 | 0.289 |
| 30618_2026-08-10 | 9 | 1.24 | 0.0401 | 0.0151 | 2.03 | 2.47 | 8.46 | 4.73 | 0.323 |
| 30618_2026-08-26 | 12 | 0.84 | 0.0282 | 0.0106 | 2.35 | 3.02 | 10.7 | 4.62 | 0.308 |
| 30618_2026-09-03 | 2 | 2.49 | 0.0513 | 0.0501 | 7.53 | 8.43 | 15.3 | 39.8 | 2.48 |
| 30639_2026-05-05 | 3 | 2.24 | 0.0639 | 0.0603 | 9.67 | 10.2 | 16.9 | 13 | 0.33 |
| 30639_2026-08-26 | 8 | 0.942 | 0.029 | 0.0102 | 2.11 | 2.7 | 8.51 | 4 | 0.725 |

## Against rover reference (medians)

e3d_mean 14.2; al_mean 12.6; al_mean_abs 12.7; al_rmse 12.9;

## Against mid reference (medians)

e3d_mean 8.1; al_mean 6.36; al_mean_abs 6.56; al_rmse 7;

## Worst bags

- **score_loss**: 30618_4d487b0d 7.09, 30618_27e994fc 3.41, 30618_28538acf 3.09, 30639_92226df0 3.07, 30618_40ffd323 2.71, 30639_4285f2bc 2.52, 30639_50956d6e 2.51, 30618_0652866c 2.5
- **al_rmse**: 30639_92226df0 15.9, 30618_4d487b0d 13.4, 30639_253671cc 10.2, 30618_28538acf 9.37, 30639_9c362687 8.84, 30618_27e994fc 8.8, 30618_88548b02 8.05, 30618_f19a4ac3 7.89
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.126, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0952, 30639_92226df0 0.0812, 30618_748832b9 0.0656
- **drift_pct**: 30618_27e994fc 4.73, 30618_0652866c 4.51, 30639_0be558e2 3.84, 30618_49fe4c54 3.52, 30618_8158f0b0 3.41, 30618_a869780d 2.17, 30618_4d487b0d 2.16, 30639_50956d6e 2.12
- **e2d_offmap_mean**: 30618_27e994fc 104, 30618_0652866c 41.5, 30618_4d487b0d 32.8, 30618_40ffd323 27.6, 30618_8158f0b0 27, 30639_4285f2bc 25.8, 30639_50956d6e 24.3, 30639_92226df0 22.2

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 22.7, min 10.2, max 414
- lat_ms_p99: median 0.233, min 0.0212, max 1.87
- lat_ms_max: median 4.32, min 0.114, max 39
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.886, min 0, max 25.3
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 119
- total_msgs: 5129019
