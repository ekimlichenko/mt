# AF_noinflate

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {'wheel_pair_sigma_inflate': 'false'}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 0.624 | 0.948 | 1.86 | 5.54 |
| v_rmse | 0.0298 | 0.0514 | 0.0825 | 0.637 |
| v_mae | 0.0208 | 0.0253 | 0.0316 | 0.114 |
| v_bias | 0.00378 (-0.00289) | 0.00694 | 0.011 | 0.0716 |
| v_bias_trans | 0.012 | 0.023 | 0.0352 | 0.422 |
| v_p99 | 0.0944 | 0.204 | 0.163 | 3.97 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 1.59 | 2.68 | 4.11 | 16.8 |
| e3d_mean | 1.63 | 2.79 | 4.27 | 17.6 |
| e3d_max | 11.1 | 16.9 | 40.3 | 86.7 |
| al_mean | 1.17 (0.876) | 2.05 | 3.89 | 15.4 |
| al_mean_abs | 1.49 | 2.46 | 4.27 | 15.6 |
| al_rmse | 1.96 | 3.14 | 7.76 | 17.6 |
| al_max | 8.13 | 11.9 | 23.7 | 68.4 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 1.32 | 2.78 | 4.86 | 23 |
| drift_pct | 0.0455 | 0.127 | 0.365 | 1.05 |
| drift_onmap_pct | 0.0457 | 0.177 | 0.128 | 6.38 |
| al_growth_pct | 0.0476 (0.0138) | 0.0895 | 0.199 | 1.03 |
| p_in95 | 0.995 | 0.947 | 1 | 1 |
| p_nees | 0.15 | 1.55 | 3.03 | 23.9 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 0.917 | 0.0304 | 0.0124 | 1.82 | 2.51 | 13.1 | 1.92 | 0.0567 |
| T2S | 30 | 0.52 | 0.0288 | 0.0102 | 1.33 | 1.61 | 4.27 | 1.47 | 0.0437 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 0.621 | 0.0295 | 0.0121 | 1.43 | 1.73 | 6.98 | 1.48 | 0.0332 |
| 30639 | 11 | 1.26 | 0.0305 | 0.0119 | 1.67 | 2.41 | 9.74 | 2.17 | 0.113 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 0.657 | 0.0285 | 0.0122 | 1.55 | 1.9 | 6.88 | 1.71 | 0.036 |
| 30618_2026-08-10 | 9 | 0.689 | 0.0403 | 0.0149 | 1.43 | 1.68 | 6.39 | 1.43 | 0.0261 |
| 30618_2026-08-26 | 12 | 0.551 | 0.0287 | 0.0106 | 1.26 | 1.61 | 8.94 | 1.31 | 0.0338 |
| 30618_2026-09-03 | 2 | 1.97 | 0.0512 | 0.05 | 9.19 | 10.2 | 16.5 | 11.1 | 0.29 |
| 30639_2026-05-05 | 3 | 2.22 | 0.064 | 0.0609 | 12.9 | 12.9 | 15.1 | 16.7 | 0.387 |
| 30639_2026-08-26 | 8 | 0.599 | 0.0291 | 0.0104 | 1.58 | 2.1 | 8.06 | 2.02 | 0.0923 |

## Worst bags

- **score_loss**: 30618_4d487b0d 5.54, 30639_92226df0 3.17, 30639_9c362687 2.22, 30618_27e994fc 2.11, 30639_253671cc 2.09, 30618_28538acf 2.08, 30618_88548b02 1.83, 30618_40ffd323 1.59
- **al_rmse**: 30639_92226df0 17.6, 30639_9c362687 12.9, 30618_88548b02 10.4, 30618_27e994fc 9.94, 30618_28538acf 8.96, 30639_253671cc 8.92, 30618_4d487b0d 7.63, 30618_616ec56b 6.74
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.128, 30618_28538acf 0.125, 30618_2dbce472 0.116, 30618_2366c74a 0.095, 30639_92226df0 0.0811, 30618_748832b9 0.0658
- **drift_pct**: 30639_0be558e2 1.05, 30618_49fe4c54 1.03, 30618_8158f0b0 0.972, 30639_92226df0 0.489, 30618_a869780d 0.464, 30639_253671cc 0.387, 30639_9c362687 0.362, 30618_27e994fc 0.323
- **e2d_offmap_mean**: 30639_92226df0 23, 30639_9c362687 18.4, 30618_27e994fc 14.6, 30618_88548b02 6.69, 30618_68d1748a 5.08, 30618_8158f0b0 4.96, 30639_4285f2bc 4.83, 30618_4d487b0d 4.38

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 26.2, min 10.6, max 294
- lat_ms_p99: median 0.224, min 0.0243, max 1.94
- lat_ms_max: median 4.16, min 0.221, max 29.5
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.759, min 0, max 24.7
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 134
- total_msgs: 5129019
