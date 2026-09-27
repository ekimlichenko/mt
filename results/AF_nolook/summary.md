# AF_nolook

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {'model_lookahead': 'false'}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 0.623 | 0.944 | 1.86 | 5.53 |
| v_rmse | 0.0295 | 0.0512 | 0.0826 | 0.637 |
| v_mae | 0.0206 | 0.0251 | 0.0315 | 0.114 |
| v_bias | 0.0037 (-0.0028) | 0.00692 | 0.0109 | 0.0716 |
| v_bias_trans | 0.0121 | 0.0229 | 0.0355 | 0.422 |
| v_p99 | 0.0931 | 0.203 | 0.164 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 1.63 | 2.67 | 4.06 | 16.7 |
| e3d_mean | 1.63 | 2.78 | 4.22 | 17.6 |
| e3d_max | 11 | 16.9 | 40.2 | 86.7 |
| al_mean | 1.1 (0.784) | 2.03 | 3.85 | 15.3 |
| al_mean_abs | 1.51 | 2.45 | 4.35 | 15.5 |
| al_rmse | 1.91 | 3.13 | 7.73 | 17.5 |
| al_max | 8.02 | 11.9 | 23.7 | 68.5 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 1.41 | 2.77 | 5.02 | 23 |
| drift_pct | 0.0516 | 0.126 | 0.365 | 1.05 |
| drift_onmap_pct | 0.0513 | 0.176 | 0.126 | 6.36 |
| al_growth_pct | 0.0455 (0.00945) | 0.09 | 0.213 | 1.05 |
| p_in95 | 0.995 | 0.947 | 1 | 1 |
| p_nees | 0.153 | 1.55 | 3.02 | 23.9 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 0.906 | 0.0298 | 0.0124 | 1.76 | 2.43 | 13 | 1.93 | 0.0521 |
| T2S | 30 | 0.524 | 0.0287 | 0.01 | 1.26 | 1.67 | 4.17 | 1.48 | 0.0458 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 0.621 | 0.0292 | 0.0121 | 1.45 | 1.76 | 6.85 | 1.46 | 0.0302 |
| 30639 | 11 | 1.26 | 0.0306 | 0.0119 | 1.66 | 2.44 | 9.66 | 2.21 | 0.118 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 0.646 | 0.0284 | 0.0121 | 1.59 | 1.85 | 6.74 | 1.67 | 0.0343 |
| 30618_2026-08-10 | 9 | 0.696 | 0.0401 | 0.0151 | 1.45 | 1.76 | 6.6 | 1.46 | 0.0202 |
| 30618_2026-08-26 | 12 | 0.546 | 0.0282 | 0.0106 | 1.15 | 1.54 | 8.87 | 1.2 | 0.0271 |
| 30618_2026-09-03 | 2 | 1.98 | 0.0513 | 0.0502 | 9.28 | 10.3 | 16.8 | 11.2 | 0.291 |
| 30639_2026-05-05 | 3 | 2.21 | 0.064 | 0.0605 | 12.8 | 12.9 | 15 | 16.7 | 0.384 |
| 30639_2026-08-26 | 8 | 0.606 | 0.0291 | 0.0102 | 1.56 | 2.06 | 8.05 | 2.01 | 0.0937 |

## Worst bags

- **score_loss**: 30618_4d487b0d 5.53, 30639_92226df0 3.16, 30639_9c362687 2.21, 30618_27e994fc 2.13, 30618_28538acf 2.09, 30639_253671cc 2.08, 30618_88548b02 1.84, 30618_40ffd323 1.59
- **al_rmse**: 30639_92226df0 17.5, 30639_9c362687 12.9, 30618_88548b02 10.5, 30618_27e994fc 10.1, 30618_28538acf 8.97, 30639_253671cc 8.83, 30618_4d487b0d 7.61, 30618_616ec56b 6.82
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.126, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0952, 30639_92226df0 0.0812, 30618_748832b9 0.0656
- **drift_pct**: 30639_0be558e2 1.05, 30618_49fe4c54 1.03, 30618_8158f0b0 0.969, 30639_92226df0 0.488, 30618_a869780d 0.459, 30639_253671cc 0.384, 30639_9c362687 0.363, 30618_27e994fc 0.329
- **e2d_offmap_mean**: 30639_92226df0 23, 30639_9c362687 18.4, 30618_27e994fc 14.9, 30618_88548b02 6.58, 30639_4285f2bc 5.12, 30618_68d1748a 5.03, 30618_8158f0b0 5.02, 30618_4d487b0d 4.35

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 28.7, min 10.4, max 339
- lat_ms_p99: median 0.258, min 0.0223, max 1.86
- lat_ms_max: median 11, min 0.227, max 468
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.76, min 0, max 24.7
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 153
- total_msgs: 5129019
