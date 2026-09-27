# AF_noloops

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {'loops_enable': 'false'}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 1.21 | 1.49 | 2.53 | 7.42 |
| v_rmse | 0.0295 | 0.0511 | 0.0826 | 0.637 |
| v_mae | 0.0206 | 0.0251 | 0.0315 | 0.114 |
| v_bias | 0.00367 (-0.00277) | 0.00692 | 0.0109 | 0.0716 |
| v_bias_trans | 0.0121 | 0.0229 | 0.0355 | 0.422 |
| v_p99 | 0.0929 | 0.203 | 0.164 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 4.53 | 6.7 | 9.96 | 72.3 |
| e3d_mean | 4.66 | 6.79 | 9.97 | 72.4 |
| e3d_max | 34.8 | 67.4 | 115 | 265 |
| al_mean | 1.41 (0.376) | 2.72 | 7.07 | 15.4 |
| al_mean_abs | 2.16 | 3.31 | 7.17 | 15.6 |
| al_rmse | 2.48 | 3.99 | 8.62 | 17.6 |
| al_max | 9.17 | 12.4 | 30.4 | 70.4 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 11.3 | 14.7 | 24.9 | 103 |
| drift_pct | 0.324 | 1.11 | 2.16 | 4.69 |
| drift_onmap_pct | 0.0516 | 0.184 | 0.134 | 6.41 |
| al_growth_pct | 0.0603 (0.00881) | 0.104 | 0.178 | 1.04 |
| p_in95 | 0.922 | 0.904 | 1 | 1 |
| p_nees | 2.31 | 5.84 | 13.8 | 23.4 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 1.85 | 0.0298 | 0.0124 | 2.53 | 3.38 | 12.7 | 6.01 | 1.87 |
| T2S | 30 | 0.685 | 0.0286 | 0.01 | 1.47 | 2.04 | 4.94 | 3.08 | 0.261 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 1.2 | 0.0292 | 0.0121 | 2.15 | 2.45 | 8.72 | 4.64 | 0.301 |
| 30639 | 11 | 2.22 | 0.0305 | 0.0119 | 2.57 | 3.25 | 9.18 | 7.54 | 0.488 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 1.21 | 0.0283 | 0.0121 | 1.71 | 2.15 | 7.57 | 4.43 | 0.297 |
| 30618_2026-08-10 | 9 | 1.19 | 0.0401 | 0.0151 | 1.53 | 1.87 | 7.28 | 4.68 | 0.332 |
| 30618_2026-08-26 | 12 | 1.02 | 0.0282 | 0.0106 | 2.54 | 3.16 | 10.8 | 4.54 | 0.309 |
| 30618_2026-09-03 | 2 | 2.59 | 0.0513 | 0.0501 | 8.51 | 9.5 | 15.8 | 40 | 2.47 |
| 30639_2026-05-05 | 3 | 2.7 | 0.0639 | 0.0603 | 14.2 | 14.2 | 16.8 | 16.8 | 0.488 |
| 30639_2026-08-26 | 8 | 0.903 | 0.029 | 0.0102 | 1.74 | 2.39 | 8.03 | 3.7 | 0.721 |

## Worst bags

- **score_loss**: 30618_4d487b0d 7.42, 30618_27e994fc 3.55, 30639_92226df0 3.26, 30618_28538acf 3.06, 30639_253671cc 2.7, 30618_40ffd323 2.63, 30639_4285f2bc 2.51, 30618_0652866c 2.47
- **al_rmse**: 30639_92226df0 17.6, 30618_4d487b0d 15.6, 30639_253671cc 14.2, 30639_9c362687 13, 30618_27e994fc 10.4, 30618_28538acf 9.11, 30618_88548b02 8.56, 30618_f19a4ac3 8.09
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.126, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0952, 30639_92226df0 0.0812, 30618_748832b9 0.0656
- **drift_pct**: 30618_27e994fc 4.69, 30618_0652866c 4.51, 30639_0be558e2 3.84, 30618_49fe4c54 3.52, 30618_8158f0b0 3.41, 30618_4d487b0d 2.19, 30618_a869780d 2.16, 30639_50956d6e 2.11
- **e2d_offmap_mean**: 30618_27e994fc 103, 30618_0652866c 41.5, 30618_4d487b0d 33.4, 30618_40ffd323 27.6, 30618_8158f0b0 27, 30639_4285f2bc 25.6, 30639_92226df0 24.7, 30639_50956d6e 24.3

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 29, min 13.3, max 420
- lat_ms_p99: median 0.25, min 0.0273, max 1.61
- lat_ms_max: median 4.78, min 0.713, max 36.7
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.816, min 0, max 25.3
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 148
- total_msgs: 5129019
