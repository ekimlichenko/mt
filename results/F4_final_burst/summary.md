# F4_final_burst

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 30.0 s + 1.5 s bursts every 120 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 0.566 | 0.815 | 1.41 | 5.05 |
| v_rmse | 0.0295 | 0.0511 | 0.0826 | 0.637 |
| v_mae | 0.0206 | 0.0251 | 0.0315 | 0.114 |
| v_bias | 0.00366 (-0.00278) | 0.00692 | 0.0109 | 0.0716 |
| v_bias_trans | 0.0121 | 0.0229 | 0.0355 | 0.422 |
| v_p99 | 0.0927 | 0.203 | 0.164 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 0.671 | 1.32 | 2.41 | 12.2 |
| e3d_mean | 0.82 | 1.47 | 3.08 | 12.3 |
| e3d_max | 9.73 | 16.6 | 52.2 | 85.4 |
| al_mean | 0.221 (0.192) | 0.76 | 1.7 | 10.9 |
| al_mean_abs | 0.484 | 1.17 | 2.29 | 10.9 |
| al_rmse | 0.834 | 1.97 | 3.71 | 17.5 |
| al_max | 7.22 | 10.5 | 22 | 66.8 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 0.617 | 1.43 | 3.53 | 12.7 |
| drift_pct | 0.0125 | 0.121 | 0.53 | 1.12 |
| drift_onmap_pct | 0.00604 | 0.023 | 0.0401 | 0.543 |
| al_growth_pct | 0.00907 (-0.000667) | 0.114 | 0.119 | 4.82 |
| p_in95 | 0.98 | 0.929 | 1 | 1 |
| p_nees | 0.431 | 4.69 | 9.17 | 89.3 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 0.804 | 0.0298 | 0.0124 | 0.695 | 1.34 | 11 | 1.13 | 0.0167 |
| T2S | 30 | 0.367 | 0.0286 | 0.0101 | 0.382 | 0.501 | 1.88 | 0.448 | 0.00907 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 0.548 | 0.0292 | 0.0121 | 0.506 | 0.82 | 6.88 | 0.772 | 0.00845 |
| 30639 | 11 | 0.767 | 0.0305 | 0.0119 | 0.403 | 0.928 | 7.6 | 1.38 | 0.0256 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 0.573 | 0.0283 | 0.0121 | 0.551 | 0.89 | 6.34 | 0.98 | 0.00815 |
| 30618_2026-08-10 | 9 | 0.598 | 0.0401 | 0.0151 | 0.405 | 0.645 | 4.64 | 0.772 | 0.0133 |
| 30618_2026-08-26 | 12 | 0.405 | 0.0282 | 0.0106 | 0.387 | 0.552 | 7.04 | 0.445 | 0.00997 |
| 30618_2026-09-03 | 2 | 1.39 | 0.0513 | 0.0502 | 1.98 | 3.49 | 33.6 | 2.53 | 0.0144 |
| 30639_2026-05-05 | 3 | 1.41 | 0.0639 | 0.0603 | 3.18 | 3.93 | 9.15 | 4.16 | 0.0401 |
| 30639_2026-08-26 | 8 | 0.39 | 0.029 | 0.0102 | 0.362 | 0.575 | 6.14 | 0.547 | 0.0197 |

## Against rover reference (medians)

e3d_mean 12.6; al_mean 12.4; al_mean_abs 12.4; al_rmse 12.5;

## Against mid reference (medians)

e3d_mean 6.29; al_mean 6.11; al_mean_abs 6.15; al_rmse 6.25;

## Worst bags

- **score_loss**: 30618_4d487b0d 5.05, 30618_28538acf 2.7, 30639_4285f2bc 2.42, 30639_9c362687 1.91, 30618_27e994fc 1.86, 30618_0652866c 1.47, 30639_253671cc 1.41, 30639_92226df0 1.37
- **al_rmse**: 30639_4285f2bc 17.5, 30618_28538acf 13.9, 30639_9c362687 11.7, 30618_616ec56b 7.14, 30618_27e994fc 5.5, 30639_253671cc 3.93, 30639_0be558e2 3.69, 30618_87afe526 3.55
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.126, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0952, 30639_92226df0 0.0812, 30618_748832b9 0.0656
- **drift_pct**: 30618_0652866c 1.12, 30618_49fe4c54 0.998, 30618_8158f0b0 0.988, 30639_4285f2bc 0.723, 30618_28538acf 0.557, 30618_dd8d0741 0.534, 30618_ab5921a4 0.53, 30618_a869780d 0.461
- **e2d_offmap_mean**: 30639_9c362687 12.7, 30618_0652866c 6.9, 30639_4285f2bc 5.23, 30618_8158f0b0 4.75, 30639_92226df0 4.12, 30639_253671cc 3.85, 30618_28538acf 3.45, 30618_68d1748a 3.36

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 25.6, min 11.2, max 345
- lat_ms_p99: median 0.229, min 0.0248, max 1.85
- lat_ms_max: median 4.48, min 0.385, max 37.8
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 1.11, min 0, max 54.2
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 132
- total_msgs: 5129019
