# AF_nopairs

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {'wheel_align_other': 'false'}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 0.651 | 1 | 2.07 | 5.59 |
| v_rmse | 0.0301 | 0.0581 | 0.104 | 0.639 |
| v_mae | 0.0212 | 0.0259 | 0.033 | 0.116 |
| v_bias | 0.00322 (-0.00186) | 0.00684 | 0.0105 | 0.0734 |
| v_bias_trans | 0.0123 | 0.0241 | 0.0403 | 0.422 |
| v_p99 | 0.0979 | 0.208 | 0.173 | 3.97 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 1.58 | 2.77 | 4.43 | 16.9 |
| e3d_mean | 1.66 | 2.88 | 4.58 | 17.7 |
| e3d_max | 11.1 | 16.9 | 40 | 87 |
| al_mean | 1.11 (0.924) | 2.08 | 4.09 | 15.5 |
| al_mean_abs | 1.55 | 2.5 | 4.63 | 15.7 |
| al_rmse | 1.94 | 3.16 | 8.31 | 17.7 |
| al_max | 7.78 | 11.9 | 24.2 | 68.9 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 1.76 | 2.98 | 5.15 | 23.1 |
| drift_pct | 0.0494 | 0.13 | 0.38 | 1.06 |
| drift_onmap_pct | 0.0592 | 0.166 | 0.137 | 5.41 |
| al_growth_pct | 0.0539 (0.0277) | 0.0876 | 0.193 | 0.6 |
| p_in95 | 0.995 | 0.946 | 1 | 1 |
| p_nees | 0.154 | 1.57 | 3.03 | 23.7 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 0.921 | 0.0303 | 0.0124 | 1.99 | 2.58 | 13.1 | 2.01 | 0.0497 |
| T2S | 30 | 0.494 | 0.03 | 0.0107 | 1.21 | 1.42 | 4.32 | 1.42 | 0.0426 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 0.649 | 0.0299 | 0.0124 | 1.25 | 1.76 | 7.13 | 1.55 | 0.0423 |
| 30639 | 11 | 1.27 | 0.0313 | 0.0119 | 1.59 | 2.37 | 10.5 | 2.13 | 0.116 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 0.705 | 0.0288 | 0.0123 | 1.67 | 1.89 | 7.07 | 1.78 | 0.0429 |
| 30618_2026-08-10 | 9 | 0.8 | 0.0716 | 0.02 | 1.5 | 2 | 6.61 | 1.61 | 0.0283 |
| 30618_2026-08-26 | 12 | 0.539 | 0.0292 | 0.0111 | 1.08 | 1.5 | 8.91 | 1.16 | 0.043 |
| 30618_2026-09-03 | 2 | 2.14 | 0.0822 | 0.0518 | 8.96 | 9.88 | 15.8 | 11.7 | 0.281 |
| 30639_2026-05-05 | 3 | 2.07 | 0.0643 | 0.0607 | 11.3 | 11.4 | 13 | 15.1 | 0.38 |
| 30639_2026-08-26 | 8 | 0.593 | 0.0303 | 0.00898 | 1.57 | 2.09 | 7.75 | 1.84 | 0.0916 |

## Worst bags

- **score_loss**: 30618_4d487b0d 5.59, 30639_92226df0 3.19, 30618_616ec56b 2.85, 30618_27e994fc 2.17, 30618_28538acf 2.11, 30618_88548b02 2.11, 30639_253671cc 2.07, 30639_9c362687 2.03
- **al_rmse**: 30639_92226df0 17.7, 30639_9c362687 11.4, 30618_27e994fc 9.91, 30618_88548b02 9.85, 30618_28538acf 8.98, 30639_253671cc 8.66, 30618_4d487b0d 8.27, 30618_616ec56b 7.32
- **v_rmse**: 30618_4d487b0d 0.639, 30618_616ec56b 0.247, 30618_40ffd323 0.18, 30618_28538acf 0.126, 30639_50956d6e 0.125, 30618_2dbce472 0.117, 30618_88548b02 0.102, 30618_2366c74a 0.0983
- **drift_pct**: 30639_0be558e2 1.06, 30618_49fe4c54 1.03, 30618_8158f0b0 0.972, 30639_92226df0 0.49, 30618_a869780d 0.463, 30618_27e994fc 0.382, 30639_253671cc 0.38, 30639_9c362687 0.317
- **e2d_offmap_mean**: 30639_92226df0 23.1, 30618_27e994fc 17.3, 30639_9c362687 16.8, 30618_88548b02 5.59, 30618_68d1748a 5.51, 30639_4285f2bc 5.32, 30618_616ec56b 5.1, 30618_8158f0b0 4.96

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 26.3, min 12.1, max 304
- lat_ms_p99: median 0.225, min 0.0261, max 1.87
- lat_ms_max: median 4.23, min 0.31, max 33.4
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.759, min 0, max 24.7
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 135
- total_msgs: 5129019
