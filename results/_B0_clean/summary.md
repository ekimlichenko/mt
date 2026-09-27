# _B0_clean

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m, cleaned reference.
Params: defaults; overrides: {}.

Bags run: 60, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 5.37 | 5.12 | 5.92 | 6.5 |
| v_rmse | 0.0532 | 0.0628 | 0.0952 | 0.184 |
| v_mae | 0.0348 | 0.0395 | 0.054 | 0.0826 |
| v_bias | 0.00323 (-0.00193) | 0.00589 | 0.00736 | 0.0611 |
| v_bias_trans | 0.0498 | 0.0592 | 0.0913 | 0.112 |
| v_p99 | 0.16 | 0.211 | 0.23 | 0.933 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 104 | 93.2 | 113 | 142 |
| e3d_mean | 104 | 93.2 | 113 | 142 |
| e3d_max | 142 | 155 | 192 | 346 |
| al_mean | 103 (103) | 90.9 | 110 | 130 |
| al_mean_abs | 103 | 90.9 | 110 | 130 |
| al_rmse | 103 | 91 | 110 | 130 |
| al_max | 110 | 101 | 134 | 148 |
| ct_rmse | 0.421 | 0.617 | 1.38 | 1.66 |
| e2d_offmap_mean | 107 | 99.8 | 122 | 168 |
| drift_pct | 2.34 | 2.6 | 3.89 | 6.14 |
| drift_onmap_pct | 2.21 | 2.97 | 2.54 | 49.5 |
| al_growth_pct | 0.0923 (0.0715) | 0.147 | 0.205 | 1.35 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 5.58 | 0.0549 | 0.0506 | 105 | 105 | 117 | 107 | 3.27 |
| T2S | 30 | 4.72 | 0.0487 | 0.0496 | 80.4 | 80.4 | 86.1 | 82.4 | 1.69 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 5.25 | 0.0507 | 0.0498 | 103 | 103 | 109 | 104 | 2.2 |
| 30639 | 11 | 5.46 | 0.0662 | 0.0563 | 107 | 107 | 114 | 109 | 2.93 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 5.06 | 0.0451 | 0.0459 | 103 | 103 | 110 | 104 | 2.19 |
| 30618_2026-08-10 | 9 | 5.55 | 0.0776 | 0.0808 | 102 | 102 | 107 | 104 | 2.3 |
| 30618_2026-08-26 | 12 | 4.91 | 0.0606 | 0.0572 | 90.6 | 90.6 | 97.7 | 91.1 | 2.1 |
| 30618_2026-09-03 | 2 | 5.11 | 0.0633 | 0.0905 | 67 | 67.4 | 83.3 | 94.3 | 3.38 |
| 30639_2026-05-05 | 3 | 6.27 | 0.0827 | 0.107 | 115 | 115 | 142 | 118 | 2.93 |
| 30639_2026-08-26 | 8 | 5.22 | 0.0527 | 0.0462 | 91.6 | 91.6 | 100 | 92.8 | 3.2 |

## Worst bags

- **score_loss**: 30618_4d487b0d 6.5, 30618_40ffd323 6.39, 30618_28538acf 6.33, 30639_253671cc 6.28, 30639_92226df0 6.27, 30618_27e994fc 5.96, 30639_0be558e2 5.91, 30618_1cc230fa 5.9
- **al_rmse**: 30639_253671cc 130, 30618_4d487b0d 129, 30618_dd8e6395 118, 30618_21dd3af3 118, 30639_9c362687 115, 30618_a53d5f6f 113, 30618_01f73500 110, 30618_3e9f4952 110
- **v_rmse**: 30618_40ffd323 0.184, 30618_4d487b0d 0.142, 30618_28538acf 0.14, 30618_2366c74a 0.11, 30618_748832b9 0.102, 30639_92226df0 0.0957, 30618_2050d396 0.0951, 30618_33bec73f 0.0951
- **drift_pct**: 30618_0652866c 6.14, 30618_27e994fc 6.13, 30639_0be558e2 5.31, 30618_49fe4c54 4.93, 30618_8158f0b0 4.81, 30639_253671cc 4.13, 30618_4d487b0d 3.86, 30639_2b4a6347 3.85
- **e2d_offmap_mean**: 30618_27e994fc 168, 30618_0652866c 135, 30618_4d487b0d 130, 30639_92226df0 128, 30639_4285f2bc 123, 30618_8158f0b0 122, 30618_40ffd323 122, 30618_21dd3af3 121

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 4.98, min 4.57, max 5.49
- lat_ms_p99: median 0.00665, min 0.00529, max 0.00843
- lat_ms_max: median 58.6, min 25.9, max 140
- pub_rate_hz: median 29.5, min 29, max 30.5
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 1, max 1
- max_step_m: median 1.44, min 0.898, max 3.77
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 18.4
- total_msgs: 2828963
