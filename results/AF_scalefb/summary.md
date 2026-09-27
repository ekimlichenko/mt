# AF_scalefb

Estimator `tram_backup_odometry.replay:run_bag`, reference antenna **master**, GNSS limit 10.0 s, tol 0.050 s, gate 5.0 m.
Params: defaults; overrides: {'trn_scale_feedback': 'true'}.

Bags run: 122, errors: 0, scored (eval_ok): 60.

## Overall (|value| over eval_ok bags; signed median in brackets)

| metric | median | mean | p90 | max |
|---|---|---|---|---|
| score_loss | 0.623 | 0.933 | 1.8 | 5.54 |
| v_rmse | 0.0292 | 0.0504 | 0.0706 | 0.637 |
| v_mae | 0.0205 | 0.0246 | 0.03 | 0.115 |
| v_bias | 0.00487 (-0.00432) | 0.00712 | 0.00904 | 0.0655 |
| v_bias_trans | 0.0119 | 0.0218 | 0.0326 | 0.422 |
| v_p99 | 0.0924 | 0.202 | 0.157 | 3.98 |
| v_cov | 1 | 1 | 1 | 1 |
| p_cov | 1 | 1 | 1 | 1 |
| e2d_mean | 1.63 | 2.69 | 4.14 | 16.8 |
| e3d_mean | 1.66 | 2.8 | 4.3 | 17.6 |
| e3d_max | 11.1 | 16.9 | 40.3 | 86.7 |
| al_mean | 1.18 (0.86) | 2.05 | 3.92 | 15.4 |
| al_mean_abs | 1.52 | 2.47 | 4.31 | 15.6 |
| al_rmse | 1.98 | 3.15 | 7.75 | 17.6 |
| al_max | 8.14 | 11.9 | 23.8 | 68.4 |
| ct_rmse | 0.205 | 0.475 | 1.37 | 1.65 |
| e2d_offmap_mean | 1.33 | 2.78 | 4.87 | 23 |
| drift_pct | 0.047 | 0.127 | 0.366 | 1.05 |
| drift_onmap_pct | 0.0457 | 0.177 | 0.125 | 6.41 |
| al_growth_pct | 0.0474 (0.0126) | 0.0895 | 0.204 | 1.04 |
| p_in95 | 0.995 | 0.947 | 1 | 1 |
| p_nees | 0.151 | 1.55 | 3.03 | 23.9 |

## By direction (medians)

| direction | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| S2T | 30 | 0.917 | 0.0297 | 0.0124 | 1.81 | 2.5 | 13.1 | 1.9 | 0.0556 |
| T2S | 30 | 0.499 | 0.0281 | 0.00852 | 1.29 | 1.58 | 4.31 | 1.45 | 0.0443 |

## By vehicle (medians)

| vehicle | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618 | 49 | 0.619 | 0.0293 | 0.0119 | 1.45 | 1.73 | 7.03 | 1.47 | 0.0326 |
| 30639 | 11 | 1.26 | 0.0292 | 0.0119 | 1.65 | 2.43 | 9.76 | 2.2 | 0.117 |

## By group (fold) (medians)

| group | n | score_loss | v_rmse | v_bias_trans | al_mean_abs | al_rmse | al_max | e3d_mean | drift_pct |
|---|---|---|---|---|---|---|---|---|---|
| 30618_2026-07-27 | 26 | 0.631 | 0.028 | 0.0115 | 1.57 | 1.9 | 6.9 | 1.72 | 0.0362 |
| 30618_2026-08-10 | 9 | 0.699 | 0.0401 | 0.0154 | 1.52 | 1.73 | 6.44 | 1.49 | 0.0245 |
| 30618_2026-08-26 | 12 | 0.541 | 0.0283 | 0.0105 | 1.2 | 1.54 | 8.95 | 1.27 | 0.032 |
| 30618_2026-09-03 | 2 | 1.93 | 0.0494 | 0.0465 | 9.21 | 10.2 | 16.5 | 11.1 | 0.288 |
| 30639_2026-05-05 | 3 | 1.98 | 0.0563 | 0.0523 | 13 | 13 | 15.1 | 16.8 | 0.387 |
| 30639_2026-08-26 | 8 | 0.592 | 0.0289 | 0.00944 | 1.57 | 2.08 | 8.1 | 2 | 0.0927 |

## Worst bags

- **score_loss**: 30618_4d487b0d 5.54, 30639_92226df0 3.01, 30618_28538acf 2.1, 30618_27e994fc 2.08, 30639_253671cc 1.98, 30639_9c362687 1.92, 30618_88548b02 1.78, 30618_40ffd323 1.59
- **al_rmse**: 30639_92226df0 17.6, 30639_9c362687 13, 30618_88548b02 10.4, 30618_27e994fc 9.97, 30618_28538acf 8.97, 30639_253671cc 8.88, 30618_4d487b0d 7.63, 30618_616ec56b 6.75
- **v_rmse**: 30618_4d487b0d 0.637, 30618_40ffd323 0.18, 30639_50956d6e 0.126, 30618_28538acf 0.126, 30618_2dbce472 0.116, 30618_2366c74a 0.0953, 30639_92226df0 0.0679, 30618_748832b9 0.0656
- **drift_pct**: 30639_0be558e2 1.05, 30618_49fe4c54 1.03, 30618_8158f0b0 0.972, 30639_92226df0 0.489, 30618_a869780d 0.462, 30639_253671cc 0.387, 30639_9c362687 0.364, 30618_27e994fc 0.325
- **e2d_offmap_mean**: 30639_92226df0 23, 30639_9c362687 18.5, 30618_27e994fc 14.7, 30618_88548b02 6.52, 30618_68d1748a 5.15, 30618_8158f0b0 4.96, 30639_4285f2bc 4.85, 30618_4d487b0d 4.36

## Robustness and timing (all bags that ran)

- cpu_us_per_msg: median 29.3, min 11.3, max 317
- lat_ms_p99: median 0.249, min 0.0259, max 1.9
- lat_ms_max: median 10.7, min 0.163, max 37.6
- pub_rate_hz: median 29.4, min 25.1, max 30.2
- stamps_from_input: median 1, min 1, max 1
- pose_frac: median 1, min 0.757, max 1
- max_step_m: median 0.759, min 0, max 24.7
- n_nonfinite_v: 0
- n_nonfinite_pose: 0
- n_neg_v: 0
- bags_without_pose (0): -
- total_cpu_s: 149
- total_msgs: 5129019
