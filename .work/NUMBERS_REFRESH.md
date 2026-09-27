# Numbers refresh after the 17:35 bug fix (pipeline.py anchor_delta)

Bug: GNSS bursts processed BEFORE the TRN had started (start window, typically when the run
starts off the map) left `anchor_delta = None`, which disabled the TRN blend for the rest of the run:
with start-only GNSS the published position ignored the TRN corrections (up to 35 m on
30639_9c362687). Fix (core/pipeline.py `_process_burst`): when TRN is not active,
`anchor_delta = anchor_s - s_offset - anchor_odo` (the anchor's offset from the TRN reference path
s_offset + s_odo). Regression test: test_gnss_aiding.py::test_start_window_burst_keeps_trn_corrections.
Host pytest 106 passed / 12 skipped; in Docker (scripts/build.sh RUN_TESTS=1, .work/build_final3.log)
120 passed / 11 skipped.
Also: launch argument vehicle_id now defaults to 30618 (launch/backup_odometry.launch.py; hidden
check = tram 30618); Params.vehicle_id default stays '' (offline tools take it from the bag name);
`vehicle_id:=''` = value from the params file.

All results files were regenerated on the fixed code at 17:36-17:58: results/F4_final,
results/F4_final_burst, results/AF_*, results/G_limit*, results/robustness, results/checker_30618_88aea4d9.
B1_naive / B1_naive_kf (separate baseline, unaffected) re-run at 17:17.

| quantity | OLD (pre-fix, may be in docs) | NEW |
|---|---|---|
| F4_final al_rmse median/mean | 2.38 / 4.32 | 1.98 / 3.15 |
| F4_final e3d_mean median/mean | 2.14 / 4.00 | 1.66 / 2.80 |
| F4_final drift_pct median/mean | 0.064 / 0.171 | 0.047 / 0.127 |
| F4_final e2d_offmap_mean median/mean | 1.89 / 4.3 | 1.33 / 2.78 |
| F4_final p_in95 median/mean | 0.998 / 0.952 | 0.995 / 0.947 |
| F4_final p_nees median/mean | 0.102 / 1.73 | 0.151 / 1.55 |
| F4_final score_loss median/mean | 0.738 / 1.06 | 0.622 / 0.946 |
| F4_final al_max median/mean | ? | 8.14 / 11.9 |
| F4_final_burst al_rmse | 0.879 / 2.41 | 0.834 / 1.97 |
| F4_final_burst e3d_mean | 0.776 / ? | 0.820 / 1.47 |
| F4_final_burst drift_pct | 0.0125 | 0.0125 / 0.121 |
| F4_final_burst e2d_offmap_mean | 0.617 | 0.617 / 1.43 |
| F4_final_burst p_in95 | 0.98 / 0.93 | 0.980 / 0.929 |
| F4_final_burst p_nees | 0.399 / 4.73 | 0.431 / 4.69 |
| F4_final_burst score_loss | 0.587 | 0.566 / 0.815 |
| F4_final_burst al_max | 7.22 | 7.22 / 10.5 |
| checker_sim final_gnss_first_35s_only 3D RMSE / median / max | 6.73 / 2.89 / 51.8 | 7.02 / 4.11 / 50.8 |
| checker_sim final (unchanged) | 1.11 m, v 0.0335 | same |

Ablations on the fixed code (median/mean; GNSS limit 10 s; source results/<tag>/summary.md):
F4_final al_rmse 1.98/3.15 e3d 1.66/2.80 score 0.622/0.946; AF_noadapt 2.74/3.78, 2.34/3.16, 0.702/1.01;
AF_nobias 2.05/3.21, 1.75/2.83, 0.635/0.952; AF_noinflate 1.96/3.14, 1.63/2.79, 0.624/0.948;
AF_nolook 1.91/3.13, 1.63/2.78, 0.623/0.944; AF_nopairs 1.94/3.16, 1.66/2.88, 0.651/1.00 (v_rmse mean 0.0581);
AF_nosc 2.12/3.21, 1.69/2.88, 0.639/0.954; AF_scalefb 1.98/3.15, 1.66/2.80, 0.623/0.933 (v_rmse 0.0292/0.0504);
AF_tail_straight = F4_final; AF_trn_off 2.38/4.49, 2.14/4.13, 0.738/1.08; AF_noloops 2.48/3.99, 4.66/6.79, 1.21/1.49
(drift 0.324/1.11); AF_noaiding 2.15/3.12, 1.74/2.75, 0.646/0.942; G_limit0 2.15/3.17, 1.77/2.80, 0.646/0.948;
G_limit05 = G_limit1 = AF_noaiding; G_limit3 2.04/3.07, 1.68/2.72, 0.640/0.938; G_limit5 1.96/3.15, 1.66/2.80, 0.622/0.946;
G_limit30 1.97/3.16, 1.61/2.81, 0.621/0.948; B1_naive al 2.81/5.91 e3d 5.23/8.57 v 0.0541/0.0728 score 2.00/2.13;
B1_naive_kf al 3.13/5.97 e3d 5.14/8.63 v 0.0296/0.0527 score 1.61/1.70 (B1 baselines have no loops).
Before the fix AF_noaiding was better than F4_final (e3d 1.74 vs 2.14); after the fix start-window aiding is
neutral-to-positive (medians 1.66 vs 1.74, means 2.80 vs 2.75).

ROS checker (organisers' hackathon_solution_checker in Docker, scripts/run_checker.sh --measure):
- results/checker_ros_30618_88aea4d9/ — image built 17:00 (pre-fix; the fix does not change this bag: the
  emulation gives identical numbers before/after): velocity RMSE 0.0322 m/s (max 0.309), x/y/z RMSE
  0.909/0.646/0.037, 3D RMSE 1.116 m (max 14.03), 38435 pairs; latency p50/p99 0.43/2.99 ms (velocity),
  0.77/3.64 ms (position); max 202/204 ms = host contention (my CV jobs 17:07-17:18, 21 of 76949 outputs > 50 ms;
  node callback max 13.4 ms); 29.4 Hz; CPU mean 0.039 core (max 0.355); RSS 62.2 -> 64.1 MiB.
- results/checker_ros_30618_88aea4d9_final/ — clean rerun on the final image (17:30-17:52), idle host. Prefer it.

FINAL clean ROS checker run (results/checker_ros_30618_88aea4d9_final/README.md, final image, idle host, 17:30-17:52 MSK):
velocity RMSE 0.0324 m/s (max 0.302); x/y/z RMSE 0.904/0.645/0.037 m; 3D RMSE 1.111 m (max 14.02); 38437 pairs.
Latency velocity p50/p95/p99 1.17/2.45/3.03 ms, max 22.8 (after 2 s) / 32.1 (all); position 1.90/3.23/3.78, max 24.9 / 32.4.
29.38 Hz; stamps = inputs; coverage 99.99 %; CPU mean 0.071 core, p95 0.100, max 0.259; RSS 62.2 -> 64.1 MiB, 11 threads.
Node callback mean 0.693 ms, p99 1.995, max 6.32 ms. Docker pytest 120 passed / 11 skipped (.work/build_final3.log).
This run supersedes the preliminary results/checker_ros_30618_88aea4d9 (202 ms peak = host contention) as the headline RT evidence.
