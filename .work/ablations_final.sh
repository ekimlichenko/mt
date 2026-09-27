#!/bin/zsh
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry
PY=/opt/miniconda3/envs/ml/bin/python
run() { tag=$1; shift; echo "== $tag"; $PY tools/run_cv.py --tag $tag --jobs 5 "$@" 2>&1 | tail -2; }
run AF_noadapt --set gain_adapt=false
run AF_nobias --set bias_enable=false
run AF_noinflate --set wheel_pair_sigma_inflate=false
run AF_nolook --set model_lookahead=false
run AF_nopairs --set wheel_align_other=false
run AF_nosc --set s_correction=false
run AF_scalefb --set trn_scale_feedback=true
run AF_tail_straight --set tail_mode=straight
run AF_trn_off --set trn_enable=false
run AF_noloops --set loops_enable=false
run AF_noaiding --set gnss_aiding=false
for g in 0 0.5 1 3 5 30; do t=${g/./}; run G_limit$t --gnss-limit $g; done
run B1_naive --estimator baseline_naive:run_bag
run B1_naive_kf --estimator baseline_naive:run_bag_kf
echo ABL_DONE
