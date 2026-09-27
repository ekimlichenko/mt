#!/usr/bin/env bash
# Regenerate the offline results used in docs/RESULTS.md and docs/MODEL.md (F4, ablations, GNSS limits,
# baselines, robustness).  Needs the bag cache .work/cache/<bag>.pkl and .work/bag_index.json (tools/README.md).
#   PY=/path/to/python JOBS=6 scripts/rerun_results.sh
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry
PY=${PY:-python3}
JOBS=${JOBS:-6}
run() { tag=$1; shift; echo "== $tag"; "$PY" tools/run_cv.py --tag "$tag" --jobs "$JOBS" "$@" 2>&1 | tail -1; }
run F4_final --bags all --antenna all
run F4_final_burst --bags all --antenna all --gnss-limit 30 --gnss-burst 120 1.5
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
for g in 0 0.5 1 3 5 30; do run "G_limit${g/./}" --gnss-limit "$g"; done
run B1_naive --estimator baseline_naive:run_bag
run B1_naive_kf --estimator baseline_naive:run_bag_kf
echo CV_DONE
mkdir -p .work
"$PY" tools/robustness.py --jobs "$JOBS" > .work/robustness.log 2>&1
echo ROBUST_DONE
