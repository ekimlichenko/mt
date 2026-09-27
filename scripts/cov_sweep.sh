#!/usr/bin/env bash
# Covariance sweep behind gnss_sigma_rtk_m / gnss_drift_frac / gnss_drift_frac_scaled (docs/PARAMETERS.md):
# GNSS bursts over the whole route, target = 95 % ellipse coverage.  Results go to results/_cov_{a,b,c}/.
#   PY=/path/to/python scripts/cov_sweep.sh
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry
PY=${PY:-python3}
COMMON=(--bags all --antenna master --jobs "${JOBS:-8}" --gnss-limit 30 --gnss-burst 120 1.5)
"$PY" tools/run_cv.py --tag _cov_a "${COMMON[@]}" --set gnss_sigma_rtk_m=0.8 > /dev/null 2>&1
"$PY" tools/run_cv.py --tag _cov_b "${COMMON[@]}" --set gnss_sigma_rtk_m=0.8 gnss_drift_frac=0.006 gnss_drift_frac_scaled=0.003 > /dev/null 2>&1
"$PY" tools/run_cv.py --tag _cov_c "${COMMON[@]}" --set gnss_sigma_rtk_m=1.2 gnss_drift_frac=0.006 gnss_drift_frac_scaled=0.004 > /dev/null 2>&1
echo DONE
