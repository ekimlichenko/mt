set -e
cd /Users/egor/Documents/sideprojects/приколы/ХакатонМосТранспорт/solution
export OMP_NUM_THREADS=1
P=/opt/miniconda3/envs/ml/bin/python
$P tools/run_cv.py --tag _cov_a --bags all --antenna master --jobs 8 --gnss-limit 30 --gnss-burst 120 1.5 --set gnss_sigma_rtk_m=0.8 > /dev/null 2>&1
$P tools/run_cv.py --tag _cov_b --bags all --antenna master --jobs 8 --gnss-limit 30 --gnss-burst 120 1.5 --set gnss_sigma_rtk_m=0.8 gnss_drift_frac=0.006 gnss_drift_frac_scaled=0.003 > /dev/null 2>&1
$P tools/run_cv.py --tag _cov_c --bags all --antenna master --jobs 8 --gnss-limit 30 --gnss-burst 120 1.5 --set gnss_sigma_rtk_m=1.2 gnss_drift_frac=0.006 gnss_drift_frac_scaled=0.004 > /dev/null 2>&1
echo DONE
