#!/usr/bin/env bash
# Latency and CPU/RSS of the node during a real-time bag play (inside the Docker image).
#
#   scripts/measure.sh <bag_dir> [out_dir] [run_bag.sh options]
#
# Runs scripts/run_bag.sh --measure: tools/latency_probe.py (input header stamp -> output with the
# same stamp, wall-clock latency p50/p95/max, output rate, 0.1 s grid coverage) and
# tools/resource_monitor.py (psutil CPU cores and RSS of the node process at 2 Hz) run next to the
# node while the bag plays at --rate 1 (default: the first 120 s; override with --duration S).
# out_dir defaults to ${TMPDIR:-/tmp}/tram_measure_<bag>_<time>.
set -euo pipefail
[ $# -ge 1 ] || { sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 1; }
BAG="$1"; shift
if [ $# -ge 1 ] && [ "${1#-}" = "$1" ]; then
    OUT="$1"; shift
else
    OUT="${TMPDIR:-/tmp}/tram_measure_$(basename "$BAG")_$(date +%Y%m%d_%H%M%S)"
fi
ARGS=("$@")
case " ${ARGS[*]:-} " in *" --duration "*) ;; *) ARGS=(--duration 120 "${ARGS[@]:+${ARGS[@]}}") ;; esac
exec "$(dirname "${BASH_SOURCE[0]}")/run_bag.sh" "$BAG" "$OUT" --measure "${ARGS[@]}"
