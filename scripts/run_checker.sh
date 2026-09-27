#!/usr/bin/env bash
# Score the backup odometry node with the ORGANISERS' checker (hackathon_solution_checker) in Docker.
#
#   scripts/run_checker.sh <bag_dir> <out_dir> [options] [-- extra launch arguments, e.g. params_file:=/path/params.yaml]
#
# Options:
#   --rate R        ros2 bag play rate (default 1.0; the checker is meant for real time)
#   --duration S    stop the playback after S wall seconds (default: whole bag)
#   --vehicle ID    vehicle_id launch argument (default: bag name prefix 30618_/30639_, else unset)
#   --measure       also run tools/latency_probe.py + tools/resource_monitor.py (as scripts/run_bag.sh)
#   --checker DIR   checker package sources (default ../check_code/check-code/src/checker_ros)
#   --image NAME    docker image (default tram_backup_odometry:humble, see scripts/build.sh)
#
# One container (--network none; bag, checker sources read-only):
#   1. copies the checker to /tmp/chk/src and builds only hackathon_solution_checker with colcon
#      on top of /ws/install, i.e. against OUR tram_vehicle_msgs (VelocitySensor is identical);
#   2. starts the node (ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=...),
#      `ros2 run hackathon_solution_checker metrics`, a recorder of /result/* and, with --measure,
#      the latency probe and the resource monitor;
#   3. plays the bag (ros2 bag play -d 1 -r R), waits 3 s, stops the checker with SIGINT (it prints
#      its final RMSE / max / n on shutdown), then the recorder and the node.
# Results in <out_dir>:
#   checker_final.txt / checker_final.json   final checker summary (the shutdown report; if it is
#                                            missing, the last periodic 5 s report, marked as such)
#   checker.log checker_build.log node.log play.log record.log
#   result_bag/     /result/* + the replayed /localization/kinematic_state + diagnostics (record time = arrival)
#   [latency.txt latency.json latency.csv resources.txt resources.json resources.csv]
set -euo pipefail

usage() { sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 2 ] || usage 1
BAG="$1"; OUT="$2"; shift 2
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RATE=1.0; DURATION=""; VEHICLE=""; MEASURE=0
CHECKER="$ROOT/../check_code/check-code/src/checker_ros"
IMAGE="${IMAGE:-tram_backup_odometry:humble}"
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --rate) RATE="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --vehicle) VEHICLE="$2"; shift 2 ;;
        --measure) MEASURE=1; shift ;;
        --checker) CHECKER="$2"; shift 2 ;;
        --image) IMAGE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        --) shift; EXTRA=("$@"); break ;;
        *) echo "unknown option: $1" >&2; usage 1 ;;
    esac
done

BAG="$(cd "$BAG" && pwd)"
[ -f "$BAG/metadata.yaml" ] || { echo "not a rosbag2 directory: $BAG" >&2; exit 1; }
CHECKER="$(cd "$CHECKER" && pwd)"
[ -f "$CHECKER/hackathon_solution_checker/metrics.py" ] || { echo "no checker sources in $CHECKER" >&2; exit 1; }
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
[ -e "$OUT/result_bag" ] && { echo "$OUT/result_bag already exists; choose another out_dir" >&2; exit 1; }
if [ -z "$VEHICLE" ]; then
    case "$(basename "$BAG")" in 30618_*) VEHICLE=30618 ;; 30639_*) VEHICLE=30639 ;; esac
fi

IFS= read -r -d '' INNER <<'INNER_EOF' || true
set -o pipefail   # no -u: ROS setup scripts use unset variables
set -m   # job control: background jobs get their own process group and keep SIGINT
export ROS_LOG_DIR=/tmp/roslog PYTHONDONTWRITEBYTECODE=1
TOOLS=/ws/tools

# 1. build the organisers' checker in an overlay on top of /ws/install (our tram_vehicle_msgs)
mkdir -p /tmp/chk/src/checker_ros
tar -C /checker_src --exclude=__pycache__ --exclude='*.pyc' --exclude=.pytest_cache -cf - . \
    | tar -C /tmp/chk/src/checker_ros -xf - || { echo "copying /checker_src failed"; exit 1; }
f=/tmp/chk/src/checker_ros/package.xml   # placeholder <maintainer> if missing, as in docker/Dockerfile
grep -q '<maintainer' "$f" || sed -i 's|</license>|</license>\n  <maintainer email="noreply@example.com">unknown</maintainer>|' "$f"
if ! (cd /tmp/chk && colcon build --packages-select hackathon_solution_checker \
        --event-handlers console_cohesion+ > /out/checker_build.log 2>&1); then
    echo "checker build failed:"; tail -30 /out/checker_build.log; exit 1
fi
source /tmp/chk/install/setup.bash
echo "checker: $(ros2 pkg prefix hackathon_solution_checker) (msgs: $(ros2 pkg prefix tram_vehicle_msgs))"

# 2. node, checker, recorder, probes
LAUNCH_ARGS=(use_sim_time:=false)
[ -n "$VEHICLE" ] && LAUNCH_ARGS+=(vehicle_id:="$VEHICLE")
LAUNCH_ARGS+=($EXTRA_ARGS)
echo "node: ros2 launch tram_backup_odometry backup_odometry.launch.py ${LAUNCH_ARGS[*]}"
ros2 launch tram_backup_odometry backup_odometry.launch.py "${LAUNCH_ARGS[@]}" > /out/node.log 2>&1 < /dev/null &
LAUNCH_PID=$!
for _ in $(seq 1 150); do grep -q 'ready:' /out/node.log && break; sleep 0.2; done
grep -q 'ready:' /out/node.log || { echo "node did not start:"; cat /out/node.log; exit 1; }

ros2 run hackathon_solution_checker metrics > /out/checker.log 2>&1 < /dev/null &
CHK_PID=$!   # `ros2 run` wrapper = process-group leader; SIGINT goes to the whole group
for _ in $(seq 1 150); do grep -q 'Comparing' /out/checker.log && break; sleep 0.2; done
grep -q 'Comparing' /out/checker.log || { echo "checker did not start:"; cat /out/checker.log; exit 1; }
grep 'Comparing' /out/checker.log | sed 's/^/checker> /'

# the replayed reference is recorded too: its record times give the arrival order the checker saw
ros2 bag record -o /out/result_bag /result/velocity /result/position /localization/kinematic_state \
    /backup_odometry/diagnostics /backup_odometry/slip > /out/record.log 2>&1 < /dev/null &
REC_PID=$!
for _ in $(seq 1 100); do   # the reference topic appears only once the play starts
    [ "$(grep -c 'Subscribed to topic' /out/record.log)" -ge 4 ] && break; sleep 0.2
done

if [ "$MEASURE" = 1 ]; then
    python3 "$TOOLS/resource_monitor.py" --csv /out/resources.csv --json /out/resources.json \
        > /out/resources.txt 2>&1 < /dev/null &
    MON_PID=$!
    python3 "$TOOLS/latency_probe.py" --json /out/latency.json --csv /out/latency.csv > /out/latency.txt 2>&1 < /dev/null &
    PROBE_PID=$!
fi
sleep 3   # discovery of the checker / probe subscriptions before the first message

# 3. play, then stop everything in order: probes, checker (final report), recorder, node
PLAY=(ros2 bag play /data/bag --disable-keyboard-controls -d 1 -r "$RATE")   # -d: let discovery finish
echo "play: ${PLAY[*]}${DURATION:+ (stopped after $DURATION s)}"
T0=$(date +%s%N)
if [ -n "$DURATION" ]; then
    timeout -s INT "$DURATION" "${PLAY[@]}" > /out/play.log 2>&1 < /dev/null
else
    "${PLAY[@]}" > /out/play.log 2>&1 < /dev/null
fi
echo "played for $(( ($(date +%s%N) - T0) / 1000000 )) ms"
sleep 3   # let the last outputs reach the checker / recorder / probes

if [ "$MEASURE" = 1 ]; then
    kill -INT "$PROBE_PID" "$MON_PID" 2>/dev/null; wait "$PROBE_PID" "$MON_PID"
fi
MARK=$(wc -l < /out/checker.log)   # everything after this line is the shutdown output
kill -INT -- -"$CHK_PID" 2>/dev/null
for _ in $(seq 1 100); do kill -0 "$CHK_PID" 2>/dev/null || break; sleep 0.2; done
if kill -0 "$CHK_PID" 2>/dev/null; then
    echo "checker did not exit 20 s after SIGINT; SIGTERM"; kill -TERM -- -"$CHK_PID" 2>/dev/null
fi
wait "$CHK_PID"; echo "checker exit code $?"
kill -INT "$REC_PID"; wait "$REC_PID"
kill -INT "$LAUNCH_PID"; wait "$LAUNCH_PID"
grep -E 'initial alignment|first pose|shutdown:|ERROR|Traceback' /out/node.log | sed 's/^/node> /' | cut -c1-400
grep -E 'ERROR|Traceback|Error' /out/checker.log | sed 's/^/checker> /' | cut -c1-400

python3 - "$MARK" <<'PY' | tee /out/checker_final.txt
# Final checker summary: the report printed on shutdown, else the last periodic one.
import json, re, sys
mark = int(sys.argv[1])
lines = open('/out/checker.log', errors='replace').read().splitlines()
pat = re.compile(r'(\w+): RMSE=([^,]+), max=([^,]+), n=(\d+)')

def last(block, kind):
    for ln in reversed(block):
        if kind in ln:
            return ln
    return None

after = lines[mark:]
vel, pos = last(after, 'Velocity metrics'), last(after, 'Position metrics')
source = 'shutdown report (printed after SIGINT)'
if vel is None or pos is None:
    vel, pos = last(lines, 'Velocity metrics'), last(lines, 'Position metrics')
    source = 'LAST PERIODIC 5 s REPORT (no shutdown report was printed)'
n_periodic = sum('Velocity metrics' in ln for ln in lines[:mark])
res = {'source': source, 'periodic_reports': n_periodic, 'velocity_line': vel, 'position_line': pos}
print('checker final summary: %s; %d periodic reports before' % (source, n_periodic))
for key, ln in (('velocity', vel), ('position', pos)):
    if ln is None:
        print('  %s: no report found' % key)
        continue
    for name, rmse, mx, n in pat.findall(ln):
        res.setdefault(key, {})[name] = {'rmse': float(rmse), 'max': float(mx), 'n': int(n)}
        print('  %-8s %-8s RMSE %10.6f  max %10.6f  n %d' % (key, name, float(rmse), float(mx), int(n)))
    print('  raw: ' + ln.strip())
json.dump(res, open('/out/checker_final.json', 'w'), indent=1)
PY
if [ "$MEASURE" = 1 ]; then cat /out/latency.txt /out/resources.txt; fi
INNER_EOF

echo "bag: $BAG"
echo "out: $OUT   image: $IMAGE   vehicle: ${VEHICLE:-unset}   rate: $RATE   measure: $MEASURE"
echo "checker sources: $CHECKER"
docker run --rm --network none -v "$BAG":/data/bag:ro -v "$CHECKER":/checker_src:ro -v "$OUT":/out \
    -e MEASURE="$MEASURE" -e RATE="$RATE" -e DURATION="$DURATION" \
    -e VEHICLE="$VEHICLE" -e EXTRA_ARGS="${EXTRA[*]:-}" \
    "$IMAGE" bash -c "$INNER"
