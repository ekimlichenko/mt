#!/usr/bin/env bash
# Play a recorded bag through the backup odometry node inside the Docker image and record the results.
#
#   scripts/run_bag.sh <bag_dir> <out_dir> [options] [-- extra launch arguments, e.g. output_frame:=utm]
#
# Options:
#   --rate R        ros2 bag play rate (default 1.0)
#   --duration S    stop the playback after S wall seconds (default: whole bag)
#   --start S       start the playback S seconds into the bag (GNSS init needs the start of a run!)
#   --vehicle ID    vehicle_id launch argument (default: bag name prefix 30618_/30639_, else unset)
#   --sim-time      launch the node with use_sim_time:=true and play with --clock 100
#   --dev           use the current src/ and tools/ (mounted read-only, the package is rebuilt
#                   offline inside the container) instead of the copy baked into the image
#   --src DIR       like --dev, but take the colcon source tree from DIR instead of src/
#   --measure       also run tools/latency_probe.py + tools/resource_monitor.py and print their summary
#   --image NAME    docker image (default tram_backup_odometry:humble, see scripts/build.sh)
#
# The container runs with --network none (DDS works over loopback), the dataset is mounted
# read-only at /data/bag, <out_dir> at /out.  Results in <out_dir>:
#   result_bag/   rosbag2 with /result/velocity /result/position /backup_odometry/diagnostics /backup_odometry/slip
#   node.log play.log record.log check.txt [latency.txt latency.json resources.txt resources.json resources.csv]
# check.txt compares the recorded outputs with the input bag: stamps must equal input header
# stamps exactly, frame ids, stamp-clock rate and coverage of the 0.1 s reference grid.
set -euo pipefail

usage() { sed -n '2,23p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 2 ] || usage 1
BAG="$1"; OUT="$2"; shift 2
RATE=1.0; DURATION=""; START=""; VEHICLE=""; SIM_TIME=0; DEV=0; MEASURE=0; DEV_SRC=""
IMAGE="${IMAGE:-tram_backup_odometry:humble}"
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --rate) RATE="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --start) START="$2"; shift 2 ;;
        --vehicle) VEHICLE="$2"; shift 2 ;;
        --sim-time) SIM_TIME=1; shift ;;
        --dev) DEV=1; shift ;;
        --src) DEV=1; DEV_SRC="$2"; shift 2 ;;
        --measure) MEASURE=1; shift ;;
        --image) IMAGE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        --) shift; EXTRA=("$@"); break ;;
        *) echo "unknown option: $1" >&2; usage 1 ;;
    esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="$(cd "$BAG" && pwd)"
[ -f "$BAG/metadata.yaml" ] || { echo "not a rosbag2 directory: $BAG" >&2; exit 1; }
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
[ -e "$OUT/result_bag" ] && { echo "$OUT/result_bag already exists; choose another out_dir" >&2; exit 1; }
if [ -z "$VEHICLE" ]; then
    case "$(basename "$BAG")" in 30618_*) VEHICLE=30618 ;; 30639_*) VEHICLE=30639 ;; esac
fi

MOUNTS=(-v "$BAG":/data/bag:ro -v "$OUT":/out)
if [ "$DEV" = 1 ]; then
    DEV_SRC="$(cd "${DEV_SRC:-$ROOT/src}" && pwd)"
    MOUNTS+=(-v "$DEV_SRC":/dev_src:ro -v "$ROOT/tools":/dev_tools:ro)
fi

IFS= read -r -d '' INNER <<'INNER_EOF' || true
set -o pipefail   # no -u: ROS setup scripts use unset variables
set -m   # job control: background jobs get their own process group and keep SIGINT
export ROS_LOG_DIR=/tmp/roslog PYTHONDONTWRITEBYTECODE=1
TOOLS=/ws/tools
if [ "$DEV" = 1 ]; then
    mkdir -p /tmp/devws/src   # copy without caches (host tools may rewrite __pycache__ meanwhile)
    tar -C /dev_src --exclude=__pycache__ --exclude='*.pyc' --exclude=.pytest_cache -cf - . \
        | tar -C /tmp/devws/src -xf - || { echo "copying /dev_src failed"; exit 1; }
    for f in /tmp/devws/src/*/package.xml; do   # placeholder <maintainer>, as in docker/Dockerfile
        grep -q '<maintainer' "$f" || sed -i 's|</license>|</license>\n  <maintainer email="noreply@example.com">unknown</maintainer>|' "$f"
    done
    if ! (cd /tmp/devws && colcon build --packages-select tram_backup_odometry > /out/dev_build.log 2>&1); then
        echo "dev build failed:"; tail -30 /out/dev_build.log; exit 1
    fi
    source /tmp/devws/install/setup.bash
    TOOLS=/dev_tools
fi

LAUNCH_ARGS=(use_sim_time:=$([ "$SIM_TIME" = 1 ] && echo true || echo false))
[ -n "$VEHICLE" ] && LAUNCH_ARGS+=(vehicle_id:="$VEHICLE")
LAUNCH_ARGS+=($EXTRA_ARGS)
echo "node: ros2 launch tram_backup_odometry backup_odometry.launch.py ${LAUNCH_ARGS[*]}"
ros2 launch tram_backup_odometry backup_odometry.launch.py "${LAUNCH_ARGS[@]}" > /out/node.log 2>&1 < /dev/null &
LAUNCH_PID=$!
for _ in $(seq 1 150); do grep -q 'ready:' /out/node.log && break; sleep 0.2; done
grep -q 'ready:' /out/node.log || { echo "node did not start:"; cat /out/node.log; exit 1; }

ros2 bag record -o /out/result_bag /result/velocity /result/position \
    /backup_odometry/diagnostics /backup_odometry/slip > /out/record.log 2>&1 < /dev/null &
REC_PID=$!
for _ in $(seq 1 100); do
    [ "$(grep -c 'Subscribed to topic' /out/record.log)" -ge 4 ] && break; sleep 0.2
done

if [ "$MEASURE" = 1 ]; then
    python3 "$TOOLS/resource_monitor.py" --csv /out/resources.csv --json /out/resources.json \
        > /out/resources.txt 2>&1 < /dev/null &
    MON_PID=$!
    python3 "$TOOLS/latency_probe.py" --json /out/latency.json --csv /out/latency.csv > /out/latency.txt 2>&1 < /dev/null &
    PROBE_PID=$!
    sleep 3   # discovery of the probe subscriptions before the first message
fi

PLAY=(ros2 bag play /data/bag --disable-keyboard-controls -d 1 -r "$RATE")   # -d: let discovery finish
[ -n "$START" ] && PLAY+=(--start-offset "$START")
[ "$SIM_TIME" = 1 ] && PLAY+=(--clock 100)
echo "play: ${PLAY[*]}${DURATION:+ (stopped after $DURATION s)}"
T0=$(date +%s%N)
if [ -n "$DURATION" ]; then
    timeout -s INT "$DURATION" "${PLAY[@]}" > /out/play.log 2>&1 < /dev/null
else
    "${PLAY[@]}" > /out/play.log 2>&1 < /dev/null
fi
echo "played for $(( ($(date +%s%N) - T0) / 1000000 )) ms"
sleep 2   # let the last outputs reach the recorder / probes

if [ "$MEASURE" = 1 ]; then
    kill -INT "$PROBE_PID" "$MON_PID" 2>/dev/null; wait "$PROBE_PID" "$MON_PID"
fi
kill -INT "$REC_PID"; wait "$REC_PID"
kill -INT "$LAUNCH_PID"; wait "$LAUNCH_PID"
grep -E 'initial alignment|first pose|shutdown:|ERROR|Traceback' /out/node.log | sed 's/^/node> /' | cut -c1-400

python3 - <<'PY' | tee /out/check.txt
# Compare the recorded outputs with the input bag (exact stamps, frame ids, rate, grid coverage).
import bisect, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, _ = r.read_next()
        out[topic].append(deserialize_message(data, get_message(types[topic])))
    return out

IN = ['/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity', '/vehicle/driver_position_cmd']
OUTT = ['/result/velocity', '/result/position', '/backup_odometry/diagnostics']
inp = read('/data/bag', IN)
res = read('/out/result_bag', OUTT)
in_stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for t in IN for m in inp[t]}
print('check: %d input messages in the bag' % sum(len(v) for v in inp.values()))
for t in OUTT[:2]:
    ms = res[t]
    if not ms:
        print('  %-18s no messages' % t); continue
    keys = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in ms]
    exact = sum(k in in_stamps for k in keys)
    st = sorted(s + ns * 1e-9 for s, ns in keys)
    span = st[-1] - st[0]
    k0, k1 = math.ceil(st[0] / 0.1), math.floor(st[-1] / 0.1)
    hit = 0
    for k in range(k0, k1 + 1):
        i = bisect.bisect_left(st, k * 0.1)
        hit += min(abs(st[j] - k * 0.1) for j in (i - 1, i) if 0 <= j < len(st)) <= 0.05 + 1e-9
    frames = sorted({m.header.frame_id for m in ms})
    extra = ''
    if t == '/result/position':
        extra = ' child_frame_id %s' % sorted({m.child_frame_id for m in ms})
    seq = [s + ns * 1e-9 for s, ns in keys]
    print('  %-18s n=%d stamps==input %d/%d (%.2f%%) | stamp rate %.2f Hz over %.1f s | '
          '0.1s-grid coverage %.2f%% | frame_id %s%s | duplicate stamps %d | stamp decreases %d'
          % (t, len(ms), exact, len(ms), 100.0 * exact / len(ms), (len(ms) - 1) / span if span > 0 else float('nan'),
             span, 100.0 * hit / max(1, k1 - k0 + 1), frames, extra, len(keys) - len(set(keys)),
             sum(b < a for a, b in zip(seq, seq[1:]))))
print('  %-18s n=%d' % (OUTT[2], len(res[OUTT[2]])))
if res[OUTT[2]]:
    last = res[OUTT[2]][-1].status[0]
    kv = {v.key: v.value for v in last.values}
    print('  last diagnostics: level %d "%s" mode=%s init_done=%s cb_p50_ms=%s cb_max_ms_total=%s'
          % (last.level[0] if isinstance(last.level, bytes) else last.level, last.message, kv.get('mode'),
             kv.get('init_done'), kv.get('cb_p50_ms'), kv.get('cb_max_ms_total')))
PY
if [ "$MEASURE" = 1 ]; then cat /out/latency.txt /out/resources.txt; fi
INNER_EOF

echo "bag: $BAG"
echo "out: $OUT   image: $IMAGE   vehicle: ${VEHICLE:-unset}   rate: $RATE   dev: $DEV${DEV_SRC:+ ($DEV_SRC)}   measure: $MEASURE"
docker run --rm --network none "${MOUNTS[@]}" \
    -e DEV="$DEV" -e MEASURE="$MEASURE" -e RATE="$RATE" -e DURATION="$DURATION" -e START="$START" \
    -e VEHICLE="$VEHICLE" -e SIM_TIME="$SIM_TIME" -e EXTRA_ARGS="${EXTRA[*]:-}" \
    "$IMAGE" bash -c "$INNER"
