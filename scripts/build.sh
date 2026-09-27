#!/usr/bin/env bash
# Build the Docker image (ROS 2 Humble + tram_vehicle_msgs + tram_backup_odometry) and
# prove that the workspace builds without internet access.
#
#   scripts/build.sh            # regenerate config/params.yaml, docker build, offline colcon build + tests
#
# Environment:
#   IMAGE=tram_backup_odometry:humble   image tag
#   GEN_PARAMS=1                        regenerate config/params.yaml from core/config.py first (0 = skip)
#   OFFLINE_CHECK=1                     run the --network none colcon build (0 = skip)
#   RUN_TESTS=0                         also run the package pytest inside the image (1 = run)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-tram_backup_odometry:humble}"
GEN_PARAMS="${GEN_PARAMS:-1}"
OFFLINE_CHECK="${OFFLINE_CHECK:-1}"
RUN_TESTS="${RUN_TESTS:-0}"

if [ "$GEN_PARAMS" = 1 ]; then
    echo "== regenerate config/params.yaml from the Params dataclass (ros:humble python, no network)"
    docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 \
        -v "$ROOT":/sol ros:humble python3 /sol/tools/gen_params_yaml.py
fi

echo "== docker build -> $IMAGE (context: src/ tools/ docker/ only)"
# bsdtar (macOS) needs these to keep xattrs / AppleDouble files out of the context;
# GNU tar (Linux) has no --no-mac-metadata and would abort the pipeline.
TAR_MAC_FLAGS=""
if tar --version 2>/dev/null | grep bsdtar >/dev/null; then TAR_MAC_FLAGS="--no-xattrs --no-mac-metadata"; fi
# shellcheck disable=SC2086  # word splitting of TAR_MAC_FLAGS is intended
COPYFILE_DISABLE=1 tar -C "$ROOT" $TAR_MAC_FLAGS \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.DS_Store' --exclude='.pytest_cache' \
    -cf - src tools docker \
  | docker build -t "$IMAGE" -f docker/Dockerfile -

if [ "$OFFLINE_CHECK" = 1 ]; then
    echo "== offline check: clean colcon build of the mounted sources with --network none"
    docker run --rm --network none --entrypoint /bin/bash \
        -v "$ROOT/src":/src_ro:ro "$IMAGE" -c '
        set -eo pipefail   # no -u: ROS setup scripts use unset variables
        python3 - <<EOF
import socket
try:
    socket.create_connection(("pypi.org", 443), timeout=3)
    print("network: REACHABLE (unexpected)")
except OSError as e:
    print("network: unreachable as intended (%s)" % e.__class__.__name__)
EOF
        source /opt/ros/humble/setup.bash
        # copy without caches (host tools may rewrite __pycache__ while we copy); fail loudly
        mkdir -p /tmp/ws/src
        tar -C /src_ro --exclude=__pycache__ --exclude="*.pyc" --exclude=.pytest_cache -cf - . | tar -C /tmp/ws/src -xf -
        cd /tmp/ws
        test "$PWD" = /tmp/ws
        for f in src/*/package.xml; do   # same placeholder <maintainer> patch as in docker/Dockerfile
            grep -q "<maintainer" "$f" || sed -i "s|</license>|</license>\n  <maintainer email=\"noreply@example.com\">unknown</maintainer>|" "$f"
        done
        colcon build --event-handlers console_cohesion- status- summary+
        source install/setup.bash
        echo "package prefix: $(ros2 pkg prefix tram_backup_odometry)"
        echo "executables:"; ros2 pkg executables tram_backup_odometry
        python3 -c "import tram_backup_odometry.node, tram_vehicle_msgs.msg; print(\"import OK\")"
        ls "$(ros2 pkg prefix tram_backup_odometry)/share/tram_backup_odometry/"{maps,config,launch}
    '
fi

if [ "$RUN_TESTS" = 1 ]; then
    echo "== package tests inside the image (network none)"
    docker run --rm --network none "$IMAGE" bash -c \
        'cd /ws && python3 -m pytest -q -p no:cacheprovider src/tram_backup_odometry/test'
fi
echo "== done: image $IMAGE"
