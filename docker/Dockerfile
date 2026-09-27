# ROS 2 Humble image with the tram backup odometry workspace built in /ws.
#
# Build context = the solution root, but only src/, tools/ and docker/ are needed;
# scripts/build.sh streams exactly those as a tar (the .work cache is ~0.5 GB):
#     tar -cf - src tools docker | docker build -t tram_backup_odometry:humble -f docker/Dockerfile -
#
# Everything needed at run time is inside the image: running and rebuilding the
# workspace works with `docker run --network none` (no internet).
FROM ros:humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-numpy python3-yaml python3-pytest python3-psutil \
        ros-humble-rosbag2-storage-default-plugins \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src
# The organisers' tram_vehicle_msgs/package.xml declares no <maintainer>, which catkin_pkg 1.x
# rejects ("must declare at least one maintainer").  Add a placeholder only where it is missing.
RUN for f in /ws/src/*/package.xml; do \
        grep -q '<maintainer' "$f" || sed -i 's|</license>|</license>\n  <maintainer email="noreply@example.com">unknown</maintainer>|' "$f"; \
    done
RUN source /opt/ros/humble/setup.bash \
 && colcon build --event-handlers console_cohesion+ \
 && rm -rf /ws/build /ws/log

COPY tools /ws/tools

# Source the workspace for every command (docker run ... <cmd>) and interactive shells.
RUN printf '%s\n' '#!/bin/bash' 'set -e' \
        'source "/opt/ros/$ROS_DISTRO/setup.bash" --' \
        'source /ws/install/setup.bash --' \
        'exec "$@"' > /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh \
 && echo 'source /ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
# `ros2 launch` runs as PID 1 and shuts down cleanly on SIGINT; with the default SIGTERM,
# `docker stop` waits for its timeout and then SIGKILLs the node.
STOPSIGNAL SIGINT
CMD ["ros2", "launch", "tram_backup_odometry", "backup_odometry.launch.py"]
