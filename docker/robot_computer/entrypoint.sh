#!/usr/bin/env bash
# Sources ROS 2 and the riptide workspace, then runs the command, e.g. the bridges script that
# `firmware_simulator bridges ROBOT --format script` writes (compose.yaml shares it in /shared).
#
#   entrypoint.sh wait-for /shared/bridges.sh   # waits for the file, then runs it
#   entrypoint.sh titan_bridge --board poac --interface vcan0
set -eo pipefail

# The setup scripts read unset variables, so they run before `set -u` would be safe
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f /ros_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /ros_ws/install/setup.bash
fi

if [ "${1:-}" = wait-for ]; then
    script=${2:?wait-for needs a file}
    for _ in $(seq 1 600); do
        [ -s "$script" ] && break
        sleep 0.5
    done
    if [ ! -s "$script" ]; then
        echo "gave up waiting for $script" >&2
        exit 1
    fi
    exec bash "$script"
fi

exec "$@"
