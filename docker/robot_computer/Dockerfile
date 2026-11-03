# The robot computer side of a "robot in a box" (PLAN.md, Orin option 1): ROS 2 Humble with titan_bridge, built
# from the titan_firmware_rs checkout, plus its upload_tool and canmore_cli. It talks to the emulated boards on the
# host's vcan interfaces (compose.yaml runs it with the host's network namespace).
#
#   docker compose -f docker/robot_computer/compose.yaml build      # from the repository root
#
# UNVERIFIED: written without a Docker daemon or network access, so it has never been built. The parts most likely
# to need attention are marked "check".
#
# Build context: the repository root (for external/titan_firmware_rs). The root .dockerignore keeps build outputs
# out of the context.
#
# For the Orin's architecture under qemu-user/binfmt, build and run with DOCKER_DEFAULT_PLATFORM=linux/arm64 (after
# `docker run --privileged --rm tonistiigi/binfmt --install arm64`); the ros:humble images are multi-arch.

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=humble
# riptide_msgs2 (the riptide message package titan_bridge publishes) lives in riptide_core's riptide_msgs
# directory, as titan_firmware's micro-ROS library generation fetches it. Set RIPTIDE_CORE_REPO to "" to skip it
# and put the packages into docker/robot_computer/ros_ws/ instead (a private fork, the full riptide stack, ...).
ARG RIPTIDE_CORE_REPO=https://github.com/osu-uwrt/riptide_core
# A branch, tag or commit; empty for the default branch
ARG RIPTIDE_CORE_REF=
# Also build upload_tool and canmore_cli (they build libusb from source).
ARG BUILD_TOOLS=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# can-utils for candump/cansend, iproute2 for `ip link` (the generated bridges script checks the interfaces),
# libclang for rclrs's bindings (check: rclrs 0.8 may ship pre-generated ones), libudev for the tools' libusb
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential ca-certificates clang curl git iproute2 can-utils libclang-dev libudev-dev pkg-config \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# Rust (edition 2024 needs 1.85 or later)
ENV RUSTUP_HOME=/usr/local/rustup CARGO_HOME=/usr/local/cargo PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable \
    && rustc --version

# ROS packages titan_bridge needs at run time: its dynamic messages load riptide_msgs2's type support. Anything put
# in docker/robot_computer/ros_ws/ (e.g. riptide_software) is built too.
WORKDIR /ros_ws
COPY docker/robot_computer/ros_ws/ /ros_ws/src/
RUN if [ -n "${RIPTIDE_CORE_REPO}" ]; then \
        git clone "${RIPTIDE_CORE_REPO}" /tmp/riptide_core \
        && if [ -n "${RIPTIDE_CORE_REF}" ]; then git -C /tmp/riptide_core checkout "${RIPTIDE_CORE_REF}"; fi \
        && cp -r /tmp/riptide_core/riptide_msgs /ros_ws/src/riptide_msgs \
        && rm -rf /tmp/riptide_core; \
    fi \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && if compgen -G "/ros_ws/src/*/package.xml" > /dev/null || compgen -G "/ros_ws/src/*/*/package.xml" > /dev/null; then \
        colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    else \
        echo "no ROS packages in /ros_ws/src: titan_bridge will not find riptide_msgs2" >&2; \
    fi

# titan_bridge (rclrs, built against the sourced ROS environment like in riptide-shell), and the tools
WORKDIR /src/titan_firmware_rs
COPY external/titan_firmware_rs/ /src/titan_firmware_rs/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi \
    && cargo build --release -p titan_bridge \
    && install -m 755 target/release/titan_bridge /usr/local/bin/ \
    && if [ "${BUILD_TOOLS}" = 1 ]; then \
        cargo build --release -p upload_tool -p canmore_cli \
        && install -m 755 target/release/upload_tool target/release/canmore_cli /usr/local/bin/; \
    fi \
    && rm -rf target /usr/local/cargo/registry

# Placeholder for the rest of the riptide stack (riptide_software's launch files, controllers, mapping, ...):
# add its packages to ros_ws/ and start them from the command, e.g.
#   command: ["ros2", "launch", "riptide_bringup", "..."]

COPY docker/robot_computer/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
