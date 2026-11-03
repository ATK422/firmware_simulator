# The emulator with the titan_firmware_rs board images, for docker/robot_computer/compose.yaml.
#
# UNVERIFIED: written without a Docker daemon or network access, so it has never been built.
#
# Build context: the repository root, with the submodules checked out (the private titan_firmware_rs one
# included). The root .dockerignore keeps local build outputs out of the context.

FROM rust:1-bookworm AS build
RUN rustup target add thumbv6m-none-eabi
WORKDIR /src
COPY . .
# The board images (the same `cargo build --workspace --release` as in the README), then the emulator
RUN cd external/titan_firmware_rs/firmware && cargo build --workspace --release
RUN cargo build --release --bin firmware_simulator
# Keep only the images: executables directly in the release directory
RUN mkdir -p /out/firmware \
    && find external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release -maxdepth 1 -type f -perm -u+x \
        -exec cp {} /out/firmware/ \;

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends can-utils iproute2 && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/target/release/firmware_simulator /usr/local/bin/
COPY --from=build /out/firmware/ /firmware/
COPY boards/ /work/boards/
COPY robots/ /work/robots/
COPY environments/ /work/environments/
# Robot files name the images by their path in the titan_firmware_rs checkout; this points that at /firmware
ENV TITAN_RS_RELEASE_DIR=/firmware
WORKDIR /work
ENTRYPOINT ["firmware_simulator"]
CMD ["--help"]
