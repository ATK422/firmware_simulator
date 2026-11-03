# firmware_simulator

An RP2040 emulator (Rust) that runs unmodified `titan_firmware` (C, pico-sdk) and `titan_firmware_rs` (Rust,
embassy) images for OSU UWRT's robot boards. `PLAN.md` is the agreed design and milestone list; follow it.

## Setup

* Submodules: `git submodule update --init` (`external/titan_firmware_rs` is the private repo
  `ATK422/titan_firmware_rs`; `external/titan_firmware` is public). For C builds (M2 onward) also run
  `git -C external/titan_firmware submodule update --init --recursive` for its pico-sdk. If the private submodule
  can't be cloned, stop and say the Claude GitHub app needs access to `ATK422/titan_firmware_rs`.
* Toolchain: `rustup target add thumbv6m-none-eabi`; from M2, `gcc-arm-none-eabi`, `cmake`, `ninja`.
* Test firmware: `cd external/titan_firmware_rs/firmware && cargo build --workspace --release`; the ELFs land in
  `external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/`.

## Working on milestones

* Work through M0 to M7 in order, without waiting for approval between milestones.
* One branch and one PR per milestone (`m0-skeleton`, `m1-embassy-boot`, …), each based on the previous milestone's
  branch so they review in order. Each PR description lists the milestone's "Done when" criteria from `PLAN.md` and
  says which were verified (with the command or test that shows it) and which were not.
* Crate layout as in `PLAN.md`. Edition 2024, `cargo clippy` clean, `cargo fmt`, tests for every instruction and
  peripheral behaviour added.
* Never modify anything under `external/`. If the firmware looks wrong rather than the emulator, write it up in the PR.
* If an open question in `PLAN.md` blocks you, take the plan's recommendation and note it in the PR. Leave the Orin
  section alone.

## Cloud limitations

Cloud environments probably have no `vcan` module or `CAP_NET_ADMIN`, and no ROS 2 / `riptide-shell`, so
`titan_bridge` can't run there. For M3 and M7, verify with the in-process CAN bus and a scripted mock bridge built on
the `titan_link` crate (`external/titan_firmware_rs/crates/titan_link`). Keep the SocketCAN bridge code, and list the
real-vcan and `titan_bridge` checks as "to verify locally" in the PR.
