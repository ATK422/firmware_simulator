# Using the emulator in another repository's CI

Firmware repositories (titan_firmware, nano_ros, a fork of titan_firmware_rs, ...) can boot their images in the
emulator on every push without building it. Each push to this repository's master publishes prebuilt binaries as the
`nightly` release, and a tag `vX.Y.Z` publishes a release of that name (`.github/workflows/release.yml`). The
repository's `action.yml` installs one on a runner and can run YAML scenarios ([scenarios.md](scenarios.md)) with
JUnit output. Runs are deterministic, so a scenario that passes once passes every time on the same images.

- [The releases](#the-releases)
- [The action](#the-action)
- [Example: a firmware repository](#example-a-firmware-repository)
- [Example: nano_ros](#example-nano_ros)
- [Without the action](#without-the-action)

## The releases

| Archive | Runs on |
|---|---|
| `firmware_simulator-x86_64-unknown-linux-gnu.tar.gz` | Linux x86_64 with glibc 2.35 or newer (Ubuntu 22.04 on) |
| `firmware_simulator-aarch64-unknown-linux-gnu.tar.gz` | Linux arm64, the same |
| `firmware_simulator-aarch64-apple-darwin.tar.gz` | macOS on Apple silicon |
| `firmware_simulator-x86_64-apple-darwin.tar.gz` | macOS on Intel (cross-built, not run in CI) |

Each holds `bin/firmware_simulator`, the data files the command line reads (`boards/`, `robots/`, `environments/`,
`vehicles/`, `scenarios/`), this README and `COMMIT` (the commit it was built from); a `.sha256` sits next to each
archive. The bootrom and the dashboard are compiled in, and the binary is built with the `fast` profile (fat LTO,
3-7 % faster than a `--release` build; [internals.md](internals.md)). No firmware images are included: bring your own.

A scenario outside this repository finds the emulator's files by their short names (`board: {file: mk2/power_board}`,
`robot: talos`): files are looked for from the scenario's directory up first, then in `$FIRMWARE_SIMULATOR_HOME`,
then next to the installed binary (the archive's top directory). So your own `boards/` or `robots/` win where you
have them.

## The action

```yaml
- uses: ATK422/firmware_simulator@nightly     # or @v0.1.0 for a fixed release
  with:
    scenarios: tests/emulator/                # optional: files or directories to run
    junit: emulator-junit.xml                 # optional, the default is firmware_simulator-junit.xml
    args: --quiet                             # optional, more `scenario` arguments
```

| Input | Default | Meaning |
|---|---|---|
| `version` | the action's ref if it is a version tag, else `nightly` | Release to install |
| `scenarios` | (none: only install) | Scenario files or directories, as `firmware_simulator scenario` takes them |
| `junit` | `firmware_simulator-junit.xml` | Where the scenarios' JUnit XML goes |
| `args` | | More arguments for `scenario` (`--quiet`, `--jobs 2`, `--image DIR`) |

It puts `firmware_simulator` on `PATH` and sets `FIRMWARE_SIMULATOR_HOME` to the installed release (also the `home`
output; `version` is the release installed). It runs on Linux (x86_64 and arm64) and macOS runners. The step fails
if a scenario fails, with the scenario's evidence in the log.

## Example: a firmware repository

titan_firmware_rs images are found by name (`image: power_board`) in `TITAN_RS_RELEASE_DIR`; C images (UF2 or
ELF) by path, from the scenario's directory up. A scenario in your repository, `tests/emulator/power.yaml`:

```yaml
name: Power board boots and connects
board: {image: power_board, file: mk2/power_board}
bridges: [1]
time: 2s
steps:
  - expect: {log: ROS connected, within: 1s}
  - expect: {status: {faults: 0}, at: 1.5s}
```

and the workflow:

```yaml
name: Emulator
on: [push, pull_request]
jobs:
  emulator:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Firmware images
        run: |
          rustup target add thumbv6m-none-eabi
          cd firmware && cargo build --workspace --release
      - uses: ATK422/firmware_simulator@nightly
        env:
          TITAN_RS_RELEASE_DIR: ${{ github.workspace }}/firmware/target/thumbv6m-none-eabi/release
        with:
          scenarios: tests/emulator/
          junit: emulator-junit.xml
      - uses: actions/upload-artifact@v4
        if: always()
        with:
          name: emulator-junit
          path: emulator-junit.xml
```

## Example: nano_ros

nano-ROS support is early ([nanoros.md](nanoros.md)). A board running a libnanoros image talks to the emulator's
nano-ROS mock agent (`agents: [nanoros:BOARD]` in a scenario), which plays the host agent's side: the handshake,
decoding what the board publishes, sending it messages. A scenario next to your test image:

```yaml
name: LED board registers and publishes
board: {image: build/nanoros_led.elf, file: talos/led_board, name: led}
agents: [nanoros:led]
time: 2s
steps:
  - expect: {board: led, log: done reg, within: 500ms}
```

Build the image as your repository does (pico-sdk with ARM GCC, for example the `gcc-arm-none-eabi` package or
uwrt_nix's firmware shell), then run the scenarios with the action as above. A `ros.toml` with the board's topics
goes in the agent spec (`nanoros:led=ros.toml`) when the board's topics differ from the test board's
([robots.md](robots.md#mock-agents-nano-ros-and-other-protocols)).

## Without the action

```bash
curl -fsSLO https://github.com/ATK422/firmware_simulator/releases/download/nightly/firmware_simulator-x86_64-unknown-linux-gnu.tar.gz
tar -xzf firmware_simulator-x86_64-unknown-linux-gnu.tar.gz
./firmware_simulator-x86_64-unknown-linux-gnu/bin/firmware_simulator scenario tests/emulator/ --junit junit.xml
```

The same works on a laptop that has no Rust toolchain.
