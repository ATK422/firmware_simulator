# Fuzzing the firmware

`firmware_simulator fuzz` drives many seeded, deterministic cases of malformed and adversarial CAN traffic at a
board from one boot, and watches for the firmware falling over: a panic, a HardFault or lockup, a watchdog reset, a
hang, a fault bit raised and never cleared, a wild memory access, or an ERROR log line. Each distinct finding is
written out as a minimised scenario reproducer, so `firmware_simulator scenario reproducer.yaml` replays it
exactly.

It is firmware-agnostic: it asserts nothing about what a board *should* do, only that bus traffic alone should not
crash, hang or fault it. What it finds is written up below under [firmware findings](#firmware-findings) and pinned
by `firmware_finding_*` tests, not judged by the harness.

- [Running it](#running-it)
- [What it does to the board](#what-it-does-to-the-board)
- [Oracles](#oracles)
- [Coverage feedback and minimisation](#coverage-feedback-and-minimisation)
- [Reproducers](#reproducers)
- [From Rust](#from-rust)
- [CI](#ci)

## Running it

```bash
cargo build --release
alias firmware_simulator=$PWD/target/release/firmware_simulator
FW=external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release

# One board, a fixed number of cases, writing reproducers
firmware_simulator fuzz esc_board --cases 5000 --jobs 2 --out findings/

# Every board, a wall-time budget shared across them, confirming each reproducer replays
firmware_simulator fuzz --time 20m --jobs 2 --out findings/ --confirm
```

| Option | Default | What it does |
|---|---|---|
| `BOARD...` | every board | Boards to fuzz by image name (`esc_board`, `power_board`, ...); none means all |
| `--time DURATION` | | Wall-time budget, shared out over the boards that boot (give this or `--cases`) |
| `--cases N` | | Run exactly this many cases per board |
| `-j, --jobs N` | 2 | Worker threads per board |
| `--seed S` | 0 | The whole run is a deterministic function of this |
| `--window T` | 200ms | How long to drive fuzz traffic in each case |
| `--settle T` | 1200ms | Quiet time after the traffic stops, to tell a hang from a slow board |
| `--max-ops N` | 16 | Largest number of operations in a generated case |
| `--out DIR` | | Write a `*.yaml` scenario and a `*.json` record per distinct finding here |
| `--confirm` | | Run each reproducer's scenario and report whether it replayed |

The boards are the titan_firmware_rs images on a CAN bus, as the smoke test assigns them (`power_board`,
`esc_board`, `camera_cage_bb`, `actuator_board`, `led_board`, `sbc_board`, `ivc_board2`, `smart_battery`,
`poac_board`, `hbridge_board`, `po_half_board`). The Puddles backplane is Ethernet-only and not fuzzed here. A
board whose image is not built is skipped.

Output is one line per board (cases run, corpus size, edges reached, distinct findings, wall time) and a line per
finding with its kind, detail and the size of its minimised case:

```
board               cases     corpus   edges   findings  wall
esc_board              5000      214   18233         0  61.2s
power_board            5000      191    2291         1   54.8s
    [fault_latched] faults 0x8 (FAULT_ROS_ERROR)  (1 op(s))
```

Runs are deterministic: the same seed, board images and options give the same cases, coverage and findings on any
number of threads (only the wall time differs). Ctrl-C ends the campaign at the next case boundary.

## What it does to the board

Each case is a seeded, ordered list of operations on the board's bus, applied at virtual-time offsets inside the
fuzz window. The generator mixes structure-aware cases with raw mutation:

* **Raw CAN frames** — arbitrary identifiers, lengths, FD flags and data, as a babbling node or adversary would
  (`cansend`). Includes CANmore framing edge cases built to the board's own client: truncated and oversized
  fragments, out-of-order sequence numbers, bad CRCs, and well-formed frames with one field mutated.
* **Register-mapped control-interface writes** (CANmore channel 14) with wild flags, page and offset, probing the
  `reg_mapped` server's address handling.
* **Encoded `titan_link` bytes** — a mutated valid message or pure noise, framed to the client so the firmware's
  `titan_link` decoder sees it whatever it is (fuzzing the postcard decode and the board dispatch).
* **Well-formed host messages** — pings (including wrong protocol versions), kill-switch updates and a few board
  messages (the structured path).
* **Bus conditions** — bit-error storms, corrupt bursts, a stuck-dominant bus, an unplugged controller, a babbling
  flood at a chosen load, and kill-switch forwarding.

The board is booted once (next to a mock `titan_bridge` that keeps pinging), run until it is up, then snapshotted;
every case runs from a cheap fork of that snapshot ([internals.md](internals.md) "Checkpoints"). After the window,
the adversarial bus conditions are cleared, so a healthy board resumes answering and a genuine hang stands out.

## Oracles

After each case the harness compares the board to its clean boot:

| Finding | How it is detected |
|---|---|
| `panic` | A `PANIC`, `Reset by panic` or `HardFault` log line (the panic handler / crash record) |
| `lockup` | A core locked up, or is locked up at the end (a fault inside a fault handler, a branch to a bad address) |
| `watchdog_reset` | The chip reset, or a `Watchdog timeout` line, that the clean boot did not have |
| `hang` | The board answered before but sent no heartbeat in the settle window after the traffic stopped, with no reset to explain it |
| `fault_latched` | A `FAULT_*` bit raised that the clean boot did not have and that is still set at the end |
| `unmodelled_access` | The firmware touched a register the emulator does not model and a clean boot did not — often a wild pointer (could also be a gap in the emulator: investigate) |
| `error_log` | An ERROR-level log line |

Findings are deduplicated by a short, stable key (the kind plus the fault name, the register, or the gist of the
log line with its numbers masked), so many inputs hitting one bug collapse to one.

## Coverage feedback and minimisation

Coverage is a branch-edge bitmap ([`cpu_armv6m::edges`](../crates/cpu_armv6m/src/edges.rs)): every taken branch
marks the hash of its source and target, AFL style. It is cheap enough to leave on while the cores run batched (it
does not force single-stepping the way `--coverage` line coverage does), and it survives chip and core resets. An
input that reaches an edge no earlier case did is kept in the corpus and mutated further (tweak a field, add, drop,
splice from another corpus entry, duplicate for bursts, retime); otherwise fresh cases are generated from the seed.

A failing case is minimised before it is reported: drop operations one at a time while the finding still occurs
(ddmin over the op list), then shrink each remaining operation's data to the shortest prefix that still triggers
it. The reproducers are the minimised cases.

## Reproducers

With `--out DIR`, each distinct finding writes two files named `fuzz_<image>_<key>`:

* a **`.yaml` scenario** ([scenarios.md](scenarios.md)) that boots the board, replays the exact traffic at its
  virtual times (using `can_send` for raw frames, `send: {bytes}` for encoded `titan_link` bytes, `send:
  {message}` for host messages and `fault` for bus conditions), clears the bus conditions, and checks for the
  finding;
* a **`.json` record** with the finding and the operation list, so a case can be re-run without the YAML.

`can_send` and `send: {bytes}` were added to the scenario format for this (a raw-frame and raw-bytes action). Run a
reproducer with `firmware_simulator scenario fuzz_power_board_fault_FAULT_ROS_ERROR.yaml`; `--confirm` does this for
every finding as the campaign writes it.

## From Rust

`sim_test::fuzz` is the same engine as a library:

```rust,ignore
use sim_test::fuzz::{self, Campaign, Config};

let target = fuzz::target("esc_board").unwrap();
let campaign = Campaign::boot(&target, &Config::default()).unwrap();
let report = fuzz::run_campaign(&campaign, 5000, 2, &|| false);
for hit in &report.hits {
    let f = &hit.findings[0];
    println!("{}: {} ({} ops)", f.kind.word(), f.detail, hit.input.ops.len());
}
```

`Campaign::run_once` runs one `FuzzInput` and judges it; `Shared::minimize` shrinks a case against a finding key;
`fuzz::repro::scenario_yaml` / `record_json` write a reproducer. Generation and mutation are in `fuzz::cases`, the
oracles in `fuzz::oracle`.

## CI

`crates/sim_test/tests/fuzz_smoke.rs` boots one board, runs a fixed number of seeded cases on two threads, checks
it found the same thing twice, and that every reproducer replays through the scenario runner. It runs in
`cargo test --release --workspace` and skips (and passes) when the image is not built, so the harness does not rot.

## Firmware findings

See [faults.md](faults.md#firmware-findings-so-far) for the running list; the ones the fuzzer turned up are marked
there and pinned by `firmware_finding_fuzz_*` tests in `crates/sim_test/tests/faults.rs`.
