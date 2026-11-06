#!/usr/bin/env bash
# Optional: builds the emulator with profile-guided optimisation (PGO), trained on the bench, in the `fast` cargo
# profile (whole-program LTO, see Cargo.toml). Normal builds never need this.
#
#   rustup component add llvm-tools      # once: llvm-profdata
#   export TITAN_RS_RELEASE_DIR=...      # the titan_firmware_rs images the bench runs (see crates/sim_test)
#   scripts/build_pgo.sh                 # the firmware_simulator binary, in target/fast/
#   scripts/build_pgo.sh -p sim_test --example bench   # or anything else `cargo build` takes
#
# It builds the bench instrumented, runs it for PGO_TRAIN_TIME of virtual time per case (default 1s; with
# TITAN_FIRMWARE_DIR set the C IVCBoard2 cases train too), merges the profile into target/pgo/merged.profdata and
# builds what the arguments name with it. The emulator does the same as without PGO (the bench's fingerprints do
# not change); only its speed does. Rebuild after changing the emulator: a stale profile still builds, but helps
# less.
set -euo pipefail

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
fi
cd "$(dirname "$0")/.."
profdata="$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin/llvm-profdata"
if [ ! -x "$profdata" ]; then
    echo "llvm-profdata not found ($profdata): rustup component add llvm-tools" >&2
    exit 1
fi
target="${CARGO_TARGET_DIR:-target}"
case "$target" in
/*) data="$target/pgo" ;;
*) data="$PWD/$target/pgo" ;;
esac
rm -rf "$data"
mkdir -p "$data"

RUSTFLAGS="${RUSTFLAGS:-} -Cprofile-generate=$data/raw" cargo build --profile fast -p sim_test --example bench
# (PGO_BENCH_ARGS: more bench arguments, e.g. "--only esc_board,talos" to train on some cases)
# shellcheck disable=SC2086
"$target/fast/examples/bench" --time "${PGO_TRAIN_TIME:-1s}" --repeat 1 ${PGO_BENCH_ARGS:-}
"$profdata" merge -o "$data/merged.profdata" "$data/raw"

# (Functions the training did not reach have no profile: not a problem.)
RUSTFLAGS="${RUSTFLAGS:-} -Cprofile-use=$data/merged.profdata -Cllvm-args=-pgo-warn-missing-function=false" \
    cargo build --profile fast "$@"
