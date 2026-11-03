#!/bin/sh
# Builds pioasm from titan_firmware's pico-sdk into build/pioasm (nothing under external/ is touched).
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SRC="$ROOT/external/titan_firmware/lib/pico-sdk/tools/pioasm"
OUT="$ROOT/build/pioasm"
if [ ! -f "$SRC/CMakeLists.txt" ]; then
    echo "pico-sdk not checked out: git -C external/titan_firmware submodule update --init lib/pico-sdk" >&2
    exit 1
fi
cmake -S "$SRC" -B "$OUT" -G Ninja -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$OUT" >/dev/null
echo "$OUT/pioasm"
