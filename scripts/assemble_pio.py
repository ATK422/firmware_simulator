#!/usr/bin/env python3
"""Assembles the PIO test programs with pioasm into test_programs/pio/*.prog, a plain format the tests read.

    scripts/build_pioasm.sh          # once
    python3 scripts/assemble_pio.py  # writes test_programs/pio/*.prog

Each .prog holds one program per `program NAME` block: `wrap_target N`, `wrap N`, `sideset BITS OPT PINDIRS`,
`define NAME VALUE` (public defines and labels) and one `instr XXXX` line per instruction. Firmware programs are
assembled from external/ in place (read only).
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIOASM = os.path.join(ROOT, "build/pioasm/pioasm")
OUT = os.path.join(ROOT, "test_programs/pio")
SOURCES = [os.path.join(OUT, f) for f in sorted(os.listdir(OUT)) if f.endswith(".pio")] + [
    os.path.join(ROOT, "external/titan_firmware_rs/firmware/esc_board/src/bidir_dshot.pio"),
    os.path.join(ROOT, "external/titan_firmware_rs/firmware/esc_board/src/uart_rx.pio"),
]


def convert(header):
    out = []
    for m in re.finditer(r"// -+ //\n// (\w+) //\n// -+ //\n(.*?)(?=\n// -+ //\n// \w+ //|\Z)", header, re.S):
        name, body = m.group(1), m.group(2)
        out.append(f"program {name}")
        wt = re.search(rf"#define {name}_wrap_target (\d+)", body)
        wr = re.search(rf"#define {name}_wrap (\d+)", body)
        out.append(f"wrap_target {wt.group(1)}")
        out.append(f"wrap {wr.group(1)}")
        ss = re.search(r"sm_config_set_sideset\(&c, (\d+), (true|false), (true|false)\)", body)
        if ss:
            out.append(f"sideset {ss.group(1)} {int(ss.group(2) == 'true')} {int(ss.group(3) == 'true')}")
        for d in re.finditer(rf"#define {name}_(?:offset_)?(\w+) (\d+)u?\n", body):
            if d.group(1) not in ("wrap_target", "wrap"):
                out.append(f"define {d.group(1)} {d.group(2)}")
        arr = re.search(r"_program_instructions\[\] = \{(.*?)\};", body, re.S).group(1)
        for ins in re.findall(r"0x([0-9a-f]{4}),", arr):
            out.append(f"instr {ins}")
        out.append("")
    return "\n".join(out)


def main():
    if not os.path.exists(PIOASM):
        sys.exit("build pioasm first: scripts/build_pioasm.sh")
    for src in SOURCES:
        header = subprocess.run([PIOASM, "-o", "c-sdk", src], check=True, capture_output=True, text=True).stdout
        name = os.path.splitext(os.path.basename(src))[0]
        if not src.startswith(OUT):
            name = "fw_" + name
        with open(os.path.join(OUT, name + ".prog"), "w") as f:
            f.write(f"# Assembled by pioasm from {os.path.relpath(src, ROOT)} (scripts/assemble_pio.py)\n")
            f.write(convert(header))
        print(name)


if __name__ == "__main__":
    main()
