//! ARMv6-M Thumb disassembler for instruction traces. Output follows GNU `objdump` syntax closely enough to diff
//! by eye; branch targets are absolute.

const CONDS: [&str; 16] = ["eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc", "hi", "ls", "ge", "lt", "gt", "le", "", ""];

fn reg(n: u32) -> &'static str {
    ["r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"][n as usize]
}

fn reglist(list: u32) -> String {
    let regs: Vec<&str> = (0..16).filter(|i| list & (1 << i) != 0).map(reg).collect();
    format!("{{{}}}", regs.join(", "))
}

fn sext(value: u32, bits: u32) -> u32 {
    let shift = 32 - bits;
    ((value << shift) as i32 >> shift) as u32
}

/// Whether the halfword `hw1` starts a 32-bit instruction.
pub fn is_32bit(hw1: u16) -> bool {
    hw1 >> 11 >= 0b11101
}

fn sysreg(sysm: u32) -> String {
    match sysm {
        0 => "APSR".into(),
        1 => "IAPSR".into(),
        2 => "EAPSR".into(),
        3 => "xPSR".into(),
        5 => "IPSR".into(),
        6 => "EPSR".into(),
        7 => "IEPSR".into(),
        8 => "MSP".into(),
        9 => "PSP".into(),
        16 => "PRIMASK".into(),
        20 => "CONTROL".into(),
        n => format!("sysm{n}"),
    }
}

/// Disassembles the instruction at `pc`. `hw2` is only used for 32-bit encodings.
pub fn disassemble(pc: u32, hw1: u16, hw2: u16) -> String {
    let i = hw1 as u32;
    let pcv = pc.wrapping_add(4);
    let rd = i & 7;
    let rn = (i >> 3) & 7;
    match i >> 11 {
        0b00000 if (i >> 6) & 31 == 0 => format!("movs {}, {}", reg(rd), reg(rn)),
        0b00000 => format!("lsls {}, {}, #{}", reg(rd), reg(rn), (i >> 6) & 31),
        0b00001 => format!(
            "lsrs {}, {}, #{}",
            reg(rd),
            reg(rn),
            match (i >> 6) & 31 {
                0 => 32,
                n => n,
            }
        ),
        0b00010 => format!(
            "asrs {}, {}, #{}",
            reg(rd),
            reg(rn),
            match (i >> 6) & 31 {
                0 => 32,
                n => n,
            }
        ),
        0b00011 => {
            let op = if i & (1 << 9) != 0 { "subs" } else { "adds" };
            let f = (i >> 6) & 7;
            if i & (1 << 10) != 0 {
                format!("{op} {}, {}, #{f}", reg(rd), reg(rn))
            } else {
                format!("{op} {}, {}, {}", reg(rd), reg(rn), reg(f))
            }
        }
        0b00100 => format!("movs {}, #{}", reg((i >> 8) & 7), i & 0xFF),
        0b00101 => format!("cmp {}, #{}", reg((i >> 8) & 7), i & 0xFF),
        0b00110 => format!("adds {}, #{}", reg((i >> 8) & 7), i & 0xFF),
        0b00111 => format!("subs {}, #{}", reg((i >> 8) & 7), i & 0xFF),
        0b01000 if i & (1 << 10) == 0 => {
            let ops = [
                "ands", "eors", "lsls", "lsrs", "asrs", "adcs", "sbcs", "rors", "tst", "rsbs", "cmp", "cmn", "orrs",
                "muls", "bics", "mvns",
            ];
            let op = (i >> 6) & 0xF;
            match op {
                9 => format!("rsbs {}, {}, #0", reg(rd), reg(rn)),
                _ => format!("{} {}, {}", ops[op as usize], reg(rd), reg(rn)),
            }
        }
        0b01000 => {
            let rm = (i >> 3) & 0xF;
            let rdn = (i & 7) | ((i >> 4) & 8);
            match (i >> 8) & 3 {
                0 => format!("add {}, {}", reg(rdn), reg(rm)),
                1 => format!("cmp {}, {}", reg(rdn), reg(rm)),
                2 => format!("mov {}, {}", reg(rdn), reg(rm)),
                _ if i & (1 << 7) != 0 => format!("blx {}", reg(rm)),
                _ => format!("bx {}", reg(rm)),
            }
        }
        0b01001 => {
            format!("ldr {}, [pc, #{}] ; {:#010x}", reg((i >> 8) & 7), (i & 0xFF) * 4, (pcv & !3) + (i & 0xFF) * 4)
        }
        0b01010 | 0b01011 => {
            let ops = ["str", "strh", "strb", "ldrsb", "ldr", "ldrh", "ldrb", "ldrsh"];
            format!("{} {}, [{}, {}]", ops[((i >> 9) & 7) as usize], reg(rd), reg(rn), reg((i >> 6) & 7))
        }
        0b01100..=0b01111 => {
            let (op, scale) = [("str", 4), ("ldr", 4), ("strb", 1), ("ldrb", 1)][((i >> 11) & 3) as usize];
            format!("{op} {}, [{}, #{}]", reg(rd), reg(rn), ((i >> 6) & 31) * scale)
        }
        0b10000 | 0b10001 => {
            let op = if i & (1 << 11) != 0 { "ldrh" } else { "strh" };
            format!("{op} {}, [{}, #{}]", reg(rd), reg(rn), ((i >> 6) & 31) * 2)
        }
        0b10010 | 0b10011 => {
            let op = if i & (1 << 11) != 0 { "ldr" } else { "str" };
            format!("{op} {}, [sp, #{}]", reg((i >> 8) & 7), (i & 0xFF) * 4)
        }
        0b10100 => {
            format!("add {}, pc, #{} ; {:#010x}", reg((i >> 8) & 7), (i & 0xFF) * 4, (pcv & !3) + (i & 0xFF) * 4)
        }
        0b10101 => format!("add {}, sp, #{}", reg((i >> 8) & 7), (i & 0xFF) * 4),
        0b10110 | 0b10111 => match (i >> 8) & 0xF {
            0x0 if i & 0x80 != 0 => format!("sub sp, #{}", (i & 0x7F) * 4),
            0x0 => format!("add sp, #{}", (i & 0x7F) * 4),
            0x2 => format!("{} {}, {}", ["sxth", "sxtb", "uxth", "uxtb"][((i >> 6) & 3) as usize], reg(rd), reg(rn)),
            0x4 | 0x5 => format!("push {}", reglist((i & 0xFF) | ((i & 0x100) << 6))),
            0x6 if i & 0xFFEF == 0xB662 => {
                format!("cps{} i", if i & 0x10 != 0 { "id" } else { "ie" })
            }
            0xA if (i >> 6) & 3 != 2 => {
                format!("{} {}, {}", ["rev", "rev16", "", "revsh"][((i >> 6) & 3) as usize], reg(rd), reg(rn))
            }
            0xC | 0xD => format!("pop {}", reglist((i & 0xFF) | ((i & 0x100) << 7))),
            0xE => format!("bkpt {:#04x}", i & 0xFF),
            0xF if i & 0xF == 0 => {
                ["nop", "yield", "wfe", "wfi", "sev"].get(((i >> 4) & 0xF) as usize).unwrap_or(&"nop").to_string()
            }
            _ => format!("undefined {i:#06x}"),
        },
        0b11000 => format!("stmia {}!, {}", reg((i >> 8) & 7), reglist(i & 0xFF)),
        0b11001 => {
            let rn = (i >> 8) & 7;
            let wb = if i & (1 << rn) == 0 { "!" } else { "" };
            format!("ldmia {}{wb}, {}", reg(rn), reglist(i & 0xFF))
        }
        0b11010 | 0b11011 => match (i >> 8) & 0xF {
            0xE => format!("udf #{}", i & 0xFF),
            0xF => format!("svc {}", i & 0xFF),
            c => format!("b{} {:#010x}", CONDS[c as usize], pcv.wrapping_add(sext(i & 0xFF, 8) << 1)),
        },
        0b11100 => format!("b {:#010x}", pcv.wrapping_add(sext(i & 0x7FF, 11) << 1)),
        _ => {
            let h2 = hw2 as u32;
            if i >> 11 == 0b11110 && h2 & 0xD000 == 0xD000 {
                let s = (i >> 10) & 1;
                let i1 = !(((h2 >> 13) & 1) ^ s) & 1;
                let i2 = !(((h2 >> 11) & 1) ^ s) & 1;
                let imm = s << 24 | i1 << 23 | i2 << 22 | (i & 0x3FF) << 12 | (h2 & 0x7FF) << 1;
                format!("bl {:#010x}", pc.wrapping_add(4).wrapping_add(sext(imm, 25)))
            } else if i & 0xFFF0 == 0xF380 && h2 & 0xFF00 == 0x8800 {
                format!("msr {}, {}", sysreg(h2 & 0xFF), reg(i & 0xF))
            } else if i == 0xF3EF && h2 & 0xF000 == 0x8000 {
                format!("mrs {}, {}", reg((h2 >> 8) & 0xF), sysreg(h2 & 0xFF))
            } else if i == 0xF3BF && h2 & 0xFF00 == 0x8F00 {
                match (h2 >> 4) & 0xF {
                    4 => "dsb sy".into(),
                    5 => "dmb sy".into(),
                    6 => "isb sy".into(),
                    _ => format!("undefined {i:04x} {h2:04x}"),
                }
            } else if i & 0xFFF0 == 0xF7F0 && h2 & 0xF000 == 0xA000 {
                format!("udf.w #{}", (i & 0xF) << 12 | (h2 & 0xFFF))
            } else {
                format!("undefined {i:04x} {h2:04x}")
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples() {
        assert_eq!(disassemble(0, 0x2001, 0), "movs r0, #1");
        assert_eq!(disassemble(0, 0x4770, 0), "bx lr");
        assert_eq!(disassemble(0, 0xB510, 0), "push {r4, lr}");
        assert_eq!(disassemble(0, 0xBD10, 0), "pop {r4, pc}");
        assert_eq!(disassemble(0x100, 0xE7FE, 0), "b 0x00000100");
        assert_eq!(disassemble(0x100, 0xD0FE, 0), "beq 0x00000100");
        assert_eq!(disassemble(0, 0xBF30, 0), "wfi");
        assert_eq!(disassemble(0, 0xB672, 0), "cpsid i");
        assert_eq!(disassemble(0x1000, 0xF000, 0xF800), "bl 0x00001004");
        assert_eq!(disassemble(0, 0xF3EF, 0x8010), "mrs r0, PRIMASK");
        assert_eq!(disassemble(0, 0xF380, 0x8814), "msr CONTROL, r0");
        assert_eq!(disassemble(0, 0xF3BF, 0x8F5F), "dmb sy");
        assert_eq!(disassemble(0, 0x4348, 0), "muls r0, r1");
        assert_eq!(disassemble(0, 0x4249, 0), "rsbs r1, r1, #0");
        assert!(is_32bit(0xF000));
        assert!(!is_32bit(0xE7FE));
    }
}
