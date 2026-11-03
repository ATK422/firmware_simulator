//! PIO programs assembled by pioasm (`scripts/assemble_pio.py`): pico-examples classics and the ESC board's own
//! `.pio` files, run on the PIO model and checked against the waveforms and data they are meant to produce.

use std::path::PathBuf;

use rp2040::periph::pio::{INSTR_MEM0, Pio, SM_STRIDE, SM0_CLKDIV, Stop, TXF0};
use sim_core::bus::WriteMode;

/// One assembled program.
#[derive(Debug, Default)]
struct Program {
    instr: Vec<u16>,
    wrap_target: u32,
    wrap: u32,
    /// (bits including the enable bit, optional, pindirs)
    sideset: Option<(u32, bool, bool)>,
    defines: Vec<(String, u32)>,
}

impl Program {
    fn define(&self, name: &str) -> u32 {
        self.defines.iter().find(|d| d.0 == name).unwrap().1
    }
}

fn load(file: &str, program: &str) -> Program {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test_programs/pio").join(file);
    let text = std::fs::read_to_string(&path).unwrap();
    let mut cur: Option<(String, Program)> = None;
    let mut found = None;
    for line in text.lines().chain(std::iter::once("program end")) {
        let f: Vec<&str> = line.split_whitespace().collect();
        match f.as_slice() {
            ["program", name] => {
                if let Some((n, p)) = cur.take()
                    && n == program
                {
                    found = Some(p);
                }
                cur = Some((name.to_string(), Program::default()));
            }
            ["wrap_target", v] => cur.as_mut().unwrap().1.wrap_target = v.parse().unwrap(),
            ["wrap", v] => cur.as_mut().unwrap().1.wrap = v.parse().unwrap(),
            ["sideset", b, o, d] => cur.as_mut().unwrap().1.sideset = Some((b.parse().unwrap(), *o == "1", *d == "1")),
            ["define", n, v] => cur.as_mut().unwrap().1.defines.push((n.to_string(), v.parse().unwrap())),
            ["instr", v] => cur.as_mut().unwrap().1.instr.push(u16::from_str_radix(v, 16).unwrap()),
            _ => {}
        }
    }
    found.unwrap_or_else(|| panic!("no program {program} in {file}"))
}

const CLKDIV: u32 = 0;
const EXECCTRL: u32 = 4;
const SHIFTCTRL: u32 = 8;
const INSTR: u32 = 0x10;
const PINCTRL: u32 = 0x14;

fn w(p: &mut Pio, off: u32, v: u32) {
    assert!(p.write(off, v, WriteMode::Normal, 0));
}

fn smreg(sm: usize, r: u32) -> u32 {
    SM0_CLKDIV + sm as u32 * SM_STRIDE + r
}

/// Loads `prog` at `offset` and configures state machine `sm` like pico-sdk's `pio_sm_init` with the program's
/// default config plus `pinctrl`, `shiftctrl`, an extra `execctrl` and a divider in 1/256ths. Not yet enabled.
#[allow(clippy::too_many_arguments)]
fn setup(
    p: &mut Pio,
    sm: usize,
    prog: &Program,
    offset: u32,
    pinctrl: u32,
    shiftctrl: u32,
    execctrl: u32,
    div256: u32,
) {
    for (i, &ins) in prog.instr.iter().enumerate() {
        // Relocate JMP targets like pio_add_program
        let ins = if ins >> 13 == 0 { (ins & !0x1F) | ((ins & 0x1F) + offset as u16) } else { ins };
        w(p, INSTR_MEM0 + 4 * (offset + i as u32), ins as u32);
    }
    let mut exec = execctrl | (prog.wrap + offset) << 12 | (prog.wrap_target + offset) << 7;
    let mut pc = pinctrl;
    if let Some((bits, opt, pindirs)) = prog.sideset {
        pc |= bits << 29;
        if opt {
            exec |= 1 << 30;
        }
        if pindirs {
            exec |= 1 << 29;
        }
    }
    w(p, smreg(sm, CLKDIV), div256 << 8);
    w(p, smreg(sm, EXECCTRL), exec);
    w(p, smreg(sm, SHIFTCTRL), shiftctrl);
    w(p, smreg(sm, PINCTRL), pc);
    // pio_sm_init: restart, clkdiv restart, jmp to the start
    w(p, 0, (1 << (4 + sm)) | (1 << (8 + sm)));
    w(p, smreg(sm, INSTR), offset);
}

fn enable(p: &mut Pio, mask: u32) {
    let cur = p.read(0).unwrap();
    w(p, 0, cur | mask);
}

/// Runs to `until`, with `input` changes (cycle, gpio levels) applied on time; returns (cycle, pad_out, pad_oe) at
/// every output change.
fn run(p: &mut Pio, until: u64, mut inputs: Vec<(u64, u32)>, mut level: u32) -> Vec<(u64, u32, u32)> {
    inputs.sort();
    let mut out = Vec::new();
    let mut i = 0;
    loop {
        let next_in = inputs.get(i).map_or(u64::MAX, |c| c.0);
        let target = until.min(next_in);
        match p.run(target, level) {
            Stop::Outputs => out.push((p.cycle, p.pad_out, p.pad_oe)),
            Stop::Reached if target == until && next_in > until => return out,
            Stop::Reached => {
                level = inputs[i].1;
                i += 1;
            }
        }
    }
}

/// High pulses on `pin` as (start, length) from an edge list.
fn pulses(edges: &[(u64, u32, u32)], pin: u32, high: bool) -> Vec<(u64, u64)> {
    let mut v = Vec::new();
    let mut start = None;
    let mut last = !high;
    for &(c, out, _) in edges {
        let l = out & (1 << pin) != 0;
        if l == last {
            continue;
        }
        last = l;
        if l == high {
            start = Some(c);
        } else if let Some(s) = start.take() {
            v.push((s, c - s));
        }
    }
    v
}

#[test]
fn squarewave() {
    let prog = load("squarewave.prog", "squarewave");
    let mut p = Pio::new(0);
    setup(&mut p, 0, &prog, 0, 1 << 26 | 3 << 5, 0x000C_0000, 0, 256);
    enable(&mut p, 1);
    let e = run(&mut p, 30, vec![], 0);
    assert_eq!(e[0].2, 1 << 3, "set pindirs first");
    // 4 cycles per period, 2 high
    let hp = pulses(&e, 3, true);
    assert!(hp.len() >= 6);
    for w in hp.windows(2) {
        assert_eq!((w[0].1, w[1].0 - w[0].0), (2, 4));
    }
}

#[test]
fn ws2812_bit_timing_and_data() {
    let prog = load("ws2812.prog", "ws2812");
    let (t1, t2, t3) = (prog.define("T1") as u64, prog.define("T2") as u64, prog.define("T3") as u64);
    let mut p = Pio::new(0);
    // Side-set pin 9, shift left, autopull at 24 bits; 800 kHz at 8 MHz SM clock from 125 MHz: 15.625
    setup(&mut p, 0, &prog, 4, 9 << 10, 1 << 17 | 24 << 25, 0, (125 * 256) / 8);
    let colours = [0xFF_00_80u32, 0x01_02_03];
    for c in colours {
        w(&mut p, TXF0, c << 8);
    }
    enable(&mut p, 1);
    let e = run(&mut p, 125 * 100, vec![], 0);
    let hp = pulses(&e, 9, true);
    assert_eq!(hp.len(), 48, "two 24-bit pixels");
    // Decode: a long pulse (T1 + T2 SM cycles) is a 1, a short one (T1) a 0
    let sm_cycle = 125.0 / 8.0;
    let mut bits = Vec::new();
    for (_, len) in &hp {
        let n = (*len as f64 / sm_cycle).round() as u64;
        assert!(n == t1 || n == t1 + t2, "pulse of {n} SM cycles");
        bits.push((n == t1 + t2) as u32);
    }
    let words: Vec<u32> = bits.chunks(24).map(|c| c.iter().fold(0, |a, b| a << 1 | b)).collect();
    assert_eq!(words, colours);
    // Bits are T1 + T2 + T3 = 10 SM cycles apart (1.25 us)
    let period = (hp[1].0 - hp[0].0) as f64 / sm_cycle;
    assert!((period - (t1 + t2 + t3) as f64).abs() < 1.0);
    // The line stays low while the state machine stalls on the empty FIFO
    assert_eq!(e.last().unwrap().1 & 1 << 9, 0);
}

/// Encodes 8N1 bytes as (cycle, level) changes on `pin`, `bit` cycles per bit, starting at `start`.
fn uart_wave(bytes: &[u8], pin: u32, bit: u64, start: u64, stop_ok: bool) -> Vec<(u64, u32)> {
    let mut v = Vec::new();
    let mut t = start;
    for &b in bytes {
        let mut frame = vec![false];
        frame.extend((0..8).map(|i| b & (1 << i) != 0));
        frame.push(stop_ok);
        for l in frame {
            v.push((t, if l { 1 << pin } else { 0 }));
            t += bit;
        }
        v.push((t, 1 << pin));
        t += bit;
    }
    v
}

#[test]
fn uart_tx_frames() {
    let prog = load("uart_tx.prog", "uart_tx");
    let mut p = Pio::new(1);
    // OUT pin 4 (count 1), side-set pin 4; shift right; 8 PIO cycles per bit, divider 2 -> 16 clk_sys per bit
    setup(&mut p, 2, &prog, 0, 4 | 1 << 20 | 4 << 10 | 4 << 5 | 1 << 26, 1 << 19, 0, 512);
    // As uart_tx_program_init: the pin an output, idle high (SET PINDIRS / SET PINS through SMx_INSTR)
    w(&mut p, smreg(2, INSTR), 0xE081);
    w(&mut p, smreg(2, INSTR), 0xE001);
    w(&mut p, TXF0 + 8, b'O' as u32);
    w(&mut p, TXF0 + 8, b'K' as u32);
    enable(&mut p, 1 << 2);
    let e = run(&mut p, 16 * 10 * 3, vec![], 0);
    // Sample the line at the middle of each 16-cycle bit and decode two frames
    let level_at = |c: u64| e.iter().rev().find(|x| x.0 <= c).is_some_and(|x| x.1 & (1 << 4) != 0);
    let start = e.iter().find(|x| x.1 & (1 << 4) == 0 && x.0 > 0).map(|x| x.0).unwrap();
    let mut bytes = Vec::new();
    let mut t = start;
    for _ in 0..2 {
        assert!(!level_at(t + 8), "start bit");
        let b = (0..8).fold(0u8, |a, i| a | (level_at(t + 8 + 16 * (i + 1)) as u8) << i);
        assert!(level_at(t + 8 + 16 * 9), "stop bit");
        bytes.push(b);
        t += 16 * 10;
    }
    assert_eq!(bytes, b"OK");
}

#[test]
fn uart_rx_bytes_and_framing_error() {
    let prog = load("uart_rx.prog", "uart_rx");
    let mut p = Pio::new(0);
    // IN pin 7, JMP pin 7; shift right; 8 PIO cycles per bit at divider 1
    setup(&mut p, 1, &prog, 0, 7 << 15, 1 << 18, 7 << 24, 256);
    enable(&mut p, 1 << 1);
    let mut wave = uart_wave(b"Hi!", 7, 8, 20, true);
    // Then a frame with a bad stop bit
    wave.extend(uart_wave(&[0x55], 7, 8, 500, false));
    run(&mut p, 800, wave, 1 << 7);
    let words: Vec<u8> = (0..3).map(|_| (p.read(0x024).unwrap() >> 24) as u8).collect();
    assert_eq!(words, b"Hi!");
    assert_eq!(p.read(0x00C).unwrap() >> 12 & 0xF, 0, "the bad frame was not pushed");
    assert_eq!(p.irq, 1 << 5, "irq 4 rel from SM1 is flag 5");
}

#[test]
fn esc_uart_rx_mini_autopush() {
    let prog = load("fw_uart_rx.prog", "uart_rx_mini");
    let mut p = Pio::new(1);
    // Shift right, autopush at 8, RX join
    setup(&mut p, 0, &prog, 0, 3 << 15, 1 << 31 | 1 << 18 | 1 << 16 | 8 << 20, 0, 256);
    enable(&mut p, 1);
    run(&mut p, 400, uart_wave(&[0xA5, 0x0F], 3, 8, 10, true), 1 << 3);
    assert_eq!(p.read(0x020).unwrap() >> 24, 0xA5);
    assert_eq!(p.read(0x020).unwrap() >> 24, 0x0F);
}

#[test]
fn handshake_between_state_machines() {
    let ping = load("handshake.prog", "ping");
    let pong = load("handshake.prog", "pong");
    let mut p = Pio::new(0);
    // SM0: side-set (pindirs) on pin 5; SM1: SET pin 6
    setup(&mut p, 0, &ping, 0, 5 << 10, 0x000C_0000, 0, 256);
    setup(&mut p, 1, &pong, 10, 1 << 26 | 6 << 5, 0x000C_0000, 0, 256);
    // Low half: "set x, 0b10110"; high half ignored
    w(&mut p, TXF0, 0xE036);
    enable(&mut p, 0b11);
    let e = run(&mut p, 40, vec![], 0);
    assert_eq!(p.read(0x020), Some(0b10110u32.reverse_bits()), "OUT EXEC set X, MOV ::X reversed it");
    // SM0 raised flag 0 with side-set pindir 1 on pin 5; SM1 consumed it and pulsed pin 6 for 3 cycles; then
    // SM0's wait completed and its side 0 released pin 5
    assert!(e.iter().any(|x| x.2 & 1 << 5 != 0));
    let hp = pulses(&e, 6, true);
    assert_eq!(hp.len(), 1);
    assert_eq!(hp[0].1, 3);
    assert_eq!(e.last().unwrap().2 & 1 << 5, 0);
    assert_eq!(p.irq, 0);
}

/// DShot600 frame bits as the ESC firmware queues them (`pio_word`): the 16-bit frame inverted and left-aligned.
fn dshot_word(frame: u16) -> u32 {
    ((!frame) as u32) << 16
}

#[test]
fn esc_bidir_dshot_frame_and_erpm_reply() {
    let prog = load("fw_bidir_dshot.prog", "bidir_dshot");
    let start = prog.define("start");
    let mut p = Pio::new(0);
    let pin = 6;
    // As init_dshot_sm: OUT/SET/IN/JMP pin 6, 40 PIO cycles per bit at 600 kHz from 125 MHz, shift left both
    // ways (OUT threshold 16, IN threshold 21), STATUS = TX level < 1
    let div = (125_000_000u64 * 256 / (600_000 * 40)) as u32;
    setup(&mut p, 0, &prog, 0, pin | 1 << 20 | pin << 5 | 1 << 26 | pin << 15, 16 << 25 | 21 << 20, pin << 24 | 1, div);
    w(&mut p, smreg(0, INSTR), start);
    enable(&mut p, 1);
    let frame: u16 = 0b1010_0101_1100_0011;
    w(&mut p, TXF0, dshot_word(frame));
    let sys_per_bit = div as f64 / 256.0 * 40.0;
    // Bidirectional DShot is inverted: idle high, each bit a low pulse of 15 (0) or 30 (1) PIO cycles
    let e = run(&mut p, (sys_per_bit * 17.0) as u64, vec![], 1 << pin);
    let low = pulses(&e, pin, false);
    assert_eq!(low.len(), 16);
    let pio_cycle = div as f64 / 256.0;
    let bits: Vec<u16> = low
        .iter()
        .map(|&(_, l)| {
            let n = (l as f64 / pio_cycle).round() as u64;
            assert!(n == 15 || n == 30, "{n}");
            (n == 30) as u16
        })
        .collect();
    assert_eq!(bits.iter().fold(0, |a, b| a << 1 | b), frame);
    // After the last bit the program releases the pin (SET PINDIRS 0) to listen
    let (end, _, oe) = *e.last().unwrap();
    assert_eq!(oe & 1 << pin, 0, "released for the reply");
    let last_rise = low.last().unwrap().0 + low.last().unwrap().1;
    assert!((end - last_rise) as f64 > 9.0 * pio_cycle, "after the 10-cycle bit tail");
    // The program then releases the pin and waits for the ESC's reply: 21 bits at 5/4 of the bit rate, 30 us later.
    // Send a level sequence with a start bit and check what the state machine pushes.
    let reply: u32 = 0b0_1101_0010_1011_0100_1110; // 21 bits, MSB first, the first is the start bit (low)
    let bit = sys_per_bit / 1.25;
    let t0 = end + (30e-6 * 125e6) as u64;
    let mut wave = Vec::new();
    for i in 0..21 {
        let l = reply >> (20 - i) & 1;
        wave.push((t0 + (i as f64 * bit) as u64, l << pin));
    }
    wave.push((t0 + (21.0 * bit) as u64, 1 << pin));
    run(&mut p, t0 + (25.0 * bit) as u64, wave, 1 << pin);
    assert_eq!(p.read(0x00C).unwrap() >> 4 & 0xF, 1, "one reply pushed");
    assert_eq!(p.read(0x020).unwrap() & 0x1F_FFFF, reply);
}
