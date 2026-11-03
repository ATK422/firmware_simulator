//! M1: the GDB stub breaks in `main`, reads registers and memory, single steps and stops on Ctrl-C, driven over
//! TCP by a minimal RSP client.

mod common;

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

use sim_host::gdb::{self, Ended};

struct Client {
    s: TcpStream,
}

impl Client {
    fn send_raw(&mut self, data: &[u8]) {
        self.s.write_all(data).unwrap();
    }

    fn cmd(&mut self, payload: &str) -> String {
        let sum = payload.bytes().fold(0u8, |a, b| a.wrapping_add(b));
        self.send_raw(format!("${payload}#{sum:02x}").as_bytes());
        self.reply()
    }

    fn reply(&mut self) -> String {
        let mut buf = Vec::new();
        let mut byte = [0u8; 1];
        loop {
            self.s.read_exact(&mut byte).unwrap();
            match byte[0] {
                b'+' if buf.is_empty() => continue,
                b'#' => {
                    let mut sum = [0u8; 2];
                    self.s.read_exact(&mut sum).unwrap();
                    self.send_raw(b"+");
                    let start = buf.iter().position(|&b| b == b'$').unwrap();
                    return String::from_utf8_lossy(&buf[start + 1..]).into_owned();
                }
                b => buf.push(b),
            }
        }
    }
}

fn le_hex_to_u32(s: &str) -> u32 {
    let b: Vec<u8> = (0..8).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect();
    u32::from_le_bytes(b.try_into().unwrap())
}

#[test]
fn break_in_main_step_and_interrupt() {
    let elf = common::repo().join("test_programs/embassy_blinky.elf");
    let image = sim_core::Image::load(&elf).unwrap();
    let main = image.symbol("main").expect("main symbol").addr & !1;
    let mut session = common::boot(&elf, None);
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = std::thread::spawn(move || {
        let ended = gdb::serve(&listener, &mut session).unwrap();
        (ended, session)
    });

    let mut c = Client { s: TcpStream::connect(("127.0.0.1", port)).unwrap() };
    assert!(c.cmd("qSupported:swbreak+").contains("qXfer:features:read+"));
    assert!(c.cmd("qXfer:features:read:target.xml:0,fff").contains("org.gnu.gdb.arm.m-profile"));
    assert_eq!(c.cmd("?"), "T05thread:1;");
    let regs = c.cmd("g");
    assert_eq!(regs.len(), 17 * 8);
    let pc = le_hex_to_u32(&regs[15 * 8..16 * 8]);
    assert_eq!(pc, image.symbol("Reset").unwrap().addr & !1, "halted at the reset handler");

    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), main, "stopped at main");
    assert_eq!(c.cmd(&format!("z0,{main:x},2")), "OK");

    // Memory: the vector table's reset vector
    let vec = c.cmd("m10000104,4");
    assert_eq!(le_hex_to_u32(&vec) & !1, image.symbol("Reset").unwrap().addr & !1);
    // PPB registers come from the core: VTOR
    assert_eq!(le_hex_to_u32(&c.cmd("me000ed08,4")), 0x1000_0100);

    // Single step moves the PC
    let before = le_hex_to_u32(&c.cmd("p0f"));
    assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
    assert_ne!(le_hex_to_u32(&c.cmd("p0f")), before);

    // Monitor command
    let out = c.cmd("qRcmd,74696d65"); // "time"
    assert!(out.starts_with('O'));
    assert_eq!(c.reply(), "OK");

    // Continue, then interrupt
    let sum = "c".bytes().fold(0u8, |a, b| a.wrapping_add(b));
    c.send_raw(format!("$c#{sum:02x}").as_bytes());
    std::thread::sleep(std::time::Duration::from_millis(50));
    c.send_raw(&[0x03]);
    assert_eq!(c.reply(), "T02thread:1;");

    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    let (ended, session) = server.join().unwrap();
    assert_eq!(ended, Ended::Detached);
    assert!(session.machine.now > sim_core::Time::ZERO);
    assert!(session.machine.breakpoints.is_empty());
}
