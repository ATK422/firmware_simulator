//! Microchip MCP2517FD / MCP2518FD: an SPI-attached CAN FD controller (datasheet DS20005688, family reference
//! DS20005678).
//!
//! Modelled: the SPI instruction set (RESET, READ, WRITE, READ_CRC, WRITE_CRC, WRITE_SAFE with CRC-16/CMS), the
//! oscillator/PLL (ready only if CLKIN really carries a clock), operating modes, RAM allocation of the TEF, TXQ and
//! FIFOs, FIFO head/tail/user address semantics (UINC, TXREQ, FRESET), acceptance filters and masks, transmit
//! priority and retransmission attempts, the transmit event FIFO with sequence numbers and timestamps, receive
//! timestamps, the interrupt flag aggregation (C1INT, C1VEC, C1RXIF/TXIF/...) driving the INT pin, fault
//! confinement, IOCON GPIO inputs and the time base counter. Bit timing comes from NBTCFG/DBTCFG and SYSCLK, so a
//! node configured for the wrong rate does not reach the bus. Not modelled: ECC errors (never reported), sleep and
//! wake-up, TDC, the INT0/INT1 pins, internal loopback.
//!
//! Fault confinement follows ISO 11898-1 as the datasheet's C1TREC describes it: TEC +8 for a transmit error
//! (bit error, or an ACK error while error active), -1 for a frame sent; REC +1 for a receive error, -1 for a frame
//! received (back to 120 from above 127); +8 for every 8 dominant bits after an error flag on a stuck bus.
//! Warning above 95, error passive above 127, bus-off above 255 (TXBO): the node stops sending, receiving and
//! acknowledging, and recovers by itself after 128 sequences of 11 recessive bits ("device went to bus-off and
//! auto-recovered", C1BDIAG1.TXBOERR), with both counters back at 0. In configuration mode TXBO reads set ("the
//! module is not on the bus"). Every change of the error state sets CERRIF; every error IVMIF; C1BDIAG0 counts the
//! errors by phase and C1BDIAG1 flags their kind and counts error-free messages. While bus-off, TEC reads 255 (the
//! datasheet does not say what it shows). Entering configuration mode resets the protocol engine, clearing both
//! counters.

use sim_core::time::Time;
use sim_core::wire::{External, Level};

use crate::DeviceCtx;
use crate::can::{BitRates, BusError, CanFrame, CanNode, TxRequest, fd_dlc_to_len, fd_len_to_dlc};
use crate::spi::SpiDevice;

/// How the controller is wired on the board.
#[derive(Clone, Debug)]
pub struct Mcp251xfdConfig {
    /// SoC pin that drives CLKIN (a GPOUT clock), if the clock comes from the SoC.
    pub clkin_pin: Option<u8>,
    /// Frequency of a crystal or oscillator on OSC1, if the clock does not come from the SoC.
    pub crystal_hz: Option<u64>,
    /// SoC pin the INT output is wired to.
    pub int_pin: u8,
    /// Level the GPIO0 input senses (the termination jumper on the UWRT CAN block).
    pub gpio0: bool,
    pub gpio1: bool,
    /// MCP2518FD reports a device ID; the MCP2517FD reads zero there.
    pub mcp2518: bool,
}

impl Default for Mcp251xfdConfig {
    fn default() -> Self {
        Mcp251xfdConfig {
            clkin_pin: Some(21),
            crystal_hz: None,
            int_pin: 22,
            gpio0: true,
            gpio1: false,
            mcp2518: false,
        }
    }
}

const RAM_START: u16 = 0x400;
const RAM_SIZE: usize = 2048;

// C1CON
const CON_RTXAT: u32 = 1 << 16;
const CON_STEF: u32 = 1 << 19;
const CON_TXQEN: u32 = 1 << 20;
const CON_RESET: u32 = 0x0498_0760;

#[allow(dead_code)]
mod mode {
    pub const NORMAL_FD: u8 = 0;
    pub const SLEEP: u8 = 1;
    pub const INT_LOOPBACK: u8 = 2;
    pub const LISTEN_ONLY: u8 = 3;
    pub const CONFIG: u8 = 4;
    pub const EXT_LOOPBACK: u8 = 5;
    pub const NORMAL_CAN20: u8 = 6;
    pub const RESTRICTED: u8 = 7;
}

// C1INT flag bits
const INT_TXIF: u16 = 1 << 0;
const INT_RXIF: u16 = 1 << 1;
const INT_TBCIF: u16 = 1 << 2;
const INT_MODIF: u16 = 1 << 3;
const INT_TEFIF: u16 = 1 << 4;
const INT_SPICRCIF: u16 = 1 << 9;
const INT_CERRIF: u16 = 1 << 13;
const INT_IVMIF: u16 = 1 << 15;
const INT_TXATIF: u16 = 1 << 10;
const INT_RXOVIF: u16 = 1 << 11;
/// Flags that latch and are cleared by writing 0 (the others mirror their sources).
const INT_STICKY: u16 = INT_TBCIF | INT_MODIF | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);

// FIFOCON
const FIFO_TFNRFNIE: u32 = 1 << 0;
const FIFO_TFHRFHIE: u32 = 1 << 1;
const FIFO_TFERFFIE: u32 = 1 << 2;
const FIFO_RXOVIE: u32 = 1 << 3;
const FIFO_TXATIE: u32 = 1 << 4;
const FIFO_RXTSEN: u32 = 1 << 5;
const FIFO_TXEN: u32 = 1 << 7;
const FIFO_UINC: u32 = 1 << 8;
const FIFO_TXREQ: u32 = 1 << 9;
const FIFO_FRESET: u32 = 1 << 10;
// FIFOSTA sticky flags
const STA_RXOVIF: u32 = 1 << 3;
const STA_TXATIF: u32 = 1 << 4;

const TEF_TEFTSEN: u32 = 1 << 5;

const SPI_RESET: u8 = 0x0;
const SPI_WRITE: u8 = 0x2;
const SPI_READ: u8 = 0x3;
const SPI_WRITE_CRC: u8 = 0xA;
const SPI_READ_CRC: u8 = 0xB;
const SPI_WRITE_SAFE: u8 = 0xC;

/// CRC-16/CMS: polynomial 0x8005, init 0xFFFF, no reflection (what the controller uses for its CRC commands).
pub fn crc16_cms(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x8005 } else { crc << 1 };
        }
    }
    crc
}

fn payload_size(plsize: u32) -> usize {
    [8, 12, 16, 20, 24, 32, 48, 64][(plsize & 7) as usize]
}

/// One of the TEF, TXQ or FIFO1-31.
#[derive(Clone, Debug, Default)]
struct Fifo {
    con: u32,
    /// Sticky FIFOSTA flags.
    sticky: u32,
    /// RAM offset of the first object, if the FIFO got RAM.
    base: Option<u16>,
    depth: u8,
    obj_size: u16,
    head: u8,
    tail: u8,
    count: u8,
    /// Transmission requested and when.
    tx_requested: Option<Time>,
    attempts: u8,
}

impl Fifo {
    fn is_tx(&self) -> bool {
        self.con & FIFO_TXEN != 0
    }

    fn reset_contents(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
        self.sticky = 0;
        self.tx_requested = None;
        self.attempts = 0;
    }

    fn obj_addr(&self, index: u8) -> Option<u16> {
        self.base.map(|b| b + index as u16 * self.obj_size)
    }

    /// FIFOSTA status flags (not empty/full and half levels, plus sticky flags) and the index.
    fn status(&self, tx: bool) -> u32 {
        let depth = self.depth as u32;
        let count = self.count as u32;
        let mut v = self.sticky;
        if tx {
            if count < depth {
                v |= 1; // TFNRFNIF: not full
            }
            if count * 2 <= depth {
                v |= 2; // TFHRFHIF: at least half empty
            }
            if count == 0 {
                v |= 4; // TFERFFIF: empty
            }
            v |= (self.tail as u32) << 8;
        } else {
            if count > 0 {
                v |= 1; // not empty
            }
            if count * 2 >= depth && count > 0 {
                v |= 2; // at least half full
            }
            if count == depth && depth > 0 {
                v |= 4; // full
            }
            v |= (self.head as u32) << 8;
        }
        v
    }

    /// Interrupt pending from this FIFO's enabled flags.
    fn irq(&self, tx: bool) -> bool {
        let st = self.status(tx) & 7;
        st & self.con & (FIFO_TFNRFNIE | FIFO_TFHRFHIE | FIFO_TFERFFIE) != 0
    }
}

/// SPI transaction state.
#[derive(Clone, Debug, Default)]
enum Spi {
    #[default]
    Idle,
    Header(Vec<u8>),
    Read(u16),
    Write(u16),
    /// Instruction, address, then the remaining parameters collected so far.
    Crc {
        cmd: u8,
        addr: u16,
        bytes: Vec<u8>,
        out: Vec<u8>,
        pos: usize,
    },
    Reset,
    Ignore,
}

#[derive(Clone, Debug)]
pub struct Mcp251xfd {
    pub config: Mcp251xfdConfig,
    con: u32,
    opmod: u8,
    nbtcfg: u32,
    dbtcfg: u32,
    tdc: u32,
    tscon: u32,
    tbc_base: u32,
    tbc_time: Time,
    int_enable: u16,
    int_sticky: u16,
    /// Error counters. TEC goes past 255 into bus-off.
    tec: u16,
    rec: u16,
    bus_off: bool,
    /// 11-recessive-bit sequences seen while bus-off.
    recovery: u64,
    /// Consecutive dominant bits on a stuck bus, and whether the node was transmitting when it got stuck.
    dominant_run: u64,
    stuck_transmitter: bool,
    bdiag: [u32; 2],
    tef: Fifo,
    /// Index 0 is the TXQ, 1-31 the FIFOs.
    fifo: [Fifo; 32],
    fltcon: [u8; 32],
    fltobj: [u32; 32],
    mask: [u32; 32],
    osc: u32,
    iocon: u32,
    crc_reg: u32,
    ecccon: u32,
    eccstat: u32,
    ram: Vec<u8>,
    spi: Spi,
    selected: bool,
    /// SYSCLK as of the last access (for timestamps computed outside SPI accesses).
    sysclk: u64,
    filhit: u8,
    int_driven: Option<bool>,
    /// MOSI/MISO bytes of the current transaction, for trace logging.
    xfer_log: Vec<(u8, u8)>,
    /// Bus deliveries since the device last updated its pins.
    pub frames_received: u64,
    pub frames_sent: u64,
    /// Times the controller went bus-off, and transmit and receive errors counted (for tests).
    pub bus_off_events: u32,
    pub tx_errors: u64,
    pub rx_errors: u64,
}

/// Fault confinement state of a CAN controller.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ErrorState {
    Active,
    /// TEC or REC above 95.
    Warning,
    /// TEC or REC above 127: passive error flags, no ACK errors counted.
    Passive,
    BusOff,
}

/// The error counters and state, as C1TREC shows them.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ErrorCounters {
    pub tec: u8,
    pub rec: u8,
    pub state: ErrorState,
}

impl Mcp251xfd {
    pub fn new(config: Mcp251xfdConfig) -> Self {
        let mut m = Mcp251xfd {
            config,
            con: CON_RESET,
            opmod: mode::CONFIG,
            nbtcfg: 0,
            dbtcfg: 0,
            tdc: 0,
            tscon: 0,
            tbc_base: 0,
            tbc_time: Time::ZERO,
            int_enable: 0,
            int_sticky: 0,
            tec: 0,
            rec: 0,
            bus_off: false,
            recovery: 0,
            dominant_run: 0,
            stuck_transmitter: false,
            bdiag: [0; 2],
            tef: Fifo::default(),
            fifo: Default::default(),
            fltcon: [0; 32],
            fltobj: [0; 32],
            mask: [0; 32],
            osc: 0,
            iocon: 0,
            crc_reg: 0,
            ecccon: 0,
            eccstat: 0,
            ram: vec![0; RAM_SIZE],
            spi: Spi::Idle,
            selected: false,
            sysclk: 0,
            filhit: 0,
            int_driven: None,
            xfer_log: Vec::new(),
            frames_received: 0,
            frames_sent: 0,
            bus_off_events: 0,
            tx_errors: 0,
            rx_errors: 0,
        };
        m.reset();
        m
    }

    /// Device reset (RESET instruction or power-on). RAM contents survive.
    fn reset(&mut self) {
        self.con = CON_RESET;
        self.opmod = mode::CONFIG;
        self.nbtcfg = 0x003E_0F0F;
        self.dbtcfg = 0x000E_0303;
        self.tdc = 0x0002_1000;
        self.tscon = 0;
        self.tbc_base = 0;
        self.int_enable = 0;
        self.int_sticky = 0;
        self.reset_protocol_engine();
        self.bdiag = [0; 2];
        self.tef = Fifo { con: 0x0000_0400 & !FIFO_FRESET, ..Fifo::default() };
        for (i, f) in self.fifo.iter_mut().enumerate() {
            *f = Fifo { con: if i == 0 { 0x0060_0000 | FIFO_TXEN } else { 0x0060_0000 }, ..Fifo::default() };
        }
        self.fltcon = [0; 32];
        self.fltobj = [0; 32];
        self.mask = [0; 32];
        self.osc = 0x0000_0060;
        self.iocon = 0x0000_0003;
        self.crc_reg = 0;
        self.ecccon = 0;
        self.eccstat = 0;
        self.filhit = 0;
    }

    /// Power lost and restored (a brownout of the board's 3.3 V rail): a reset, with RAM contents lost and no
    /// SPI transaction in progress.
    pub fn power_on_reset(&mut self) {
        self.reset();
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        for b in self.ram.iter_mut() {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            *b = x as u8;
        }
        self.spi = Spi::Idle;
        self.selected = false;
        self.int_driven = None;
    }

    /// Clears fault confinement (device reset, or entering configuration mode).
    fn reset_protocol_engine(&mut self) {
        self.tec = 0;
        self.rec = 0;
        self.bus_off = false;
        self.recovery = 0;
        self.dominant_run = 0;
        self.stuck_transmitter = false;
    }

    /// Error counters and state.
    pub fn error_counters(&self) -> ErrorCounters {
        let state = if self.bus_off {
            ErrorState::BusOff
        } else if self.tec > 127 || self.rec > 127 {
            ErrorState::Passive
        } else if self.tec > 95 || self.rec > 95 {
            ErrorState::Warning
        } else {
            ErrorState::Active
        };
        ErrorCounters { tec: self.tec.min(255) as u8, rec: self.rec.min(255) as u8, state }
    }

    fn error_passive(&self) -> bool {
        self.tec > 127 || self.rec > 127
    }

    /// The error state bits of C1TREC (EWARN to TXBO) from the counters alone.
    fn trec_state_bits(&self) -> u32 {
        let mut v = 0;
        if self.rec > 95 && self.rec < 128 {
            v |= 1 << 17;
        }
        if self.tec > 95 && self.tec < 128 {
            v |= 1 << 18;
        }
        if v != 0 {
            v |= 1 << 16;
        }
        if self.rec > 127 {
            v |= 1 << 19;
        }
        if self.tec > 127 {
            v |= 1 << 20;
        }
        if self.bus_off {
            v |= 1 << 21;
        }
        v
    }

    /// Changes the counters with `f`, then handles bus-off and flags a change of error state (CERRIF).
    fn confine(&mut self, f: impl FnOnce(&mut Self)) {
        let before = self.trec_state_bits();
        f(self);
        self.rec = self.rec.min(255);
        if self.tec > 255 && !self.bus_off {
            self.bus_off = true;
            self.recovery = 0;
            self.bus_off_events += 1;
            self.bdiag[1] |= 1 << 23; // TXBOERR
            log::debug!("MCP251xFD bus-off");
        }
        if self.trec_state_bits() != before {
            self.int_sticky |= INT_CERRIF;
        }
    }

    /// Counts an error in C1BDIAG0 (`shift` selects NRERRCNT, NTERRCNT, DRERRCNT or DTERRCNT) and flags it in
    /// C1BDIAG1.
    fn diag(&mut self, shift: u32, flag: u32) {
        let c = (self.bdiag[0] >> shift) & 0xFF;
        if c < 0xFF {
            self.bdiag[0] += 1 << shift;
        }
        self.bdiag[1] |= flag;
        self.int_sticky |= INT_IVMIF;
    }

    fn error_free_message(&mut self) {
        let n = (self.bdiag[1] & 0xFFFF).wrapping_add(1) & 0xFFFF;
        self.bdiag[1] = (self.bdiag[1] & !0xFFFF) | n;
    }

    /// A transmission of FIFO `i` failed; gives up once the retransmission attempts are used up.
    fn tx_failed(&mut self, i: usize) {
        let limit = self.retransmit_limit(i);
        let f = &mut self.fifo[i];
        f.attempts = f.attempts.saturating_add(1);
        if limit.is_some_and(|l| f.attempts >= l) {
            f.sticky |= STA_TXATIF | 1 << 5;
            f.tx_requested = None;
            f.attempts = 0;
        }
    }

    fn on_bus(&self) -> bool {
        matches!(self.opmod, mode::NORMAL_FD | mode::NORMAL_CAN20 | mode::RESTRICTED | mode::EXT_LOOPBACK)
            && self.sysclk > 0
    }

    // ------------------------------------------------------------------ clocks and timing

    fn clkin_hz(&self, ctx: &dyn DeviceCtx) -> u64 {
        match (self.config.crystal_hz, self.config.clkin_pin) {
            (Some(hz), _) => hz,
            (None, Some(pin)) => ctx.pin_clock_hz(pin),
            (None, None) => 0,
        }
    }

    fn compute_sysclk(&self, clkin: u64) -> u64 {
        if self.osc & (1 << 2) != 0 {
            return 0; // OSCDIS
        }
        let mut f = clkin;
        if self.osc & 1 != 0 {
            f *= 10;
        }
        if self.osc & (1 << 4) != 0 {
            f /= 2;
        }
        f
    }

    fn refresh_clock(&mut self, ctx: &dyn DeviceCtx) {
        let clkin = self.clkin_hz(ctx);
        let new = self.compute_sysclk(clkin);
        if new != self.sysclk {
            // Keep the time base continuous across clock changes
            self.tbc_base = self.tbc(ctx.now());
            self.tbc_time = ctx.now();
            self.sysclk = new;
        }
    }

    fn tbc(&self, now: Time) -> u32 {
        if self.tscon & (1 << 16) == 0 || self.sysclk == 0 {
            return self.tbc_base;
        }
        let pre = (self.tscon & 0x3FF) as u64 + 1;
        let elapsed = now.saturating_sub(self.tbc_time).as_ps() as u128;
        let ticks = elapsed * self.sysclk as u128 / pre as u128 / 1_000_000_000_000;
        self.tbc_base.wrapping_add(ticks as u32)
    }

    /// Bit rates from the bit timing registers.
    fn rates(&self) -> (u64, u64) {
        let rate = |cfg: u32, tseg1_mask: u32, tseg2_mask: u32| -> u64 {
            let brp = (cfg >> 24) as u64 + 1;
            let tseg1 = ((cfg >> 16) & tseg1_mask) as u64 + 1;
            let tseg2 = ((cfg >> 8) & tseg2_mask) as u64 + 1;
            self.sysclk / (brp * (1 + tseg1 + tseg2))
        };
        (rate(self.nbtcfg, 0xFF, 0x7F), rate(self.dbtcfg, 0x1F, 0xF))
    }

    // ------------------------------------------------------------------ interrupts

    fn int_flags(&self) -> u16 {
        let mut f = self.int_sticky;
        if self.c1_txif() != 0 {
            f |= INT_TXIF;
        }
        if self.c1_rxif() != 0 {
            f |= INT_RXIF;
        }
        if self.tef_irq() {
            f |= INT_TEFIF;
        }
        if self.c1_txatif() != 0 {
            f |= INT_TXATIF;
        }
        if self.c1_rxovif() != 0 {
            f |= INT_RXOVIF;
        }
        if self.crc_reg & 0x0003_0000 & (self.crc_reg >> 8) != 0 {
            f |= INT_SPICRCIF;
        }
        f
    }

    fn c1_txif(&self) -> u32 {
        let mut v = 0;
        for (i, f) in self.fifo.iter().enumerate() {
            let is_tx = i == 0 || f.is_tx();
            if is_tx && (i != 0 || self.con & CON_TXQEN != 0) && f.base.is_some() && f.irq(true) {
                v |= 1 << i;
            }
        }
        v
    }

    fn c1_rxif(&self) -> u32 {
        let mut v = 0;
        for (i, f) in self.fifo.iter().enumerate().skip(1) {
            if !f.is_tx() && f.base.is_some() && f.irq(false) {
                v |= 1 << i;
            }
        }
        v
    }

    fn c1_rxovif(&self) -> u32 {
        let mut v = 0;
        for (i, f) in self.fifo.iter().enumerate().skip(1) {
            if f.sticky & STA_RXOVIF != 0 && f.con & FIFO_RXOVIE != 0 {
                v |= 1 << i;
            }
        }
        v
    }

    fn c1_txatif(&self) -> u32 {
        let mut v = 0;
        for (i, f) in self.fifo.iter().enumerate() {
            if f.sticky & STA_TXATIF != 0 && f.con & FIFO_TXATIE != 0 {
                v |= 1 << i;
            }
        }
        v
    }

    fn tef_status(&self) -> u32 {
        let t = &self.tef;
        let mut v = t.sticky & (1 << 3);
        if t.count > 0 {
            v |= 1;
        }
        if t.count as u32 * 2 >= t.depth as u32 && t.count > 0 {
            v |= 2;
        }
        if t.count == t.depth && t.depth > 0 {
            v |= 4;
        }
        v
    }

    fn tef_irq(&self) -> bool {
        self.tef_status() & self.tef.con & 0xF != 0
    }

    fn txreq_bits(&self) -> u32 {
        let mut v = 0;
        for (i, f) in self.fifo.iter().enumerate() {
            if f.tx_requested.is_some() {
                v |= 1 << i;
            }
        }
        v
    }

    fn vec(&self) -> u32 {
        let flags = self.int_flags() & self.int_enable;
        let tx = self.c1_txif();
        let rx = self.c1_rxif();
        let lowest = |m: u32| if m == 0 { 0x40 } else { m.trailing_zeros() };
        let icode = if flags & (INT_TXIF | INT_RXIF) != 0 {
            lowest((tx | rx) & if flags & INT_TXIF != 0 { u32::MAX } else { !tx })
        } else if flags & (1 << 13) != 0 {
            0x41
        } else if flags & (1 << 14) != 0 {
            0x42
        } else if flags & INT_RXOVIF != 0 {
            0x43
        } else if flags & INT_TBCIF != 0 {
            0x46
        } else if flags & INT_MODIF != 0 {
            0x47
        } else if flags & (1 << 15) != 0 {
            0x48
        } else if flags & INT_TEFIF != 0 {
            0x49
        } else if flags & INT_TXATIF != 0 {
            0x4A
        } else {
            0x40
        };
        lowest(rx) << 24 | lowest(tx) << 16 | (self.filhit as u32) << 8 | icode
    }

    /// Whether the INT output is asserted (active low).
    pub fn int_asserted(&self) -> bool {
        self.int_flags() & self.int_enable != 0
    }

    fn drive_int(&mut self, ctx: &mut dyn DeviceCtx) {
        let asserted = self.int_asserted();
        if self.int_driven != Some(asserted) {
            self.int_driven = Some(asserted);
            let open_drain = self.iocon & (1 << 30) != 0;
            let level = if asserted {
                External::Driven(Level::Low)
            } else if open_drain {
                External::Floating
            } else {
                External::Driven(Level::High)
            };
            ctx.set_pin(self.config.int_pin, level);
        }
    }

    // ------------------------------------------------------------------ RAM allocation

    fn allocate_ram(&mut self) {
        let mut addr: usize = 0;
        let place = |f: &mut Fifo, used: bool, obj_size: usize, addr: &mut usize| {
            let depth = ((f.con >> 24) & 0x1F) as usize + 1;
            f.depth = depth as u8;
            f.obj_size = obj_size as u16;
            f.reset_contents();
            if used && *addr + depth * obj_size <= RAM_SIZE {
                f.base = Some(*addr as u16);
                *addr += depth * obj_size;
            } else {
                f.base = None;
            }
        };
        let tef_size = if self.tef.con & TEF_TEFTSEN != 0 { 12 } else { 8 };
        let stef = self.con & CON_STEF != 0;
        place(&mut self.tef, stef, tef_size, &mut addr);
        let txq_size = 8 + payload_size(self.fifo[0].con >> 29);
        let txqen = self.con & CON_TXQEN != 0;
        place(&mut self.fifo[0], txqen, txq_size, &mut addr);
        for i in 1..32 {
            let f = &self.fifo[i];
            let ts = !f.is_tx() && f.con & FIFO_RXTSEN != 0;
            let size = 8 + if ts { 4 } else { 0 } + payload_size(f.con >> 29);
            place(&mut self.fifo[i], true, size, &mut addr);
        }
    }

    fn set_mode(&mut self, new: u8, now: Time) {
        if new == self.opmod {
            return;
        }
        let leaving_config = self.opmod == mode::CONFIG;
        if new == mode::CONFIG {
            for f in self.fifo.iter_mut() {
                f.reset_contents();
            }
            self.tef.reset_contents();
            self.reset_protocol_engine();
        }
        if leaving_config {
            self.allocate_ram();
        }
        log::debug!("[{now}] MCP251xFD mode {} -> {}", self.opmod, new);
        self.opmod = new;
        self.int_sticky |= INT_MODIF;
    }

    // ------------------------------------------------------------------ registers

    fn fifo_index(addr: u16) -> Option<(usize, u16)> {
        match addr {
            0x050..=0x058 => Some((0, addr - 0x050)),
            0x05C..=0x1CB => Some((((addr - 0x05C) / 12 + 1) as usize, (addr - 0x05C) % 12)),
            _ => None,
        }
    }

    fn read_word(&mut self, addr: u16, now: Time) -> u32 {
        let config = self.opmod == mode::CONFIG;
        match addr {
            0x000 => (self.con & !(7 << 21) & !(1 << 11)) | (self.opmod as u32) << 21,
            0x004 => self.nbtcfg,
            0x008 => self.dbtcfg,
            0x00C => self.tdc,
            0x010 => self.tbc(now),
            0x014 => self.tscon,
            0x018 => self.vec(),
            0x01C => (self.int_enable as u32) << 16 | self.int_flags() as u32,
            0x020 => self.c1_rxif(),
            0x024 => self.c1_txif(),
            0x028 => self.c1_rxovif(),
            0x02C => self.c1_txatif(),
            0x030 => self.txreq_bits(),
            0x034 => {
                let c = self.error_counters();
                let mut v = c.rec as u32 | (c.tec as u32) << 8 | self.trec_state_bits();
                if config {
                    v |= 1 << 21; // TXBO: not on the bus
                }
                v
            }
            0x038 => self.bdiag[0],
            0x03C => self.bdiag[1],
            0x040 => self.tef.con & !FIFO_FRESET,
            0x044 => self.tef_status(),
            0x048 => {
                if config {
                    0
                } else {
                    self.tef.obj_addr(self.tef.head).unwrap_or(0) as u32
                }
            }
            0x1D0..=0x1EF => {
                let i = (addr - 0x1D0) as usize;
                u32::from_le_bytes([self.fltcon[i], self.fltcon[i + 1], self.fltcon[i + 2], self.fltcon[i + 3]])
            }
            0x1F0..=0x2EF => {
                let i = ((addr - 0x1F0) / 8) as usize;
                if (addr - 0x1F0).is_multiple_of(8) { self.fltobj[i] } else { self.mask[i] }
            }
            0x400..=0xBFF => {
                let o = (addr - RAM_START) as usize;
                u32::from_le_bytes(self.ram[o..o + 4].try_into().unwrap())
            }
            0xE00 => {
                let mut v = self.osc & 0x7F;
                if self.sysclk > 0 {
                    v |= 1 << 10; // OSCRDY
                    if self.osc & 1 != 0 {
                        v |= 1 << 8; // PLLRDY
                    }
                }
                // SCLKRDY is the synchronized SCLKDIV value, not a ready flag
                v |= ((self.osc >> 4) & 1) << 12;
                v
            }
            0xE04 => {
                let mut v = self.iocon & !(3 << 16);
                if self.config.gpio0 {
                    v |= 1 << 16;
                }
                if self.config.gpio1 {
                    v |= 1 << 17;
                }
                v
            }
            0xE08 => self.crc_reg,
            0xE0C => self.ecccon,
            0xE10 => self.eccstat,
            0xE14 => {
                if self.config.mcp2518 {
                    0x0000_0014
                } else {
                    0
                }
            }
            _ => match Self::fifo_index(addr) {
                Some((i, 0)) => {
                    let f = &self.fifo[i];
                    let mut v = f.con & !(FIFO_UINC | FIFO_FRESET | FIFO_TXREQ);
                    if f.tx_requested.is_some() {
                        v |= FIFO_TXREQ;
                    }
                    if i == 0 {
                        v |= FIFO_TXEN;
                    }
                    v
                }
                Some((i, 4)) => {
                    let tx = i == 0 || self.fifo[i].is_tx();
                    self.fifo[i].status(tx)
                }
                Some((i, 8)) => {
                    let f = &self.fifo[i];
                    if config {
                        0
                    } else {
                        let tx = i == 0 || f.is_tx();
                        f.obj_addr(if tx { f.tail } else { f.head }).unwrap_or(0) as u32
                    }
                }
                _ => 0,
            },
        }
    }

    /// Writes the bytes of the word register at `addr` selected by `mask`.
    fn write_word(&mut self, addr: u16, value: u32, mask: u32, now: Time) {
        let config = self.opmod == mode::CONFIG;
        let merge = |old: u32| (old & !mask) | (value & mask);
        match addr {
            0x000 => {
                let new = merge(self.con);
                // REQOP is acted on at once; the other fields only change in configuration mode
                let cfg_bits = 0x00FF_FFFF & !(7 << 21) & !(1 << 11);
                if config {
                    self.con = (self.con & !cfg_bits) | (new & cfg_bits);
                }
                self.con = (self.con & !(0xFF << 24)) | (new & (0xFF << 24));
                if mask & 0x0700_0000 != 0 {
                    let reqop = ((self.con >> 24) & 7) as u8;
                    self.set_mode(reqop, now);
                }
                if self.con & (1 << 27) != 0 {
                    // ABAT: abort all pending transmissions
                    for f in self.fifo.iter_mut() {
                        f.tx_requested = None;
                    }
                    self.con &= !(1 << 27);
                }
            }
            0x004 if config => self.nbtcfg = merge(self.nbtcfg),
            0x008 if config => self.dbtcfg = merge(self.dbtcfg),
            0x00C if config => self.tdc = merge(self.tdc),
            0x010 => {
                self.tbc_base = merge(self.tbc(now));
                self.tbc_time = now;
            }
            0x014 => {
                self.tbc_base = self.tbc(now);
                self.tbc_time = now;
                self.tscon = merge(self.tscon) & 0x0007_03FF;
            }
            0x01C => {
                let new = merge((self.int_enable as u32) << 16 | 0xFFFF);
                self.int_enable = (new >> 16) as u16;
                if mask & 0xFFFF != 0 {
                    // Writing 0 clears the latched flags
                    self.int_sticky &= (new as u16) | !INT_STICKY;
                }
            }
            0x038 => self.bdiag[0] = merge(self.bdiag[0]),
            0x03C => self.bdiag[1] = merge(self.bdiag[1]),
            0x040 => {
                let new = merge(self.tef.con);
                let writable = if config { 0x1F00_002F } else { 0x0000_000F };
                self.tef.con = (self.tef.con & !writable) | (new & writable);
                if !config && mask & 0xFF00 != 0 {
                    if new & FIFO_UINC != 0 && self.tef.count > 0 {
                        self.tef.head = (self.tef.head + 1) % self.tef.depth.max(1);
                        self.tef.count -= 1;
                    }
                    if new & FIFO_FRESET != 0 {
                        self.tef.reset_contents();
                    }
                }
            }
            0x044 => {
                // TEFOVIF clears by writing 0
                if mask & 0xFF != 0 && value & (1 << 3) == 0 {
                    self.tef.sticky &= !(1 << 3);
                }
            }
            0x1D0..=0x1EF => {
                for lane in 0..4 {
                    if mask & (0xFF << (8 * lane)) != 0 {
                        self.fltcon[(addr - 0x1D0) as usize + lane] = (value >> (8 * lane)) as u8 & 0x9F;
                    }
                }
            }
            0x1F0..=0x2EF => {
                let i = ((addr - 0x1F0) / 8) as usize;
                // Filter objects can only change while the filter is disabled
                if self.fltcon[i] & 0x80 == 0 {
                    if (addr - 0x1F0).is_multiple_of(8) {
                        self.fltobj[i] = merge(self.fltobj[i]) & 0x7FFF_FFFF;
                    } else {
                        self.mask[i] = merge(self.mask[i]) & 0x7FFF_FFFF;
                    }
                }
            }
            0x400..=0xBFF => {
                let o = (addr - RAM_START) as usize;
                let new = merge(u32::from_le_bytes(self.ram[o..o + 4].try_into().unwrap()));
                self.ram[o..o + 4].copy_from_slice(&new.to_le_bytes());
            }
            // LPMEN (bit 3) only exists on the MCP2518FD; drivers tell the parts apart by whether it sticks
            0xE00 => self.osc = merge(self.osc) & if self.config.mcp2518 { 0x7D } else { 0x75 },
            0xE04 => self.iocon = merge(self.iocon) & 0x7303_0343,
            0xE08 => {
                let new = merge(self.crc_reg);
                // Flags clear by writing 0, enables are read/write
                let flags = self.crc_reg & new & 0x0003_0000;
                self.crc_reg = (new & 0x0300_0000) | flags | (self.crc_reg & 0xFFFF);
            }
            0xE0C => self.ecccon = merge(self.ecccon) & 0x7F07,
            0xE10 => self.eccstat = merge(self.eccstat) & 0x0FFF_0006,
            _ => {
                if let Some((i, off)) = Self::fifo_index(addr) {
                    self.write_fifo(i, off, value, mask, now);
                }
            }
        }
    }

    fn write_fifo(&mut self, i: usize, off: u16, value: u32, mask: u32, now: Time) {
        let config = self.opmod == mode::CONFIG;
        match off {
            0 => {
                let f = &mut self.fifo[i];
                let new = (f.con & !mask) | (value & mask);
                // Interrupt enables, TXPRI and TXAT any time; layout (TXEN, RXTSEN, RTREN, FSIZE, PLSIZE) in config
                let any_time = 0x007F_001F;
                let config_only = 0xFF00_00E0 & if i == 0 { !FIFO_TXEN } else { u32::MAX };
                let writable = any_time | if config { config_only } else { 0 };
                f.con = (f.con & !writable) | (new & writable);
                if config || f.base.is_none() || mask & 0xFF00 == 0 {
                    return;
                }
                let tx = i == 0 || f.is_tx();
                if new & FIFO_FRESET != 0 {
                    f.reset_contents();
                    return;
                }
                if new & FIFO_UINC != 0 {
                    if tx {
                        if f.count < f.depth {
                            f.tail = (f.tail + 1) % f.depth;
                            f.count += 1;
                        }
                    } else if f.count > 0 {
                        f.head = (f.head + 1) % f.depth;
                        f.count -= 1;
                    }
                }
                if tx && new & FIFO_TXREQ != 0 && f.count > 0 && f.tx_requested.is_none() {
                    f.tx_requested = Some(now);
                    f.attempts = 0;
                }
            }
            // RXOVIF and TXATIF (and the TX error flags) clear by writing 0
            4 if mask & 0xFF != 0 => self.fifo[i].sticky &= value | !0xF8,
            _ => {}
        }
    }

    fn read_byte(&mut self, addr: u16, now: Time) -> u8 {
        let word = self.read_word(addr & !3, now);
        (word >> (8 * (addr & 3))) as u8
    }

    fn write_byte(&mut self, addr: u16, b: u8, now: Time) {
        let lane = (addr & 3) as u32;
        self.write_word(addr & !3, (b as u32) << (8 * lane), 0xFF << (8 * lane), now);
    }

    fn write_bytes(&mut self, addr: u16, data: &[u8], now: Time) {
        // Group bytes by word so multi-byte register writes see the whole value at once (UINC with TXREQ etc.)
        let mut a = addr;
        let mut i = 0;
        while i < data.len() {
            let word = a & !3 & 0xFFF;
            let mut value = 0u32;
            let mut mask = 0u32;
            while i < data.len() && (a & !3 & 0xFFF) == word {
                let lane = (a & 3) as u32;
                value |= (data[i] as u32) << (8 * lane);
                mask |= 0xFF << (8 * lane);
                a = (a + 1) & 0xFFF;
                i += 1;
            }
            self.write_word(word, value, mask, now);
        }
    }

    fn in_ram(addr: u16) -> bool {
        (RAM_START..RAM_START + RAM_SIZE as u16).contains(&addr)
    }

    // ------------------------------------------------------------------ CAN side helpers

    fn tx_allowed(&self) -> bool {
        matches!(self.opmod, mode::NORMAL_FD | mode::NORMAL_CAN20 | mode::EXT_LOOPBACK) && !self.bus_off
    }

    fn rx_allowed(&self) -> bool {
        !self.bus_off
            && matches!(
                self.opmod,
                mode::NORMAL_FD | mode::NORMAL_CAN20 | mode::LISTEN_ONLY | mode::EXT_LOOPBACK | mode::RESTRICTED
            )
    }

    fn next_tx(&self) -> Option<usize> {
        if !self.tx_allowed() {
            return None;
        }
        let mut best: Option<(usize, u32)> = None;
        for (i, f) in self.fifo.iter().enumerate() {
            let usable = if i == 0 { self.con & CON_TXQEN != 0 } else { f.is_tx() };
            if usable && f.tx_requested.is_some() && f.count > 0 && f.base.is_some() {
                let pri = (f.con >> 16) & 0x1F;
                if best.is_none_or(|(_, p)| pri > p) {
                    best = Some((i, pri));
                }
            }
        }
        best.map(|(i, _)| i)
    }

    fn read_tx_object(&self, i: usize) -> (CanFrame, u32) {
        let f = &self.fifo[i];
        let a = f.obj_addr(f.head).unwrap() as usize;
        let w = |o: usize| u32::from_le_bytes(self.ram[a + o..a + o + 4].try_into().unwrap());
        let id = w(0);
        let flags = w(4);
        let extended = flags & (1 << 4) != 0;
        let fd = flags & (1 << 7) != 0 && self.opmod != mode::NORMAL_CAN20;
        let dlc = (flags & 0xF) as u8;
        let len = if fd { fd_dlc_to_len(dlc) } else { (dlc as usize).min(8) };
        let len = len.min(f.obj_size as usize - 8);
        let can_id = if extended { (id & 0x7FF) << 18 | (id >> 11) & 0x3FFFF } else { id & 0x7FF };
        let frame = CanFrame {
            id: can_id,
            extended,
            fd,
            brs: fd && flags & (1 << 6) != 0,
            rtr: !fd && flags & (1 << 5) != 0,
            esi: flags & (1 << 8) != 0,
            data: self.ram[a + 8..a + 8 + len].to_vec(),
        };
        (frame, flags)
    }

    fn retransmit_limit(&self, i: usize) -> Option<u8> {
        if self.con & CON_RTXAT == 0 {
            return None;
        }
        match (self.fifo[i].con >> 21) & 3 {
            0 => Some(1),
            1 => Some(3),
            _ => None,
        }
    }

    fn push_tef(&mut self, id_word: u32, flags: u32, at: Time) {
        let t = &mut self.tef;
        let Some(a) = t.obj_addr(t.tail) else { return };
        if t.count >= t.depth {
            t.sticky |= 1 << 3; // TEFOVIF
            return;
        }
        let a = a as usize;
        let ts = self.tef.con & TEF_TEFTSEN != 0;
        let stamp = self.tbc(at);
        self.ram[a..a + 4].copy_from_slice(&id_word.to_le_bytes());
        self.ram[a + 4..a + 8].copy_from_slice(&flags.to_le_bytes());
        if ts {
            self.ram[a + 8..a + 12].copy_from_slice(&stamp.to_le_bytes());
        }
        let t = &mut self.tef;
        t.tail = (t.tail + 1) % t.depth;
        t.count += 1;
    }

    fn filter_match(&self, n: usize, frame: &CanFrame) -> bool {
        let obj = self.fltobj[n];
        let mask = self.mask[n];
        let (sid, eid) = if frame.extended { (frame.id >> 18, frame.id & 0x3FFFF) } else { (frame.id & 0x7FF, 0) };
        let mide = mask & (1 << 30) != 0;
        let exide = obj & (1 << 30) != 0;
        if mide && exide != frame.extended {
            return false;
        }
        if (sid ^ obj) & mask & 0x7FF != 0 {
            return false;
        }
        if frame.extended && (eid ^ (obj >> 11)) & (mask >> 11) & 0x3FFFF != 0 {
            return false;
        }
        true
    }

    fn store_rx(&mut self, fifo: usize, frame: &CanFrame, filter: usize, at: Time) {
        let f = &self.fifo[fifo];
        let Some(a) = f.obj_addr(f.tail) else { return };
        if f.count >= f.depth {
            self.fifo[fifo].sticky |= STA_RXOVIF;
            return;
        }
        let ts = f.con & FIFO_RXTSEN != 0;
        let payload = f.obj_size as usize - 8 - if ts { 4 } else { 0 };
        let a = a as usize;
        let id_word =
            if frame.extended { (frame.id >> 18) & 0x7FF | (frame.id & 0x3FFFF) << 11 } else { frame.id & 0x7FF };
        let dlc = if frame.fd { fd_len_to_dlc(frame.data.len()) } else { frame.data.len().min(8) as u8 };
        let flags = dlc as u32
            | (frame.extended as u32) << 4
            | (frame.rtr as u32) << 5
            | (frame.brs as u32) << 6
            | (frame.fd as u32) << 7
            | (frame.esi as u32) << 8
            | (filter as u32) << 11;
        self.ram[a..a + 4].copy_from_slice(&id_word.to_le_bytes());
        self.ram[a + 4..a + 8].copy_from_slice(&flags.to_le_bytes());
        let mut d = a + 8;
        if ts {
            let stamp = self.tbc(at);
            self.ram[d..d + 4].copy_from_slice(&stamp.to_le_bytes());
            d += 4;
        }
        let len = frame.data.len().min(payload);
        self.ram[d..d + len].copy_from_slice(&frame.data[..len]);
        for b in &mut self.ram[d + len..d + payload] {
            *b = 0;
        }
        let f = &mut self.fifo[fifo];
        f.tail = (f.tail + 1) % f.depth;
        f.count += 1;
        self.filhit = filter as u8;
    }

    // ------------------------------------------------------------------ SPI

    /// Data and CRC a READ_CRC of `len` (bytes for SFRs, words for RAM) at `addr` shifts out.
    fn read_crc_data(&mut self, addr: u16, len: u8, now: Time) -> Vec<u8> {
        let n = len as usize * if Self::in_ram(addr) { 4 } else { 1 };
        let mut data: Vec<u8> = (0..n).map(|i| self.read_byte((addr + i as u16) & 0xFFF, now)).collect();
        let mut all = vec![(SPI_READ_CRC << 4) | (addr >> 8) as u8, addr as u8, len];
        all.extend_from_slice(&data);
        data.extend_from_slice(&crc16_cms(&all).to_be_bytes());
        data
    }

    fn finish_crc_write(&mut self, cmd: u8, addr: u16, bytes: &[u8], now: Time) {
        // bytes: [len?] data... crc_hi crc_lo (len only for WRITE_CRC)
        let header = [(cmd << 4) | (addr >> 8) as u8, addr as u8];
        let (data, crc) = bytes.split_at(bytes.len() - 2);
        let mut all = header.to_vec();
        all.extend_from_slice(data);
        let ok = crc16_cms(&all) == u16::from_be_bytes([crc[0], crc[1]]);
        if !ok {
            self.crc_reg |= 1 << 16; // CRCERRIF
            log::debug!("MCP251xFD SPI CRC mismatch on {:#x}", addr);
        }
        let payload = if cmd == SPI_WRITE_CRC { &data[1..] } else { data };
        if ok || cmd == SPI_WRITE_CRC {
            self.write_bytes(addr, payload, now);
        }
    }
}

impl SpiDevice for Mcp251xfd {
    fn name(&self) -> String {
        if self.config.mcp2518 { "MCP2518FD".into() } else { "MCP2517FD".into() }
    }

    fn select(&mut self, ctx: &mut dyn DeviceCtx, selected: bool) {
        self.refresh_clock(ctx);
        let now = ctx.now();
        if selected && !self.selected {
            self.spi = Spi::Header(Vec::new());
            self.xfer_log.clear();
        } else if !selected && self.selected {
            if log::log_enabled!(log::Level::Trace) {
                let mosi: Vec<String> = self.xfer_log.iter().map(|(o, _)| format!("{o:02x}")).collect();
                let miso: Vec<String> = self.xfer_log.iter().map(|(_, i)| format!("{i:02x}")).collect();
                log::trace!("[{now}] MCP SPI mosi {} miso {}", mosi.join(" "), miso.join(" "));
            }
            match std::mem::take(&mut self.spi) {
                Spi::Reset => {
                    log::debug!("[{now}] MCP251xFD reset");
                    self.reset();
                }
                Spi::Crc { cmd, addr, bytes, .. } if cmd == SPI_WRITE_CRC || cmd == SPI_WRITE_SAFE => {
                    let need = if cmd == SPI_WRITE_SAFE {
                        (if Self::in_ram(addr) { 4 } else { 1 }) + 2
                    } else {
                        let len = *bytes.first().unwrap_or(&0) as usize;
                        1 + len * if Self::in_ram(addr) { 4 } else { 1 } + 2
                    };
                    if bytes.len() == need {
                        self.finish_crc_write(cmd, addr, &bytes, now);
                    } else {
                        self.crc_reg |= 1 << 17; // FERRIF: wrong number of bytes
                    }
                }
                _ => {}
            }
            self.spi = Spi::Idle;
        }
        self.selected = selected;
        self.drive_int(ctx);
    }

    fn transfer(&mut self, ctx: &mut dyn DeviceCtx, mosi: u16, bits: u8) -> u16 {
        let b = (mosi & 0xFF) as u8;
        if bits != 8 {
            log::warn!("MCP251xFD: {bits}-bit SPI frame");
        }
        let now = ctx.now();
        if !self.selected {
            return 0xFF;
        }
        let (next, miso) = match std::mem::take(&mut self.spi) {
            s @ (Spi::Idle | Spi::Ignore | Spi::Reset) => (s, 0),
            Spi::Header(mut h) => {
                h.push(b);
                if h.len() < 2 {
                    (Spi::Header(h), 0)
                } else {
                    let cmd = h[0] >> 4;
                    let addr = ((h[0] as u16 & 0xF) << 8) | h[1] as u16;
                    let next = match cmd {
                        SPI_RESET => Spi::Reset,
                        SPI_READ => Spi::Read(addr),
                        SPI_WRITE => Spi::Write(addr),
                        SPI_READ_CRC | SPI_WRITE_CRC | SPI_WRITE_SAFE => {
                            Spi::Crc { cmd, addr, bytes: Vec::new(), out: Vec::new(), pos: 0 }
                        }
                        _ => {
                            log::warn!("MCP251xFD: unknown SPI instruction {cmd:#x}");
                            Spi::Ignore
                        }
                    };
                    (next, 0)
                }
            }
            Spi::Read(a) => {
                let v = self.read_byte(a, now);
                (Spi::Read((a + 1) & 0xFFF), v)
            }
            Spi::Write(a) => {
                self.write_byte(a, b, now);
                self.refresh_clock(ctx);
                (Spi::Write((a + 1) & 0xFFF), 0)
            }
            Spi::Crc { cmd: SPI_READ_CRC, addr, mut bytes, out, pos } => {
                if bytes.is_empty() {
                    // The length byte: read the data now and append its CRC
                    bytes.push(b);
                    let out = self.read_crc_data(addr, b, now);
                    (Spi::Crc { cmd: SPI_READ_CRC, addr, bytes, out, pos: 0 }, 0)
                } else {
                    let v = out.get(pos).copied().unwrap_or(0);
                    (Spi::Crc { cmd: SPI_READ_CRC, addr, bytes, out, pos: pos + 1 }, v)
                }
            }
            Spi::Crc { cmd, addr, mut bytes, out, pos } => {
                bytes.push(b);
                (Spi::Crc { cmd, addr, bytes, out, pos }, 0)
            }
        };
        self.spi = next;
        self.drive_int(ctx);
        if self.xfer_log.len() < 128 {
            self.xfer_log.push((b, miso));
        }
        miso as u16
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        self.refresh_clock(ctx);
        self.drive_int(ctx);
    }

    fn can_node(&mut self) -> Option<&mut dyn CanNode> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn clone_box(&self) -> Box<dyn SpiDevice> {
        Box::new(self.clone())
    }
}

impl CanNode for Mcp251xfd {
    fn bit_rates(&self) -> Option<BitRates> {
        // A bus-off node is still attached to the wire, counting recessive bits to recover
        if !(self.rx_allowed() || self.bus_off && self.on_bus()) || self.sysclk == 0 {
            return None;
        }
        let (nominal, data) = self.rates();
        Some(BitRates { nominal, data: (self.opmod != mode::NORMAL_CAN20).then_some(data) })
    }

    fn acknowledges(&self) -> bool {
        matches!(self.opmod, mode::NORMAL_FD | mode::NORMAL_CAN20 | mode::RESTRICTED)
            && self.sysclk > 0
            && !self.bus_off
    }

    fn tx_request(&mut self) -> Option<TxRequest> {
        let i = self.next_tx()?;
        let (frame, _) = self.read_tx_object(i);
        Some(TxRequest { token: i as u64, frame, ready_at: self.fifo[i].tx_requested.unwrap() })
    }

    fn tx_done(&mut self, token: u64, at: Time, acked: bool) {
        let i = token as usize;
        let f = &self.fifo[i];
        if f.count == 0 {
            return;
        }
        let a = f.obj_addr(f.head).unwrap() as usize;
        let id_word = u32::from_le_bytes(self.ram[a..a + 4].try_into().unwrap());
        let flags = u32::from_le_bytes(self.ram[a + 4..a + 8].try_into().unwrap());
        let self_ack = self.opmod == mode::EXT_LOOPBACK;
        if acked || self_ack {
            self.frames_sent += 1;
            self.confine(|m| m.tec = m.tec.saturating_sub(1));
            self.error_free_message();
            if self_ack {
                let (frame, _) = self.read_tx_object(i);
                self.receive(&frame, at);
            }
            let f = &mut self.fifo[i];
            f.head = (f.head + 1) % f.depth;
            f.count -= 1;
            f.attempts = 0;
            if f.count == 0 {
                f.tx_requested = None;
            }
            if self.con & CON_STEF != 0 {
                // TEF object: the transmitted ID and flags (including the sequence number)
                self.push_tef(id_word, flags, at);
            }
        } else {
            // No acknowledgement: TEC + 8, except for an error passive transmitter whose passive error flag is not
            // overwritten (ISO 11898-1 rule 3, exception 1), so an ACK error alone never leads to bus-off
            self.tx_errors += 1;
            if !self.error_passive() {
                self.confine(|m| m.tec += 8);
            }
            self.diag(8, 1 << 18); // NTERRCNT, NACKERR
            self.tx_failed(i);
        }
    }

    fn tx_error(&mut self, token: u64, _at: Time, error: BusError) {
        if !self.on_bus() || self.opmod == mode::EXT_LOOPBACK {
            return;
        }
        self.tx_errors += 1;
        // The transmitter monitors every bit it sends: wanting recessive and seeing dominant is a bit-1 error
        match error {
            BusError::Bit { data_phase: true } => self.diag(24, 1 << 25), // DTERRCNT, DBIT1ERR
            _ => self.diag(8, 1 << 17),                                   // NTERRCNT, NBIT1ERR
        }
        if error == BusError::StuckDominant {
            self.stuck_transmitter = true;
        }
        self.confine(|m| m.tec += 8);
        self.tx_failed(token as usize);
    }

    fn rx_error(&mut self, _at: Time, error: BusError) {
        if !self.rx_allowed() || self.opmod == mode::LISTEN_ONLY {
            return;
        }
        self.rx_errors += 1;
        // Receivers see the transmitter's error flag as six equal bits: a stuff error
        match error {
            BusError::Bit { data_phase: true } => self.diag(16, 1 << 28), // DRERRCNT, DSTUFERR
            _ => self.diag(0, 1 << 20),                                   // NRERRCNT, NSTUFERR
        }
        self.confine(|m| m.rec += 1);
    }

    fn dominant_bits(&mut self, _at: Time, bits: u64) {
        if self.bus_off || !self.on_bus() || self.opmod == mode::LISTEN_ONLY {
            return;
        }
        let before = self.dominant_run;
        self.dominant_run += bits;
        if before == 0 {
            // The first six dominant bits are a stuff error (a frame cut off already counted its own)
            if !self.stuck_transmitter {
                self.rx_errors += 1;
                self.diag(0, 1 << 20);
                self.confine(|m| m.rec += 1);
            }
        }
        // After the error flag, every sequence of 8 more dominant bits adds 8 (rule 6), to the TEC of a node that
        // was transmitting and the REC of a receiver
        let steps = (self.dominant_run.saturating_sub(12) / 8).saturating_sub(before.saturating_sub(12) / 8);
        if steps > 0 {
            let add = (8 * steps).min(1024) as u16;
            if self.stuck_transmitter {
                self.confine(|m| m.tec = m.tec.saturating_add(add));
            } else {
                self.confine(|m| m.rec = m.rec.saturating_add(add));
            }
        }
    }

    fn error_passive(&self) -> bool {
        Mcp251xfd::error_passive(self) && !self.bus_off
    }

    fn recessive_sequences(&mut self, _at: Time, n: u64) {
        self.dominant_run = 0;
        self.stuck_transmitter = false;
        if self.bus_off && self.on_bus() {
            self.recovery += n;
            if self.recovery >= 128 {
                // Recovered: error active again with both counters cleared
                self.confine(|m| {
                    m.bus_off = false;
                    m.tec = 0;
                    m.rec = 0;
                    m.recovery = 0;
                });
                log::debug!("MCP251xFD recovered from bus-off");
            }
        }
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        if !self.rx_allowed() || (frame.fd && self.opmod == mode::NORMAL_CAN20) {
            return;
        }
        // A frame received without error: REC - 1, or back into error active from error passive
        self.confine(|m| m.rec = if m.rec > 127 { 120 } else { m.rec.saturating_sub(1) });
        self.error_free_message();
        for n in 0..32 {
            let fc = self.fltcon[n];
            if fc & 0x80 != 0 && self.filter_match(n, frame) {
                let fifo = (fc & 0x1F) as usize;
                if fifo >= 1 && !self.fifo[fifo].is_tx() {
                    self.frames_received += 1;
                    self.store_rx(fifo, frame, n, at);
                }
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    fn ctx() -> TestCtx {
        let mut c = TestCtx::default();
        c.clocks[21] = 4_000_000;
        c
    }

    /// One SPI transaction: select, exchange `bytes`, deselect.
    fn xfer(dev: &mut Mcp251xfd, c: &mut TestCtx, bytes: &[u8]) -> Vec<u8> {
        dev.select(c, true);
        let out = bytes.iter().map(|&b| dev.transfer(c, b as u16, 8) as u8).collect();
        dev.select(c, false);
        out
    }

    fn write(dev: &mut Mcp251xfd, c: &mut TestCtx, addr: u16, data: &[u8]) {
        let mut b = vec![(SPI_WRITE << 4) | (addr >> 8) as u8, addr as u8];
        b.extend_from_slice(data);
        xfer(dev, c, &b);
    }

    fn write32(dev: &mut Mcp251xfd, c: &mut TestCtx, addr: u16, v: u32) {
        write(dev, c, addr, &v.to_le_bytes());
    }

    fn read(dev: &mut Mcp251xfd, c: &mut TestCtx, addr: u16, n: usize) -> Vec<u8> {
        let mut b = vec![(SPI_READ << 4) | (addr >> 8) as u8, addr as u8];
        b.extend(std::iter::repeat_n(0, n));
        xfer(dev, c, &b)[2..].to_vec()
    }

    fn read32(dev: &mut Mcp251xfd, c: &mut TestCtx, addr: u16) -> u32 {
        u32::from_le_bytes(read(dev, c, addr, 4).try_into().unwrap())
    }

    /// The titan_rp2040 driver's configuration: 1 Mbit/s / 5 Mbit/s, TEF 8, TXQ 8x64 B, FIFO1 RX 16x64 B, one filter.
    fn configured() -> (Mcp251xfd, TestCtx) {
        let mut c = ctx();
        let mut d = Mcp251xfd::new(Mcp251xfdConfig::default());
        xfer(&mut d, &mut c, &[0, 0]); // RESET
        write32(&mut d, &mut c, 0xE00, 1); // PLLEN
        assert_eq!(read32(&mut d, &mut c, 0xE00) & 0x1500, 0x500, "PLLRDY | OSCRDY, SCLKRDY follows SCLKDIV = 0");
        write32(&mut d, &mut c, 0x000, 0x0419_0760); // config mode, STEF, TXQEN, RTXAT
        write32(&mut d, &mut c, 0x004, 30 << 16 | 7 << 8 | 7);
        write32(&mut d, &mut c, 0x008, 4 << 16 | 1 << 8 | 1);
        write32(&mut d, &mut c, 0x040, 7 << 24 | 1);
        write32(&mut d, &mut c, 0x050, 7 << 29 | 7 << 24 | 1 << 21);
        write32(&mut d, &mut c, 0x05C, 7 << 29 | 15 << 24 | FIFO_RXOVIE | FIFO_TFNRFNIE);
        write32(&mut d, &mut c, 0x1F0, 0x123);
        write32(&mut d, &mut c, 0x1F4, 0x7FF | 1 << 30);
        write(&mut d, &mut c, 0x1D0, &[0x81]);
        write32(&mut d, &mut c, 0x01C, ((INT_RXIF | INT_TEFIF | INT_TXATIF | INT_RXOVIF) as u32) << 16);
        write(&mut d, &mut c, 0x003, &[mode::NORMAL_FD]);
        (d, c)
    }

    #[test]
    fn crc16_check_value() {
        assert_eq!(crc16_cms(b"123456789"), 0xAEE7);
    }

    #[test]
    fn lpmen_only_on_the_mcp2518() {
        let mut c = ctx();
        let mut d = Mcp251xfd::new(Mcp251xfdConfig::default());
        write(&mut d, &mut c, 0xE00, &[0x09]);
        assert_eq!(read(&mut d, &mut c, 0xE00, 1)[0], 0x01);
        let mut d = Mcp251xfd::new(Mcp251xfdConfig { mcp2518: true, ..Default::default() });
        write(&mut d, &mut c, 0xE00, &[0x19]);
        assert_eq!(read(&mut d, &mut c, 0xE00, 1)[0], 0x19);
        assert_eq!(read(&mut d, &mut c, 0xE01, 1)[0], 0x15, "PLLRDY, OSCRDY, SCLKRDY = SCLKDIV");
    }

    #[test]
    fn oscillator_needs_clkin() {
        let mut c = TestCtx::default();
        let mut d = Mcp251xfd::new(Mcp251xfdConfig::default());
        assert_eq!(read32(&mut d, &mut c, 0xE00) & (1 << 10), 0, "no CLKIN: OSCRDY clear");
        c.clocks[21] = 4_000_000;
        assert_ne!(read32(&mut d, &mut c, 0xE00) & (1 << 10), 0);
    }

    #[test]
    fn mode_change_bit_timing_and_ram_layout() {
        let (mut d, mut c) = configured();
        assert_eq!((read32(&mut d, &mut c, 0x000) >> 21) & 7, mode::NORMAL_FD as u32);
        assert_eq!(d.bit_rates(), Some(BitRates { nominal: 1_000_000, data: Some(5_000_000) }));
        // TEF 8 x 8 B at 0, TXQ 8 x 72 B at 64, FIFO1 16 x 72 B at 640
        assert_eq!(read32(&mut d, &mut c, 0x048), 0);
        assert_eq!(read32(&mut d, &mut c, 0x058), 64);
        assert_eq!(read32(&mut d, &mut c, 0x064), 640);
        assert!(d.acknowledges());
    }

    fn queue_frame(d: &mut Mcp251xfd, c: &mut TestCtx, id: u32, data: &[u8], seq: u32) {
        let ua = read32(d, c, 0x058) as u16;
        let mut obj = id.to_le_bytes().to_vec();
        let flags = fd_len_to_dlc(data.len()) as u32 | 1 << 7 | 1 << 6 | seq << 9;
        obj.extend_from_slice(&flags.to_le_bytes());
        obj.extend_from_slice(data);
        while !obj.len().is_multiple_of(4) {
            obj.push(0);
        }
        write(d, c, 0x400 + ua, &obj);
        write(d, c, 0x051, &[0x03]); // UINC | TXREQ
    }

    #[test]
    fn transmit_through_txq_and_tef() {
        let (mut d, mut c) = configured();
        let ua = read32(&mut d, &mut c, 0x058) as u16;
        let mut obj = 0x321u32.to_le_bytes().to_vec();
        obj.extend_from_slice(&(3u32 | 1 << 7 | 1 << 6 | 2 << 9).to_le_bytes());
        obj.extend_from_slice(&[1, 2, 3, 0]);
        write(&mut d, &mut c, 0x400 + ua, &obj);
        assert_eq!(read32(&mut d, &mut c, 0x054) & 4, 4, "TXQ still empty before UINC");
        write(&mut d, &mut c, 0x051, &[0x03]); // UINC | TXREQ
        assert_eq!(read32(&mut d, &mut c, 0x030), 1, "TXREQ for the TXQ");
        let req = d.tx_request().unwrap();
        assert_eq!(req.frame, CanFrame::fd(0x321, false, &[1, 2, 3]));
        d.tx_done(req.token, Time::from_us(50), true);
        assert!(d.tx_request().is_none());
        assert_eq!(read32(&mut d, &mut c, 0x030), 0);
        // TEF holds the sequence number; TEFIF raises INT
        assert_eq!(read32(&mut d, &mut c, 0x044) & 1, 1);
        assert!(d.int_asserted());
        d.update(&mut c);
        assert_eq!(c.pins[22], External::Driven(Level::Low));
        let tefua = read32(&mut d, &mut c, 0x048) as u16;
        let te1 = u32::from_le_bytes(read(&mut d, &mut c, 0x404 + tefua, 4).try_into().unwrap());
        assert_eq!((te1 >> 9) & 0x7F, 2);
        write(&mut d, &mut c, 0x041, &[0x01]); // TEF UINC
        assert!(!d.int_asserted());
        d.update(&mut c);
        assert_eq!(c.pins[22], External::Driven(Level::High));
    }

    #[test]
    fn retransmission_attempts_exhausted() {
        let (mut d, mut c) = configured();
        queue_frame(&mut d, &mut c, 0x10, &[0], 1);
        for _ in 0..3 {
            let r = d.tx_request().expect("still requested");
            d.tx_done(r.token, Time::from_us(10), false);
        }
        assert!(d.tx_request().is_none(), "TXAT = 3 attempts with RTXAT");
        assert_ne!(read32(&mut d, &mut c, 0x054) & STA_TXATIF, 0);
        // C1TXATIF only reports FIFOs with TXATIE set (titan's driver leaves it clear on the TXQ)
        assert_eq!(read32(&mut d, &mut c, 0x02C), 0);
        write(&mut d, &mut c, 0x050, &[FIFO_TXATIE as u8]);
        assert_eq!(read32(&mut d, &mut c, 0x02C), 1);
        assert!(d.int_asserted());
        assert_eq!((read32(&mut d, &mut c, 0x034) >> 8) & 0xFF, 24, "TEC");
        write(&mut d, &mut c, 0x054, &[0]);
        assert_eq!(read32(&mut d, &mut c, 0x054) & STA_TXATIF, 0);
    }

    #[test]
    fn receive_filters_and_fifo() {
        let (mut d, mut c) = configured();
        d.receive(&CanFrame::fd(0x124, false, &[9]), Time::from_us(5));
        assert_eq!(read32(&mut d, &mut c, 0x060) & 1, 0, "filtered out");
        d.receive(&CanFrame::fd(0x123, false, &[7, 8, 9]), Time::from_us(5));
        assert_eq!(read32(&mut d, &mut c, 0x060) & 1, 1);
        assert!(d.int_asserted());
        assert_eq!(read32(&mut d, &mut c, 0x020), 1 << 1, "C1RXIF FIFO1");
        assert_eq!(read32(&mut d, &mut c, 0x018) >> 24, 1, "RXCODE");
        let ua = read32(&mut d, &mut c, 0x064) as u16;
        let obj = read(&mut d, &mut c, 0x400 + ua, 12);
        assert_eq!(u32::from_le_bytes(obj[0..4].try_into().unwrap()), 0x123);
        let r1 = u32::from_le_bytes(obj[4..8].try_into().unwrap());
        assert_eq!(r1 & 0xF, 3);
        assert_ne!(r1 & (1 << 7), 0, "FDF");
        assert_eq!(&obj[8..11], &[7, 8, 9]);
        write(&mut d, &mut c, 0x05D, &[0x01]); // UINC
        assert!(!d.int_asserted());
    }

    #[test]
    fn rx_overflow() {
        let (mut d, mut c) = configured();
        for _ in 0..17 {
            d.receive(&CanFrame::fd(0x123, false, &[0]), Time::ZERO);
        }
        assert_ne!(read32(&mut d, &mut c, 0x060) & STA_RXOVIF, 0);
        assert_ne!(read32(&mut d, &mut c, 0x028), 0);
        write(&mut d, &mut c, 0x060, &[0]);
        assert_eq!(read32(&mut d, &mut c, 0x060) & STA_RXOVIF, 0);
    }

    #[test]
    fn extended_filter() {
        let (mut d, mut c) = configured();
        write(&mut d, &mut c, 0x003, &[mode::CONFIG]);
        let id: u32 = 0x1234_5678 & 0x1FFF_FFFF;
        let obj = (id >> 18) & 0x7FF | (id & 0x3FFFF) << 11 | 1 << 30;
        write(&mut d, &mut c, 0x1D1, &[0x00]);
        write32(&mut d, &mut c, 0x1F8, obj);
        write32(&mut d, &mut c, 0x1FC, 0x7FFF_FFFF);
        write(&mut d, &mut c, 0x1D1, &[0x81]);
        write(&mut d, &mut c, 0x003, &[mode::NORMAL_FD]);
        d.receive(&CanFrame::fd(id, false, &[1]), Time::ZERO);
        assert_eq!(read32(&mut d, &mut c, 0x060) & 1, 0, "standard frame does not match an EXIDE filter");
        d.receive(&CanFrame::fd(id, true, &[1]), Time::ZERO);
        assert_eq!(read32(&mut d, &mut c, 0x060) & 1, 1);
        let ua = read32(&mut d, &mut c, 0x064) as u16;
        let r1 = u32::from_le_bytes(read(&mut d, &mut c, 0x404 + ua, 4).try_into().unwrap());
        assert_eq!(r1 >> 11 & 0x1F, 1, "FILHIT");
    }

    #[test]
    fn crc_read_and_safe_write() {
        let (mut d, mut c) = configured();
        // READ_CRC of 4 SFR bytes of IOCON
        let addr = 0xE04u16;
        let out = xfer(&mut d, &mut c, &[(SPI_READ_CRC << 4) | (addr >> 8) as u8, addr as u8, 4, 0, 0, 0, 0, 0, 0]);
        let data = &out[3..7];
        let crc = u16::from_be_bytes([out[7], out[8]]);
        let mut all = vec![(SPI_READ_CRC << 4) | (addr >> 8) as u8, addr as u8, 4];
        all.extend_from_slice(data);
        assert_eq!(crc, crc16_cms(&all));
        assert_eq!(u32::from_le_bytes(data.try_into().unwrap()) & (1 << 16), 1 << 16, "GPIO0 senses termination");
        // WRITE_SAFE with a good CRC writes, with a bad one does not
        let a = 0x410u16;
        let mut cmd = vec![(SPI_WRITE_SAFE << 4) | (a >> 8) as u8, a as u8, 0xAA, 0xBB, 0xCC, 0xDD];
        let crc = crc16_cms(&cmd);
        cmd.extend_from_slice(&crc.to_be_bytes());
        xfer(&mut d, &mut c, &cmd);
        assert_eq!(read(&mut d, &mut c, a, 4), vec![0xAA, 0xBB, 0xCC, 0xDD]);
        let mut bad = vec![(SPI_WRITE_SAFE << 4) | (a >> 8) as u8, a as u8, 1, 2, 3, 4, 0, 0];
        bad[6] = 0x12;
        xfer(&mut d, &mut c, &bad);
        assert_eq!(read(&mut d, &mut c, a, 4), vec![0xAA, 0xBB, 0xCC, 0xDD]);
        assert_ne!(read32(&mut d, &mut c, 0xE08) & (1 << 16), 0, "CRCERRIF");
        // WRITE_CRC: length in words for RAM
        let mut wc = vec![(SPI_WRITE_CRC << 4) | (a >> 8) as u8, a as u8, 1, 5, 6, 7, 8];
        let crc = crc16_cms(&wc);
        wc.extend_from_slice(&crc.to_be_bytes());
        xfer(&mut d, &mut c, &wc);
        assert_eq!(read(&mut d, &mut c, a, 4), vec![5, 6, 7, 8]);
    }

    #[test]
    fn reset_returns_to_config_mode() {
        let (mut d, mut c) = configured();
        xfer(&mut d, &mut c, &[0, 0]);
        assert_eq!((read32(&mut d, &mut c, 0x000) >> 21) & 7, mode::CONFIG as u32);
        assert!(d.bit_rates().is_none());
        assert_eq!(read32(&mut d, &mut c, 0x000), CON_RESET);
    }

    fn trec(d: &mut Mcp251xfd, c: &mut TestCtx) -> (u32, u32, u32) {
        let v = read32(d, c, 0x034);
        (v & 0xFF, (v >> 8) & 0xFF, v >> 16)
    }

    #[test]
    fn ack_errors_stop_at_error_passive() {
        let (mut d, mut c) = configured();
        queue_frame(&mut d, &mut c, 0x10, &[0], 1);
        write(&mut d, &mut c, 0x050 + 2, &[3 << 5]); // TXAT = unlimited, so it keeps trying
        for n in 1..=40 {
            let r = d.tx_request().expect("still requested");
            d.tx_done(r.token, Time::from_us(10 * n), false);
        }
        let (rec, tec, state) = trec(&mut d, &mut c);
        assert_eq!((rec, tec), (0, 128), "+8 per ACK error until error passive, then no more");
        assert_eq!(state, 1 << 4, "TXBP only");
        assert_eq!(d.error_counters().state, ErrorState::Passive);
        assert_ne!(read32(&mut d, &mut c, 0x01C) & INT_CERRIF as u32, 0, "CERRIF on the state changes");
        assert_ne!(read32(&mut d, &mut c, 0x01C) & INT_IVMIF as u32, 0);
        let bdiag1 = read32(&mut d, &mut c, 0x03C);
        assert_ne!(bdiag1 & 1 << 18, 0, "NACKERR");
        assert_eq!((read32(&mut d, &mut c, 0x038) >> 8) & 0xFF, 40, "NTERRCNT");
        // A frame that gets through counts down again
        let r = d.tx_request().unwrap();
        d.tx_done(r.token, Time::from_ms(1), true);
        assert_eq!(trec(&mut d, &mut c).1, 127);
        assert_eq!(d.error_counters().state, ErrorState::Warning);
        assert_eq!(read32(&mut d, &mut c, 0x03C) & 0xFFFF, 1, "EFMSGCNT");
    }

    #[test]
    fn bit_errors_lead_to_bus_off_and_automatic_recovery() {
        let (mut d, mut c) = configured();
        queue_frame(&mut d, &mut c, 0x10, &[0], 1);
        write(&mut d, &mut c, 0x050 + 2, &[3 << 5]);
        for n in 0..32 {
            let r = d.tx_request().expect("sending until bus-off");
            d.tx_error(r.token, Time::from_us(30 * n), BusError::Bit { data_phase: n % 2 == 1 });
        }
        assert_eq!(d.error_counters().state, ErrorState::BusOff, "TEC 256 > 255");
        let (_, tec, state) = trec(&mut d, &mut c);
        assert_eq!(tec, 255);
        assert_eq!(state & (1 << 5), 1 << 5, "TXBO");
        assert!(d.tx_request().is_none(), "a bus-off node does not send");
        assert!(!d.acknowledges(), "or acknowledge");
        assert!(d.bit_rates().is_some(), "but stays on the wire to count recessive bits");
        d.receive(&CanFrame::fd(0x123, false, &[1]), Time::from_ms(1));
        assert_eq!(read32(&mut d, &mut c, 0x060) & 1, 0, "or receive");
        let bdiag0 = read32(&mut d, &mut c, 0x038);
        assert_eq!(((bdiag0 >> 8) & 0xFF, bdiag0 >> 24), (16, 16), "NTERRCNT and DTERRCNT by phase");
        assert_eq!(d.bus_off_events, 1);
        // 128 sequences of 11 recessive bits
        d.recessive_sequences(Time::from_ms(2), 100);
        assert_eq!(d.error_counters().state, ErrorState::BusOff);
        d.recessive_sequences(Time::from_ms(3), 28);
        assert_eq!(d.error_counters(), ErrorCounters { tec: 0, rec: 0, state: ErrorState::Active });
        assert_ne!(read32(&mut d, &mut c, 0x03C) & 1 << 23, 0, "TXBOERR: went bus-off and recovered");
        assert!(d.tx_request().is_some(), "the queued frame goes out again");
    }

    #[test]
    fn receive_errors_and_recovery_from_error_passive() {
        let (mut d, mut c) = configured();
        for _ in 0..130 {
            d.rx_error(Time::ZERO, BusError::Bit { data_phase: false });
        }
        assert_eq!(trec(&mut d, &mut c), (130, 0, 1 << 3), "RXBP");
        assert_eq!(read32(&mut d, &mut c, 0x038) & 0xFF, 130, "NRERRCNT");
        assert_ne!(read32(&mut d, &mut c, 0x03C) & 1 << 20, 0, "NSTUFERR");
        assert!(d.acknowledges(), "error passive nodes still acknowledge");
        d.receive(&CanFrame::fd(0x124, false, &[9]), Time::ZERO);
        assert_eq!(trec(&mut d, &mut c).0, 120, "back to 119-127 after a good frame");
        assert_eq!(d.error_counters().state, ErrorState::Warning);
        d.receive(&CanFrame::fd(0x124, false, &[9]), Time::ZERO);
        assert_eq!(trec(&mut d, &mut c).0, 119);
    }

    #[test]
    fn stuck_dominant_bus() {
        // A receiver: a stuff error, then +8 for every 8 dominant bits after its error flag
        let (mut d, mut c) = configured();
        d.dominant_bits(Time::ZERO, 6);
        assert_eq!(trec(&mut d, &mut c).0, 1);
        d.dominant_bits(Time::ZERO, 1000);
        let (rec, tec, _) = trec(&mut d, &mut c);
        assert!(rec > 127 && tec == 0, "error passive, never bus-off: {rec}");
        assert_eq!(d.error_counters().state, ErrorState::Passive);
        d.recessive_sequences(Time::ZERO, 1);
        // A transmitter whose frame the bus cut off: its TEC climbs instead, to bus-off
        let (mut d, mut c) = configured();
        queue_frame(&mut d, &mut c, 0x10, &[0], 1);
        let r = d.tx_request().unwrap();
        d.tx_error(r.token, Time::ZERO, BusError::StuckDominant);
        d.dominant_bits(Time::ZERO, 300);
        assert_eq!(d.error_counters().state, ErrorState::BusOff);
        assert_eq!(trec(&mut d, &mut c).0, 0);
    }

    #[test]
    fn configuration_mode_and_error_interrupts() {
        let (mut d, mut c) = configured();
        assert_eq!(trec(&mut d, &mut c).2, 0);
        for _ in 0..100 {
            d.rx_error(Time::ZERO, BusError::Bit { data_phase: true });
        }
        assert_eq!(trec(&mut d, &mut c).2, 0b11, "RXWARN | EWARN");
        assert!(!d.int_asserted(), "the titan driver leaves CERRIF and IVMIF disabled");
        let int = read32(&mut d, &mut c, 0x01C);
        write32(&mut d, &mut c, 0x01C, int | ((INT_CERRIF as u32) << 16));
        assert!(d.int_asserted());
        assert_eq!(read32(&mut d, &mut c, 0x018) & 0x7F, 0x41, "ICODE: error interrupt");
        write(&mut d, &mut c, 0x01D, &[!(INT_CERRIF >> 8) as u8]);
        assert!(!d.int_asserted(), "CERRIF clears by writing 0");
        // Configuration mode resets the protocol engine; TXBO reads set while off the bus
        write(&mut d, &mut c, 0x003, &[mode::CONFIG]);
        assert_eq!(trec(&mut d, &mut c), (0, 0, 1 << 5));
        write(&mut d, &mut c, 0x003, &[mode::NORMAL_FD]);
        assert_eq!(trec(&mut d, &mut c), (0, 0, 0));
    }

    #[test]
    fn power_loss_resets_the_controller() {
        let (mut d, mut c) = configured();
        d.power_on_reset();
        assert!(d.bit_rates().is_none(), "back in configuration mode, off the bus");
        assert_eq!(read32(&mut d, &mut c, 0x000), CON_RESET);
    }
}
