# Fault injection

Faults model what goes wrong around a board the way the hardware fails, not by poking firmware state: a bit error
destroys a frame on the wire and the CAN controllers count it; an absent sensor NAKs its address; a brownout
removes power from the RP2040 and the chips on its 3.3 V rail. The firmware's real safety code then does whatever
it does, and tests check that against what it is meant to do.

- [Injecting faults](#injecting-faults)
- [Every fault](#every-fault)
- [What the models do](#what-the-models-do)
- [Firmware findings so far](#firmware-findings-so-far)

## Injecting faults

From the command line, at a virtual time (repeatable):

```bash
# run: TIME@FAULT
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 \
    --fault 1s@can-stuck --fault 3s@can-release --time 4s
#   [INFO  sim_host::fault] [1.000000s] fault: CanStuckDominant(true)
#   [WARN  sim_devices::can] CAN bus can: endpoint 0's transmit queue is full, dropping its frames
#   [1.600303s] 1.468418 INFO  Lost connection to ROS
#   [INFO  sim_host::fault] [3.000000s] fault: CanStuckDominant(false)
#   [3.000500s] 2.868612 INFO  ROS connected

# robot: TIME@BOARD:FAULT
firmware_simulator robot robots/talos.toml --mock-bridge internal:4 --fault 1.5s@camera_cage:i2c-absent:0x76
#   [1.521600s] camera_cage: 1.389617 ERROR Depth sensor failed: Comm(NoAcknowledge)
#   [1.521700s] camera_cage: 1.389639 ERROR Fault FAULT_DEPTH_INIT_ERROR raised (arg 0x5) at camera_cage_bb/src/cage.rs:131
```

With `--candump`, frames destroyed by an injected error are marked `(destroyed: Bit { data_phase: true })`.

From a test ([testing.md](testing.md#faults)): `Bench::inject(fault)` now or `Bench::inject_at(time, fault)`;
`RobotBench::inject(board, fault)` and `inject_at(time, board, fault)`. Faults are `sim_test::Fault` values
(`sim_host::fault::Fault`), and `"i2c-absent:0x76".parse::<Fault>()` gives the same as the command line.

A fault that does not fit the board is refused with a reason (a CAN fault on a board without a bus, a bridge
fault without a mock bridge, an I2C address nobody answers, Ethernet faults without Ethernet): the CLI exits with
an error, `inject` returns it, and a scheduled `inject_at` panics.

## Every fault

In specs, numbers may be hex (`0x76`). Times inside a fault spec take `us`, `ms` or `s` and mean milliseconds
without a unit (`brownout:50` is 50 ms); the `TIME@` in front follows `--time` (`1.5s`, a bare number is seconds).

| Spec | `Fault` | What it models |
|---|---|---|
| `can-error-rate:P` | `CanErrorRate(P)` | Every frame on the bus is hit by a bit error with probability `P` (0 ends it) |
| `can-corrupt:N` | `CanCorrupt(N)` | The next `N` frames on the bus are hit by a bit error |
| `can-stuck`, `can-release` | `CanStuckDominant(bool)` | The bus held dominant (a shorted transceiver): nothing can start, a frame on the wire is cut off, every node's error counters climb |
| `can-unplug`, `can-plug` | `CanUnplugged(bool)` | The board's CAN controller unplugged: it sends alone with no ACK and hears nothing; the rest of the bus carries on |
| `can-flood:ID:LOAD[:LEN]`, `can-flood-stop` | `CanFlood { id, extended, len, load }`, `CanFloodStop` | A babbling node sending frame `ID` (`LEN` bytes, 8 by default; CAN FD on an FD bus; extended above 0x7FF) at `LOAD` (0-1) of the bus capacity, competing in arbitration |
| `i2c-absent:ADDR` | `I2c { address, fault: Some(I2cFault::Absent) }` | The device does not answer: address NAK (unpowered, unplugged, dead) |
| `i2c-intermittent:ADDR:P` | `I2cFault::Intermittent(P)` | Each START is NAKed with probability `P` (a marginal connector) |
| `i2c-stuck:ADDR` | `I2cFault::Stuck` | Answers, but every read replays the last answer to the same command (checksums stay valid; only a frozen value shows it) |
| `i2c-glitch:ADDR:P` | `I2cFault::Glitch(P)` | Each byte read has a bit flipped with probability `P` (noise on SDA) |
| `i2c-stretch:ADDR:TIME` | `I2cFault::Stretch(t)` | The device holds SCL low for `TIME` before every byte |
| `i2c-ok:ADDR` | `I2c { fault: None, .. }` | Repairs it |
| `adc-short:IN:VOLTS` | `Adc { input, fault: Some(AnalogFault::Shorted(v)) }` | ADC input `IN` (0-3) shorted to a voltage (0 V: to ground, or the measured wire broken off) |
| `adc-open:IN` | `AnalogFault::Open` | The pin open: it reads a value wandering from the last real one, about 0.01 V per conversion |
| `adc-noise:IN:SIGMA` | `AnalogFault::Noise(sigma)` | Gaussian noise of `SIGMA` volts on top (a bad ground) |
| `adc-spikes:IN:P:VOLTS` | `AnalogFault::Spikes(p, v)` | With probability `P` a conversion reads `±VOLTS` off (transients) |
| `adc-ok:IN` | `Adc { fault: None, .. }` | Repairs it |
| `gpio:PIN:high`, `gpio:PIN:low`, `gpio:PIN:release` | `Gpio { pin, level }` | A pin driven from outside, or released: the power board's physical kill switch is GPIO 1, low while it asserts kill |
| `brownout:TIME` | `Brownout { off }` | The supply drops out for `TIME`: the RP2040 and the chips on its 3.3 V rail (CAN and Ethernet controllers) lose power, and the RP2040 comes back through a power-on reset. In a robot the board stays off for that long; on a single `Bench` its clock jumps over the gap |
| `hang:CORE` | `Hang { core, interrupts_masked: false }` | A task that never yields: the core spins on a `b .` (from the image, or placed in XIP SRAM for images without one), so the safety task stops feeding the watchdog |
| `hang-masked:CORE` | `Hang { interrupts_masked: true, .. }` | The same inside a critical section that never ends |
| `panic` | `Panic` | Core 0 calls the firmware's panic path (`__defmt_default_panic`, the target of `defmt::panic!`) |
| `watchdog` | `WatchdogTimeout` | The watchdog resets the chip as if its timer ran out (`WATCHDOG_REASON.TIMER`) |
| `bridge-stop`, `bridge-start` | `Bridge(bool)` | titan_bridge (the mock bridge on the board's bus or cable) stops or starts again |
| `kill:on`, `kill:off`, `kill:none` | `KillSwitch(Some(true) / Some(false) / None)` | The `state/kill` the bridge forwards with every ping: kill, enable, or stop forwarding (the firmware's kill switch then times out) |
| `eth-unplug`, `eth-plug` | `EthernetLink(bool)` | The Puddles Ethernet cable: the W5500's PHY link follows it |
| `packet-loss:TO:FROM` | `PacketLoss { to_board, from_board }` | Frames between the cable and the host lost with these probabilities (0 and 0 end it) |

`Bench` also has `inject_panic` (the same as `Fault::Panic`), `watchdog_reset` (a forced watchdog reset, scratch
registers and SRAM kept) and `power_cycle` (power off and on, SRAM lost).

## What the models do

**CAN.** A bit error cuts the frame off where it struck and is followed by an error frame (flag, delimiter,
intermission); the frame is not delivered and its transmitter retries. The MCP2517FD/MCP2518FD model keeps
ISO 11898-1 fault confinement as its datasheet describes it: TEC/REC counting, error warning and error passive,
bus-off above 255 with automatic recovery after 128 x 11 recessive bits, an error-passive transmitter's suspend
transmission. The firmware sees it where the chip shows it: `C1TREC`, `C1BDIAG0/1` (error counts by phase,
error kinds, `TXBOERR`), `CERRIF`, `IVMIF`. In a test, `Bench::can_errors()` returns the controller's counters and
`can_controller()` the model. Assumptions (not checked against a real chip): TEC reads 255 while bus-off; entering
configuration mode clears the counters; when retransmission attempts run out, `TXREQ` is cleared and the message
stays at the head of the TXQ. Local errors seen by only one receiver are not modelled.

**Mock bridge queue.** The mock bridge queues at most 10 frames, Linux's default `txqueuelen`; titan_bridge drops a
ping its SocketCAN write refuses (`ENOBUFS`). Without that, a stuck or flooded bus made the mock bridge pile up
hundreds of pings that then overflowed the board's RX FIFO, which a real bridge would not do. A SocketCAN
interface or a cannelloni tunnel queues at most 1024 frames (`BRIDGE_TX_QUEUE`: room for the bursts a batching
cannelloni peer or a lagging real-time run delivers). Beyond the limit the bus drops the newest frames, counts them
in `CanBus::endpoint_dropped` and logs the first.

**Parts.** `FaultyI2c` wraps any I2C device (on the controllers or bit-banged) and `FaultyAnalog` any ADC
source; repaired, they pass everything through. Their random numbers are seeded and copied with snapshots, so
fault runs reproduce. Only the MCP251xFD and the W5500 are reset by a brownout; I2C sensors keep their state (the
firmware resets them when it reboots).

**Chip.** A hang during the first ~130 ms of boot, before the safety code starts the watchdog, hangs for good,
as on hardware.

**Lockups.** A fault inside HardFault (or NMI), or one HardFault cannot be entered for, locks a Cortex-M0+ core up:
on a real RP2040 it stops executing (its PC reads `0xfffffffe`; the emulator reports the PC of the instruction
that faulted) while the other core, the peripherals and the watchdog go on. So `run` and `robot` keep a
locked-up board running: the core stays stopped, and if the watchdog is counting down it resets the chip when it
runs out, which boots again as any watchdog reset does (through the real bootrom with `--rom-boot`, or directly
again after a direct boot; [running.md](running.md#boot-modes-and-the-flash)). A scheduled `brownout` or
`watchdog` fault restarts the chip too, so the run waits for it. Each lockup is logged
(`core0 locked up at ...: it stays stopped until the watchdog resets the chip at 0.010001s`), and the end of the
run says how many there were (`3 lockup(s), 2 chip reset(s); core0 is locked up now`). The run stops with an error
only when nothing can end the lockup: the watchdog is not running, no such fault is scheduled, and the other core
is off, locked up too or asleep in the bootrom (core 1 never launched). `--stop-on-lockup` stops at the first
lockup, as before. In code, `Session::stop_on_lockup` (true by default, so tests see `StopReason::Lockup`) and
`Machine::lockup_is_final`; `test_programs/lockup.S` and `crates/rp2040/tests/m6_resets.rs` show it.

Modelled: the core stopping, the other core and the peripherals running on, the watchdog reset ending it, the
lockup counted per board. Not modelled: the debug halt a connected debugger would make of a lockup (with
`--gdb` the core simply stays locked up), the `LOCKUP` signal the RP2040 routes to the watchdog's
`WDSEL`/`PSM` lockup reset options (unused by pico-sdk and embassy) and whatever the chip does with
`CTRL.PAUSE_DBG0/1` while a core is locked up (the emulator does not pause the watchdog). What happens after the
reset is the real bootrom's doing with `--rom-boot`: if the flash is still busy programming or erasing (a
multi-second erase was interrupted by the watchdog), the chip ignores the bootrom's reads of boot2, and after
about half a second of trying the bootrom falls back to USB boot (BOOTSEL) and stays there
(`a_rom_boot_while_the_flash_is_erasing_falls_back_to_usb_boot`). A direct boot (the default) skips the bootrom:
the core fetches from the busy flash at once, faults, and the HardFault handler, in flash too, locks it up for
good (the watchdog reset left the watchdog off). Run firmware that erases or programs the flash with
`--rom-boot` to see what a board would do.

## Firmware findings so far

Behaviour of the firmware (not the emulator) that looks wrong, each pinned down by a test asserting the behaviour
of the pinned `external/titan_firmware_rs`, so a fix shows up as a test to update. No firmware was changed here.
Findings 1-3 were fixed in titan_firmware_rs (its PR #1, merged as 4d86d47; pinned here from 94a107e): their tests
now check the fixed behaviour, and the rows say which commit fixed them. Unless noted, the tests are in
`crates/sim_test/tests/faults.rs`.

| # | Finding | Test |
|---|---|---|
| 1 | **Fixed in titan_firmware_rs c59474f.** The CAN transmit queue stalled after its retransmission attempts ran out, muting the board for up to 10 s on a healthy bus: the TXQ used 3 attempts, each exhausted set cleared TXREQ, and `Mcp251xfd::transmit` only set TXREQ again when it queued a new frame, which it could not once the queue was full. After a cable was re-plugged the board heard the bridge at once ("ROS connected") but sent nothing until the driver's offline reset about 10 s later. Now the TXQ retries without limit and drains as soon as the cable is back. Depends on the modelled TXAT semantics: check on hardware | `can_tx_queue_drains_when_the_cable_is_plugged_back_in` |
| 2 | **Fixed in titan_firmware_rs 8d4dfd6.** Every loss of ROS raised `FAULT_WATCHDOG_WARNING`, and it was never lowered: after `deinit` the safety task compared the setup timeout's 500 ms margin with the 250 ms timeout it last fed | `ros_disconnect_raises_no_watchdog_warning` |
| 3 | **Fixed in titan_firmware_rs b2afb53.** A panic while ROS was connected was recorded as a watchdog timeout: the panic handler busy-waited 300 ms before triggering the watchdog, longer than the 250 ms active timeout, and the `TimedOut` path ignored the panic marker. Now the handler reloads the watchdog before its 100 ms log drain, and a timeout with the marker set counts as a panic | `panic_while_connected_is_recorded_as_a_panic`; `crates/sim_host/tests/flash.rs`, `panic_leaves_a_crash_record_across_watchdog_resets` |
| 4 | **On the power board, `state/kill` lands on the physical kill switch's slot** (`KILL_SWITCH_ROS` and `KILL_SWITCH_PHYSICAL` are both 0 there), so a forwarded ROS kill only pulses the kill output for ~10 ms. titan_bridge skips that subscription for power boards; the firmware relies on it | `firmware_finding_power_board_ros_kill_lands_on_the_physical_switch` |
| 5 | **CAN error states are invisible to the firmware.** The Rust driver does not enable CERRIF, so error passive and bus-off are never logged or raised as faults; at a 60 % error rate the board goes bus-off, recovers, and ends with no fault and no log line | `can_bus_off_and_automatic_recovery` |
| 6 | **Latched faults that never clear**: `FAULT_DEPTH_ERROR` after the depth sensor answers again (also in the C camera cage), `FAULT_BQ40_COMMAND_FAIL` after the gauge comes back | `depth_sensor_failures_on_the_camera_cage`, `smart_battery_gauge_lost_and_found` |
| 7 | **No plausibility checks on sensor values**: bit flips on the MS5837's reads publish wild depths; a frozen depth sensor goes unnoticed while the vehicle dives; a shorted or open battery sense input publishes 0 V or a wandering value. (The SHT41's CRC does catch glitches, and an absent MCP3426 raises `FAULT_ADC_ERROR`) | `depth_sensor_failures_on_the_camera_cage`, `stuck_depth_sensor_goes_unnoticed`, `sensor_faults_on_the_power_board` |
| 8 | **The first-generation IVC modem loses symbols to pool reverberation at full receive gain.** A symbol counts only if the wanted band is above 300 *and the other band below 300*; mid-slot, the previous symbol still arrives over reflected paths about 30 dB down, which with the assumed 40 dB gain reads around that fixed threshold (over it in about 7 % of the captures at 5.6 m). Which symbols are lost depends on where the board is and on when its captures fall in the slot, which moves with the firmware build; at 5.6 m most packets lose one, retransmissions included (the echoes are the same every time), and a moving board passes through such spots. A threshold relative to the stronger band would not care. Depends on the assumed reflection coefficients and gain | `crates/sim_host/tests/ivc_link.rs`, `firmware_finding_ivc_symbols_lost_to_pool_reverberation` |
| 9 | **The bootrom acknowledges PICOBOOT commands without a data phase whatever their result** (B1 and B2 alike), so a misaligned FLASH_ERASE "succeeds" and erases nothing | `crates/sim_host/tests/usb.rs`, `picoboot_rejects_bad_commands_and_recovers` |
| 10 | **The C IVCBoard2 (titan_firmware `blake/ivc`) overwrites a packet in flight with the next queued byte.** `attempt_writing` dequeues on every main loop pass, also while transmitting, and only replaces `data_to_write`; two bytes enqueued together go out as one packet with the second, and the first is lost. The Rust `ivc_board2` waits for the packet to end. (Read but not run: `debug_tx`/`tx_debug` register `void tx_disable(void *)` as a pico-sdk alarm callback, which takes `(alarm_id_t, void *)` and returns the reschedule time, so the debug tone's end alarm gets the alarm id as its context and returns whatever is in r0) | `crates/sim_host/tests/ivc_link.rs`, `firmware_finding_c_ivc_board2_overwrites_the_packet_in_flight` (on `test_programs/c_ivc_modem`, [acoustics.md](acoustics.md#c-and-rust-ivcboard2-interop)) |
| 11 | **The IVC surface vehicle's answer is lost when Talos has another packet queued** (protocol, the same schedule as the C firmware). SSFB may only transmit in the 400 ms after its ACK, so its answer starts about 830 ms into Talos's packet period, while Talos waits out its interframe space until 1410 ms and then sends its next packet. Talos never hears the answer, and SSFB, listening for its ACK from 760 ms after its own start, hears Talos's packet instead: a 1 symbol there is taken for the ACK, so the answer counts as delivered and is never retransmitted. Related, seen in the pool runs but not pinned down: every pinger ping starts a phantom packet (all 13 symbols missing) that keeps a receiver busy for 825 ms and opens SSFB's transmit window like a real packet; and both ends retransmit every 1410 ms, so two packets that collided keep colliding until both links break | `crates/sim_host/tests/ivc_link.rs`, `firmware_finding_ivc_answer_lost_when_talos_sends_on` |

Observations not pinned down by a test: in a Talos run IVCBoard2's CANmore heartbeat comes every 523 ms and its
link heartbeat every 105 ms instead of 500/100 ms like the other boards (deterministic, so either the firmware's
timing on IVCBoard2 or an emulation effect). From a test campaign with the C IVCBoard2 (titan_firmware `blake/ivc`, with micro-ROS, not
in this repository's tests): `storage_init(true)` erases the top 512 KiB of the flash (`flash_range_erase`: eight 64 KiB
block erases, about 1.2 s at the flash model's typical 150 ms each, up to 16 s at the datasheet's maximum) with interrupts disabled, while the safety code's
watchdog runs with a 250 ms timeout, so the watchdog resets the chip in the middle of the erase. With the default
direct boot the board then locks up for good (see [Lockups](#what-the-models-do)); on hardware, as with
`--rom-boot`, it should end up in USB boot until power-cycled. Either way it is a firmware bug: the erase needs the
watchdog fed (or paused) around it, or erasing sector by sector between feeds.

**To verify locally**: the CAN error model against a real MCP2517FD/MCP2518FD (bus-off reporting, recovery timing,
what TXAT exhaustion does: finding 1 depends on it); findings 1 and 3 on a real board; `--fault` with the real
titan_bridge over vcan (it should log "Ping failed" under `can-stuck`).
