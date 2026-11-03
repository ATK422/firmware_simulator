//! Fault injection against the real firmware: how the Titan safety system (`titan_rp2040::safety`, the link and
//! the CAN driver) reacts when the bus, a sensor, the power or the robot computer misbehaves.
//!
//! Tests named `firmware_finding_*` pin down behaviour that looks wrong or questionable in the firmware (written
//! up in the PR); they assert what the firmware does today, so a fix shows up as a failing test to update.

use sim_devices::can::BusError;
use sim_devices::dshot::Esc;
use sim_test::titan_link::{DeviceToHost, HostToDevice};
use sim_test::{AnalogFault, Bench, ErrorState, Fault, I2cFault, RobotBench, Time};

const FAULT_WATCHDOG_RESET: u32 = 1 << 0;
const FAULT_WATCHDOG_WARNING: u32 = 1 << 1;
const FAULT_CAN_INTERNAL_ERROR: u32 = 1 << 2;

fn ms(t: u64) -> Time {
    Time::from_ms(t)
}

/// The Talos power board, connected to its mock bridge (client 1) after 1 s.
fn power_board() -> Option<Bench> {
    let elf = sim_test::titan_rs("power_board")?;
    let mut b = Bench::builder(&elf).board("mk2/power_board").bridge(1).record_bus().boot().unwrap();
    b.run_until(ms(1000)).unwrap();
    assert!(b.logged("ROS connected"));
    Some(b)
}

/// CANmore heartbeats the bridge received after `t`.
fn heartbeats_after(b: &Bench, t: Time) -> Vec<Time> {
    b.bridge.as_ref().unwrap().state().heartbeats.iter().map(|h| h.0).filter(|&h| h > t).collect()
}

fn statuses_after(b: &Bench, t: Time) -> usize {
    b.statuses().iter().filter(|s| s.0 > t).count()
}

/// Runs scenarios on copies of `base`, in parallel.
fn scenarios<R: Send>(base: &Bench, cases: Vec<Box<dyn FnOnce(Bench) -> R + Send>>) -> Vec<R> {
    let benches: Vec<Bench> = cases.iter().map(|_| base.snapshot()).collect();
    std::thread::scope(|s| {
        let handles: Vec<_> = benches.into_iter().zip(cases).map(|(b, f)| s.spawn(move || f(b))).collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    })
}

// ------------------------------------------------------------------------------------------------ CAN

#[test]
fn can_bit_errors_are_counted_and_survived() {
    let Some(mut b) = power_board() else { return };
    // A burst of interference: three frames in ten hit by a bit error for 3 s
    b.inject(Fault::CanErrorRate(0.3)).unwrap();
    let mut peak = 0;
    for _ in 0..30 {
        b.run_for(ms(100));
        peak = peak.max(b.can_errors().unwrap().tec);
    }
    b.inject(Fault::CanErrorRate(0.0)).unwrap();
    let errors = b.frames().iter().filter(|r| matches!(r.error, Some(BusError::Bit { .. }))).count();
    assert!(errors > 10, "{errors} error frames");
    assert!(peak > 24, "the transmit error counter climbs: {peak}");
    assert!(b.can_controller().unwrap().rx_errors > 0, "and the receive errors on the bridge's frames count too");
    // Every destroyed frame was retransmitted: the link held and the board kept reporting
    assert!(!b.logged("Lost connection to ROS"), "{:#?}", b.session.logs);
    assert!(statuses_after(&b, ms(2000)) >= 2);
    // Once the interference stops, each frame sent takes one off the TEC
    let tec = b.can_errors().unwrap().tec;
    b.run_for(ms(3000));
    let after = b.can_errors().unwrap();
    assert!(after.tec < tec || after.tec == 0, "{tec} -> {after:?}");
    assert_eq!(b.faults(), Some(0), "bit errors the controller retransmits are not a firmware fault");
}

#[test]
fn can_bus_off_and_automatic_recovery() {
    let Some(mut b) = power_board() else { return };
    // Heavy interference: most frames are destroyed, the board's TEC passes 255 and it goes bus-off
    b.inject(Fault::CanErrorRate(0.6)).unwrap();
    let mut states = Vec::new();
    for _ in 0..40 {
        b.run_for(ms(100));
        states.push(b.can_errors().unwrap().state);
    }
    let c = b.can_controller().unwrap();
    assert!(c.bus_off_events >= 1, "bus-off: {states:?}");
    // The MCP2517FD recovers by itself after 128 x 11 recessive bits, and says so in C1BDIAG1.TXBOERR
    assert!(states.contains(&ErrorState::BusOff) || c.bus_off_events > 0);
    b.inject(Fault::CanErrorRate(0.0)).unwrap();
    b.run_for(ms(3000));
    assert_ne!(b.can_errors().unwrap().state, ErrorState::BusOff);
    let hb = heartbeats_after(&b, b.now() - ms(2000));
    assert!(hb.len() >= 3, "heartbeats go out again: {hb:?}");
    // The firmware never noticed: bus-off is not a fault, not logged, and the link survived
    assert!(!b.logged("CAN controller error"));
    assert!(!b.logged("Lost connection to ROS"));
    assert_eq!(b.faults(), Some(0));
}

#[test]
fn can_stuck_dominant_bus_kills_the_vehicle_and_recovers() {
    let Some(mut b) = power_board() else { return };
    let soft_kill = |b: &Bench| b.session.machine.sys.gpio.input(2);
    assert!(soft_kill(&b), "enabled: the software kill output is high");
    b.inject(Fault::CanStuckDominant(true)).unwrap();
    b.run_for(ms(1500));
    // Nothing gets through: the bridge's pings stop reaching the board, which drops ROS after 700 ms and kills
    let last_ping = b.bridge.as_ref().unwrap().state().last_ping.unwrap();
    let lost = b.log_time("Lost connection to ROS").expect("link lost");
    assert!(lost > ms(1000) && lost < ms(1000) + ms(800), "{lost}");
    assert!(!soft_kill(&b), "killed");
    // Every node on a dominant bus counts receive errors into error passive; nothing it sends starts, so its TEC
    // stays put and it never goes bus-off
    let e = b.can_errors().unwrap();
    assert_eq!(e.state, ErrorState::Passive, "{e:?}");
    assert_eq!(e.tec, 0);
    assert!(b.bridge.as_ref().unwrap().state().pings > 0 && last_ping > ms(1500));
    // The bridge's interface queue fills and its later pings are refused (ENOBUFS), so they do not pile up
    assert!(b.session.can_bus.as_ref().unwrap().endpoint_dropped > 0);
    b.inject(Fault::CanStuckDominant(false)).unwrap();
    let released = b.now();
    let again = b.wait_for_log_after("ROS connected", released, ms(1000)).unwrap();
    assert!(again - released < ms(150), "reconnected {again}");
    b.run_for(ms(1500));
    assert!(soft_kill(&b), "enabled again");
    assert_eq!(b.can_errors().unwrap().state, ErrorState::Active, "REC back down after good frames");
    assert!(statuses_after(&b, released) >= 1);
    // No more than the interface queue's ten frames burst out on release, which the RX FIFO takes
    assert_eq!(b.faults().unwrap() & FAULT_CAN_INTERNAL_ERROR, 0);
}

#[test]
fn can_flood_starves_the_board_until_it_stops() {
    let Some(mut b) = power_board() else { return };
    // A babbling node at the highest priority takes the whole bus: lower IDs lose every arbitration
    b.inject(Fault::CanFlood { id: 0x000, extended: false, len: 8, load: 1.0 }).unwrap();
    b.run_for(ms(1500));
    assert!(b.logged("Lost connection to ROS"));
    let flood = &b.session.can_bus.as_ref().unwrap().faults.floods[0];
    assert!(flood.sent > 5000, "{} flood frames", flood.sent);
    assert_eq!(b.can_errors().unwrap().state, ErrorState::Active, "losing arbitration is not an error");
    b.inject(Fault::CanFloodStop).unwrap();
    let stop = b.now();
    b.wait_for_log_after("ROS connected", stop, ms(500)).unwrap();
    // A flood at a lower priority and a third of the bus leaves room for everyone
    b.inject(Fault::CanFlood { id: 0x7FF, extended: false, len: 8, load: 0.3 }).unwrap();
    let t = b.now();
    b.run_for(ms(2000));
    assert_eq!(b.log_times("Lost connection to ROS").len(), 1, "no second disconnect");
    assert!(statuses_after(&b, t) >= 1);
    assert!(heartbeats_after(&b, t).len() >= 3);
}

#[test]
fn can_unplugged_board_goes_error_passive_not_bus_off() {
    let Some(mut b) = power_board() else { return };
    b.inject(Fault::CanUnplugged(true)).unwrap();
    b.run_for(ms(1500));
    assert!(b.logged("Lost connection to ROS"));
    // No ACK: TEC +8 per attempt up to error passive, then ACK errors stop counting (ISO 11898-1 rule 3,
    // exception 1), so a lone node never goes bus-off
    let e = b.can_errors().unwrap();
    assert_eq!((e.tec, e.state), (128, ErrorState::Passive), "{e:?}");
    assert_eq!(b.can_controller().unwrap().bus_off_events, 0);
    // The rest of the bus no longer hears it
    let unplugged = ms(1000);
    assert!(heartbeats_after(&b, unplugged + ms(10)).is_empty());
}

/// Once its frames run out of retransmission attempts with the transmit queue full, the board stays silent on a
/// healthy bus until the 10 s "offline" reset of the CAN driver.
#[test]
fn firmware_finding_tx_queue_stalls_after_retransmission_attempts_run_out() {
    let Some(mut b) = power_board() else { return };
    b.inject(Fault::CanUnplugged(true)).unwrap();
    b.run_until(ms(3000)).unwrap();
    b.inject(Fault::CanUnplugged(false)).unwrap();
    // Plugged back in: the board hears the bridge again straight away...
    let back = b.wait_for_log_after("ROS connected", ms(3000), ms(500)).unwrap();
    assert!(back < ms(3200));
    b.run_until(ms(10_000)).unwrap();
    // ...but sends nothing: every queued frame used its 3 attempts (TXQ with TXAT = 3), the MCP2517FD cleared
    // TXREQ, and the driver only sets TXREQ when it queues a new frame, which it cannot while the queue is full
    assert!(heartbeats_after(&b, ms(3000)).is_empty(), "silent on a working bus");
    assert_eq!(statuses_after(&b, ms(3000)), 0);
    // The driver's offline reset (10 s without a confirmed heartbeat) re-initialises the controller
    let reset = b.wait_for_log_after("CAN offline for", ms(3000), ms(3000)).unwrap();
    assert!(reset > ms(10_500) && reset < ms(11_500), "{reset}");
    b.run_until(ms(12_500)).unwrap();
    assert!(!heartbeats_after(&b, reset).is_empty(), "talking again after the reset");
    assert!(statuses_after(&b, reset) >= 1);
}

/// Every loss of the ROS link raises FAULT_WATCHDOG_WARNING, although the watchdog was never close to firing.
#[test]
fn firmware_finding_watchdog_warning_on_every_disconnect() {
    let Some(mut b) = power_board() else { return };
    b.inject(Fault::Bridge(false)).unwrap();
    let lost = b.wait_for_log("Lost connection to ROS", ms(1000)).unwrap();
    let warn = b.wait_for_log("Fault FAULT_WATCHDOG_WARNING raised", ms(100)).unwrap();
    // One safety tick later: `safety_task` switches to the 3 s setup timeout and its 500 ms margin, but compares
    // them against the 250 ms timeout it last fed with (elapsed 10 ms + margin 500 ms > 250 ms). The argument is
    // the "remaining" time, 240 ms.
    assert!(warn - lost < ms(15), "{lost} {warn}");
    let line = &b.session.logs.iter().find(|(_, l)| l.message.contains("FAULT_WATCHDOG_WARNING")).unwrap().1.message;
    assert!(line.contains("0x3a981"), "{line}");
    b.inject(Fault::Bridge(true)).unwrap();
    b.run_for(ms(1500));
    // It is never lowered, so the board reports it (and the heartbeat error bit) from then on
    assert_eq!(b.faults(), Some(FAULT_WATCHDOG_WARNING));
}

// ------------------------------------------------------------------------------------------------ links and kill

#[test]
fn bridge_loss_and_physical_kill_switch_on_the_power_board() {
    let Some(base) = power_board() else { return };
    let soft_kill = |b: &Bench| b.session.machine.sys.gpio.input(2);
    let results = scenarios(
        &base,
        vec![
            // titan_bridge stops: no pings, ROS counts as lost 700 ms after the last one, which kills
            Box::new(move |mut b: Bench| {
                b.inject(Fault::Bridge(false)).unwrap();
                let last = b.bridge.as_ref().unwrap().state().last_ping.unwrap();
                let lost = b.wait_for_log("Lost connection to ROS", ms(1500)).unwrap();
                let lost_after = lost - last;
                b.run_for(ms(20));
                let killed = !soft_kill(&b);
                b.inject(Fault::Bridge(true)).unwrap();
                b.wait_for_log_after("ROS connected", b.now(), ms(500)).unwrap();
                b.run_for(ms(200));
                (lost_after, killed, true, soft_kill(&b))
            }),
            // The physical kill switch (GPIO 1, low asserts kill) pulled and reinserted
            Box::new(move |mut b: Bench| {
                b.inject(Fault::Gpio { pin: 1, level: Some(false) }).unwrap();
                b.run_for(ms(20));
                let killed = !soft_kill(&b);
                b.inject(Fault::Gpio { pin: 1, level: None }).unwrap();
                b.run_for(ms(20));
                (Time::ZERO, killed, false, soft_kill(&b))
            }),
        ],
    );
    let (lost_after, killed, _, enabled) = results[0];
    assert!(lost_after >= ms(700) && lost_after < ms(720), "lost {lost_after} after the last ping");
    assert!(killed && enabled, "killed while the bridge was gone, enabled again once it is back");
    let (_, killed, _, enabled) = results[1];
    assert!(killed && enabled, "physical kill switch");
}

/// On the power board, the `state/kill` the link forwards lands on kill switch 0, which is also the physical kill
/// switch (`KILL_SWITCH_PHYSICAL = 0`) that `physical_kill_task` rewrites every 10 ms: a ROS kill only pulses the
/// software kill output.
#[test]
fn firmware_finding_power_board_ros_kill_lands_on_the_physical_switch() {
    let Some(mut b) = power_board() else { return };
    b.inject(Fault::KillSwitch(Some(true))).unwrap();
    let mut low = 0;
    let mut pulses = 0;
    let mut was_high = true;
    for _ in 0..1000 {
        b.run_for(ms(1));
        let high = b.session.machine.sys.gpio.input(2);
        if !high {
            low += 1;
            if was_high {
                pulses += 1;
            }
        }
        was_high = high;
    }
    // Killed for a moment at every ping (each carries the kill switch), enabled again at the next safety tick
    assert!(pulses >= 9, "{pulses} kill pulses");
    eprintln!("{pulses} kill pulses, low for {low} ms of 1000");
    assert!(low <= pulses * 15, "low for {low} ms in all: the vehicle is mostly enabled despite `state/kill`");
}

/// The ESC board: thruster commands stop (neutral DShot frames) when the vehicle is killed, and losing core 1
/// resets the board.
#[test]
fn esc_thrusters_go_neutral_when_killed() {
    let Some(elf) = sim_test::titan_rs("esc_board") else { return };
    use sim_test::titan_link::esc::HostToDevice as EscCmd;
    let mut base = Bench::builder(&elf).board("mk2/esc_board").bridge(2).boot().unwrap();
    base.run_until(ms(300)).unwrap();
    base.send(HostToDevice::Esc(EscCmd::SetRawMode(true)));
    // The ESCs get neutral for 5 s after power-up; keep commanding like the controller (targets last 500 ms)
    let targets = [300i16, -300, 200, 400, 0, 0, 0, 0];
    let mut t = 300;
    while t < 5600 {
        base.send(HostToDevice::Esc(EscCmd::ThrusterRpm(targets)));
        t += 100;
        base.run_until(ms(t)).unwrap();
    }
    let escs = |b: &Bench| -> Vec<Esc> {
        b.session.machine.sys.pin_devices.iter().filter_map(|d| d.as_any().downcast_ref::<Esc>()).cloned().collect()
    };
    assert!(
        escs(&base).iter().all(|e| e.frames.last().unwrap().throttle == 348 || e.frames.last().unwrap().throttle > 0)
    );
    // When every thruster's frames went neutral (throttle 0) for good, after `from`
    fn neutral_from(e: &Esc, from: Time) -> Option<Time> {
        let after: Vec<_> = e.frames.iter().filter(|f| f.at >= from).collect();
        let last_on = after.iter().rposition(|f| f.throttle != 0);
        match last_on {
            None => after.first().map(|f| f.at),
            Some(i) => after.get(i + 1).map(|f| f.at),
        }
    }
    let commanding = move |b: &mut Bench, d: u64| {
        let end = b.now() + ms(d);
        while b.now() < end {
            b.send(HostToDevice::Esc(EscCmd::ThrusterRpm(targets)));
            b.run_for(ms(100));
        }
    };
    #[allow(clippy::type_complexity)]
    let results: Vec<(Vec<Option<Time>>, Bench)> = scenarios(
        &base,
        vec![
            // titan_bridge goes away (with it the thruster commands)
            Box::new(move |mut b: Bench| {
                let t = b.now();
                b.inject(Fault::Bridge(false)).unwrap();
                b.run_for(ms(1000));
                (escs(&b).iter().map(|e| neutral_from(e, t).map(|n| n - t)).collect(), b)
            }),
            // Commands keep coming but the kill switch stops being forwarded and times out
            Box::new(move |mut b: Bench| {
                let t = b.now();
                b.inject(Fault::KillSwitch(None)).unwrap();
                commanding(&mut b, 1500);
                (escs(&b).iter().map(|e| neutral_from(e, t).map(|n| n - t)).collect(), b)
            }),
            // `state/kill` asserts kill (the kill switch pulled, or the operator killing the vehicle)
            Box::new(move |mut b: Bench| {
                let t = b.now();
                b.inject(Fault::KillSwitch(Some(true))).unwrap();
                commanding(&mut b, 500);
                (escs(&b).iter().map(|e| neutral_from(e, t).map(|n| n - t)).collect(), b)
            }),
            // Core 1, which runs the control loop and sends the DShot frames, hangs
            Box::new(move |mut b: Bench| {
                let t = b.now();
                b.inject(Fault::Hang { core: 1, interrupts_masked: false }).unwrap();
                commanding(&mut b, 1000);
                // The last frame before the board went down
                let last: Vec<Option<Time>> = escs(&b)
                    .iter()
                    .map(|e| e.frames.iter().map(|f| f.at).rfind(|&a| a > t && a < t + ms(250)))
                    .collect();
                (last, b)
            }),
        ],
    );
    // Bridge gone: the targets go stale after 500 ms (DSHOT_MIN_UPDATE_RATE_MS) and the thrusters go neutral; the
    // kill follows when ROS is lost at 700 ms
    let (neutral, b) = &results[0];
    for n in neutral {
        let n = n.expect("neutral");
        assert!(n > ms(300) && n < ms(620), "neutral {n} after the bridge stopped");
    }
    assert!(b.logged("Lost connection to ROS"));
    assert!(b.logged("FAULT_RPM_CMD_TIMEOUT"), "the stale command is reported");
    // Kill switch timed out: neutral within 500 ms of its last update although commands keep coming
    let (neutral, b) = &results[1];
    for n in neutral {
        let n = n.expect("neutral although commanded");
        assert!(n > ms(390) && n < ms(520), "neutral {n} after the kill switch stopped");
    }
    assert!(!b.logged("Lost connection"));
    let st = b.statuses();
    let last = &st.last().unwrap().1;
    assert_eq!((last.kill_switches_timed_out, last.kill_switches_asserting_kill), (1, 0), "{last:?}");
    // Kill asserted: neutral at the next ping (every 100 ms), within a control loop tick of it
    let (neutral, _) = &results[2];
    for n in neutral {
        let n = n.expect("neutral");
        assert!(n < ms(101), "neutral {n} after the kill");
    }
    // Core 1 hung: no more DShot frames at all (the ESCs see the signal stop), core 0 notices after 250 ms and
    // panics, which resets the board
    let (last, b) = &results[3];
    for l in last {
        assert!(l.is_none_or(|l| l < base.now() + ms(5)), "frames after the hang: {l:?}");
    }
    let noticed = b.log_time("Core 1 did not check in").expect("core 0 notices");
    assert!(noticed - base.now() > ms(240) && noticed - base.now() < ms(280), "{noticed}");
    // The panic resets the board (counted as a watchdog timeout, see the panic finding); after the reboot core 1
    // sends neutral again, since the ESCs are held neutral for 5 s after power-up
    let reboot = b.log_time("Crash counts since power on: CrashCounts { watchdog_timeouts: 1").expect("rebooted");
    for e in escs(b) {
        let after: Vec<_> = e.frames.iter().filter(|f| f.at > reboot).collect();
        assert!(!after.is_empty() && after.iter().all(|f| f.throttle == 0), "{}", e.name);
        assert!(e.frames.iter().all(|f| f.at < noticed || f.at > reboot), "no frames while it was down");
    }
}

#[test]
fn puddles_cable_unplugged_and_packet_loss() {
    let Some(elf) = sim_test::titan_rs("puddles_backplane") else { return };
    let mut base = Bench::builder(&elf).board("mk2/puddles_backplane").bridge(1).boot().unwrap();
    base.wait_for_log("ROS connected", ms(2000)).unwrap();
    base.run_for(ms(1000));
    let results = scenarios(
        &base,
        vec![
            Box::new(|mut b: Bench| {
                let t = b.now();
                b.inject(Fault::EthernetLink(false)).unwrap();
                b.run_for(ms(1500));
                let down = b.log_time("Ethernet link down").map(|d| d - t);
                let lost = b.log_time("Lost connection to ROS").map(|d| d - t);
                let msgs = b.messages().iter().filter(|m| m.0 > t + ms(50)).count();
                let replug = b.now();
                b.inject(Fault::EthernetLink(true)).unwrap();
                let up = b.wait_for_log_after("Ethernet link up", replug, ms(500)).ok();
                let again = b.wait_for_log_after("ROS connected", replug, ms(1000)).ok();
                b.run_for(ms(1200));
                let st = statuses_after(&b, replug);
                (down, lost, msgs, up.map(|u| u - replug), again.map(|a| a - replug), st)
            }),
            Box::new(|mut b: Bench| {
                let t = b.now();
                // A third of the datagrams lost both ways: the link survives (the board drops ROS only after 700 ms
                // without a ping)
                b.inject(Fault::PacketLoss { to_board: 0.3, from_board: 0.3 }).unwrap();
                b.run_for(ms(3000));
                let lost = b.log_time("Lost connection to ROS").map(|d| d - t);
                let st = statuses_after(&b, t);
                let loss = b.session.eth_loss.as_ref().unwrap();
                let dropped = (loss.dropped_to_board + loss.dropped_from_board) as usize;
                (None, lost, dropped, None, None, st)
            }),
        ],
    );
    let (down, lost, msgs, up, again, st) = results[0];
    eprintln!(
        "cable out: link down after {down:?}, ROS lost after {lost:?}; back in: up after {up:?}, ROS after {again:?}"
    );
    assert!(down.unwrap() < ms(800), "the W5500 driver polls the PHY link: {down:?}");
    let lost = lost.expect("ROS lost with the cable out");
    assert!(lost > ms(600) && lost < ms(800), "{lost}");
    assert_eq!(msgs, 0, "nothing reaches the bridge while the cable is out");
    assert!(up.unwrap() < ms(800));
    assert!(again.unwrap() < ms(1000), "{again:?}");
    assert!(st >= 1, "firmware statuses again");
    let (_, lost, dropped, _, _, st) = results[1];
    assert!(lost.is_none(), "survives 30 % loss");
    assert!(dropped > 10 && st >= 1, "{dropped} dropped, {st} statuses");
}

// ------------------------------------------------------------------------------------------------ sensors

/// The camera cage with its MS5837 depth sensor zeroed and publishing.
fn camera_cage() -> Option<Bench> {
    let elf = sim_test::titan_rs("camera_cage_bb")?;
    let mut b = Bench::builder(&elf).board("mk2/camera_cage_bb").bridge(4).boot().unwrap();
    b.wait_for_log("Depth zeroed", ms(4000)).unwrap();
    b.run_for(ms(500));
    Some(b)
}

fn depths(b: &Bench, after: Time) -> Vec<(Time, f32)> {
    b.picked(|m| match m {
        DeviceToHost::CameraCage(sim_test::titan_link::camera_cage::DeviceToHost::Depth(d)) => Some(*d),
        _ => None,
    })
    .into_iter()
    .filter(|d| d.0 > after)
    .collect()
}

const FAULT_DEPTH_ERROR: u32 = 1 << 7;

#[test]
fn depth_sensor_failures_on_the_camera_cage() {
    let Some(base) = camera_cage() else { return };
    assert!(!depths(&base, Time::ZERO).is_empty());
    assert_eq!(base.faults(), Some(0));
    let t0 = base.now();
    let results = scenarios(
        &base,
        vec![
            // The sensor stops answering (unplugged connector): every conversion fails with an address NAK
            Box::new(move |mut b: Bench| {
                b.inject(Fault::I2c { address: 0x76, fault: Some(I2cFault::Absent) }).unwrap();
                b.run_for(ms(1500));
                let raised = b.log_time("Fault FAULT_DEPTH_ERROR raised").map(|t| t - t0);
                let faults = b.faults().unwrap();
                let during = depths(&b, t0 + ms(100)).len();
                // Plugged back in: readings come back
                b.inject(Fault::I2c { address: 0x76, fault: None }).unwrap();
                let back = b.now();
                b.run_for(ms(1500));
                (raised, faults, during, depths(&b, back).len(), b.faults().unwrap())
            }),
            // Noise on SDA: bits flip in what the sensor sends. The ADC reads carry no CRC
            Box::new(move |mut b: Bench| {
                b.inject(Fault::I2c { address: 0x76, fault: Some(I2cFault::Glitch(0.02)) }).unwrap();
                b.run_for(ms(3000));
                let d = depths(&b, t0);
                let wild = d.iter().filter(|(_, d)| d.abs() > 0.2).count();
                (None, b.faults().unwrap(), d.len(), wild, 0)
            }),
            // The device stretches the clock for 30 ms per byte: conversions run late
            Box::new(move |mut b: Bench| {
                b.inject(Fault::I2c { address: 0x76, fault: Some(I2cFault::Stretch(ms(30))) }).unwrap();
                b.run_for(ms(2000));
                let raised = b.log_time("Fault FAULT_DEPTH_ERROR raised").map(|t| t - t0);
                (raised, b.faults().unwrap(), depths(&b, t0 + ms(100)).len(), 0, 0)
            }),
        ],
    );
    // Absent: FAULT_DEPTH_ERROR after 5 failed polls in a row (BAD_READS_FAULT_COUNT, 50 ms apart), reported in
    // the firmware status; no depth is published while the sensor is gone
    let (raised, faults, during, after, faults_after) = results[0];
    let raised = raised.expect("FAULT_DEPTH_ERROR");
    assert!(raised >= ms(150) && raised < ms(400), "raised after {raised}");
    assert_eq!(faults & FAULT_DEPTH_ERROR, FAULT_DEPTH_ERROR);
    assert_eq!(during, 0);
    assert!(after >= 20, "{after} readings after the sensor came back");
    assert_eq!(faults_after & FAULT_DEPTH_ERROR, FAULT_DEPTH_ERROR, "firmware finding: the fault is never lowered");
    // Glitches: wrong depths are published as if real; nothing is raised (firmware finding: no plausibility check)
    let (_, faults, n, wild, _) = results[1];
    assert!(n > 40);
    assert!(wild > 0, "corrupted readings reach ROS");
    assert_eq!(faults & FAULT_DEPTH_ERROR, 0);
    // Clock stretching: each conversion takes longer than the 50 ms poll period; the overruns raise the fault
    let (raised, faults, n, _, _) = results[2];
    assert!(raised.is_some(), "overruns reported");
    assert_eq!(faults & FAULT_DEPTH_ERROR, FAULT_DEPTH_ERROR);
    assert!(n < 20, "{n}");
}

#[test]
fn stuck_depth_sensor_goes_unnoticed() {
    let Some(mut b) = camera_cage() else { return };
    b.inject(Fault::I2c { address: 0x76, fault: Some(I2cFault::Stuck) }).unwrap();
    // The first conversion after the fault is the last one the sensor gets right
    b.run_for(ms(200));
    let t = b.now();
    // The vehicle dives to 2 m, but the sensor's readings are frozen
    let surface = b.get("depth.pressure_pa");
    b.set("depth.pressure_pa", surface + 2.0 * 997.0 * 9.80665);
    b.run_for(ms(1000));
    let d = depths(&b, t + ms(100));
    assert!(d.len() > 10 && d.iter().all(|(_, d)| d.abs() < 0.05), "frozen at the surface: {d:?}");
    assert_eq!(b.faults(), Some(0), "nothing notices a frozen sensor");
    b.inject(Fault::I2c { address: 0x76, fault: None }).unwrap();
    let t = b.now();
    b.run_for(ms(500));
    let d = depths(&b, t + ms(100));
    assert!(d.iter().all(|(_, d)| (d + 2.0).abs() < 0.05), "2 m once it answers again: {d:?}");
}

#[test]
fn sensor_faults_on_the_power_board() {
    let Some(base) = power_board() else { return };
    let electrical = |b: &Bench, after: Time| {
        b.picked(|m| match m {
            DeviceToHost::Power(sim_test::titan_link::power::DeviceToHost::Electrical(e)) => Some(*e),
            _ => None,
        })
        .into_iter()
        .filter(|e| e.0 > after)
        .map(|e| e.1)
        .collect::<Vec<_>>()
    };
    let t0 = base.now();
    let results = scenarios(
        &base,
        vec![
            // The port battery sense input shorted to ground
            Box::new(move |mut b: Bench| {
                b.inject(Fault::Adc { input: 0, fault: Some(AnalogFault::Shorted(0.0)) }).unwrap();
                b.run_for(ms(1500));
                let e = electrical(&b, t0 + ms(300));
                (e.last().map(|e| (e.port_voltage, e.stbd_voltage)), b.faults().unwrap())
            }),
            // The sense input open: it floats
            Box::new(move |mut b: Bench| {
                b.inject(Fault::Adc { input: 0, fault: Some(AnalogFault::Open) }).unwrap();
                b.run_for(ms(1500));
                let e = electrical(&b, t0 + ms(300));
                (e.last().map(|e| (e.port_voltage, e.stbd_voltage)), b.faults().unwrap())
            }),
            // The SHT41 glitches: its CRC catches every bad reading
            Box::new(move |mut b: Bench| {
                b.inject(Fault::I2c { address: 0x44, fault: Some(I2cFault::Glitch(0.5)) }).unwrap();
                b.run_for(ms(3000));
                (None, b.faults().unwrap())
            }),
            // 50 mV of noise on the sense pin (0.55 V at the battery through the 10k/1k divider): the spread of
            // the published voltage, as (mean, standard deviation)
            Box::new(move |mut b: Bench| {
                b.inject(Fault::Adc { input: 0, fault: Some(AnalogFault::Noise(0.05)) }).unwrap();
                b.run_for(ms(20_000));
                let v: Vec<f32> = electrical(&b, t0 + ms(300)).iter().map(|e| e.port_voltage).collect();
                let mean = v.iter().sum::<f32>() / v.len() as f32;
                let sd = (v.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / v.len() as f32).sqrt();
                (Some((mean, sd)), b.faults().unwrap())
            }),
            // The MCP3426 rail monitor stops answering
            Box::new(move |mut b: Bench| {
                b.inject(Fault::I2c { address: 0x68, fault: Some(I2cFault::Absent) }).unwrap();
                b.run_for(ms(3000));
                let e = electrical(&b, t0 + ms(2000));
                (e.last().map(|e| (e.three_volt_voltage, e.balanced_voltage)), b.faults().unwrap())
            }),
        ],
    );
    let (v, faults) = results[0];
    let (port, stbd) = v.unwrap();
    assert!(port.abs() < 0.1 && (stbd - 16.4).abs() < 0.3, "{port} {stbd}");
    assert_eq!(faults, 0, "firmware finding: a 0 V battery reading raises nothing");
    let (v, _) = results[1];
    let (port, _) = v.unwrap();
    assert!((port - 16.2).abs() < 2.0, "an open input wanders near its last value: {port}");
    let (_, faults) = results[2];
    assert_eq!(faults, 1 << 7, "the power board's FAULT_SHT41_ERROR (7) is raised");
    let (v, faults) = results[3];
    let (mean, sd) = v.unwrap();
    eprintln!("port battery with 50 mV pin noise: {mean:.2} V, sd {sd:.2} V");
    // Each published value is one raw conversion: the noise goes straight to ROS, unfiltered and unflagged
    assert!((mean - 16.2).abs() < 0.3 && sd > 0.3, "{mean} {sd}");
    assert_eq!(faults, 0);
    let (v, faults) = results[4];
    let (three, balanced) = v.unwrap();
    assert!(three.is_nan() && balanced.is_nan(), "rails without a reading are NaN: {three} {balanced}");
    assert_eq!(faults, 1 << 6, "FAULT_ADC_ERROR (6)");
}

/// The smart battery's BQ40Z80 gauge (bit-banged SMBus) drops off and comes back: FAULT_BQ40_NOT_CONNECTED is
/// raised while it is gone and lowered when the board reconnects to it.
#[test]
fn smart_battery_gauge_lost_and_found() {
    let Some(elf) = sim_test::titan_rs("smart_battery") else { return };
    const NOT_CONNECTED: u32 = 1 << 6;
    let mut b = Bench::builder(&elf).board("mk2/sbh_mcu").bridge(21).boot().unwrap();
    b.run_until(ms(1500)).unwrap();
    assert!(b.logged("Connected to gauge, serial 21"));
    assert_eq!(b.faults(), Some(0));
    b.inject(Fault::I2c { address: 0x0B, fault: Some(I2cFault::Absent) }).unwrap();
    let lost = b.wait_for_log("Lost the gauge", ms(2000)).unwrap();
    b.run_for(ms(500));
    assert_eq!(b.faults().unwrap() & NOT_CONNECTED, NOT_CONNECTED);
    b.inject(Fault::I2c { address: 0x0B, fault: None }).unwrap();
    let back = b.now();
    let found = b.wait_for_log_after("Connected to gauge", back, ms(2000)).unwrap();
    assert!(b.logged("Fault FAULT_BQ40_NOT_CONNECTED lowered"));
    b.run_for(ms(1200));
    assert_eq!(b.faults().unwrap() & NOT_CONNECTED, 0, "lowered once the gauge answers (lost {lost}, found {found})");
    // The link to the bridge was never affected
    assert!(!b.logged("Lost connection to ROS"));
}

// ------------------------------------------------------------------------------------------------ power and resets

#[test]
fn hung_task_trips_the_watchdog_with_a_crash_record() {
    let Some(base) = power_board() else { return };
    let results = scenarios(
        &base,
        [false, true]
            .into_iter()
            .map(|masked| -> Box<dyn FnOnce(Bench) -> Bench + Send> {
                Box::new(move |mut b: Bench| {
                    // A task that never yields, or one stuck in a critical section with interrupts masked
                    b.inject(Fault::Hang { core: 0, interrupts_masked: masked }).unwrap();
                    let t = b.now();
                    b.wait_for_log_after("Watchdog timeout after", t, ms(1000)).unwrap();
                    b.run_for(ms(1500));
                    b
                })
            })
            .collect(),
    );
    for b in &results {
        let t = ms(1000);
        // The safety task stops feeding: the watchdog (250 ms once ROS is connected) resets the chip
        assert_eq!(b.session.machine.sys.watchdog.reason, 1, "WATCHDOG_REASON.TIMER");
        let reboot = b.log_time("Watchdog timeout after").unwrap();
        assert!(reboot > t + ms(250) && reboot < t + ms(450), "{reboot}");
        // The crash record survives in the scratch registers: counted, reported, and the fault raised
        assert!(b.logged("watchdog_timeouts: 1, panics: 0"), "{:#?}", b.session.logs);
        assert_eq!(b.faults().unwrap() & FAULT_WATCHDOG_RESET, FAULT_WATCHDOG_RESET);
        assert!(b.log_times("ROS connected").iter().any(|&c| c > reboot));
    }
}

/// The same hang in a C (pico-sdk) image, which has no busy loop of its own to hang in: `titan_safety` reports the
/// watchdog reset and its crash count on UART0 after the reboot.
#[test]
fn hung_c_firmware_reports_a_watchdog_reset() {
    let Some(elf) = sim_test::titan_c("basic_template/template_project.elf") else { return };
    let mut b = Bench::builder(&elf).boot().unwrap();
    b.run_until(ms(500)).unwrap();
    assert!(b.uart(0).contains("[safety]: Clean Boot: Power-on Reset"), "{}", b.uart(0));
    b.inject(Fault::Hang { core: 0, interrupts_masked: true }).unwrap();
    b.run_until(ms(2500)).unwrap();
    let out = b.uart(0);
    assert!(out.contains("[safety]: Watchdog Reset (Total Crashes: 1) - UNKNOWN_SAFETY_ACTIVE"), "{out}");
    assert!(out.contains("Fault FAULT_WATCHDOG_RESET (0) Raised"), "{out}");
}

/// A panic before ROS connects (3 s watchdog) is recorded as a panic; a brownout then clears the crash record.
#[test]
fn panic_and_brownout_resets() {
    let Some(elf) = sim_test::titan_rs("power_board") else { return };
    // No bridge: the board stays in its setup phase
    let mut b = Bench::builder(&elf).board("mk2/power_board").boot().unwrap();
    b.run_until(ms(500)).unwrap();
    b.inject(Fault::Panic).unwrap();
    b.wait_for_log("Reset by panic", ms(1000)).unwrap();
    assert!(b.logged("watchdog_timeouts: 0, panics: 1"), "{:#?}", b.session.logs);
    assert_eq!(b.session.machine.sys.watchdog.reason, 2, "WATCHDOG_REASON.FORCE");
    b.run_for(ms(300));
    b.inject(Fault::Brownout { off: ms(50) }).unwrap();
    b.run_for(ms(500));
    // A power-on reset clears the scratch registers: the crash counts start again and no fault is raised
    let counts: Vec<&str> = b
        .session
        .logs
        .iter()
        .filter(|(_, l)| l.message.contains("Crash counts"))
        .map(|(_, l)| l.message.as_str())
        .collect();
    assert_eq!(counts.len(), 3, "{counts:?}");
    assert!(counts[2].contains("watchdog_timeouts: 0, panics: 0"));
    assert_eq!(b.session.machine.sys.vreg.chip_reset & (1 << 8), 1 << 8, "CHIP_RESET.HAD_POR");
    assert_eq!(b.session.machine.sys.watchdog.reason, 0);
}

/// A panic while ROS is connected is recorded as a watchdog timeout: the panic handler busy-waits
/// `cortex_m::asm::delay(12_500_000)` (12.5 M loop iterations of 3 cycles, 300 ms at 125 MHz) before it triggers
/// the watchdog, longer than the 250 ms active timeout, so the timer fires first and `setup` takes the
/// `TimedOut` branch, which ignores the panic marker.
#[test]
fn firmware_finding_panic_while_connected_counts_as_watchdog_timeout() {
    let Some(mut b) = power_board() else { return };
    b.inject(Fault::Panic).unwrap();
    let t = b.now();
    b.wait_for_log_after("Crash counts since power on", t, ms(1000)).unwrap();
    assert!(b.logged("PANIC: panicked"));
    assert!(!b.logged("Reset by panic"));
    assert!(b.logged("Watchdog timeout after"));
    assert!(b.logged("watchdog_timeouts: 1, panics: 0"), "{:#?}", b.session.logs);
    assert_eq!(b.session.machine.sys.watchdog.reason, 1, "WATCHDOG_REASON.TIMER, not FORCE");
}

// ------------------------------------------------------------------------------------------------ robot

/// Three Talos boards on the internal bus: one is unplugged, browns out, or babbles; the others carry on.
#[test]
fn robot_board_faults_stay_local() {
    for elf in ["poac_board", "camera_cage_bb", "sbc_board"] {
        if sim_test::titan_rs(elf).is_none() {
            return;
        }
    }
    let keep = ["poac", "camera_cage", "sbc"];
    let mut r = RobotBench::load("talos.toml", &[("internal", 1)], |spec| {
        spec.board.retain(|b| keep.contains(&b.name.as_str()));
        spec.bus.retain(|b| b.name == "internal");
    })
    .unwrap();
    let mut bridges = vec![r.bridges[0].clone().unwrap()];
    for client in [4, 7] {
        let b = sim_test::MockBridge::new(client, true, Time::ZERO);
        b.set_kill(Some(false));
        r.robot.buses[0].endpoints.push(Box::new(b.clone()));
        bridges.push(b);
    }
    r.inject_at(ms(1000), "camera_cage", Fault::CanUnplugged(true));
    r.inject_at(ms(1000), "sbc", Fault::Brownout { off: ms(100) });
    r.run_until(ms(3000));
    let logs = |n: &str| r.board(n).logs.iter().map(|(t, l)| (*t, l.message.clone())).collect::<Vec<_>>();
    // The camera cage is off the bus: it lost ROS and its bridge stopped hearing from it
    assert!(logs("camera_cage").iter().any(|(_, l)| l.contains("Lost connection to ROS")));
    assert!(bridges[1].state().heartbeats.iter().all(|h| h.0 < ms(1010)));
    // The SBC board browned out, rebooted and reconnected
    let sbc = logs("sbc");
    assert_eq!(sbc.iter().filter(|(_, l)| l.contains("Crash counts")).count(), 2);
    assert!(sbc.iter().any(|(t, l)| *t > ms(1100) && l.contains("ROS connected")), "{sbc:#?}");
    assert!(bridges[2].statuses().iter().any(|s| s.0 > ms(2000)));
    // The POAC board noticed nothing
    assert!(!logs("poac").iter().any(|(_, l)| l.contains("Lost connection")));
    assert!(bridges[0].statuses().iter().filter(|s| s.0 > ms(1000)).count() >= 2);
    let poac = r.robot.index("poac").unwrap();
    let e = sim_host::fault::can_error_counters(&mut r.robot.boards[poac]);
    assert_eq!(e[0].state, ErrorState::Active);
}
