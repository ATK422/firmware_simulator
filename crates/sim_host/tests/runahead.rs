//! Boards of a robot run ahead of each other between the slices where they can see each other
//! (`sim_host::runahead`). The result must be exactly that of taking turns slice by slice: every CAN frame at the
//! same time, every log line at the same time and in the same order across boards, the same machine state, the
//! same statistics.

mod common;

use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};

use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::Robot;
use sim_host::socketcan::Live;
use sim_host::timetravel::{Ran, TimeTravel, Travelled};

/// How a run is done.
#[derive(Clone, Default)]
struct How {
    threads: usize,
    run_ahead: bool,
    /// Boards on these threads (see `Robot::fix_threads`).
    fixed: Option<Vec<usize>>,
    /// `Robot::run_ahead_stress`.
    stress: Option<u64>,
    /// Pieces to run (`run_until` ends); one piece to `END` if empty.
    ends: Vec<Time>,
    /// Edits the robot (after loading it, before running).
    sink_vehicle: bool,
    /// The mock bridge behind a live endpoint (as titan_bridge behind SocketCAN): what it sends is handed in at
    /// the step that reads it.
    live: bool,
    /// Also frames from a live endpoint at chosen times (`ARRIVALS`).
    arrivals: bool,
    /// Paced to the wall clock.
    realtime: bool,
    /// Traces GPIO and exceptions on the camera cage (which follows the vehicle), the actuator and the battery
    /// board, and profiles esc0 and the camera cage.
    watch: bool,
    /// A host thread writes frames to a live endpoint as the run goes, on the wall clock, and their arrivals are
    /// logged here (`Arrivals`).
    fed: Option<sim_host::canlog::CandumpWriter>,
    /// A log of arrivals replayed where the fed endpoint was.
    replayed: Option<Arc<Vec<sim_host::canlog::LogFrame>>>,
    /// Notes the virtual time of every bus step that reads a live endpoint, and the wall clock then.
    probe: Option<Reads>,
    /// Records the run for time travel, with a checkpoint this often (`sim_host::timetravel`)...
    record: Option<Time>,
    /// ...and once at the end, goes back to this time and runs to the end again (a replay).
    rewind: Option<Time>,
    /// Collects code coverage.
    coverage: bool,
}

/// A live endpoint that cannot tell that a frame is waiting before it hands it in (`CanEndpoint::waiting`): its
/// frames cut windows only where they are read, after the boards may have run past.
struct Late<E>(E);

impl<E: sim_devices::can::CanEndpoint> sim_devices::can::CanEndpoint for Late<E> {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, sim_devices::can::CanFrame)> {
        self.0.take_tx(until)
    }
    fn receive(&mut self, frame: &sim_devices::can::CanFrame, at: Time) {
        self.0.receive(frame, at);
    }
    fn live(&self) -> bool {
        self.0.live()
    }
}

/// Each read of a live endpoint: the step's virtual time and the wall clock then.
type Reads = Arc<Mutex<Vec<(Time, std::time::Instant)>>>;

/// A live endpoint handing in what a host thread put in its queue (as a SocketCAN interface does what host tools
/// write).
#[derive(Clone, Default)]
struct Queue(Arc<Mutex<std::collections::VecDeque<sim_devices::can::CanFrame>>>);

impl sim_devices::can::CanEndpoint for Queue {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, sim_devices::can::CanFrame)> {
        self.0.lock().unwrap().drain(..).map(|f| (until, f)).collect()
    }
    fn receive(&mut self, _frame: &sim_devices::can::CanFrame, _at: Time) {}
    fn live(&self) -> bool {
        true
    }
    fn waiting(&mut self, _until: Time) -> bool {
        !self.0.lock().unwrap().is_empty()
    }
}

/// A shared buffer to write a log into.
#[derive(Clone, Default)]
struct Buffer(Arc<Mutex<Vec<u8>>>);

impl std::io::Write for Buffer {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// A live endpoint that sends nothing and notes when it is read.
struct Probe(Reads);

impl sim_devices::can::CanEndpoint for Probe {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, sim_devices::can::CanFrame)> {
        self.0.lock().unwrap().push((until, std::time::Instant::now()));
        Vec::new()
    }
    fn receive(&mut self, _frame: &sim_devices::can::CanFrame, _at: Time) {}
    fn live(&self) -> bool {
        true
    }
}

/// Frames reaching a live endpoint from outside, at chosen virtual times: some alone, some together, two in the
/// same slice, one right at a slice boundary. No board takes them, but every frame changes every controller.
fn arrivals() -> Vec<(Time, sim_devices::can::CanFrame)> {
    use sim_devices::can::CanFrame;
    let mut out = Vec::new();
    for (k, us) in
        [3_333u64, 3_360, 47_000, 47_000, 47_000, 120_050, 250_000, 250_099, 401_234, 650_777].iter().enumerate()
    {
        let f = if k % 2 == 0 {
            CanFrame::fd(0x7F0 + k as u32, true, &[k as u8; 12])
        } else {
            CanFrame::classic(0x7E0, false, &[1])
        };
        out.push((Time::from_us(*us), f));
    }
    out
}

const END: Time = Time::from_ms(700);

/// The ESC boards on threads of their own, the rest on the calling one.
const FIXED: [usize; 9] = [0, 1, 2, 0, 0, 0, 0, 0, 0];

/// Everything the run produced.
#[derive(PartialEq, Debug)]
struct Run {
    frames: Vec<String>,
    /// Every log and UART line from every board as the hooks saw them, in the order they saw them.
    hooks: Vec<String>,
    state: Vec<String>,
    bridge: String,
    /// Every board's code coverage (with `How::coverage`).
    coverage: Vec<Option<Box<rp2040::coverage::CoverageMap>>>,
}

/// Talos (one board switched on later, as in the robot file) with a mock bridge, run as `how` says. Returns the
/// run and what running ahead did.
fn talos(how: &How) -> Option<(Run, sim_host::runahead::Stats)> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    assert_eq!(r.names[1..3], ["esc0", "esc1"]);
    r.threads = how.threads;
    r.run_ahead = how.run_ahead;
    r.run_ahead_stress = how.stress;
    if let Some(f) = &how.fixed {
        r.fix_threads(f.clone());
    }
    if how.sink_vehicle {
        // Started below where it floats, the vehicle rises from the first slice: the camera cage's depth sensor
        // (its pressure input written by the vehicle) changes every millisecond
        let v = r.vehicle("talos").unwrap();
        let mut s = v.vehicle.state;
        s.position[2] += 1.5;
        v.vehicle.place(s);
        v.update_outputs();
    }
    for rb in &mut r.buses {
        rb.bus.record = Some(Vec::new());
    }
    r.realtime = how.realtime;
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    let eps = &mut r.bus("internal").unwrap().endpoints;
    if how.live {
        eps.push(Box::new(Live(bridge.clone())));
    } else {
        eps.push(Box::new(bridge.clone()));
    }
    if how.arrivals {
        eps.push(Box::new(Late(Live(sim_devices::can::Monitor { outbox: arrivals(), ..Default::default() }))));
    }
    if let Some(p) = &how.probe {
        eps.push(Box::new(Probe(p.clone())));
    }
    let mut feeder = None;
    if let Some(log) = &how.fed {
        let queue = Queue::default();
        let inner = Box::new(queue.clone());
        eps.push(Box::new(sim_host::canlog::Arrivals { inner, iface: "internal".into(), log: log.clone() }));
        feeder = Some(std::thread::spawn(move || {
            for k in 0..40u32 {
                std::thread::sleep(std::time::Duration::from_micros(4_000 + 1_300 * (k % 5) as u64));
                queue.0.lock().unwrap().push_back(sim_devices::can::CanFrame::fd(0x700 + k, false, &[k as u8; 8]));
            }
        }));
    }
    if let Some(log) = &how.replayed {
        eps.push(Box::new(sim_host::canlog::arrivals_replay(log, "internal")));
    }
    // The smart battery is alone on the external bus: without the Orin's interface there to acknowledge its frames,
    // it retries each one without end (as CAN does, since titan_firmware_rs c59474f), a frame on the bus every
    // 284 us, and every one of them ends a window
    let battery = MockBridge::new(21, false, Time::ZERO);
    battery.set_kill(Some(false));
    if how.live {
        r.bus("external").unwrap().endpoints.push(Box::new(Live(battery)));
    } else {
        r.bus("external").unwrap().endpoints.push(Box::new(battery));
    }
    let hooks = Arc::new(Mutex::new(Vec::new()));
    for (i, s) in r.boards.iter_mut().enumerate() {
        let h = hooks.clone();
        s.on_log = Some(Box::new(move |t, line| h.lock().unwrap().push(format!("{i} [{t}] {line}"))));
        let h = hooks.clone();
        s.on_uart_line = Some(Box::new(move |t, n, line| h.lock().unwrap().push(format!("{i} [{t}] uart{n} {line}"))));
    }
    let traces: Vec<sim_core::trace::VecSink> = if how.watch {
        [3, 4, 8]
            .iter()
            .map(|&i| {
                let sink = sim_core::trace::VecSink::new(&[
                    sim_core::trace::Category::Gpio,
                    sim_core::trace::Category::Exception,
                ]);
                r.boards[i].machine.set_trace(Box::new(sink.clone()));
                sink
            })
            .collect()
    } else {
        Vec::new()
    };
    if how.watch {
        for i in [1, 3] {
            r.boards[i].start_profile(Time::from_us(37));
        }
    }
    if how.coverage {
        for s in &mut r.boards {
            s.machine.enable_coverage();
        }
    }
    let ends = if how.ends.is_empty() { vec![END] } else { how.ends.clone() };
    if let Some(p) = &how.probe {
        // Where the wall clock starts (pacing starts with the run)
        p.lock().unwrap().push((r.now, std::time::Instant::now()));
    }
    let inputs = r.inputs.clone();
    let mut tt = how.record.map(|every| {
        let mut tt = TimeTravel::new(&mut r, inputs);
        tt.every = every;
        tt
    });
    for &end in &ends {
        match &mut tt {
            Some(tt) => assert_eq!(tt.run_until(&mut r, end), Ran::Reached),
            None => assert!(r.run_until(end).is_none()),
        }
        assert_eq!(r.now, end);
    }
    if let (Some(tt), Some(every)) = (&tt, how.record) {
        // A checkpoint every `every`, or where the first window that ends after it ends (64 slices at most)
        let at_least = ends.last().unwrap().as_ps() / (every.as_ps() + r.slice.as_ps() * 64);
        assert!(tt.stats().taken >= at_least, "{:?}", tt.stats());
    }
    if let (Some(tt), Some(t)) = (&mut tt, how.rewind) {
        let before = r.run_ahead_stats;
        assert_eq!(tt.rewind_to(&mut r, t), Travelled::Arrived);
        assert!(r.now <= t && r.now + r.slice > t, "{} for {t}", r.now);
        let end = *ends.last().unwrap();
        assert_eq!(tt.run_until(&mut r, end), Ran::Reached);
        assert!(tt.at_frontier());
        // The replay ran ahead too (its bridge's frames come from the record)
        if how.run_ahead && how.threads != 1 {
            let replayed = r.run_ahead_stats.slices - before.slices;
            assert!(replayed * 10 >= (end - t).as_ps() / r.slice.as_ps() * 9, "{before:?} {:?}", r.run_ahead_stats);
        }
    }
    if let Some(fixed) = &how.fixed {
        assert_eq!(&r.thread_of_boards(), fixed);
    }
    if let Some(f) = feeder {
        f.join().unwrap();
    }
    let mut frames = Vec::new();
    for rb in &r.buses {
        for rec in rb.bus.record.as_ref().unwrap() {
            frames.push(format!("{} {} {:?} {} {:?}", rec.start, rec.end, rec.sender, rec.acked, rec.frame));
        }
    }
    let mut state = Vec::new();
    for (name, s) in r.names.iter().zip(&r.boards) {
        for (t, line) in &s.logs {
            state.push(format!("{name} [{t}] {line}"));
        }
        state.push(format!("{name} rtt {:?} uart {:?}", s.rtt_text, s.uart_text));
        let m = &s.machine;
        for (c, core) in m.cores.iter().enumerate() {
            let regs: Vec<u32> = (0..core.num_regs()).map(|n| core.read_reg(n)).collect();
            state.push(format!(
                "{name} core{c} {} {} {} {regs:x?}",
                m.core_time(c),
                core.cycles(),
                core.instructions()
            ));
        }
        let mut h = std::collections::hash_map::DefaultHasher::new();
        m.sys.sram.hash(&mut h);
        m.sys.flash[..].hash(&mut h);
        state.push(format!("{name} now {} memory {:x}", m.now, h.finish()));
        state.push(format!(
            "{name} skipped {} timer waits {} fused {}",
            m.spin_skipped, m.timer_waits_skipped, m.quanta_fused
        ));
        let devices: Vec<String> = m
            .sys
            .pin_devices
            .iter()
            .filter_map(|d| d.as_any().downcast_ref::<sim_devices::dshot::Esc>())
            .map(|e| format!("{} {:?} {}", e.name, e.motor(), e.frames.len()))
            .collect();
        state.push(format!("{name} escs {devices:?}"));
    }
    for v in &r.vehicles {
        state.push(format!("{} {:?} {:?}", v.vehicle.name, v.vehicle.state, v.rpms(&r.boards, r.now)));
    }
    for t in &traces {
        let events = t.take();
        assert!(
            events.len() > 100
                && r.boards.iter().all(|b| !b.machine.trace.wants(sim_core::trace::Category::Instruction))
        );
        state.extend(events.iter().map(|(t, e)| format!("trace [{t}] {e}")));
    }
    for (name, s) in r.names.iter().zip(&r.boards) {
        if let Some(p) = &s.profiler {
            let report = p.report();
            assert!(p.samples(0) > 1000, "{name}: {} samples", p.samples(0));
            state.push(format!("{name} profile {report:?}"));
        }
    }
    let bridge = format!("{:?}", bridge.state().received);
    let hooks = std::mem::take(&mut *hooks.lock().unwrap());
    let coverage = r.boards.iter().map(|s| s.machine.coverage.clone()).collect();
    Some((Run { frames, hooks, state, bridge, coverage }, r.run_ahead_stats))
}

/// Asserts two runs are the same, showing where they first differ.
#[track_caller]
fn same(want: &Run, got: &Run, what: &str) {
    for (name, a, b) in
        [("frames", &want.frames, &got.frames), ("hooks", &want.hooks, &got.hooks), ("state", &want.state, &got.state)]
    {
        if let Some(i) = (0..a.len().max(b.len())).find(|&i| a.get(i) != b.get(i)) {
            panic!("{what}: {name} differ at {i}:\n  want {:?}\n  got  {:?}", a.get(i), b.get(i));
        }
    }
    assert_eq!(want.bridge, got.bridge, "{what}: what the bridge received");
    for (i, (a, b)) in want.coverage.iter().zip(&got.coverage).enumerate() {
        assert!(a == b, "{what}: board {i}'s coverage differs");
    }
}

fn turns() -> How {
    How { threads: 1, ..Default::default() }
}

fn ahead(threads: usize) -> How {
    How { threads, run_ahead: true, ..Default::default() }
}

#[test]
fn running_ahead_gives_the_same_run_as_turns() {
    let Some((want, none)) = talos(&turns()) else { return };
    assert_eq!(none, Default::default(), "one thread takes turns");
    assert!(want.frames.len() > 20, "{} frames", want.frames.len());
    assert!(want.hooks.len() > 20, "{} lines", want.hooks.len());
    assert!(want.bridge.len() > 100);
    // On threads slice by slice, as before
    let (lockstep, none) = talos(&How { threads: 3, fixed: Some(FIXED.to_vec()), ..Default::default() }).unwrap();
    assert_eq!(none, Default::default());
    same(&want, &lockstep, "slice by slice");
    for how in [ahead(3), ahead(2), ahead(0), How { fixed: Some(FIXED.to_vec()), ..ahead(3) }] {
        let (got, stats) = talos(&how).unwrap();
        same(&want, &got, &format!("{} threads", how.threads));
        // Most of the run was ahead: windows are many slices long
        assert!(stats.slices >= 6900 && stats.windows < stats.slices / 8, "{stats:?}");
    }
}

/// Every board ignoring where its window ends until it has been rolled back once, and every window cut at a random
/// boundary: boards are rolled back all the time, and the run stays the same.
#[test]
fn rolled_back_boards_rerun_the_same() {
    let Some((want, _)) = talos(&turns()) else { return };
    for seed in [1, 2] {
        let (got, stats) = talos(&How { stress: Some(seed), fixed: Some(FIXED.to_vec()), ..ahead(3) }).unwrap();
        same(&want, &got, &format!("seed {seed}"));
        assert!(stats.rollbacks > 2000, "{stats:?}");
    }
}

/// A rising vehicle changes the camera cage's depth input every millisecond; the camera cage follows the vehicle
/// in the commit rather than running ahead, and its frames cut windows short.
#[test]
fn boards_on_a_moving_vehicle_see_it_move_as_in_turns() {
    let sink = How { sink_vehicle: true, ..Default::default() };
    let Some((want, _)) = talos(&How { threads: 1, ..sink.clone() }) else { return };
    let (got, stats) =
        talos(&How { threads: 3, run_ahead: true, fixed: Some(FIXED.to_vec()), ..sink.clone() }).unwrap();
    same(&want, &got, "");
    assert!(stats.follower_cuts > 0, "{stats:?}");
    let (got, _) = talos(&How { threads: 3, run_ahead: true, stress: Some(7), ..sink }).unwrap();
    same(&want, &got, "");
    // It did move
    let (still, _) = talos(&turns()).unwrap();
    assert_ne!(still.state.last(), want.state.last());
}

/// Runs in pieces of any length (short ones take turns; a piece ends a window) give the same run.
#[test]
fn any_pieces_give_the_same_run() {
    let mut ends = Vec::new();
    let mut t = Time::ZERO;
    for d in [30, 1230, 700, 50_000, 200, 6_431, 777, 23_000].iter().cycle() {
        t = (t + Time::from_us(*d)).min(END);
        ends.push(t);
        if t == END {
            break;
        }
    }
    let Some((want, _)) = talos(&How { ends: ends.clone(), ..turns() }) else { return };
    for stress in [None, Some(3)] {
        let (got, _) = talos(&How { ends: ends.clone(), stress, fixed: Some(FIXED.to_vec()), ..ahead(3) }).unwrap();
        same(&want, &got, &format!("stress {stress:?}"));
    }
}

/// Which board stopped and why, the time every board got to and the robot's time.
type Stopped = (Option<(usize, rp2040::StopReason)>, Vec<Time>, Time);

/// Talos with breakpoints on two boards, run ahead on threads (`stress` as `Robot::run_ahead_stress`) or slice by
/// slice on threads.
fn stops(run_ahead: bool, stress: Option<u64>) -> Option<Stopped> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    r.threads = 3;
    r.run_ahead = run_ahead;
    r.run_ahead_stress = stress;
    r.fix_threads(vec![0, 0, 1, 0, 0, 2, 0, 0, 0]);
    assert!(r.run_until(Time::from_ms(300)).is_none());
    for name in ["led", "esc1"] {
        let m = &mut r.board(name).unwrap().machine;
        let image = m.image.clone().unwrap();
        m.breakpoints.insert((image.symbol("TIMER_IRQ_0").unwrap().addr & !1) + 2);
    }
    let stop = r.run_until(Time::from_ms(310));
    Some((stop, r.boards.iter().map(|b| b.machine.now).collect(), r.now))
}

/// A board that stops ends its window there and is reported as on threads slice by slice: lowest index first, the
/// others run to the end of that slice, the robot left at the slice's start.
#[test]
fn a_stop_ends_the_window_as_on_threads() {
    let Some(want) = stops(false, None) else { return };
    assert!(matches!(want.0, Some((_, rp2040::StopReason::Breakpoint { .. }))), "{want:?}");
    assert_eq!(stops(true, None).unwrap(), want);
    assert_eq!(stops(true, Some(5)).unwrap(), want);
}

/// An endpoint that cannot tell when it will send (as a bridge to real host tools).
struct Unknown;

impl sim_devices::can::CanEndpoint for Unknown {
    fn take_tx(&mut self, _until: Time) -> Vec<(Time, sim_devices::can::CanFrame)> {
        Vec::new()
    }
    fn receive(&mut self, _frame: &sim_devices::can::CanFrame, _at: Time) {}
}

/// A board traced to the instruction, or an endpoint that cannot tell when it sends, keeps the boards in step slice
/// by slice on threads; the switch does too. Other trace sinks and live endpoints do not.
#[test]
fn what_cannot_be_run_ahead_runs_slice_by_slice() {
    if common::titan_elf("esc_board").is_none() {
        return;
    }
    let windows = |edit: &dyn Fn(&mut Robot)| {
        let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
        r.threads = 3;
        edit(&mut r);
        assert!(r.run_until(Time::from_ms(20)).is_none());
        r.run_ahead_stats.windows
    };
    assert!(windows(&|_| {}) > 0);
    assert_eq!(windows(&|r| r.run_ahead = false), 0);
    assert_eq!(windows(&|r| r.threads = 1), 0);
    assert_eq!(
        windows(&|r| {
            let sink = sim_core::trace::VecSink::new(&[sim_core::trace::Category::Instruction]);
            r.boards[4].machine.set_trace(Box::new(sink));
        }),
        0
    );
    assert!(
        windows(&|r| {
            let sink = sim_core::trace::VecSink::new(&[sim_core::trace::Category::Gpio]);
            r.boards[4].machine.set_trace(Box::new(sink));
        }) > 0
    );
    assert!(windows(&|r| r.buses[0].endpoints.push(Box::new(Live(Unknown)))) > 0);
    assert_eq!(windows(&|r| r.buses[0].endpoints.push(Box::new(Unknown))), 0);
    assert!(windows(&|r| r.buses[0].endpoints.push(Box::new(sim_devices::can::Monitor::default()))) > 0);
}

fn live() -> How {
    How { live: true, arrivals: true, ..Default::default() }
}

/// Live endpoints (SocketCAN, cannelloni) cannot tell when they will send; their frames are handed in at the step
/// that reads them, which running ahead does where it commits a boundary, cutting the window where they can finish.
/// Given the same arrivals (a scripted endpoint behind one), running ahead gives the run taking turns gives.
#[test]
fn live_endpoints_give_the_same_run_as_turns() {
    let Some((want, _)) = talos(&How { threads: 1, ..live() }) else { return };
    // The scripted frames went over the bus, at their arrival's slice
    for (t, f) in arrivals() {
        let id = format!("id: {}", f.id);
        assert!(want.frames.iter().any(|l| l.contains(&id)), "{t}: {f:?}");
    }
    let (lockstep, _) = talos(&How { threads: 3, fixed: Some(FIXED.to_vec()), ..live() }).unwrap();
    same(&want, &lockstep, "slice by slice");
    for how in [How { threads: 3, run_ahead: true, ..live() }, How { threads: 0, run_ahead: true, ..live() }] {
        let (got, stats) = talos(&how).unwrap();
        same(&want, &got, &format!("{} threads", how.threads));
        // The scripted arrivals cut windows where they are read, the bridge's as soon as they are waiting
        assert!(stats.live_cuts > 0 && stats.early_cuts > 0, "{stats:?}");
        assert!(stats.slices >= 6900 && stats.windows < stats.slices / 4, "{stats:?}");
    }
    // Different from the same bridge telling when it sends (its frames are ready when it made them)
    let (told, _) = talos(&turns()).unwrap();
    assert_ne!(told.frames, want.frames);
}

/// A frame from a live endpoint lands inside the windows of boards that ran ahead of where it was read: they are
/// rolled back to before it and see it as taking turns does.
#[test]
fn a_host_frame_inside_a_window_rolls_the_boards_past_it_back() {
    let Some((want, _)) = talos(&How { threads: 1, ..live() }) else { return };
    // The ESC boards on the calling thread, which also commits: the light boards on the workers run far ahead of
    // where the commit reads the host's frames
    let fixed = vec![1, 0, 0, 2, 1, 2, 1, 2, 1];
    let (got, stats) = talos(&How { threads: 3, run_ahead: true, fixed: Some(fixed.clone()), ..live() }).unwrap();
    same(&want, &got, "light boards ahead");
    assert!(stats.live_cuts > 0 && stats.rollbacks > 0, "{stats:?}");
    // Every board past the window's end when a host frame is read (stressed): rolled back every time
    for seed in [4, 9] {
        let (got, stats) = talos(&How { threads: 3, run_ahead: true, stress: Some(seed), ..live() }).unwrap();
        same(&want, &got, &format!("seed {seed}"));
        assert!(stats.live_cuts > 0 && stats.rollbacks > 2000, "{stats:?}");
    }
}

/// In real time with a live endpoint, no window runs more than `LIVE_LEAD` (2 ms) ahead of the wall clock, so
/// host frames are read about when they arrive; the run is still the one taking turns gives the same arrivals.
#[test]
fn in_real_time_live_windows_stay_close_to_the_wall_clock() {
    let Some((want, _)) = talos(&How { threads: 1, ends: vec![Time::from_ms(300)], ..live() }) else { return };
    let probe = Arc::new(Mutex::new(Vec::new()));
    let how = How {
        threads: 3,
        run_ahead: true,
        realtime: true,
        ends: vec![Time::from_ms(300)],
        probe: Some(probe.clone()),
        ..live()
    };
    let start = std::time::Instant::now();
    let (got, stats) = talos(&how).unwrap();
    let wall = start.elapsed();
    same(&want, &got, "real time");
    assert!(wall >= std::time::Duration::from_millis(295), "{wall:?}: paced");
    assert!(stats.windows > 0 && stats.live_cuts + stats.early_cuts > 0, "{stats:?}");
    // Read at every boundary, in order, never more than the lead (and the slice being stepped) ahead of the wall
    // clock since the run started
    let reads = probe.lock().unwrap();
    assert_eq!(reads.len(), 3001, "the start, then one read per slice");
    assert!(reads.windows(2).all(|w| w[0].0 < w[1].0));
    let (v0, w0) = reads[0];
    let ahead = reads.iter().map(|(v, w)| (*v - v0).as_secs_f64() - (*w - w0).as_secs_f64()).fold(f64::MIN, f64::max);
    assert!(ahead < 0.002 + 0.0001 + 0.001, "{:.2} ms ahead of the wall clock", ahead * 1e3);
}

/// `complete_controller`'s raw thruster frames (CANmore utility channel 0 of ESC boards 2 and 3), every 20 ms.
struct Controller {
    next: Time,
}

impl sim_devices::can::CanEndpoint for Controller {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, sim_devices::can::CanFrame)> {
        let mut out = Vec::new();
        while self.next <= until {
            for client in [2u32, 3] {
                let rpm: Vec<u8> = [1200i16, -800, 0, 400].iter().flat_map(|r| r.to_be_bytes()).collect();
                out.push((
                    self.next,
                    sim_devices::can::CanFrame::classic((client << 6) | (1 << 5) | (1 << 4), false, &rpm),
                ));
            }
            self.next += Time::from_ms(20);
        }
        out
    }
    fn receive(&mut self, _frame: &sim_devices::can::CanFrame, _at: Time) {}
}

/// `robots/talos_nereus.toml` as nereus drives it, with every host side behind a live endpoint (titan_bridge for
/// each board, the controller's thruster frames) and the external vehicle's pose posted every 10 ms between runs,
/// as the dashboard does for the nereus bridge.
fn nereus(how: &How) -> Option<(Run, sim_host::runahead::Stats)> {
    for elf in ["esc_board", "poac_board", "camera_cage_bb"] {
        common::titan_elf(elf)?;
    }
    let mut r = Robot::load(&common::repo().join("robots/talos_nereus.toml")).unwrap();
    assert!(r.vehicles[0].external);
    r.threads = how.threads;
    r.run_ahead = how.run_ahead;
    r.run_ahead_stress = how.stress;
    r.buses[0].bus.record = Some(Vec::new());
    let mut bridges = Vec::new();
    for client in 1..=4 {
        let b = MockBridge::new(client, true, Time::ZERO);
        b.set_kill(Some(false));
        r.buses[0].endpoints.push(Box::new(Live(b.clone())));
        bridges.push(b);
    }
    // (Off the 10 ms grid of the runs, so that its frames arrive inside windows)
    r.buses[0].endpoints.push(Box::new(Live(Controller { next: Time::from_us(50_330) })));
    let hooks = Arc::new(Mutex::new(Vec::new()));
    for (i, s) in r.boards.iter_mut().enumerate() {
        let h = hooks.clone();
        s.on_log = Some(Box::new(move |t, line| h.lock().unwrap().push(format!("{i} [{t}] {line}"))));
    }
    // As with the dashboard, which records the run for time travel by default
    let inputs = r.inputs.clone();
    let mut tt = how.record.map(|every| {
        let mut tt = TimeTravel::new(&mut r, inputs);
        tt.every = every;
        tt
    });
    while r.now < END {
        // nereus moves it: down and back up, turning
        let x = r.now.as_secs_f64();
        let pose = sim_host::vehicle::ExternalPose {
            position: [x, 0.0, 0.6 + 0.4 * (6.0 * x).sin()],
            attitude: sim_physics::Quat::from_euler(0.0, 0.0, x),
            velocity: None,
            rate: None,
        };
        r.vehicles[0].set_pose(&pose);
        let until = r.now + Time::from_ms(10);
        match &mut tt {
            Some(tt) => assert_eq!(tt.run_until(&mut r, until), Ran::Reached),
            None => assert!(r.run_until(until).is_none()),
        }
    }
    let frames = r.buses[0]
        .bus
        .record
        .as_ref()
        .unwrap()
        .iter()
        .map(|rec| format!("{} {} {:?} {} {:?}", rec.start, rec.end, rec.sender, rec.acked, rec.frame))
        .collect();
    let mut state = Vec::new();
    for (name, s) in r.names.iter().zip(&r.boards) {
        let m = &s.machine;
        let regs: Vec<u32> = (0..m.cores[0].num_regs()).map(|n| m.cores[0].read_reg(n)).collect();
        let mut h = std::collections::hash_map::DefaultHasher::new();
        m.sys.sram.hash(&mut h);
        state.push(format!(
            "{name} {} {} {regs:x?} {:x} {}",
            m.now,
            m.cores[0].instructions(),
            h.finish(),
            m.spin_skipped
        ));
    }
    let v = &r.vehicles[0];
    state.push(format!("{:?} {:?}", v.vehicle.state, v.rpms(&r.boards, r.now)));
    let bridge = bridges.iter().map(|b| format!("{:?}", b.state().received)).collect();
    let hooks = std::mem::take(&mut *hooks.lock().unwrap());
    Some((Run { frames, hooks, state, bridge, coverage: Vec::new() }, r.run_ahead_stats))
}

/// The nereus setup (live host sides, an external vehicle posted between runs) runs ahead, and as taking turns,
/// also recorded for time travel (as the dashboard does by default: its live endpoints are tapped).
/// The camera cage, whose depth input the vehicle writes, need not follow the vehicle slice by slice: an external
/// vehicle stays where it was put during a run.
#[test]
fn the_nereus_setup_runs_ahead_as_it_runs_in_turns() {
    let Some((want, _)) = nereus(&turns()) else { return };
    assert!(want.bridge.contains("CameraCage") && want.frames.len() > 100, "{}", want.bridge);
    let recorded = How { record: Some(Time::from_ms(10)), ..ahead(3) };
    for how in [ahead(3), ahead(2), How { stress: Some(11), ..ahead(3) }, recorded] {
        let (got, stats) = nereus(&how).unwrap();
        same(&want, &got, &format!("{} threads, stress {:?}", how.threads, how.stress));
        assert_eq!(stats.follower_cuts, 0, "{stats:?}");
        assert!(stats.live_cuts + stats.early_cuts > 30 && stats.slices >= 6900, "{stats:?}");
    }
}

/// Trace sinks and profilers see only what the boards finally did: the events and samples of a run that is rolled
/// back are dropped, those of the rerun kept, so they get exactly what taking turns gives them.
#[test]
fn traces_and_profiles_of_boards_run_ahead_are_those_of_turns() {
    let watched = How { watch: true, ..Default::default() };
    let Some((want, _)) = talos(&How { threads: 1, ..watched.clone() }) else { return };
    for how in [
        How { threads: 3, run_ahead: true, fixed: Some(FIXED.to_vec()), ..watched.clone() },
        How { threads: 3, run_ahead: true, stress: Some(21), ..watched.clone() },
    ] {
        let (got, stats) = talos(&how).unwrap();
        same(&want, &got, &format!("stress {:?}", how.stress));
        assert!(stats.slices >= 6900 && stats.windows < stats.slices / 4, "{stats:?}");
    }
}

/// A run with frames from outside arriving whenever they arrive (a host thread on the wall clock, in real time)
/// cannot be repeated by itself; with their arrivals logged (`canlog::Arrivals`) and replayed, it can, to the
/// frame, taking turns or running ahead.
#[test]
fn a_live_run_replays_exactly_from_its_logged_arrivals() {
    if common::titan_elf("esc_board").is_none() {
        return;
    }
    let out = Buffer::default();
    let log = sim_host::canlog::CandumpWriter::new(Box::new(out.clone()));
    let ends = vec![Time::from_ms(300)];
    let fed = How { threads: 3, run_ahead: true, realtime: true, ends: ends.clone(), fed: Some(log.clone()), ..live() };
    let (want, stats) = talos(&fed).unwrap();
    assert!(stats.live_cuts > 0, "{stats:?}");
    log.flush().unwrap();
    let text = String::from_utf8(out.0.lock().unwrap().clone()).unwrap();
    let logged = sim_host::canlog::parse(&text).unwrap();
    // The feeder sends its 40 frames on the wall clock over about 264 ms of the 300 ms run; a machine whose sleeps
    // overshoot (macOS runners) gets fewer in before the end. What matters is that whatever arrived replays exactly.
    assert!((10..=40).contains(&logged.len()), "{} logged:\n{text}", logged.len());
    let replayed = Some(Arc::new(logged));
    for how in [
        How { threads: 1, ends: ends.clone(), replayed: replayed.clone(), ..live() },
        How { threads: 3, run_ahead: true, ends: ends.clone(), stress: Some(2), replayed: replayed.clone(), ..live() },
    ] {
        let (got, _) = talos(&how).unwrap();
        same(&want, &got, &format!("replayed, {} threads", how.threads));
    }
}

/// Recording for time travel takes its checkpoints at boundaries the robot committed (a run is cut into pieces
/// that end where a checkpoint is due, and a window never goes past the end of a run), and the recorded bridge
/// still tells when it will send: the robot runs ahead while recording, and in replays, with the same result as
/// taking turns without recording.
#[test]
fn recording_for_time_travel_runs_ahead_the_same() {
    let Some((want, _)) = talos(&turns()) else { return };
    let record = |how: How| How { record: Some(Time::from_ms(10)), ..how };
    let (got, none) = talos(&record(turns())).unwrap();
    same(&want, &got, "recorded, in turns");
    assert_eq!(none, Default::default());
    for how in [ahead(3), How { stress: Some(4), fixed: Some(FIXED.to_vec()), ..ahead(3) }] {
        let (got, stats) = talos(&record(how.clone())).unwrap();
        same(&want, &got, &format!("recorded, ahead, stress {:?}", how.stress));
        assert!(stats.slices >= 6900 && stats.windows < stats.slices / 6, "{stats:?}");
    }
    // Back to a time and on to the end again: replayed ahead
    let (got, _) = talos(&How { rewind: Some(Time::from_us(237_450)), ..record(ahead(3)) }).unwrap();
    same(&want, &got, "recorded, ahead, replayed");
    let (got, _) = talos(&How { rewind: Some(Time::from_us(237_450)), ..record(turns()) }).unwrap();
    same(&want, &got, "recorded, in turns, replayed");
}

/// Code coverage counts what the committed run executed: what a board ran before it was rolled back does not
/// count, so the coverage is the same running ahead as taking turns (and with rollbacks forced).
#[test]
fn coverage_is_the_same_running_ahead() {
    let end = Time::from_ms(250);
    let cov = |how: How| How { coverage: true, ends: vec![end], ..how };
    let Some((want, _)) = talos(&cov(turns())) else { return };
    assert!(want.coverage.iter().all(Option::is_some));
    for how in [ahead(3), How { stress: Some(9), fixed: Some(FIXED.to_vec()), ..ahead(3) }] {
        let (got, stats) = talos(&cov(how.clone())).unwrap();
        same(&want, &got, &format!("coverage, ahead, stress {:?}", how.stress));
        assert!(stats.windows > 0 && stats.rollbacks > 0, "{stats:?}");
    }
}

/// A recorded robot takes turns while a debugger could stop it (GDB attached, so a `BKPT` stops; a breakpoint set),
/// so that stops are as a replay taking turns gives them; otherwise it runs ahead.
#[test]
fn a_recorded_robot_takes_turns_while_debugged() {
    if common::titan_elf("esc_board").is_none() {
        return;
    }
    let windows = |edit: &dyn Fn(&mut Robot)| {
        let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
        r.threads = 3;
        edit(&mut r);
        let inputs = r.inputs.clone();
        let mut tt = TimeTravel::new(&mut r, inputs);
        assert_eq!(tt.run_until(&mut r, Time::from_ms(20)), Ran::Reached);
        r.run_ahead_stats.windows
    };
    assert!(windows(&|_| {}) > 0);
    assert_eq!(windows(&|r| r.boards[2].machine.bkpt_policy = rp2040::machine::BkptPolicy::Stop), 0);
    assert_eq!(windows(&|r| _ = r.boards[4].machine.breakpoints.insert(0x2004_0000)), 0);
}

/// A robot with live endpoints recorded for time travel runs ahead (the tapped endpoints are live as their
/// endpoints are), and rewound, replays ahead from the recording: both as taking turns gives with the same
/// arrivals.
#[test]
fn recorded_live_endpoints_run_ahead_and_replay_the_same() {
    let Some((want, _)) = talos(&How { threads: 1, ..live() }) else { return };
    let record = |how: How| How { record: Some(Time::from_ms(10)), ..how };
    for how in
        [How { threads: 3, run_ahead: true, ..live() }, How { stress: Some(6), threads: 3, run_ahead: true, ..live() }]
    {
        let (got, stats) = talos(&record(how.clone())).unwrap();
        same(&want, &got, &format!("recorded, stress {:?}", how.stress));
        assert!(stats.live_cuts + stats.early_cuts > 0 && stats.windows < stats.slices / 4, "{stats:?}");
    }
    let (got, _) =
        talos(&How { rewind: Some(Time::from_us(237_450)), ..record(How { threads: 3, run_ahead: true, ..live() }) })
            .unwrap();
    same(&want, &got, "recorded, replayed ahead");
}
