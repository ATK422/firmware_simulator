//! Boards in an acoustic medium run on threads, ahead of each other as far as the water lets them
//! (`sim_host::acoustic::Lookahead`): the run must be exactly the one taking turns gives, sound included. Compared
//! here: every CAN frame, every log and UART line with its time and its order across boards, what the bridges
//! received, every ADC sample the listening board's hydrophone took, and the machines' state.

mod common;

use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};

use sim_acoustics::medium::{LivePath, Trajectory};
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::{Robot, RobotSpec};
use titan_link::{DeviceToHost, HostToDevice, ivc2};

/// How a run is done.
#[derive(Clone, Copy, Default)]
struct How {
    threads: usize,
    /// Off: on threads slice by slice.
    run_ahead: bool,
    /// `Robot::run_ahead_stress`: boards rolled back all the time.
    stress: Option<u64>,
    /// Every hydrophone computes every tap, silent origins too (`Hydrophone::skip_quiet` off).
    every_tap: bool,
}

const TURNS: How = How { threads: 1, run_ahead: true, stress: None, every_tap: false };
const AHEAD: How = How { threads: 2, run_ahead: true, stress: None, every_tap: false };

#[derive(Debug, PartialEq)]
struct Run {
    frames: Vec<String>,
    hooks: Vec<String>,
    state: Vec<String>,
    bridges: Vec<String>,
    /// The listening board's hydrophone: every ADC sample (time, volts).
    heard: Vec<(Time, u32)>,
}

/// Loads a robot file from `robots/`, changed by `edit`, with a bridge for client 8 on each bus.
fn load(file: &str, edit: impl FnOnce(&mut RobotSpec)) -> (Robot, Vec<MockBridge>) {
    let path = common::repo().join("robots").join(file);
    let mut spec: RobotSpec = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    edit(&mut spec);
    let mut r = Robot::from_spec(&spec, path.parent().unwrap()).unwrap();
    let mut bridges = Vec::new();
    for b in r.buses.iter_mut() {
        b.bus.record = Some(Vec::new());
        let br = MockBridge::new(8, true, Time::ZERO);
        br.set_kill(Some(false));
        b.endpoints.push(Box::new(br.clone()));
        bridges.push(br);
    }
    (r, bridges)
}

/// Runs `r` as `how` says in `pieces` (`run_until` ends), Talos's board sending a byte from `send` on, and
/// collects everything observable.
fn run(mut r: Robot, bridges: &[MockBridge], how: How, send: Option<Time>, pieces: &[Time]) -> (Run, Robot) {
    r.threads = how.threads;
    r.run_ahead = how.run_ahead;
    r.run_ahead_stress = how.stress;
    for b in &mut r.boards {
        for h in sim_host::acoustic::hydrophones(&mut b.machine) {
            h.skip_quiet = !how.every_tap;
        }
    }
    let hooks = Arc::new(Mutex::new(Vec::new()));
    for (i, s) in r.boards.iter_mut().enumerate() {
        let h = hooks.clone();
        s.on_log = Some(Box::new(move |t, line| h.lock().unwrap().push(format!("{i} [{t}] {line}"))));
        let h = hooks.clone();
        s.on_uart_line = Some(Box::new(move |t, n, line| h.lock().unwrap().push(format!("{i} [{t}] uart{n} {line}"))));
    }
    sim_host::acoustic::hydrophones(&mut r.boards[1].machine).next().unwrap().recording = Some(Vec::new());
    let mut sent = false;
    for &end in pieces {
        if let Some(at) = send.filter(|&at| !sent && at <= r.now) {
            assert!(r.boards[0].logged("ROS connected"), "by {at}");
            bridges[0].send(HostToDevice::Ivc2(ivc2::HostToDevice::EnqueueData(0xA5)));
            sent = true;
        }
        assert!(r.run_until(end).is_none());
        assert_eq!(r.now, end);
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
            let at = m.core_time(c);
            state.push(format!("{name} core{c} {at} {} {} {regs:x?}", core.cycles(), core.instructions()));
        }
        let mut h = std::collections::hash_map::DefaultHasher::new();
        m.sys.sram.hash(&mut h);
        state.push(format!("{name} now {} memory {:x}", m.now, h.finish()));
    }
    let edges: Vec<u64> =
        r.medium.as_ref().unwrap().lock().unwrap().projectors.iter().map(|p| p.edge_count()).collect();
    state.push(format!("projector edges {edges:?}"));
    let bridges = bridges.iter().map(|b| format!("{:?}", b.state().received)).collect();
    let heard = sim_host::acoustic::hydrophones(&mut r.boards[1].machine)
        .next()
        .unwrap()
        .recording
        .take()
        .unwrap()
        .into_iter()
        .map(|(t, v)| (t, v.to_bits()))
        .collect();
    let hooks = std::mem::take(&mut *hooks.lock().unwrap());
    (Run { frames, hooks, state, bridges, heard }, r)
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
    if let Some(i) = (0..want.heard.len().max(got.heard.len())).find(|&i| want.heard.get(i) != got.heard.get(i)) {
        panic!("{what}: ADC sample {i} differs: want {:?}, got {:?}", want.heard.get(i), got.heard.get(i));
    }
    assert_eq!(want.bridges, got.bridges, "{what}: what the bridges received");
}

/// The run to 1.4 s, Talos's board transmitting from 0.4 s, in uneven pieces.
const PIECES: [Time; 4] = [Time::from_ms(400), Time::from_ms(731), Time::from_ms(1_000), Time::from_ms(1_400)];

#[test]
fn a_pool_on_threads_gives_the_same_run_as_turns() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let pool = || load("ivc2_pool.toml", |_| {});
    let (r, b) = pool();
    let (want, r) = run(r, &b, TURNS, Some(Time::from_ms(400)), &PIECES);
    assert_eq!(r.run_ahead_stats, Default::default(), "one thread takes turns");
    // The listener heard the packet's tones: its ADC samples follow the other board's projector
    let edges = r.medium.as_ref().unwrap().lock().unwrap().projectors[0].edge_count();
    assert!(edges > 50_000, "{edges} level changes on Talos's projector");
    assert!(want.heard.len() > 100_000, "{} ADC samples", want.heard.len());
    assert!(
        want.hooks.len() > 10 && want.frames.len() > 10,
        "{} lines, {} frames",
        want.hooks.len(),
        want.frames.len()
    );
    for how in [AHEAD, How { threads: 0, ..AHEAD }, How { run_ahead: false, ..AHEAD }, How { every_tap: true, ..TURNS }]
    {
        let (r, b) = pool();
        let (got, r) = run(r, &b, how, Some(Time::from_ms(400)), &PIECES);
        same(&want, &got, &format!("{} threads, ahead {}", how.threads, how.run_ahead));
        let stats = r.run_ahead_stats;
        if how.run_ahead && how.threads != 1 {
            // 5 m of water, 3.4 ms: windows of about 33 slices (fewer where a CAN frame ends one)
            assert!(stats.slices == 14_000 && stats.windows < stats.slices / 16, "{stats:?}");
        } else {
            assert_eq!(stats, Default::default());
        }
    }
    // Boards rolled back all the time: their projectors' drives go back with them
    let (r, b) = pool();
    let (got, r) = run(r, &b, How { stress: Some(7), ..AHEAD }, Some(Time::from_ms(400)), &PIECES);
    same(&want, &got, "stressed");
    assert!(r.run_ahead_stats.rollbacks > 300, "{:?}", r.run_ahead_stats);
}

/// Both vehicles drive towards each other at 1.5 m/s, from 5 m to 0.8 m apart: the windows shrink as the sound has
/// less far to go (and Doppler shifts what each hears), and the run is still that of taking turns.
#[test]
fn closing_vehicles_on_threads_give_the_same_run_as_turns() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let pool = || {
        load("ivc2_pool.toml", |spec| {
            for (b, (x, v)) in spec.board.iter_mut().zip([(5.0, 1.5), (10.0, -1.5)]) {
                b.position_m = None;
                b.trajectory = Some(Trajectory::Linear { start_m: [x, 6.0, 1.5], velocity_mps: [v, 0.0, 0.0] });
            }
        })
    };
    let (r, b) = pool();
    let (want, _) = run(r, &b, TURNS, Some(Time::from_ms(400)), &PIECES);
    for how in [AHEAD, How { stress: Some(3), ..AHEAD }] {
        let (r, b) = pool();
        let (got, r) = run(r, &b, how, Some(Time::from_ms(400)), &PIECES);
        same(&want, &got, &format!("stress {:?}", how.stress));
        assert!(r.run_ahead_stats.windows > 0, "{:?}", r.run_ahead_stats);
    }
}

/// Where the boards could hear each other within a slice (closer than 15 cm of water), or a transducer's path is not
/// known ahead (on a simulated vehicle), they take turns, threads or not.
#[test]
fn boards_that_cannot_run_apart_take_turns() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let windows = |edit: &dyn Fn(&mut Robot)| {
        let (mut r, _) = load("ivc2_pool.toml", |spec| spec.board[1].position_m = Some([5.1, 6.0, 1.5]));
        r.threads = 2;
        edit(&mut r);
        assert!(r.run_until(Time::from_ms(30)).is_none());
        r.run_ahead_stats.windows
    };
    assert_eq!(windows(&|_| {}), 0, "10 cm apart");
    assert!(windows(&|r| r.place(1, Trajectory::Fixed([10.0, 6.0, 1.5]))) > 0, "5 m apart");
    assert_eq!(windows(&|r| r.place(1, Trajectory::Live(LivePath::new([10.0, 6.0, 1.5])))), 0, "a path recorded live");
}

/// The whole exchange of ivc_link.rs's test on threads: the byte Talos sends reaches the surface vehicle's bridge at
/// the same time as taking turns.
#[test]
fn a_byte_crosses_the_pool_on_threads_as_in_turns() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let received = |how: How| {
        let (r, b) = load("ivc2_pool.toml", |_| {});
        let pieces = [Time::from_ms(3_500), Time::from_ms(13_000)];
        let (out, _) = run(r, &b, how, Some(Time::from_ms(3_500)), &pieces);
        let bytes: Vec<(Time, u8)> = b[1]
            .state()
            .received
            .iter()
            .filter_map(|(t, m)| match m {
                DeviceToHost::Ivc2(ivc2::DeviceToHost::RxNew(b)) => Some((*t, *b)),
                _ => None,
            })
            .collect();
        (out, bytes)
    };
    let (want, bytes) = received(TURNS);
    assert_eq!(bytes.iter().map(|b| b.1).collect::<Vec<_>>(), [0xA5]);
    let (got, threaded) = received(AHEAD);
    assert_eq!(bytes, threaded);
    same(&want, &got, "on threads");
}

/// Two C IVCBoard2s (titan_firmware's blake/ivc, through the bootrom and their bootloader), when
/// `TITAN_FIRMWARE_DIR` has a build of it.
#[test]
fn a_pool_of_c_boards_on_threads_gives_the_same_run_as_turns() {
    if sim_host::titan_firmware_image("talos/IVCBoard2/build/IVCBoard2_with_bl.uf2").is_none() {
        return;
    }
    let pieces = [Time::from_ms(613), Time::from_ms(1_500)];
    let (r, b) = load("ivc2_pool_c.toml", |_| {});
    let (want, _) = run(r, &b, TURNS, None, &pieces);
    assert!(want.hooks.len() > 5, "{:?}", want.hooks);
    let (r, b) = load("ivc2_pool_c.toml", |_| {});
    let (got, r) = run(r, &b, AHEAD, None, &pieces);
    same(&want, &got, "C boards on threads");
    assert!(r.run_ahead_stats.windows > 0);
}
