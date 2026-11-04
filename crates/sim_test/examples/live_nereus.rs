//! The nereus setup as it runs on a laptop, without vcan or ROS: `robots/talos_nereus.toml` on threads, its bus
//! behind a live endpoint (as SocketCAN is) whose far side is a "robot computer" thread on the wall clock (a mock
//! titan_bridge for each board and the controller's thruster frames every 20 ms, on a bus of its own, the frames
//! carried both ways as through vcan), and the external vehicle's pose set every 10 ms of virtual time between runs,
//! as the nereus bridge posts it to the dashboard. Reports how the emulator kept up with the wall clock.
//!
//! ```text
//! cargo run --release -p sim_test --example live_nereus -- [--time 10s] [--threads 3] [--no-run-ahead] [--no-realtime]
//! ```

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sim_devices::can::{CanBus, CanEndpoint, CanFrame};
use sim_test::{MockBridge, RobotBench, Time};

type Queue = Arc<Mutex<VecDeque<CanFrame>>>;

/// The emulator's end of the "vcan interface": what the far side wrote, handed in at the step that reads it.
struct Near {
    to_robot: Queue,
    from_robot: Queue,
}

impl CanEndpoint for Near {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.to_robot.lock().unwrap().drain(..).map(|f| (until, f)).collect()
    }
    fn receive(&mut self, frame: &CanFrame, _at: Time) {
        self.from_robot.lock().unwrap().push_back(frame.clone());
    }
    fn live(&self) -> bool {
        true
    }
    fn waiting(&mut self, _until: Time) -> bool {
        !self.to_robot.lock().unwrap().is_empty()
    }
}

/// The far side's end, on the far side's (wall) clock.
struct Far {
    to_robot: Queue,
    from_robot: Queue,
}

impl CanEndpoint for Far {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.from_robot.lock().unwrap().drain(..).map(|f| (until, f)).collect()
    }
    fn receive(&mut self, frame: &CanFrame, _at: Time) {
        self.to_robot.lock().unwrap().push_back(frame.clone());
    }
}

/// `complete_controller`'s raw thruster frames (CANmore utility channel 0 of ESC boards 2 and 3), every 20 ms.
struct Controller {
    next: Time,
}

impl CanEndpoint for Controller {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut out = Vec::new();
        while self.next <= until {
            for client in [2u32, 3] {
                let rpm: Vec<u8> = [1200i16, -800, 0, 400].iter().flat_map(|r| r.to_be_bytes()).collect();
                out.push((self.next, CanFrame::classic((client << 6) | (1 << 5) | (1 << 4), false, &rpm)));
            }
            self.next += Time::from_ms(20);
        }
        out
    }
    fn receive(&mut self, _frame: &CanFrame, _at: Time) {}
}

fn parse_time(s: &str) -> Time {
    let (num, scale) = match s.strip_suffix("ms") {
        Some(n) => (n, 1e-3),
        None => (s.strip_suffix('s').unwrap_or(s), 1.0),
    };
    Time::from_secs_f64(num.parse::<f64>().expect("a time like 10s or 500ms") * scale)
}

fn main() {
    let (mut time, mut threads, mut run_ahead, mut realtime) = (Time::from_secs(10), 3, true, true);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--time" => time = parse_time(&args.next().expect("--time T")),
            "--threads" => threads = args.next().and_then(|n| n.parse().ok()).expect("--threads N"),
            "--no-run-ahead" => run_ahead = false,
            "--no-realtime" => realtime = false,
            other => panic!("unknown argument {other}"),
        }
    }
    let mut r = RobotBench::load("talos_nereus.toml", &[], |_| {}).unwrap_or_else(|e| panic!("{e}"));
    r.robot.threads = threads;
    r.robot.run_ahead = run_ahead;
    r.robot.realtime = realtime;
    let (to_robot, from_robot) = (Queue::default(), Queue::default());
    r.robot.buses[0].endpoints.push(Box::new(Near { to_robot: to_robot.clone(), from_robot: from_robot.clone() }));
    // The robot computer: titan_bridge for each board and the controller, on its own bus and the wall clock
    let rates = r.robot.buses[0].bus.rates;
    let done = Arc::new(AtomicBool::new(false));
    let bridges: Vec<MockBridge> = (1..=4)
        .map(|client| {
            let b = MockBridge::new(client, true, Time::ZERO);
            b.set_kill(Some(false));
            b
        })
        .collect();
    let far = {
        let (done, bridges) = (done.clone(), bridges.clone());
        std::thread::spawn(move || {
            let mut bus = CanBus::new("can0", rates);
            let mut eps: Vec<Box<dyn CanEndpoint>> = bridges.into_iter().map(|b| Box::new(b) as _).collect();
            eps.push(Box::new(Controller { next: Time::from_ms(50) }));
            eps.push(Box::new(Far { to_robot, from_robot }));
            let start = Instant::now();
            while !done.load(Ordering::Relaxed) {
                let now = Time::from_ns(start.elapsed().as_nanos() as u64);
                let mut e: Vec<&mut dyn CanEndpoint> = eps.iter_mut().map(|e| &mut **e as _).collect();
                bus.step(&mut [], &mut e, now);
                std::thread::sleep(Duration::from_micros(500));
            }
        })
    };
    let start = Instant::now();
    let mut worst_behind = Duration::ZERO;
    let mut worst_ahead = Duration::ZERO;
    while r.now() < time {
        // nereus moves the vehicle on the wall clock: down and back up, turning
        let x = start.elapsed().as_secs_f64();
        let pose = sim_host::vehicle::ExternalPose {
            position: [x, 0.0, 0.6 + 0.4 * (6.0 * x).sin()],
            attitude: sim_physics::Quat::from_euler(0.0, 0.0, x),
            velocity: None,
            rate: None,
        };
        r.robot.vehicles[0].set_pose(&pose);
        r.run_until((r.now() + Time::from_ms(10)).min(time));
        let (virt, wall) = (Duration::from_nanos(r.now().as_ns()), start.elapsed());
        worst_behind = worst_behind.max(wall.saturating_sub(virt));
        worst_ahead = worst_ahead.max(virt.saturating_sub(wall));
    }
    let wall = start.elapsed();
    done.store(true, Ordering::Relaxed);
    far.join().unwrap();
    let s = r.robot.run_ahead_stats;
    println!(
        "{} threads{}{}: {time} virtual in {:.2}s wall ({:.2}x), at most {:.1} ms behind / {:.1} ms ahead of the wall \
         clock between runs",
        threads,
        if run_ahead { ", running ahead" } else { ", slice by slice" },
        if realtime { ", real time" } else { "" },
        wall.as_secs_f64(),
        time.as_secs_f64() / wall.as_secs_f64(),
        worst_behind.as_secs_f64() * 1e3,
        worst_ahead.as_secs_f64() * 1e3,
    );
    println!(
        "windows {} ({:.1} slices each), rollbacks {}, cut by host frames {} (as soon as they were waiting {})",
        s.windows,
        s.slices as f64 / s.windows.max(1) as f64,
        s.rollbacks,
        s.live_cuts + s.early_cuts,
        s.early_cuts
    );
    for (b, name) in bridges.iter().zip(["poac", "esc0", "esc1", "camera_cage"]) {
        let st = b.state();
        println!("{name}: {} message(s) to the bridge, {} error(s)", st.received.len(), st.errors.len());
    }
}
