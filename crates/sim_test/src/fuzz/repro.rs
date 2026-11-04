//! Writing a failing case out as a reproducer: a YAML scenario (docs/scenarios.md) that replays the exact traffic
//! and checks for the finding, plus a JSON record of the case. `firmware_simulator scenario reproducer.yaml`
//! replays it.

use serde_yaml::{Mapping, Value};
use sim_core::time::Time;

use super::cases::{FuzzInput, Op, fault_spec};
use super::oracle::{Finding, Kind};
use super::{Campaign, Target};

fn secs(t: Time) -> Value {
    Value::String(format!("{}s", t.as_secs_f64()))
}

fn mapping(pairs: Vec<(&str, Value)>) -> Value {
    let mut m = Mapping::new();
    for (k, v) in pairs {
        m.insert(Value::String(k.to_string()), v);
    }
    Value::Mapping(m)
}

/// A step that performs `op` at its absolute time (snapshot + the op's offset), on the `client`'s bridge.
fn op_step(op: &Op, snap_at: Time, client: u8, fd: bool) -> Value {
    let at = secs(snap_at + super::grid(Time::from_us(op.at_us() as u64)));
    let bridge = Value::Number((client as u64).into());
    match op {
        Op::Raw { id, extended, fd: f, brs, data, .. } => {
            let mut frame = if *f {
                let mut fr = sim_devices::can::CanFrame::fd(*id, *extended, data);
                fr.brs = *brs;
                fr
            } else {
                sim_devices::can::CanFrame::classic(*id, *extended, data)
            };
            // compact() uses the frame's fields; pad FD like the bus does
            if frame.fd {
                let n = sim_devices::can::fd_dlc_to_len(sim_devices::can::fd_len_to_dlc(frame.data.len()));
                frame.data.resize(n, 0);
            }
            mapping(vec![(
                "can_send",
                mapping(vec![
                    ("bridge", bridge),
                    ("frames", Value::Sequence(vec![Value::String(sim_host::canlog::compact(&frame))])),
                    ("at", at),
                ]),
            )])
        }
        Op::Encoded { bytes, .. } => {
            let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
            mapping(vec![("send", mapping(vec![("bridge", bridge), ("bytes", Value::String(hex)), ("at", at)]))])
        }
        Op::Host { msg, .. } => {
            // Through JSON so the externally-tagged enum becomes a plain map (`{Ping: {...}}`), which is what the
            // scenario parser reads, not a serde_yaml `!Ping` tag
            let json = serde_json::to_string(msg).unwrap_or_default();
            let m: Value = serde_yaml::from_str(&json).unwrap_or(Value::Null);
            mapping(vec![("send", mapping(vec![("bridge", bridge), ("message", m), ("at", at)]))])
        }
        Op::Fault { fault, .. } => {
            let _ = fd;
            mapping(vec![("fault", Value::String(fault_spec(fault))), ("at", at)])
        }
    }
}

/// The scenario YAML for `input` against `finding`, replaying the traffic and checking the finding.
pub fn scenario_yaml(
    target: &Target,
    snap_at: Time,
    window: Time,
    settle: Time,
    input: &FuzzInput,
    finding: &Finding,
    fd: bool,
) -> String {
    let end = snap_at + window + settle;
    let board_file =
        std::fs::canonicalize(crate::board_file(&target.board)).unwrap_or_else(|_| crate::board_file(&target.board));
    let image = crate::titan_rs(&target.image)
        .and_then(|p| std::fs::canonicalize(&p).ok())
        .unwrap_or_else(|| target.image.clone().into());

    let mut steps: Vec<Value> = input.ops.iter().map(|op| op_step(op, snap_at, target.client, fd)).collect();
    // Clear the adversarial bus conditions after the window, as the campaign does, so a hang can be told apart
    let clear_at = secs(snap_at + window);
    for spec in ["can-release", "can-plug", "can-flood-stop", "can-error-rate:0", "kill:off"] {
        steps.push(mapping(vec![("fault", Value::String(spec.to_string())), ("at", clear_at.clone())]));
    }
    // The oracle check for this finding
    let within = secs(window + settle);
    let check = match finding.kind {
        Kind::Panic => mapping(vec![(
            "expect",
            mapping(vec![
                ("log", mapping(vec![("regex", Value::String("PANIC|Reset by panic|HardFault".into()))])),
                ("within", within),
            ]),
        )]),
        Kind::WatchdogReset => mapping(vec![(
            "expect",
            mapping(vec![("resets", mapping(vec![("min", Value::Number(1.into()))])), ("within", within)]),
        )]),
        Kind::Lockup => mapping(vec![(
            "expect",
            mapping(vec![("lockups", mapping(vec![("min", Value::Number(1.into()))])), ("within", within)]),
        )]),
        Kind::Hang => {
            mapping(vec![("expect", mapping(vec![("link", Value::String("down".into())), ("at", secs(end))]))])
        }
        Kind::FaultLatched => {
            // Replay and check a status is still received; the exact fault bits are in the JSON record
            mapping(vec![("expect", mapping(vec![("status", Value::String("any".into())), ("within", within)]))])
        }
        Kind::ErrorLog => mapping(vec![(
            "expect",
            mapping(vec![
                (
                    "log",
                    mapping(vec![("regex", Value::String(super::oracle::gist(&finding.detail).replace('#', ".")))]),
                ),
                ("level", Value::String("error".into())),
                ("within", within),
            ]),
        )]),
        Kind::UnmodelledAccess => {
            // The scenario format cannot assert on unmodelled registers; replay the traffic and sanity-check no panic
            mapping(vec![("never", mapping(vec![("log", Value::String("PANIC".into()))]))])
        }
    };
    steps.push(check);

    let doc = mapping(vec![
        ("name", Value::String(format!("firmware_finding fuzz {} / {}", target.image, finding.key))),
        (
            "description",
            Value::String(format!("{}: {} (fuzz reproducer, seed-generated)", finding.kind.word(), finding.detail)),
        ),
        (
            "board",
            mapping(vec![
                ("image", Value::String(image.to_string_lossy().into_owned())),
                ("file", Value::String(board_file.to_string_lossy().into_owned())),
            ]),
        ),
        ("bridges", Value::Sequence(vec![Value::Number((target.client as u64).into())])),
        ("time", secs(end)),
        ("steps", Value::Sequence(steps)),
    ]);
    serde_yaml::to_string(&doc).unwrap_or_default()
}

/// The JSON record of a case: the finding, the board, and the op list (so a case can be re-run without the YAML).
pub fn record_json(target: &Target, input: &FuzzInput, finding: &Finding) -> serde_json::Value {
    serde_json::json!({
        "image": target.image,
        "board": target.board,
        "client": target.client,
        "finding": finding,
        "ops": input.ops,
    })
}

/// Re-runs `input` and says whether it still produces a finding with `key` (the authoritative check: the harness,
/// not the scenario runner). Used to confirm a minimised reproducer.
pub fn reproduces(campaign: &Campaign, input: &FuzzInput, key: &str) -> bool {
    campaign.run_once(input).0.iter().any(|f| f.key == key)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fuzz::cases::Op;
    use sim_host::fault::Fault;

    #[test]
    fn fault_specs_round_trip() {
        for f in [
            Fault::CanStuckDominant(true),
            Fault::CanUnplugged(true),
            Fault::CanCorrupt(5),
            Fault::CanErrorRate(0.3),
            Fault::CanFlood { id: 0x123, extended: false, len: 8, load: 0.5 },
            Fault::KillSwitch(Some(true)),
            Fault::KillSwitch(None),
        ] {
            let spec = fault_spec(&f);
            assert_eq!(spec.parse::<Fault>().unwrap(), f, "{spec}");
        }
    }

    #[test]
    fn a_scenario_parses() {
        let target = Target {
            image: "power_board".into(),
            board: "mk2/power_board".into(),
            client: 1,
            boot_log: "ROS connected".into(),
        };
        let input = FuzzInput {
            ops: vec![
                Op::Raw { at_us: 1000, id: 0x123, extended: false, fd: false, brs: false, data: vec![1, 2, 3] },
                Op::Encoded { at_us: 2000, bytes: vec![0xff, 0x00, 0x7f] },
                Op::Fault { at_us: 3000, fault: Fault::CanStuckDominant(true) },
            ],
        };
        let finding = Finding { kind: Kind::Panic, key: "panic".into(), detail: "PANIC: wild".into(), at: None };
        let yaml =
            scenario_yaml(&target, Time::from_ms(500), Time::from_ms(200), Time::from_ms(800), &input, &finding, false);
        // It parses as a scenario
        let scenarios = crate::scenario::parse_str(&yaml, std::path::Path::new("repro.yaml")).expect("parses");
        assert_eq!(scenarios.len(), 1);
        assert!(yaml.contains("can_send"));
        assert!(yaml.contains("bytes"));
    }
}
