//! M7: the Puddles backplane on Ethernet. Its W5500 (MACRAW, under embassy-net) talks UDP to a scripted
//! `titan_bridge` on the robot computer's address: ARP, the titan_link datagrams on port 2203 both ways, and the
//! CANmore heartbeat broadcast on port 2201.

use sim_host::ethernet::PuddlesBridge;
use sim_test::titan_link::puddles_backplane as pb;
use sim_test::titan_link::{DeviceToHost, HostToDevice};
use sim_test::{Bench, Time};

fn backplane() -> Option<(Bench, PuddlesBridge)> {
    let elf = sim_test::titan_rs("puddles_backplane")?;
    let mut b = Bench::builder(&elf).board("mk2/puddles_backplane").boot().unwrap();
    let bridge = PuddlesBridge::new(Time::ZERO);
    bridge.set_kill(Some(false));
    b.session.eth_endpoints.push(Box::new(bridge.clone()));
    Some((b, bridge))
}

#[test]
fn puddles_backplane_connects_over_udp() {
    let Some((mut b, bridge)) = backplane() else { return };
    b.wait_for_log("Ethernet up as 192.168.1.42", Time::from_secs(1)).unwrap();
    b.wait_for_log("ROS connected", Time::from_secs(2)).unwrap();
    b.run_until(Time::from_ms(3000)).unwrap();
    let s = bridge.state();
    assert!(s.errors.is_empty(), "{:?}", s.errors);
    let st: Vec<_> = s
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::FirmwareStatus(f) => Some((*t, f.clone())),
            _ => None,
        })
        .collect();
    assert!(st.len() >= 2, "{st:?}");
    assert_eq!(st.last().unwrap().1.board_name.as_str(), "mk2/puddles_backplane");
    assert_eq!(st.last().unwrap().1.faults, 0);
    // CANmore heartbeats broadcast every 500 ms
    assert!(s.heartbeats.len() >= 4, "{:?}", s.heartbeats);
    let wire = b.session.machine.sys.ethernet.as_ref().unwrap().lock().unwrap();
    assert_eq!(wire.dropped, 0);
}

#[test]
fn puddles_thrusters_follow_udp_commands() {
    let Some((mut b, bridge)) = backplane() else { return };
    // Replies sent before the firmware sees the PHY link are dropped, like micro-ROS writes with the cable out
    b.wait_for_log("Ethernet link up", Time::from_secs(2)).unwrap();
    // Thruster commands are only applied 5 s after boot, when the ESCs have woken up (`WAKEUP_DELAY_US`)
    b.run_until(Time::from_ms(5100)).unwrap();
    let cmd = [100, -200, 0, 300, 0, 0, 0, 400];
    // Commands keep coming like from the controller, every 20 ms
    for _ in 0..50 {
        bridge.send(HostToDevice::PuddlesBackplane(pb::HostToDevice::Dshot(cmd)));
        b.run_for(Time::from_ms(20));
    }
    let rpm: Vec<pb::RpmFeedback> = bridge
        .state()
        .received
        .iter()
        .filter_map(|(_, m)| match m {
            DeviceToHost::PuddlesBackplane(pb::DeviceToHost::Rpm(r)) => Some(*r),
            _ => None,
        })
        .collect();
    assert!(rpm.len() >= 40, "an answer to (almost) every command: {}", rpm.len());
    let last = rpm.last().unwrap();
    eprintln!("{last:?}");
    assert_eq!(last.valid_mask, 0xFF, "every thruster answers with eRPM");
    // Thruster 8 runs forwards, 2 backwards, 3 stands still (signed mechanical RPM)
    assert!(last.rpm[7] > 0 && last.rpm[1] < 0 && last.rpm[2] == 0, "{last:?}");
    assert!(last.rpm[7].abs() > last.rpm[0].abs());
}
