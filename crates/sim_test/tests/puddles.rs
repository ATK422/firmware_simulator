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

/// The relay to a real host socket, as `titan_bridge` would use it: a UDP socket on loopback plays the bridge.
#[test]
fn puddles_backplane_through_a_host_udp_socket() {
    let Some(elf) = sim_test::titan_rs("puddles_backplane") else { return };
    let mut b = Bench::builder(&elf).board("mk2/puddles_backplane").boot().unwrap();
    let host = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    host.set_nonblocking(true).unwrap();
    let relay = sim_host::ethernet::UdpRelay::bind("127.0.0.1:0".parse().unwrap(), host.local_addr().unwrap()).unwrap();
    let relay_addr = relay.local_addr().unwrap();
    b.session.eth_endpoints.push(Box::new(relay));
    let mut buf = [0u8; 512];
    let mut got = Vec::new();
    for i in 0..30 {
        // A ping every 100 ms, as titan_bridge sends them
        let mut d = b"TL".to_vec();
        let mut enc = [0u8; 64];
        d.extend_from_slice(
            sim_test::titan_link::encode(
                &HostToDevice::Ping { protocol_version: sim_test::titan_link::PROTOCOL_VERSION },
                &mut enc,
            )
            .unwrap(),
        );
        host.send_to(&d, relay_addr).unwrap();
        b.run_until(Time::from_ms(100 * (i + 1))).unwrap();
        while let Ok((n, _)) = host.recv_from(&mut buf) {
            if let Some(Ok(m)) = buf[..n].strip_prefix(b"TL").map(sim_test::titan_link::decode::<DeviceToHost>) {
                got.push(m);
            }
        }
    }
    assert!(b.logged("ROS connected"));
    assert!(got.iter().any(|m| matches!(m, DeviceToHost::FirmwareStatus(_))), "{got:?}");
    assert!(got.iter().filter(|m| matches!(m, DeviceToHost::Heartbeat)).count() >= 10);
}
