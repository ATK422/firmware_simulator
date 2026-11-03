//! M7: a whole robot in one process. Talos's boards share the internal CAN FD bus, a smart battery sits on the
//! external classic bus; a mock bridge per client sees every board connect and report.

use sim_test::{RobotBench, Time};

#[test]
fn talos_boards_share_their_buses() {
    for elf in ["poac_board", "esc_board", "camera_cage_bb", "actuator_board", "led_board", "sbc_board", "ivc_board2"] {
        if sim_test::titan_rs(elf).is_none() {
            return;
        }
    }
    let clients = [("internal", 1), ("internal", 2), ("internal", 3), ("internal", 4), ("internal", 5)];
    let more = [("internal", 6), ("internal", 7), ("internal", 8), ("external", 21)];
    let all: Vec<(&str, u8)> = clients.iter().chain(&more).copied().collect();
    // One bridge per client: RobotBench takes one per bus, so add the rest by hand
    let mut r = RobotBench::load("talos.toml", &all[..1], |spec| {
        for b in &mut spec.bus {
            b.record = true;
        }
    })
    .unwrap();
    let mut bridges = vec![r.bridges[0].clone().unwrap()];
    for (bus, client) in &all[1..] {
        let j = r.robot.bus_names.iter().position(|n| n == bus).unwrap();
        let fd = r.robot.buses[j].bus.rates.data.is_some();
        let b = sim_test::MockBridge::new(*client, fd, Time::ZERO);
        b.set_kill(Some(false));
        r.robot.buses[j].endpoints.push(Box::new(b.clone()));
        bridges.push(b);
    }
    let wall = std::time::Instant::now();
    r.run_until(Time::from_ms(2000));
    eprintln!("9 boards, 2 s virtual: {:?} wall", wall.elapsed());
    let expect = [
        "talos/poac_board",
        "mk2/esc_board",
        "mk2/esc_board",
        "mk2/camera_cage_bb",
        "mk2/actuator_board",
        "talos/led_board",
        "talos/sbc_board",
        "talos/ivc_board",
        "mk2/sbh_mcu",
    ];
    for ((bridge, (bus, client)), name) in bridges.iter().zip(&all).zip(expect) {
        let st = bridge.statuses();
        let last = st.last().unwrap_or_else(|| panic!("client {client} on {bus}: no status"));
        assert_eq!(last.1.board_name.as_str(), name, "client {client}");
        assert_eq!(last.1.client_id, *client);
        assert!(bridge.state().errors.is_empty(), "client {client}: {:?}", bridge.state().errors);
    }
    // Every board's CANmore heartbeat is on its bus, and frames are acknowledged by the other nodes
    let internal = r.robot.buses[0].bus.record.as_ref().unwrap();
    let mut heartbeat_clients: Vec<u8> = internal
        .iter()
        .filter_map(|rec| {
            let f = canmore::id::Fields::decode(rec.frame.id, rec.frame.extended);
            (f.frame_type == canmore::id::FrameType::Util && f.noc == canmore::CHAN_HEARTBEAT).then_some(f.client_id)
        })
        .collect();
    heartbeat_clients.sort();
    heartbeat_clients.dedup();
    assert_eq!(heartbeat_clients, [1, 2, 3, 4, 5, 6, 7, 8]);
    assert!(internal.iter().all(|rec| rec.acked));
}
