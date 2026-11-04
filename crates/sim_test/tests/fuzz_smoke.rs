//! A short, deterministic fuzz smoke test, so the harness does not rot: boot one board, run a fixed number of
//! seeded cases on two threads, and check it ran, found the same thing twice, and that any finding's reproducer
//! replays. It skips (and passes) when the image is not built, like the other suites.

use sim_test::Time;
use sim_test::fuzz::{self, Campaign, Config};

fn cfg() -> Config {
    Config { seed: 1, window: Time::from_ms(120), settle: Time::from_ms(700), max_ops: 12, ..Config::default() }
}

#[test]
fn fuzz_smoke_runs_and_is_deterministic() {
    let Some(target) = fuzz::target("power_board") else { return };
    if sim_test::titan_rs(&target.image).is_none() {
        eprintln!("skipping: {} not built", target.image);
        return;
    }
    let run = || {
        let c = Campaign::boot(&target, &cfg()).unwrap();
        let r = fuzz::run_campaign(&c, 80, 2, &|| false);
        let mut keys: Vec<String> = r.hits.iter().flat_map(|h| h.findings.iter().map(|f| f.key.clone())).collect();
        keys.sort();
        (r.cases, r.edges > 0, keys, c)
    };
    let (cases, saw_edges, keys_a, campaign) = run();
    assert_eq!(cases, 80, "every case ran");
    assert!(saw_edges, "a clean boot reaches some edges");
    let (_, _, keys_b, _) = run();
    assert_eq!(keys_a, keys_b, "the same seed finds the same things on two threads");

    // Every reproducer this seed produces replays through the scenario runner
    for hit in fuzz::run_campaign(&campaign, 80, 2, &|| false).hits {
        let f = &hit.findings[0];
        let yaml = fuzz::repro::scenario_yaml(
            &target,
            campaign.shared().snap_at(),
            campaign.shared().window(),
            campaign.shared().settle(),
            &hit.input,
            f,
            campaign.params().fd,
        );
        let scenarios = sim_test::scenario::parse_str(&yaml, std::path::Path::new("fuzz_smoke.yaml")).unwrap();
        let opts = sim_test::scenario::Options::default();
        let outcomes = sim_test::scenario::run_all(&scenarios, &opts, 1);
        assert!(
            outcomes.iter().all(|o| o.passed()),
            "reproducer for {} did not replay:\n{yaml}\n{:#?}",
            f.key,
            outcomes
        );
        // And the harness itself still reproduces it (authoritative)
        assert!(fuzz::repro::reproduces(&campaign, &hit.input, &f.key));
    }
}
