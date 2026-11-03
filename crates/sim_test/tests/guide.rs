//! Keeps `docs/testing.md` honest: its walkthrough (`examples/walkthrough.rs`) runs as a test, and every code block
//! the guide marks as taken from that file must still be in it.

#[path = "../examples/walkthrough.rs"]
mod walkthrough;

#[test]
fn the_walkthrough_runs() {
    walkthrough::main();
}

/// Lines with their indentation removed, blank lines dropped.
fn lines(text: &str) -> Vec<&str> {
    text.lines().map(str::trim).filter(|l| !l.is_empty()).collect()
}

#[test]
fn the_guide_quotes_the_walkthrough() {
    let repo = sim_test::repo();
    let guide = std::fs::read_to_string(repo.join("docs/testing.md")).unwrap();
    let source = std::fs::read_to_string(repo.join("crates/sim_test/examples/walkthrough.rs")).unwrap();
    let source = lines(&source);
    let marker = "<!-- from crates/sim_test/examples/walkthrough.rs -->";
    let mut quoted = 0;
    for part in guide.split(marker).skip(1) {
        let block = part.trim_start().strip_prefix("```rust\n").expect("a ```rust block after the marker");
        let block = &block[..block.find("```").expect("the end of the block")];
        let want = lines(block);
        assert!(
            source.windows(want.len()).any(|w| w == want.as_slice()),
            "docs/testing.md quotes code that is not in examples/walkthrough.rs:\n{block}"
        );
        quoted += 1;
    }
    assert!(quoted >= 5, "only {quoted} quoted blocks found");
}
