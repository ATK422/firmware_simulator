//! Just enough HTTP/1.1 to talk to a dashboard (`sim_host::dashboard`): one request per connection, JSON bodies.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use serde_json::Value;

/// A dashboard's address from what a user would paste: `http://127.0.0.1:8080/`, `127.0.0.1:8080`,
/// `localhost:8080`.
pub fn parse_addr(url: &str) -> Result<SocketAddr, String> {
    let s = url.trim();
    let s = s.strip_prefix("http://").unwrap_or(s);
    if s.starts_with("https://") {
        return Err("the dashboard serves plain http".into());
    }
    let host = s.split('/').next().unwrap_or("");
    host.to_socket_addrs().map_err(|e| format!("{url:?}: {e}"))?.next().ok_or_else(|| format!("{url:?}: no address"))
}

/// Sends a request and returns the status code and the body as JSON.
pub fn request(addr: SocketAddr, method: &str, path: &str, body: Option<&Value>) -> Result<(u16, Value), String> {
    let mut s = TcpStream::connect_timeout(&addr, Duration::from_secs(3))
        .map_err(|e| format!("cannot reach the dashboard at {addr}: {e}"))?;
    // A POST waits up to 10 s for the run loop
    s.set_read_timeout(Some(Duration::from_secs(20))).map_err(|e| e.to_string())?;
    let body = body.map(Value::to_string).unwrap_or_default();
    write!(
        s,
        "{method} {path} HTTP/1.1\r\nHost: {addr}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    )
    .map_err(|e| format!("writing to {addr}: {e}"))?;
    let mut out = Vec::new();
    s.read_to_end(&mut out).map_err(|e| format!("reading from {addr}: {e}"))?;
    parse_response(&out)
}

pub fn get(addr: SocketAddr, path: &str) -> Result<(u16, Value), String> {
    request(addr, "GET", path, None)
}

pub fn post(addr: SocketAddr, path: &str, body: &Value) -> Result<(u16, Value), String> {
    request(addr, "POST", path, Some(body))
}

fn parse_response(raw: &[u8]) -> Result<(u16, Value), String> {
    let at = raw.windows(4).position(|w| w == b"\r\n\r\n").ok_or("the dashboard's answer was cut short")?;
    let head = String::from_utf8_lossy(&raw[..at]);
    let code = head
        .split_whitespace()
        .nth(1)
        .and_then(|c| c.parse().ok())
        .ok_or_else(|| format!("not an HTTP answer: {head:?}"))?;
    let body = &raw[at + 4..];
    let v = serde_json::from_slice(body)
        .map_err(|e| format!("the dashboard's answer is not JSON ({e}): {}", String::from_utf8_lossy(body)))?;
    Ok((code, v))
}

/// Percent-encodes a query parameter value.
pub fn encode(s: &str) -> String {
    s.bytes()
        .map(|b| match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => (b as char).to_string(),
            _ => format!("%{b:02X}"),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addresses() {
        let a: SocketAddr = "127.0.0.1:8080".parse().unwrap();
        assert_eq!(parse_addr("http://127.0.0.1:8080/").unwrap(), a);
        assert_eq!(parse_addr("127.0.0.1:8080").unwrap(), a);
        assert_eq!(parse_addr(" http://127.0.0.1:8080/api/state ").unwrap(), a);
        assert!(parse_addr("localhost:8080").unwrap().ip().is_loopback());
        assert!(parse_addr("https://127.0.0.1:8080").is_err());
        assert!(parse_addr("127.0.0.1").is_err(), "no port");
    }

    #[test]
    fn responses() {
        let r = b"HTTP/1.1 422 Unprocessable Entity\r\nContent-Length: 14\r\n\r\n{\"ok\": false}\n";
        assert_eq!(parse_response(r).unwrap(), (422, serde_json::json!({"ok": false})));
        assert!(parse_response(b"HTTP/1.1 200 OK\r\n").is_err());
        assert!(parse_response(b"HTTP/1.1 200 OK\r\n\r\n<html>").is_err());
    }

    #[test]
    fn encoding() {
        assert_eq!(encode("camera cage/1&x"), "camera%20cage%2F1%26x");
        assert_eq!(encode("esc0"), "esc0");
    }

    #[test]
    fn talks_to_a_real_dashboard() {
        let d = sim_host::dashboard::Dashboard::start("127.0.0.1:0", vec![], vec![]).unwrap();
        assert_eq!(get(d.addr, "/api/state").unwrap(), (200, serde_json::json!({})));
        assert_eq!(get(d.addr, "/nope").unwrap().0, 404);
        let (code, v) = post(d.addr, "/api/fault", &serde_json::json!({"fault": "warp"})).unwrap();
        assert_eq!(code, 400, "{v}");
    }
}
