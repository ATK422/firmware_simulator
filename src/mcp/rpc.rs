//! JSON-RPC 2.0 framing for the MCP stdio transport: one message per line, no batches.

use serde_json::{Value, json};

pub const PARSE_ERROR: i64 = -32700;
pub const INVALID_REQUEST: i64 = -32600;
pub const METHOD_NOT_FOUND: i64 = -32601;
pub const INVALID_PARAMS: i64 = -32602;

/// A message read from the client.
#[derive(Debug, PartialEq)]
pub enum Incoming {
    /// A request, to be answered with its `id`.
    Request { id: Value, method: String, params: Value },
    /// A notification: no answer.
    Notification { method: String, params: Value },
    /// An answer to a request of ours (we send none), ignored.
    Response,
    /// Not a valid message: answer with this error (`id` null when it could not be read).
    Invalid { id: Value, code: i64, message: String },
}

/// Reads one line of the transport.
pub fn parse(line: &str) -> Incoming {
    let v: Value = match serde_json::from_str(line) {
        Ok(v) => v,
        Err(e) => {
            return Incoming::Invalid { id: Value::Null, code: PARSE_ERROR, message: format!("parse error: {e}") };
        }
    };
    let Value::Object(m) = v else {
        let message = if v.is_array() { "batches are not supported" } else { "a message must be a JSON object" };
        return Incoming::Invalid { id: Value::Null, code: INVALID_REQUEST, message: message.into() };
    };
    let id = m.get("id").cloned();
    let id_ok = id.as_ref().is_none_or(|i| i.is_string() || i.is_number());
    let reply_id = id.clone().filter(|_| id_ok).unwrap_or(Value::Null);
    if m.get("jsonrpc").and_then(Value::as_str) != Some("2.0") {
        return Incoming::Invalid { id: reply_id, code: INVALID_REQUEST, message: "jsonrpc must be \"2.0\"".into() };
    }
    if !id_ok {
        return Incoming::Invalid {
            id: Value::Null,
            code: INVALID_REQUEST,
            message: "id must be a string or number".into(),
        };
    }
    let params = m.get("params").cloned().unwrap_or(Value::Null);
    if !(params.is_null() || params.is_object() || params.is_array()) {
        return Incoming::Invalid { id: reply_id, code: INVALID_REQUEST, message: "params must be an object".into() };
    }
    match (m.get("method"), id) {
        (Some(Value::String(method)), Some(id)) => Incoming::Request { id, method: method.clone(), params },
        (Some(Value::String(method)), None) => Incoming::Notification { method: method.clone(), params },
        (Some(_), _) => {
            Incoming::Invalid { id: reply_id, code: INVALID_REQUEST, message: "method must be a string".into() }
        }
        (None, Some(_)) if m.contains_key("result") || m.contains_key("error") => Incoming::Response,
        (None, _) => Incoming::Invalid { id: reply_id, code: INVALID_REQUEST, message: "no method".into() },
    }
}

pub fn result(id: &Value, result: Value) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "result": result})
}

pub fn error(id: &Value, code: i64, message: &str) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}})
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_notifications_and_responses() {
        assert_eq!(
            parse(r#"{"jsonrpc":"2.0","id":1,"method":"tools/list"}"#),
            Incoming::Request { id: json!(1), method: "tools/list".into(), params: Value::Null }
        );
        assert_eq!(
            parse(r#"{"jsonrpc":"2.0","id":"a","method":"tools/call","params":{"name":"x"}}"#),
            Incoming::Request { id: json!("a"), method: "tools/call".into(), params: json!({"name": "x"}) }
        );
        assert_eq!(
            parse(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#),
            Incoming::Notification { method: "notifications/initialized".into(), params: Value::Null }
        );
        assert_eq!(parse(r#"{"jsonrpc":"2.0","id":3,"result":{}}"#), Incoming::Response);
    }

    #[test]
    fn malformed_messages_get_errors() {
        let code = |line: &str| match parse(line) {
            Incoming::Invalid { code, id, .. } => (code, id),
            other => panic!("{line}: {other:?}"),
        };
        assert_eq!(code("{not json"), (PARSE_ERROR, Value::Null));
        assert_eq!(code(r#"[{"jsonrpc":"2.0","id":1,"method":"ping"}]"#), (INVALID_REQUEST, Value::Null));
        assert_eq!(code("42"), (INVALID_REQUEST, Value::Null));
        assert_eq!(code(r#"{"id":7,"method":"ping"}"#), (INVALID_REQUEST, json!(7)), "jsonrpc missing");
        assert_eq!(code(r#"{"jsonrpc":"1.0","id":7,"method":"ping"}"#), (INVALID_REQUEST, json!(7)));
        assert_eq!(code(r#"{"jsonrpc":"2.0","id":{},"method":"ping"}"#), (INVALID_REQUEST, Value::Null));
        assert_eq!(code(r#"{"jsonrpc":"2.0","id":2,"method":5}"#), (INVALID_REQUEST, json!(2)));
        assert_eq!(code(r#"{"jsonrpc":"2.0","id":2,"method":"x","params":3}"#), (INVALID_REQUEST, json!(2)));
        assert_eq!(code(r#"{"jsonrpc":"2.0","id":2}"#), (INVALID_REQUEST, json!(2)));
    }

    #[test]
    fn answers() {
        assert_eq!(result(&json!(1), json!({})), json!({"jsonrpc": "2.0", "id": 1, "result": {}}));
        let e = error(&json!("x"), METHOD_NOT_FOUND, "no such method");
        assert_eq!(e["error"]["code"], -32601);
        assert_eq!(e["id"], "x");
    }
}
