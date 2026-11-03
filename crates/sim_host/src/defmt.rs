//! defmt log decoding from the ELF's `.defmt` table.

use defmt_decoder::{DecodeError, StreamDecoder, Table};

/// One decoded log line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogLine {
    pub level: Option<String>,
    /// The firmware's own timestamp, as formatted by its `defmt::timestamp!`.
    pub timestamp: Option<String>,
    pub message: String,
}

impl std::fmt::Display for LogLine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(t) = &self.timestamp {
            write!(f, "{t} ")?;
        }
        if let Some(l) = &self.level {
            write!(f, "{:<5} ", l.to_uppercase())?;
        }
        f.write_str(&self.message)
    }
}

/// Decodes a defmt byte stream (as read from RTT) into log lines.
pub struct DefmtDecoder {
    table: &'static Table,
    decoder: Box<dyn StreamDecoder + Send + Sync>,
}

impl DefmtDecoder {
    /// Builds a decoder from an ELF with a `.defmt` section; `None` if it has none.
    pub fn from_elf(elf: &[u8]) -> anyhow_free::Result<Option<Self>> {
        let table = Table::parse(elf).map_err(|e| anyhow_free::Error(e.to_string()))?;
        Ok(table.map(|t| {
            // The stream decoder borrows the table; one table lives for the whole run
            let table: &'static Table = Box::leak(Box::new(t));
            DefmtDecoder { table, decoder: table.new_stream_decoder() }
        }))
    }

    /// A decoder for the same table with no partial frame buffered (for a snapshot taken between frames).
    pub fn fresh(&self) -> Self {
        DefmtDecoder { table: self.table, decoder: self.table.new_stream_decoder() }
    }

    /// Feeds bytes and returns every complete frame.
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<LogLine> {
        self.decoder.received(bytes);
        let mut out = Vec::new();
        loop {
            match self.decoder.decode() {
                Ok(frame) => out.push(LogLine {
                    level: frame.level().map(|l| l.as_str().to_string()),
                    timestamp: frame.display_timestamp().map(|t| t.to_string()),
                    message: frame.display_message().to_string(),
                }),
                Err(DecodeError::UnexpectedEof) => break,
                Err(DecodeError::Malformed) if self.table.encoding().can_recover() => {
                    // rzCOBS: the decoder dropped the frame up to its separator
                    log::warn!("malformed defmt frame");
                    continue;
                }
                Err(DecodeError::Malformed) => {
                    // Raw frames have no separator to resynchronise on, and the decoder keeps the bad bytes
                    // buffered: drop them rather than decode the same frame again forever
                    log::warn!("malformed defmt frame in a raw stream; dropping the buffered bytes");
                    self.decoder = self.table.new_stream_decoder();
                    break;
                }
            }
        }
        out
    }
}

/// Minimal error type (the decoder's errors are `anyhow`, which this crate does not otherwise need).
pub mod anyhow_free {
    #[derive(Debug)]
    pub struct Error(pub String);
    impl std::fmt::Display for Error {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.write_str(&self.0)
        }
    }
    impl std::error::Error for Error {}
    pub type Result<T> = std::result::Result<T, Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decoder(encoding: &str) -> DefmtDecoder {
        let json = format!(r#"{{"timestamp":null,"entries":{{}},"bitflags":{{}},"encoding":"{encoding}"}}"#);
        let table: &'static Table = Box::leak(Box::new(serde_json::from_str(&json).unwrap()));
        DefmtDecoder { table, decoder: table.new_stream_decoder() }
    }

    /// A malformed frame in a raw-encoded stream (here an index the table doesn't have, as after RTT dropped
    /// bytes) can't be skipped: the raw decoder keeps it buffered and returns `Malformed` again, so retrying spun
    /// forever and hung the emulator. rzCOBS frames are skipped to the next separator as before.
    #[test]
    fn malformed_frames_do_not_hang_the_decoder() {
        for encoding in ["Raw", "Rzcobs"] {
            let (tx, rx) = std::sync::mpsc::channel();
            std::thread::spawn(move || {
                let mut d = decoder(encoding);
                let lines = d.feed(&[0x05, 0x00, 0x01, 0x02, 0x00]);
                let more = d.feed(&[0x07, 0x00, 0x00]);
                tx.send((lines.len(), more.len())).unwrap();
            });
            let got = rx.recv_timeout(std::time::Duration::from_secs(5));
            assert_eq!(got, Ok((0, 0)), "{encoding}: feed must return");
        }
    }
}
