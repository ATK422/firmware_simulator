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
    decoder: Box<dyn StreamDecoder + Send + Sync>,
}

impl DefmtDecoder {
    /// Builds a decoder from an ELF with a `.defmt` section; `None` if it has none.
    pub fn from_elf(elf: &[u8]) -> anyhow_free::Result<Option<Self>> {
        let table = Table::parse(elf).map_err(|e| anyhow_free::Error(e.to_string()))?;
        Ok(table.map(|t| {
            // The stream decoder borrows the table; one table lives for the whole run
            let table: &'static Table = Box::leak(Box::new(t));
            DefmtDecoder { decoder: table.new_stream_decoder() }
        }))
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
                Err(DecodeError::Malformed) => {
                    log::warn!("malformed defmt frame");
                    continue;
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
