use core::fmt;

/// Errors from atom record parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Buffer shorter than the fixed atom record header.
    TooShort,
    /// Declared record length is inconsistent with the buffer or below header size.
    InvalidLength,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => write!(f, "buffer too short for atom record header"),
            Self::InvalidLength => write!(f, "invalid atom record length field"),
        }
    }
}
