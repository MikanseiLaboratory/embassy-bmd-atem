//! No-alloc construction of atom command blobs.
//!
//! An ATEM command is a sequence of atom records. The **same bytes** are carried
//! by every transport — UDP/BURP puts them in the payload of a 12-byte packet,
//! USB-C 8.6+ announces them with a `0x04` header and then streams the blob, and
//! the OpenSwitcher TCP proxy wraps them again. Only the framing differs, so a
//! command built here can be sent over any of them unchanged.
//!
//! ```
//! use bmd_atem_atom::{AtomWriter, FourCc};
//!
//! let mut buf = [0u8; 64];
//! let mut w = AtomWriter::new(&mut buf);
//! w.push(FourCc(*b"DCut"), &[0, 0, 0, 0]).unwrap();
//! assert_eq!(w.as_slice(), &[0x00, 0x0c, 0x00, 0x00, b'D', b'C', b'u', b't', 0, 0, 0, 0]);
//! ```

use crate::{FourCc, RawAtomSlice};

/// Builds a contiguous atom blob (the transport-independent command payload).
///
/// The writer never allocates: it hands out a view of the caller's buffer, so
/// the same bytes can be handed to a UDP packet encoder or a USB tunnel writer.
#[derive(Debug)]
pub struct AtomWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> AtomWriter<'a> {
    /// Create a writer over `buf`.
    #[must_use]
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes written so far.
    #[must_use]
    pub fn len(&self) -> usize {
        self.pos
    }

    /// `true` when nothing has been written yet.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }

    /// Remaining capacity.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Append one atom record: `u16 BE total length | 2 pad | FourCC | data`.
    ///
    /// # Errors
    /// [`crate::Error::InvalidLength`] when the record does not fit: the write
    /// is all-or-nothing, so a failed call leaves the blob unchanged (callers
    /// can then decide to flush what fits and retry).
    pub fn push(&mut self, name: FourCc, data: &[u8]) -> Result<(), crate::Error> {
        let total = 8usize.checked_add(data.len()).ok_or(crate::Error::InvalidLength)?;
        let total_u16 = u16::try_from(total).map_err(|_| crate::Error::InvalidLength)?;
        if self.remaining() < total {
            return Err(crate::Error::InvalidLength);
        }
        let start = self.pos;
        let [hi, lo] = total_u16.to_be_bytes();
        self.buf[start] = hi;
        self.buf[start + 1] = lo;
        self.buf[start + 2] = 0;
        self.buf[start + 3] = 0;
        self.buf[start + 4..start + 8].copy_from_slice(&(name.0));
        self.buf[start + 8..start + total].copy_from_slice(data);
        self.pos += total;
        Ok(())
    }

    /// Append a command whose payload is a single `u8` followed by three pad
    /// bytes (`DAut`, `DCut`, `FtbA`, …).
    ///
    /// # Errors
    /// As [`push`](Self::push).
    pub fn push_me(&mut self, name: FourCc, me: u8) -> Result<(), crate::Error> {
        self.push(name, &[me, 0, 0, 0])
    }

    /// Append a command whose payload is `u8 me | pad | u16 BE source`
    /// (`CPgI`, `CPvI`, `CInL`-style routing).
    ///
    /// # Errors
    /// As [`push`](Self::push).
    pub fn push_source(&mut self, name: FourCc, me: u8, source: u16) -> Result<(), crate::Error> {
        let [sh, sl] = source.to_be_bytes();
        self.push(name, &[me, 0, sh, sl])
    }

    /// The atoms written so far, as a slice the transports can carry directly.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    /// Consume the writer and return the written bytes.
    #[must_use]
    pub fn finish(self) -> &'a [u8] {
        let n = self.pos;
        &self.buf[..n]
    }

    /// Iterate the records written so far (sanity checks, logging).
    pub fn iter(&self) -> crate::AtomIter<'_> {
        crate::AtomIter::new(self.as_slice())
    }
}

/// Split a command blob into records without allocating.
#[must_use]
pub fn split_atoms(blob: &[u8]) -> crate::AtomIter<'_> {
    crate::AtomIter::new(blob)
}

/// `true` when `blob` is a sequence of well-formed atom records.
#[must_use]
pub fn is_valid_atom_blob(blob: &[u8]) -> bool {
    let mut rest = blob;
    while !rest.is_empty() {
        if rest.len() < 8 {
            return false;
        }
        let alen = usize::from(u16::from_be_bytes([rest[0], rest[1]]));
        if alen < 8 || rest.len() < alen {
            return false;
        }
        rest = &rest[alen..];
    }
    true
}

/// Iterate the atom records of a blob as borrows.
pub fn atoms(blob: &[u8]) -> impl Iterator<Item = Result<RawAtomSlice<'_>, crate::Error>> {
    crate::AtomIter::new(blob)
}

#[cfg(test)]
mod tests {
    use alloc::string::ToString as _;
    use alloc::vec::Vec;

    use super::*;
    use crate::{
        encode_atom_change_program_input, encode_atom_do_ftb_cut, encode_atom_do_transition_cut,
    };

    #[test]
    fn writer_matches_the_hand_encoded_helpers() {
        let mut buf = [0u8; 64];
        let mut w = AtomWriter::new(&mut buf);
        w.push(FourCc(*b"DCut"), &[0, 0, 0, 0]).unwrap();
        assert_eq!(w.as_slice(), &encode_atom_do_transition_cut(0)[..]);

        let mut w = AtomWriter::new(&mut buf);
        w.push_source(FourCc(*b"CPgI"), 0, 3).unwrap();
        assert_eq!(w.as_slice(), &encode_atom_change_program_input(0, 3)[..]);

        let mut w = AtomWriter::new(&mut buf);
        w.push(FourCc(*b"FCut"), &[0, 1, 0, 0]).unwrap();
        assert_eq!(w.as_slice(), &encode_atom_do_ftb_cut(0, true)[..]);
    }

    #[test]
    fn multiple_atoms_concatenate_and_split_back() {
        let mut buf = [0u8; 64];
        let mut w = AtomWriter::new(&mut buf);
        w.push_me(FourCc(*b"DAut"), 0).unwrap();
        w.push_source(FourCc(*b"CPgI"), 0, 2).unwrap();
        w.push_me(FourCc(*b"DCut"), 1).unwrap();
        assert_eq!(w.len(), 36);

        let names: Vec<_> = w
            .iter()
            .collect::<Result<Vec<_>, _>>()
            .unwrap()
            .iter()
            .map(|a| a.name.display_lossy().to_string())
            .collect();
        assert_eq!(names, alloc::vec!["DAut", "CPgI", "DCut"]);
    }

    #[test]
    fn overflow_is_all_or_nothing() {
        let mut buf = [0u8; 12];
        let mut w = AtomWriter::new(&mut buf);
        w.push_me(FourCc(*b"DAut"), 0).unwrap();
        assert!(!w.is_empty());
        let before = w.len();
        // A second record does not fit.
        assert_eq!(w.push_me(FourCc(*b"DCut"), 0), Err(crate::Error::InvalidLength));
        assert_eq!(w.len(), before);
        assert_eq!(w.as_slice(), &[0x00, 0x0c, 0x00, 0x00, b'D', b'A', b'u', b't', 0, 0, 0, 0]);
    }

    #[test]
    fn blob_validation_and_split_helpers() {
        let good = encode_atom_do_transition_cut(0);
        assert!(is_valid_atom_blob(&good));
        assert!(!is_valid_atom_blob(&[0x00, 0x03, 0, 0, 0, 0, 0, 0]));
        assert!(!is_valid_atom_blob(&[0x00, 0x0c, 0, 0]));
        assert_eq!(split_atoms(&good).count(), 1);
        assert_eq!(atoms(&good).count(), 1);
    }
}
