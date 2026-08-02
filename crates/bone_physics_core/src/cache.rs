use crate::{SolverError, WorldFingerprint};

pub const CACHE_FORMAT_VERSION: u16 = 1;
pub const SOLVER_STATE_VERSION: u16 = 1;
const MAGIC: [u8; 8] = *b"XPBDS001";
const HEADER_BYTES: usize = 80;
const MAX_ITEMS: u32 = 1_000_000;

pub struct Encoder {
    bytes: Vec<u8>,
}

impl Encoder {
    #[must_use]
    pub fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    pub fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    pub fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    pub fn f32(&mut self, value: f32) {
        self.u32(value.to_bits());
    }

    #[must_use]
    pub fn finish(self) -> Vec<u8> {
        self.bytes
    }
}

pub struct Decoder<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Decoder<'a> {
    #[must_use]
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    pub fn u8(&mut self) -> Result<u8, SolverError> {
        let value = *self.bytes.get(self.offset).ok_or_else(truncated)?;
        self.offset += 1;
        Ok(value)
    }

    pub fn u32(&mut self) -> Result<u32, SolverError> {
        let bytes = self.take::<4>()?;
        Ok(u32::from_le_bytes(bytes))
    }

    pub fn f32(&mut self) -> Result<f32, SolverError> {
        let value = f32::from_bits(self.u32()?);
        if value.is_finite() {
            Ok(value)
        } else {
            Err(SolverError::invalid_value("checkpoint_float", value))
        }
    }

    pub fn count(&mut self, field: &'static str) -> Result<usize, SolverError> {
        let value = self.u32()?;
        if value > MAX_ITEMS {
            return Err(SolverError::invalid_value(field, value));
        }
        Ok(value as usize)
    }

    pub fn is_done(&self) -> bool {
        self.offset == self.bytes.len()
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], SolverError> {
        let end = self.offset.checked_add(N).ok_or_else(truncated)?;
        let value = self.bytes.get(self.offset..end).ok_or_else(truncated)?;
        self.offset = end;
        Ok(value.try_into().expect("fixed width"))
    }
}

pub fn wrap_payload(fingerprint: WorldFingerprint, payload: Vec<u8>) -> Vec<u8> {
    let mut result = Vec::with_capacity(HEADER_BYTES + payload.len());
    result.extend_from_slice(&MAGIC);
    result.extend_from_slice(&CACHE_FORMAT_VERSION.to_le_bytes());
    result.extend_from_slice(&SOLVER_STATE_VERSION.to_le_bytes());
    result.extend_from_slice(&fingerprint.bytes());
    result.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    result.extend_from_slice(blake3::hash(&payload).as_bytes());
    result.extend_from_slice(&payload);
    result
}

pub fn unwrap_payload(bytes: &[u8], expected: WorldFingerprint) -> Result<&[u8], SolverError> {
    if bytes.len() < HEADER_BYTES {
        return Err(truncated());
    }
    if bytes[..8] != MAGIC {
        return Err(SolverError::invalid_value("checkpoint_magic", 0));
    }
    let cache_version = u16::from_le_bytes([bytes[8], bytes[9]]);
    let solver_version = u16::from_le_bytes([bytes[10], bytes[11]]);
    if cache_version != CACHE_FORMAT_VERSION || solver_version != SOLVER_STATE_VERSION {
        return Err(SolverError::invalid_value(
            "checkpoint_version",
            u32::from(cache_version),
        ));
    }
    let actual = WorldFingerprint::from_bytes(bytes[12..44].try_into().expect("fixed width"));
    if actual != expected {
        return Err(SolverError::invalid_value("checkpoint_fingerprint", 0));
    }
    let payload_len = u32::from_le_bytes(bytes[44..48].try_into().expect("fixed width")) as usize;
    let end = HEADER_BYTES
        .checked_add(payload_len)
        .ok_or_else(truncated)?;
    if end != bytes.len() {
        return Err(truncated());
    }
    let payload = &bytes[HEADER_BYTES..end];
    if blake3::hash(payload).as_bytes() != &bytes[48..80] {
        return Err(SolverError::invalid_value("checkpoint_checksum", 0));
    }
    Ok(payload)
}

fn truncated() -> SolverError {
    SolverError::invalid_value("checkpoint_truncated", 0)
}
