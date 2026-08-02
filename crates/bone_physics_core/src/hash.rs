use blake3::Hash;

/// Stable 256-bit identity of all inputs that affect a solver world.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct WorldFingerprint(pub [u8; 32]);

impl WorldFingerprint {
    #[must_use]
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    #[must_use]
    pub const fn bytes(self) -> [u8; 32] {
        self.0
    }

    #[must_use]
    pub fn hash_canonical(bytes: &[u8]) -> Self {
        let hash: Hash = blake3::hash(bytes);
        Self(*hash.as_bytes())
    }
}
