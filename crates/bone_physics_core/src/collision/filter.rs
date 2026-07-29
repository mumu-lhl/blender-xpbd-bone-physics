use std::collections::BTreeSet;

use crate::{BodyId, ConstraintId};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollisionFilter {
    pub layer: u32,
    pub mask: u32,
    pub excluded_bodies: BTreeSet<BodyId>,
    pub excluded_constraints: BTreeSet<ConstraintId>,
}

impl Default for CollisionFilter {
    fn default() -> Self {
        Self {
            layer: 1,
            mask: u32::MAX,
            excluded_bodies: BTreeSet::new(),
            excluded_constraints: BTreeSet::new(),
        }
    }
}
