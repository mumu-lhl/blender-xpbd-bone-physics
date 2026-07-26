use std::collections::BTreeSet;

use crate::{BodyId, ConstraintId, ConstraintKind, PersistentConstraint, SolverError};

const CLASS_ORDER: [ConstraintClass; 5] = [
    ConstraintClass::Anchor,
    ConstraintClass::Recovery,
    ConstraintClass::Structural,
    ConstraintClass::AngularLimit,
    ConstraintClass::Contact,
];

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ConstraintClass {
    Anchor,
    Recovery,
    Structural,
    AngularLimit,
    Contact,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstraintColor {
    constraint_ids: Vec<ConstraintId>,
}

impl ConstraintColor {
    #[must_use]
    pub fn constraint_ids(&self) -> &[ConstraintId] {
        &self.constraint_ids
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ConstraintSchedule {
    anchor: Vec<ConstraintColor>,
    recovery: Vec<ConstraintColor>,
    structural: Vec<ConstraintColor>,
    angular_limit: Vec<ConstraintColor>,
    contact: Vec<ConstraintColor>,
}

impl ConstraintSchedule {
    pub fn build(constraints: &[PersistentConstraint]) -> Result<Self, SolverError> {
        let mut seen_ids = BTreeSet::new();
        for constraint in constraints {
            if !seen_ids.insert(constraint.id) {
                return Err(SolverError::DuplicateId {
                    kind: "constraint",
                    id: constraint.id.get(),
                });
            }
        }

        let mut sorted = constraints.iter().collect::<Vec<_>>();
        sorted.sort_by_key(|constraint| (classify(constraint), constraint.id));

        let mut schedule = Self::default();
        for class in CLASS_ORDER {
            *schedule.colors_mut(class) = color_constraints(
                sorted
                    .iter()
                    .copied()
                    .filter(|constraint| classify(constraint) == class),
            );
        }
        Ok(schedule)
    }

    #[must_use]
    pub fn colors(&self, class: ConstraintClass) -> &[ConstraintColor] {
        match class {
            ConstraintClass::Anchor => &self.anchor,
            ConstraintClass::Recovery => &self.recovery,
            ConstraintClass::Structural => &self.structural,
            ConstraintClass::AngularLimit => &self.angular_limit,
            ConstraintClass::Contact => &self.contact,
        }
    }

    #[must_use]
    pub fn iteration_order(&self, iteration: u32) -> Vec<(ConstraintClass, ConstraintId)> {
        let mut result = Vec::new();
        for class in CLASS_ORDER {
            let colors = self.colors(class);
            if iteration.is_multiple_of(2) {
                for color in colors {
                    result.extend(color.constraint_ids.iter().copied().map(|id| (class, id)));
                }
            } else {
                for color in colors.iter().rev() {
                    result.extend(
                        color
                            .constraint_ids
                            .iter()
                            .rev()
                            .copied()
                            .map(|id| (class, id)),
                    );
                }
            }
        }
        result
    }

    fn colors_mut(&mut self, class: ConstraintClass) -> &mut Vec<ConstraintColor> {
        match class {
            ConstraintClass::Anchor => &mut self.anchor,
            ConstraintClass::Recovery => &mut self.recovery,
            ConstraintClass::Structural => &mut self.structural,
            ConstraintClass::AngularLimit => &mut self.angular_limit,
            ConstraintClass::Contact => &mut self.contact,
        }
    }
}

fn classify(constraint: &PersistentConstraint) -> ConstraintClass {
    match &constraint.kind {
        ConstraintKind::Anchor { .. } => ConstraintClass::Anchor,
        ConstraintKind::Attachment { .. } | ConstraintKind::OrientationRecovery => {
            ConstraintClass::Recovery
        }
        ConstraintKind::Distance { .. } | ConstraintKind::DistanceRange { .. } => {
            ConstraintClass::Structural
        }
        ConstraintKind::AngularLimit { .. } => ConstraintClass::AngularLimit,
    }
}

fn color_constraints<'a>(
    constraints: impl Iterator<Item = &'a PersistentConstraint>,
) -> Vec<ConstraintColor> {
    let mut colors: Vec<(ConstraintColor, BTreeSet<BodyId>)> = Vec::new();
    for constraint in constraints {
        if let Some((color, bodies)) = colors
            .iter_mut()
            .find(|(_, bodies)| !writes_body(bodies, constraint))
        {
            color.constraint_ids.push(constraint.id);
            insert_bodies(bodies, constraint);
        } else {
            let mut bodies = BTreeSet::new();
            insert_bodies(&mut bodies, constraint);
            colors.push((
                ConstraintColor {
                    constraint_ids: vec![constraint.id],
                },
                bodies,
            ));
        }
    }
    colors.into_iter().map(|(color, _)| color).collect()
}

fn writes_body(bodies: &BTreeSet<BodyId>, constraint: &PersistentConstraint) -> bool {
    bodies.contains(&constraint.body_a)
        || constraint
            .body_b
            .is_some_and(|body_b| bodies.contains(&body_b))
}

fn insert_bodies(bodies: &mut BTreeSet<BodyId>, constraint: &PersistentConstraint) {
    bodies.insert(constraint.body_a);
    if let Some(body_b) = constraint.body_b {
        bodies.insert(body_b);
    }
}
