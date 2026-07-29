use super::triangle::MeshContactGeometry;
use crate::SolverError;
use crate::error::{validate_finite, validate_nonnegative};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshManifoldLimits {
    pub max_contacts: usize,
    pub position_threshold: f32,
    pub normal_min_dot: f32,
}

impl MeshManifoldLimits {
    pub fn try_new(
        max_contacts: usize,
        position_threshold: f32,
        normal_min_dot: f32,
    ) -> Result<Self, SolverError> {
        let limits = Self {
            max_contacts,
            position_threshold,
            normal_min_dot,
        };
        limits.validate()?;
        Ok(limits)
    }

    fn validate(self) -> Result<(), SolverError> {
        if !(1..=4).contains(&self.max_contacts) {
            return Err(SolverError::invalid_value(
                "mesh_manifold_max_contacts",
                self.max_contacts as f64,
            ));
        }
        validate_nonnegative("mesh_manifold_position_threshold", self.position_threshold)?;
        validate_finite("mesh_manifold_normal_min_dot", self.normal_min_dot)?;
        if !(-1.0..=1.0).contains(&self.normal_min_dot) {
            return Err(SolverError::invalid_value(
                "mesh_manifold_normal_min_dot",
                self.normal_min_dot,
            ));
        }
        Ok(())
    }
}

pub fn cluster_mesh_contacts(
    mut raw: Vec<MeshContactGeometry>,
    limits: MeshManifoldLimits,
) -> Result<Vec<MeshContactGeometry>, SolverError> {
    limits.validate()?;
    for contact in &raw {
        contact.validate()?;
    }
    raw.sort_by_key(|contact| contact.key);
    let threshold_squared = limits.position_threshold * limits.position_threshold;
    let penetration_hysteresis = (limits.position_threshold * 1.0e-3).max(1.0e-6);
    let mut clustered: Vec<MeshContactGeometry> = Vec::new();
    for contact in raw {
        let matching = clustered.iter().position(|representative| {
            representative
                .geometry
                .point_b
                .distance_squared(contact.geometry.point_b)
                <= threshold_squared
                && representative.geometry.normal.dot(contact.geometry.normal)
                    >= limits.normal_min_dot
        });
        if let Some(index) = matching {
            let representative = &mut clustered[index];
            if contact.geometry.penetration
                > representative.geometry.penetration + penetration_hysteresis
            {
                *representative = contact;
            }
        } else {
            clustered.push(contact);
        }
    }
    let mut selected = Vec::with_capacity(limits.max_contacts.min(clustered.len()));
    while !clustered.is_empty() && selected.len() < limits.max_contacts {
        let mut best_index = 0;
        for candidate_index in 1..clustered.len() {
            if clustered[candidate_index].geometry.penetration
                > clustered[best_index].geometry.penetration + penetration_hysteresis
            {
                best_index = candidate_index;
            }
        }
        selected.push(clustered.remove(best_index));
    }
    selected.sort_by_key(|contact| contact.key);
    Ok(selected)
}
