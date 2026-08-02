use super::capsule::{capsule_manifold, CapsuleManifold};
use super::shape::CapsuleCollider;
use super::triangle::{capsule_triangle_contact, MeshContactGeometry, MovingTriangle};
use super::ContactGeometry;
use crate::error::validate_finite;
use crate::{Pose, SolverError};

const SAMPLE_COUNT: u32 = 64;
const REFINE_ITERATIONS: u32 = 24;

#[derive(Clone, Debug, PartialEq)]
pub enum CcdGeometry {
    Capsule(ContactGeometry),
    Triangle(MeshContactGeometry),
}

#[derive(Clone, Debug, PartialEq)]
pub enum CcdResult {
    Impact {
        toi: f32,
        geometry: CcdGeometry,
    },
    NoImpact,
    Failed {
        fallback: Option<CcdGeometry>,
    },
}

/// Find the earliest contact over one fixed step for two linearly/rotationally
/// moving capsule colliders. The search is deterministic and bounded; the
/// discrete manifold remains the single source of contact geometry.
pub fn capsule_capsule_toi(
    previous_a: &CapsuleCollider,
    current_a: &CapsuleCollider,
    previous_b: &CapsuleCollider,
    current_b: &CapsuleCollider,
    dt: f32,
) -> Result<CcdResult, SolverError> {
    validate_dt(dt)?;
    previous_a.validate()?;
    current_a.validate()?;
    previous_b.validate()?;
    current_b.validate()?;
    if previous_a.id != current_a.id || previous_b.id != current_b.id {
        return Err(SolverError::invalid_value("ccd_collider_id", 0));
    }

    let impact = search_interval(SAMPLE_COUNT, |time| {
        let a = interpolate_collider(previous_a, current_a, time);
        let b = interpolate_collider(previous_b, current_b, time);
        capsule_manifold(&a, &b, None).map(|value| {
            value.map(|manifold| {
                CcdGeometry::Capsule(first_geometry(&manifold))
            })
        })
    })?;
    Ok(impact.unwrap_or(CcdResult::NoImpact))
}

/// Find the earliest contact between a moving capsule and a linearly moving
/// triangle over one fixed step.
pub fn capsule_triangle_toi(
    previous_capsule: &CapsuleCollider,
    current_capsule: &CapsuleCollider,
    triangle: &MovingTriangle,
    dt: f32,
) -> Result<CcdResult, SolverError> {
    validate_dt(dt)?;
    previous_capsule.validate()?;
    current_capsule.validate()?;
    triangle.validate()?;
    if previous_capsule.id != current_capsule.id {
        return Err(SolverError::invalid_value("ccd_collider_id", 0));
    }

    let impact = search_interval(SAMPLE_COUNT, |time| {
        let capsule = interpolate_collider(previous_capsule, current_capsule, time);
        let triangle = interpolate_triangle(triangle, dt, time);
        capsule_triangle_contact(&capsule, &triangle, None)
            .map(|value| value.map(CcdGeometry::Triangle))
    })?;
    Ok(impact.unwrap_or(CcdResult::NoImpact))
}

fn search_interval(
    samples: u32,
    mut contact_at: impl FnMut(f32) -> Result<Option<CcdGeometry>, SolverError>,
) -> Result<Option<CcdResult>, SolverError> {
    let first = contact_at(0.0)?;
    if let Some(geometry) = first {
        return Ok(Some(CcdResult::Impact { toi: 0.0, geometry }));
    }

    let mut lower = 0.0;
    for sample in 1..=samples {
        let upper = sample as f32 / samples as f32;
        if contact_at(upper)?.is_some() {
            let mut impact_time = upper;
            let mut left = lower;
            for _ in 0..REFINE_ITERATIONS {
                let midpoint = (left + impact_time) * 0.5;
                if contact_at(midpoint)?.is_some() {
                    impact_time = midpoint;
                } else {
                    left = midpoint;
                }
            }
            let geometry = contact_at(impact_time)?.ok_or_else(|| {
                SolverError::invalid_value("ccd_refined_geometry", impact_time)
            })?;
            return Ok(Some(CcdResult::Impact {
                toi: impact_time.clamp(0.0, 1.0),
                geometry,
            }));
        }
        lower = upper;
    }
    Ok(None)
}

fn interpolate_collider(
    previous: &CapsuleCollider,
    current: &CapsuleCollider,
    time: f32,
) -> CapsuleCollider {
    let mut collider = current.clone();
    collider.pose = Pose {
        translation: previous
            .pose
            .translation
            .lerp(current.pose.translation, time),
        rotation: previous
            .pose
            .rotation
            .slerp(current.pose.rotation, time)
            .normalize(),
    };
    collider
}

fn interpolate_triangle(triangle: &MovingTriangle, dt: f32, time: f32) -> MovingTriangle {
    let mut result = *triangle;
    for (vertex, velocity) in result.vertices.iter_mut().zip(triangle.velocities) {
        *vertex += velocity * (dt * time);
    }
    result
}

fn first_geometry(manifold: &CapsuleManifold) -> ContactGeometry {
    manifold.points[0]
}

fn validate_dt(dt: f32) -> Result<(), SolverError> {
    validate_finite("ccd_dt", dt)?;
    if dt <= 0.0 {
        return Err(SolverError::invalid_value("ccd_dt", dt));
    }
    Ok(())
}
