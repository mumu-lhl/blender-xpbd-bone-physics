use core::cmp::Ordering;

use glam::Vec3;

use super::capsule::{PairKey, usable_normal};
use super::shape::CapsuleCollider;
use crate::error::{validate_finite, validate_nonnegative};
use crate::pose::validate_vec3;
use crate::{ColliderId, Pose, SolverError};

const GEOMETRY_EPSILON_SQUARED: f32 = 1.0e-12;
const PARALLEL_AXIS_DOT_THRESHOLD: f32 = 0.999_999;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AnalyticShape {
    Sphere {
        radius: f32,
    },
    Capsule {
        local_a: Vec3,
        local_b: Vec3,
        radius: f32,
    },
    RoundedBox {
        half_extents: Vec3,
        radius: f32,
    },
}

impl AnalyticShape {
    fn validate(self) -> Result<(), SolverError> {
        match self {
            Self::Sphere { radius } => validate_nonnegative("analytic_sphere_radius", radius),
            Self::Capsule {
                local_a,
                local_b,
                radius,
            } => {
                validate_vec3("analytic_capsule_local_a", local_a)?;
                validate_vec3("analytic_capsule_local_b", local_b)?;
                validate_nonnegative("analytic_capsule_radius", radius)
            }
            Self::RoundedBox {
                half_extents,
                radius,
            } => {
                validate_nonnegative("analytic_rounded_box_half_extents", half_extents.x)?;
                validate_nonnegative("analytic_rounded_box_half_extents", half_extents.y)?;
                validate_nonnegative("analytic_rounded_box_half_extents", half_extents.z)?;
                validate_nonnegative("analytic_rounded_box_radius", radius)
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnalyticProxy {
    pub id: ColliderId,
    pub pose: Pose,
    pub linear_velocity: Vec3,
    pub shape: AnalyticShape,
    pub collision_enabled: bool,
}

impl AnalyticProxy {
    pub(crate) fn validate(self) -> Result<(), SolverError> {
        self.pose.validate()?;
        validate_vec3("analytic_proxy_linear_velocity", self.linear_velocity)?;
        self.shape.validate()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnalyticContact {
    pub point_capsule: Vec3,
    pub point_proxy: Vec3,
    pub normal: Vec3,
    pub separation: f32,
    pub speculative: bool,
}

pub fn capsule_analytic_contact(
    capsule: &CapsuleCollider,
    proxy: &AnalyticProxy,
    dt: f32,
) -> Result<Option<AnalyticContact>, SolverError> {
    validate_finite("dt", dt)?;
    if dt <= 0.0 {
        return Err(SolverError::invalid_value("dt", dt));
    }

    capsule.validate()?;
    let capsule_radius = capsule.shape.effective_radius()?;
    proxy.validate()?;
    let pair = PairKey::ordered(capsule.id, proxy.id)?;

    if !proxy.collision_enabled {
        return Ok(None);
    }

    let (capsule_a, capsule_b) = ordered_segment(capsule.world_segment()?);
    let relative_velocity = proxy.linear_velocity - capsule.linear_velocity;
    validate_vec3("analytic_relative_linear_velocity", relative_velocity)?;
    let contact = match proxy.shape {
        AnalyticShape::Sphere { radius } => rounded_segment_contact(
            capsule_a,
            capsule_b,
            proxy.pose.translation,
            proxy.pose.translation,
            capsule_radius,
            radius,
            pair,
            relative_velocity,
        )?,
        AnalyticShape::Capsule {
            local_a,
            local_b,
            radius,
        } => {
            let proxy_segment = ordered_segment((
                proxy.pose.translation + proxy.pose.rotation * local_a,
                proxy.pose.translation + proxy.pose.rotation * local_b,
            ));
            validate_vec3("analytic_capsule_world_a", proxy_segment.0)?;
            validate_vec3("analytic_capsule_world_b", proxy_segment.1)?;
            rounded_segment_contact(
                capsule_a,
                capsule_b,
                proxy_segment.0,
                proxy_segment.1,
                capsule_radius,
                radius,
                pair,
                relative_velocity,
            )?
        }
        AnalyticShape::RoundedBox {
            half_extents,
            radius,
        } => rounded_box_contact(
            capsule_a,
            capsule_b,
            capsule_radius,
            proxy.pose,
            half_extents,
            radius,
            pair,
            relative_velocity,
        )?,
    };

    select_current_or_speculative(contact, relative_velocity, dt)
}

#[allow(clippy::too_many_arguments)]
fn rounded_segment_contact(
    capsule_a: Vec3,
    capsule_b: Vec3,
    proxy_a: Vec3,
    proxy_b: Vec3,
    capsule_radius: f32,
    proxy_radius: f32,
    pair: PairKey,
    relative_velocity: Vec3,
) -> Result<AnalyticContact, SolverError> {
    let radius_sum = checked_radius_sum(capsule_radius, proxy_radius)?;
    let (center_capsule, center_proxy) =
        closest_segment_points(capsule_a, capsule_b, proxy_a, proxy_b);
    let delta = center_proxy - center_capsule;
    let distance_squared = delta.length_squared();
    validate_nonnegative("analytic_center_distance_squared", distance_squared)?;
    let normal = if distance_squared > GEOMETRY_EPSILON_SQUARED {
        delta / distance_squared.sqrt()
    } else {
        fallback_normal(
            pair,
            relative_velocity,
            capsule_b - capsule_a,
            proxy_b - proxy_a,
        )
    };
    make_contact(
        center_capsule,
        center_proxy,
        normal,
        capsule_radius,
        proxy_radius,
        radius_sum,
        false,
    )
}

#[allow(clippy::too_many_arguments)]
fn rounded_box_contact(
    capsule_a: Vec3,
    capsule_b: Vec3,
    capsule_radius: f32,
    box_pose: Pose,
    half_extents: Vec3,
    box_radius: f32,
    pair: PairKey,
    relative_velocity: Vec3,
) -> Result<AnalyticContact, SolverError> {
    let radius_sum = checked_radius_sum(capsule_radius, box_radius)?;
    let inverse_rotation = box_pose.rotation.conjugate();
    let local_a = inverse_rotation * (capsule_a - box_pose.translation);
    let local_b = inverse_rotation * (capsule_b - box_pose.translation);
    validate_vec3("analytic_rounded_box_local_capsule_a", local_a)?;
    validate_vec3("analytic_rounded_box_local_capsule_b", local_b)?;
    let (local_a, local_b) = ordered_segment((local_a, local_b));
    let (local_capsule, local_box) = closest_segment_aabb(local_a, local_b, half_extents);
    let local_delta = local_box - local_capsule;
    let distance_squared = local_delta.length_squared();
    validate_nonnegative("analytic_center_distance_squared", distance_squared)?;

    if distance_squared > GEOMETRY_EPSILON_SQUARED {
        let local_normal = local_delta / distance_squared.sqrt();
        let normal = normalized_or_fallback(
            box_pose.rotation * local_normal,
            pair,
            relative_velocity,
            capsule_b - capsule_a,
            Vec3::ZERO,
        );
        let center_capsule = box_pose.translation + box_pose.rotation * local_capsule;
        let center_proxy = box_pose.translation + box_pose.rotation * local_box;
        return make_contact(
            center_capsule,
            center_proxy,
            normal,
            capsule_radius,
            box_radius,
            radius_sum,
            false,
        );
    }

    // A zero center distance means the segment intersects the core AABB.
    // Choose the shortest separating translation from the complete
    // segment/AABB SAT intervals rather than from an arbitrary intersection
    // point. The canonical contact normal is opposite the capsule's selected
    // translation direction.
    let (local_capsule_support, local_surface, local_normal) =
        segment_aabb_minimum_translation(local_a, local_b, half_extents, radius_sum, pair)?;
    let normal = normalized_or_fallback(
        box_pose.rotation * local_normal,
        pair,
        relative_velocity,
        capsule_b - capsule_a,
        Vec3::ZERO,
    );
    let center_capsule = box_pose.translation + box_pose.rotation * local_capsule_support;
    let core_surface = box_pose.translation + box_pose.rotation * local_surface;
    let point_capsule = center_capsule + normal * capsule_radius;
    let point_proxy = core_surface - normal * box_radius;
    let separation = (point_proxy - point_capsule).dot(normal);
    validated_contact(point_capsule, point_proxy, normal, separation, false)
}

fn make_contact(
    center_capsule: Vec3,
    center_proxy: Vec3,
    normal: Vec3,
    capsule_radius: f32,
    proxy_radius: f32,
    radius_sum: f32,
    speculative: bool,
) -> Result<AnalyticContact, SolverError> {
    let point_capsule = center_capsule + normal * capsule_radius;
    let point_proxy = center_proxy - normal * proxy_radius;
    let separation = (center_proxy - center_capsule).dot(normal) - radius_sum;
    validated_contact(point_capsule, point_proxy, normal, separation, speculative)
}

fn validated_contact(
    point_capsule: Vec3,
    point_proxy: Vec3,
    normal: Vec3,
    separation: f32,
    speculative: bool,
) -> Result<AnalyticContact, SolverError> {
    validate_vec3("analytic_contact_point_capsule", point_capsule)?;
    validate_vec3("analytic_contact_point_proxy", point_proxy)?;
    validate_vec3("analytic_contact_normal", normal)?;
    let normal_length = normal.length();
    validate_finite("analytic_contact_normal_length", normal_length)?;
    if (normal_length - 1.0).abs() > 1.0e-4 {
        return Err(SolverError::invalid_value(
            "analytic_contact_normal_length",
            normal_length,
        ));
    }
    validate_finite("analytic_contact_separation", separation)?;
    Ok(AnalyticContact {
        point_capsule,
        point_proxy,
        normal,
        separation,
        speculative,
    })
}

fn select_current_or_speculative(
    mut contact: AnalyticContact,
    relative_velocity: Vec3,
    dt: f32,
) -> Result<Option<AnalyticContact>, SolverError> {
    if contact.separation <= 0.0 {
        return Ok(Some(contact));
    }
    let closing_speed = -relative_velocity.dot(contact.normal);
    validate_finite("analytic_closing_speed", closing_speed)?;
    if closing_speed > 0.0 && closing_speed * dt >= contact.separation {
        contact.speculative = true;
        Ok(Some(contact))
    } else {
        Ok(None)
    }
}

fn checked_radius_sum(a: f32, b: f32) -> Result<f32, SolverError> {
    let sum = a + b;
    validate_nonnegative("analytic_radius_sum", sum)?;
    Ok(sum)
}

fn normalized_or_fallback(
    candidate: Vec3,
    pair: PairKey,
    relative_velocity: Vec3,
    axis_capsule: Vec3,
    axis_proxy: Vec3,
) -> Vec3 {
    usable_normal(candidate)
        .unwrap_or_else(|| fallback_normal(pair, relative_velocity, axis_capsule, axis_proxy))
}

fn fallback_normal(
    pair: PairKey,
    relative_velocity: Vec3,
    axis_capsule: Vec3,
    axis_proxy: Vec3,
) -> Vec3 {
    if let Some(normal) = usable_normal(relative_velocity) {
        return normal;
    }
    if let Some(normal) = usable_normal(axis_capsule.cross(axis_proxy)) {
        return normal;
    }

    let selector = pair_selector(pair);
    match selector % 3 {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

fn pair_selector(pair: PairKey) -> u32 {
    pair.first().get().wrapping_mul(31) ^ pair.second().get()
}

fn ordered_segment((a, b): (Vec3, Vec3)) -> (Vec3, Vec3) {
    if compare_vec3(a, b) == Ordering::Greater {
        (b, a)
    } else {
        (a, b)
    }
}

fn compare_vec3(a: Vec3, b: Vec3) -> Ordering {
    a.x.total_cmp(&b.x)
        .then_with(|| a.y.total_cmp(&b.y))
        .then_with(|| a.z.total_cmp(&b.z))
}

fn closest_segment_aabb(start: Vec3, end: Vec3, half_extents: Vec3) -> (Vec3, Vec3) {
    let (start, end) = ordered_segment((start, end));
    let direction = end - start;
    let mut breaks = vec![0.0_f32, 1.0];
    let start_values = start.to_array();
    let direction_values = direction.to_array();
    let extents = half_extents.to_array();
    for axis in 0..3 {
        if direction_values[axis].abs() <= GEOMETRY_EPSILON_SQUARED.sqrt() {
            continue;
        }
        for plane in [-extents[axis], extents[axis]] {
            let t = (plane - start_values[axis]) / direction_values[axis];
            if t > 0.0 && t < 1.0 {
                breaks.push(t);
            }
        }
    }
    breaks.sort_by(f32::total_cmp);
    breaks.dedup_by(|a, b| a.to_bits() == b.to_bits());

    let mut best = segment_aabb_candidate(start, direction, half_extents, 0.0);
    for &t in &breaks[1..] {
        update_segment_aabb_best(
            &mut best,
            segment_aabb_candidate(start, direction, half_extents, t),
        );
    }
    for interval in breaks.windows(2) {
        let low = interval[0];
        let high = interval[1];
        let midpoint = (low + high) * 0.5;
        let point = start + direction * midpoint;
        let point_values = point.to_array();
        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for axis in 0..3 {
            let bound = if point_values[axis] < -extents[axis] {
                -extents[axis]
            } else if point_values[axis] > extents[axis] {
                extents[axis]
            } else {
                continue;
            };
            numerator += direction_values[axis] * (start_values[axis] - bound);
            denominator += direction_values[axis] * direction_values[axis];
        }
        if denominator > GEOMETRY_EPSILON_SQUARED {
            let t = (-numerator / denominator).clamp(low, high);
            update_segment_aabb_best(
                &mut best,
                segment_aabb_candidate(start, direction, half_extents, t),
            );
        }
    }
    (best.1, best.2)
}

fn segment_aabb_candidate(
    start: Vec3,
    direction: Vec3,
    half_extents: Vec3,
    t: f32,
) -> (f32, Vec3, Vec3) {
    let point = start + direction * t;
    let closest = point.clamp(-half_extents, half_extents);
    (point.distance_squared(closest), point, closest)
}

fn segment_support(start: Vec3, end: Vec3, direction: Vec3) -> Vec3 {
    if end.dot(direction) > start.dot(direction) {
        end
    } else {
        start
    }
}

fn aabb_support(point: Vec3, half_extents: Vec3, direction: Vec3) -> Vec3 {
    let mut witness = point.clamp(-half_extents, half_extents);
    for axis in 0..3 {
        if direction[axis] > 0.0 {
            witness[axis] = half_extents[axis];
        } else if direction[axis] < 0.0 {
            witness[axis] = -half_extents[axis];
        }
    }
    witness
}

fn segment_aabb_minimum_translation(
    start: Vec3,
    end: Vec3,
    half_extents: Vec3,
    radius_sum: f32,
    pair: PairKey,
) -> Result<(Vec3, Vec3, Vec3), SolverError> {
    let segment_direction = end - start;
    let mut axes = vec![(0_usize, Vec3::X), (1, Vec3::Y), (2, Vec3::Z)];
    for (slot, box_axis) in [(3, Vec3::X), (4, Vec3::Y), (5, Vec3::Z)] {
        let Some(axis) = usable_normal(segment_direction.cross(box_axis)) else {
            continue;
        };
        if axes
            .iter()
            .any(|(_, existing)| axis.dot(*existing).abs() >= PARALLEL_AXIS_DOT_THRESHOLD)
        {
            continue;
        }
        axes.push((slot, axis));
    }

    let selector = pair_selector(pair);
    let start_axis = selector as usize % axes.len();
    let mut best_motion = Vec3::ZERO;
    let mut best_magnitude = f32::INFINITY;
    for offset in 0..axes.len() {
        let (slot, axis) = axes[(start_axis + offset) % axes.len()];
        let projection_start = start.dot(axis);
        let projection_end = end.dot(axis);
        let projection_min = projection_start.min(projection_end);
        let projection_max = projection_start.max(projection_end);
        let box_support = axis.abs().dot(half_extents);
        validate_nonnegative("analytic_mtd_box_support", box_support)?;

        let positive_magnitude = box_support + radius_sum - projection_min;
        let negative_magnitude = box_support + radius_sum + projection_max;
        validate_nonnegative("analytic_mtd_positive", positive_magnitude)?;
        validate_nonnegative("analytic_mtd_negative", negative_magnitude)?;

        let positive_first = (selector & (1 << ((slot + 3) % 32))) == 0;
        let candidates = if positive_first {
            [(axis, positive_magnitude), (-axis, negative_magnitude)]
        } else {
            [(-axis, negative_magnitude), (axis, positive_magnitude)]
        };
        for (motion, magnitude) in candidates {
            if magnitude < best_magnitude {
                best_motion = motion;
                best_magnitude = magnitude;
            }
        }
    }

    let normal = -best_motion;
    let capsule_support = segment_support(start, end, normal);
    let box_support = aabb_support(capsule_support, half_extents, best_motion);
    Ok((capsule_support, box_support, normal))
}

fn update_segment_aabb_best(best: &mut (f32, Vec3, Vec3), candidate: (f32, Vec3, Vec3)) {
    if candidate.0 < best.0 {
        *best = candidate;
    }
}

fn closest_segment_points(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (mut s, t);
    if a <= GEOMETRY_EPSILON_SQUARED && e <= GEOMETRY_EPSILON_SQUARED {
        return (p1, p2);
    }
    if a <= GEOMETRY_EPSILON_SQUARED {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= GEOMETRY_EPSILON_SQUARED {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denominator = a * e - b * b;
            s = if denominator.abs() > GEOMETRY_EPSILON_SQUARED {
                ((b * f - c * e) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let projected = b * s + f;
            if projected < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if projected > e {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            } else {
                t = projected / e;
            }
        }
    }
    (p1 + d1 * s, p2 + d2 * t)
}
