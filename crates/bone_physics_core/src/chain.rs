use glam::Vec3;

use crate::error::{validate_finite, validate_nonnegative};
use crate::{
    BodyId, BoneBodyDesc, ConstraintId, ParameterCurve, PersistentConstraint, SolverError,
};

#[derive(Clone, Debug, PartialEq)]
pub struct ChainDesc {
    pub bodies: Vec<BodyId>,
    pub rest_lengths: Vec<f32>,
    pub closed: bool,
}

impl ChainDesc {
    fn validate(&self) -> Result<(), SolverError> {
        if self.bodies.is_empty() || self.bodies.len() != self.rest_lengths.len() {
            return Err(SolverError::invalid_value(
                "chain_length_count",
                self.rest_lengths.len() as f64 - self.bodies.len() as f64,
            ));
        }

        let mut sorted_bodies = self.bodies.clone();
        sorted_bodies.sort_unstable();
        for pair in sorted_bodies.windows(2) {
            if pair[0] == pair[1] {
                return Err(SolverError::DuplicateId {
                    kind: "chain_body",
                    id: pair[0].get(),
                });
            }
        }

        for length in &self.rest_lengths {
            validate_finite("chain_rest_length", *length)?;
            if *length <= 0.0 {
                return Err(SolverError::invalid_value("chain_rest_length", *length));
            }
        }
        Ok(())
    }

    fn total_length(&self) -> Result<f32, SolverError> {
        self.validate()?;
        let total: f32 = self.rest_lengths.iter().sum();
        validate_finite("chain_total_length", total)?;
        if total <= 0.0 {
            return Err(SolverError::invalid_value("chain_total_length", total));
        }
        Ok(total)
    }
}

pub fn sample_chain_s(chain: &ChainDesc) -> Result<Vec<f32>, SolverError> {
    let total = chain.total_length()?;
    let mut result = Vec::with_capacity(chain.rest_lengths.len() + 1);
    let mut accumulated = 0.0;
    result.push(0.0);
    for length in &chain.rest_lengths {
        accumulated += length;
        result.push((accumulated / total).min(1.0));
    }
    Ok(result)
}

#[derive(Clone, Debug)]
pub struct BodyLookup {
    bodies: Vec<BoneBodyDesc>,
}

impl BodyLookup {
    pub fn try_new(mut bodies: Vec<BoneBodyDesc>) -> Result<Self, SolverError> {
        bodies.sort_by_key(|body| body.id);
        for pair in bodies.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(SolverError::DuplicateId {
                    kind: "body",
                    id: pair[0].id.get(),
                });
            }
        }
        for body in &bodies {
            body.validate()?;
        }
        Ok(Self { bodies })
    }

    fn get(&self, id: BodyId) -> Result<&BoneBodyDesc, SolverError> {
        self.bodies
            .binary_search_by_key(&id, |body| body.id)
            .map(|index| &self.bodies[index])
            .map_err(|_| SolverError::MissingBody(id.get()))
    }
}

#[derive(Clone, Debug)]
pub struct CrossLinkSpec {
    pub first_constraint_id: ConstraintId,
    pub chain_a: ChainDesc,
    pub chain_b: ChainDesc,
    pub samples: Vec<f32>,
    pub diagonals: bool,
    pub compliance_curve: ParameterCurve,
}

#[derive(Clone, Copy, Debug)]
struct AnchorSample {
    body: BodyId,
    local: Vec3,
    world: Vec3,
}

pub fn build_cross_links(
    spec: &CrossLinkSpec,
    bodies: &BodyLookup,
) -> Result<Vec<PersistentConstraint>, SolverError> {
    spec.chain_a.validate()?;
    spec.chain_b.validate()?;
    validate_samples(&spec.samples)?;
    validate_body_references(&spec.chain_a, bodies)?;
    validate_body_references(&spec.chain_b, bodies)?;

    let diagonal_count = if spec.diagonals {
        spec.samples.len().saturating_sub(1) * 2
    } else {
        0
    };
    let link_count =
        spec.samples
            .len()
            .checked_add(diagonal_count)
            .ok_or(SolverError::invalid_value(
                "cross_link_count",
                f64::INFINITY,
            ))?;
    if link_count == 0 {
        return Err(SolverError::invalid_value("cross_link_count", 0.0));
    }
    let final_offset = u32::try_from(link_count - 1)
        .map_err(|_| SolverError::invalid_value("constraint_id", f64::INFINITY))?;
    spec.first_constraint_id
        .get()
        .checked_add(final_offset)
        .ok_or(SolverError::invalid_value("constraint_id", f64::INFINITY))?;

    let anchors_a = sample_anchors(&spec.chain_a, &spec.samples, bodies)?;
    let anchors_b = sample_anchors(&spec.chain_b, &spec.samples, bodies)?;
    let cycle_forming = spec.samples.len() > 1;
    let mut constraints = Vec::with_capacity(link_count);

    for (index, s) in spec.samples.iter().copied().enumerate() {
        push_link(
            &mut constraints,
            spec,
            anchors_a[index],
            anchors_b[index],
            s,
            cycle_forming,
        )?;
    }
    if spec.diagonals {
        for index in 0..spec.samples.len().saturating_sub(1) {
            let sample_s = (spec.samples[index] + spec.samples[index + 1]) * 0.5;
            push_link(
                &mut constraints,
                spec,
                anchors_a[index],
                anchors_b[index + 1],
                sample_s,
                true,
            )?;
            push_link(
                &mut constraints,
                spec,
                anchors_a[index + 1],
                anchors_b[index],
                sample_s,
                true,
            )?;
        }
    }

    Ok(constraints)
}

fn validate_samples(samples: &[f32]) -> Result<(), SolverError> {
    if samples.is_empty() {
        return Err(SolverError::invalid_value("cross_link_sample_count", 0.0));
    }
    for sample in samples {
        validate_finite("cross_link_sample", *sample)?;
        if !(0.0..=1.0).contains(sample) {
            return Err(SolverError::invalid_value("cross_link_sample", *sample));
        }
    }
    for pair in samples.windows(2) {
        if pair[0] >= pair[1] {
            return Err(SolverError::invalid_value(
                "cross_link_sample_order",
                pair[1] - pair[0],
            ));
        }
    }
    Ok(())
}

fn validate_body_references(chain: &ChainDesc, bodies: &BodyLookup) -> Result<(), SolverError> {
    for body in &chain.bodies {
        bodies.get(*body)?;
    }
    Ok(())
}

fn sample_anchors(
    chain: &ChainDesc,
    samples: &[f32],
    bodies: &BodyLookup,
) -> Result<Vec<AnchorSample>, SolverError> {
    let total_length = chain.total_length()?;
    samples
        .iter()
        .map(|sample| anchor_at(chain, *sample, total_length, bodies))
        .collect()
}

fn anchor_at(
    chain: &ChainDesc,
    s: f32,
    total_length: f32,
    bodies: &BodyLookup,
) -> Result<AnchorSample, SolverError> {
    let target = s * total_length;
    let mut start = 0.0;
    let last_index = chain.bodies.len() - 1;
    for (index, length) in chain.rest_lengths.iter().copied().enumerate() {
        let end = start + length;
        if target < end || index == last_index {
            let fraction = ((target - start) / length).clamp(0.0, 1.0);
            let body_id = chain.bodies[index];
            let body = bodies.get(body_id)?;
            let local = body.local_head.lerp(body.local_tail, fraction);
            let world = body.rest_pose.translation + body.rest_pose.rotation * local;
            return Ok(AnchorSample {
                body: body_id,
                local,
                world,
            });
        }
        start = end;
    }
    Err(SolverError::invalid_value("cross_link_sample", s))
}

fn push_link(
    constraints: &mut Vec<PersistentConstraint>,
    spec: &CrossLinkSpec,
    anchor_a: AnchorSample,
    anchor_b: AnchorSample,
    sample_s: f32,
    cycle_forming: bool,
) -> Result<(), SolverError> {
    let compliance = spec.compliance_curve.sample(sample_s)?;
    validate_nonnegative("cross_link_compliance", compliance)?;
    if cycle_forming && compliance == 0.0 {
        return Err(SolverError::invalid_value(
            "cyclic_cross_link_compliance",
            compliance,
        ));
    }

    let offset = u32::try_from(constraints.len())
        .map_err(|_| SolverError::invalid_value("constraint_id", f64::INFINITY))?;
    let raw_id = spec
        .first_constraint_id
        .get()
        .checked_add(offset)
        .ok_or(SolverError::invalid_value("constraint_id", f64::INFINITY))?;
    constraints.push(PersistentConstraint::distance(
        ConstraintId::new(raw_id),
        anchor_a.body,
        anchor_b.body,
        anchor_a.local,
        anchor_b.local,
        anchor_a.world.distance(anchor_b.world),
        compliance,
        0.0,
    ));
    Ok(())
}
