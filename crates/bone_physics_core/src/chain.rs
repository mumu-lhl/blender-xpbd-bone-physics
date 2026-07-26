use std::collections::{BTreeMap, BTreeSet};

use glam::Vec3;

use crate::error::{validate_finite, validate_nonnegative};
use crate::pose::validate_vec3;
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
        if self.closed && self.bodies.len() < 2 {
            return Err(SolverError::invalid_value(
                "closed_chain_body_count",
                self.bodies.len() as f64,
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
    for (index, length) in chain.rest_lengths.iter().copied().enumerate() {
        accumulated += length;
        validate_finite("chain_accumulated_length", accumulated)?;
        let boundary = if index + 1 == chain.rest_lengths.len() {
            1.0
        } else {
            accumulated / total
        };
        validate_finite("chain_normalized_boundary", boundary)?;
        if boundary <= *result.last().expect("the zero boundary is present") {
            return Err(SolverError::invalid_value(
                "chain_normalized_span",
                boundary,
            ));
        }
        result.push(boundary);
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

#[derive(Clone, Debug)]
pub struct CrossLinkLatticeSpec {
    pub first_constraint_id: ConstraintId,
    pub chains: Vec<ChainDesc>,
    pub samples: Vec<f32>,
    pub diagonals: bool,
    pub closed: bool,
    pub compliance_curve: ParameterCurve,
}

#[derive(Clone, Copy, Debug)]
struct AnchorSample {
    body: BodyId,
    local: Vec3,
    world: Vec3,
}

#[derive(Clone, Copy, Debug)]
struct LinkCandidate {
    anchor_a: AnchorSample,
    anchor_b: AnchorSample,
    sample_s: f32,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct AnchorKey {
    body: BodyId,
    local_bits: [u32; 3],
}

impl AnchorKey {
    fn new(anchor: AnchorSample) -> Self {
        Self {
            body: anchor.body,
            local_bits: [
                canonical_float_bits(anchor.local.x),
                canonical_float_bits(anchor.local.y),
                canonical_float_bits(anchor.local.z),
            ],
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct LinkKey(AnchorKey, AnchorKey);

impl LinkKey {
    fn new(a: AnchorKey, b: AnchorKey) -> Self {
        if a <= b {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
}

#[derive(Default)]
struct RigidGraph {
    nodes: BTreeMap<AnchorKey, usize>,
    parents: Vec<usize>,
}

impl RigidGraph {
    fn add_chain(&mut self, anchors: &[AnchorSample], closed: bool) {
        let nodes = anchors
            .iter()
            .copied()
            .map(AnchorKey::new)
            .map(|key| self.ensure_node(key))
            .collect::<Vec<_>>();
        for pair in nodes.windows(2) {
            self.union_indices(pair[0], pair[1]);
        }
        if closed && nodes.len() > 1 {
            self.union_indices(nodes[0], nodes[nodes.len() - 1]);
        }
    }

    fn connected(&mut self, a: AnchorKey, b: AnchorKey) -> bool {
        let a = self.ensure_node(a);
        let b = self.ensure_node(b);
        self.root(a) == self.root(b)
    }

    fn union(&mut self, a: AnchorKey, b: AnchorKey) {
        let a = self.ensure_node(a);
        let b = self.ensure_node(b);
        self.union_indices(a, b);
    }

    fn ensure_node(&mut self, key: AnchorKey) -> usize {
        if let Some(index) = self.nodes.get(&key) {
            *index
        } else {
            let index = self.parents.len();
            self.nodes.insert(key, index);
            self.parents.push(index);
            index
        }
    }

    fn root(&mut self, index: usize) -> usize {
        let parent = self.parents[index];
        if parent == index {
            index
        } else {
            let root = self.root(parent);
            self.parents[index] = root;
            root
        }
    }

    fn union_indices(&mut self, a: usize, b: usize) {
        let root_a = self.root(a);
        let root_b = self.root(b);
        if root_a != root_b {
            let (parent, child) = if root_a < root_b {
                (root_a, root_b)
            } else {
                (root_b, root_a)
            };
            self.parents[child] = parent;
        }
    }
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

    let link_count = link_count_for_samples(spec.samples.len(), spec.diagonals)?;
    validate_constraint_id_range(spec.first_constraint_id, link_count)?;

    let anchors_a = sample_anchors(&spec.chain_a, &spec.samples, bodies)?;
    let anchors_b = sample_anchors(&spec.chain_b, &spec.samples, bodies)?;
    let candidates = link_candidates(
        &anchors_a,
        &anchors_b,
        &spec.samples,
        spec.diagonals,
        link_count,
    );
    let mut graph = RigidGraph::default();
    graph.add_chain(&anchors_a, spec.chain_a.closed);
    graph.add_chain(&anchors_b, spec.chain_b.closed);
    build_constraints(
        spec.first_constraint_id,
        &candidates,
        &spec.compliance_curve,
        graph,
    )
}

pub fn build_cross_link_lattice(
    spec: &CrossLinkLatticeSpec,
    bodies: &BodyLookup,
) -> Result<Vec<PersistentConstraint>, SolverError> {
    if spec.chains.len() < 2 || (spec.closed && spec.chains.len() < 3) {
        return Err(SolverError::invalid_value(
            "cross_link_chain_count",
            spec.chains.len() as f64,
        ));
    }
    validate_samples(&spec.samples)?;
    for chain in &spec.chains {
        chain.validate()?;
        validate_body_references(chain, bodies)?;
    }

    let pair_count = spec
        .chains
        .len()
        .checked_sub(1)
        .and_then(|count| count.checked_add(if spec.closed { 1 } else { 0 }))
        .ok_or(SolverError::invalid_value(
            "cross_link_pair_count",
            f64::INFINITY,
        ))?;
    let links_per_pair = link_count_for_samples(spec.samples.len(), spec.diagonals)?;
    let link_count = pair_count
        .checked_mul(links_per_pair)
        .ok_or(SolverError::invalid_value(
            "cross_link_count",
            f64::INFINITY,
        ))?;
    validate_constraint_id_range(spec.first_constraint_id, link_count)?;

    let anchors = spec
        .chains
        .iter()
        .map(|chain| sample_anchors(chain, &spec.samples, bodies))
        .collect::<Result<Vec<_>, _>>()?;
    let mut graph = RigidGraph::default();
    for (chain, chain_anchors) in spec.chains.iter().zip(&anchors) {
        graph.add_chain(chain_anchors, chain.closed);
    }

    let mut candidates = Vec::with_capacity(link_count);
    for pair in anchors.windows(2) {
        candidates.extend(link_candidates(
            &pair[0],
            &pair[1],
            &spec.samples,
            spec.diagonals,
            links_per_pair,
        ));
    }
    if spec.closed {
        candidates.extend(link_candidates(
            &anchors[anchors.len() - 1],
            &anchors[0],
            &spec.samples,
            spec.diagonals,
            links_per_pair,
        ));
    }

    build_constraints(
        spec.first_constraint_id,
        &candidates,
        &spec.compliance_curve,
        graph,
    )
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
    let boundaries = sample_chain_s(chain)?;
    samples
        .iter()
        .map(|sample| anchor_at(chain, *sample, &boundaries, bodies))
        .collect()
}

fn anchor_at(
    chain: &ChainDesc,
    s: f32,
    boundaries: &[f32],
    bodies: &BodyLookup,
) -> Result<AnchorSample, SolverError> {
    let mut index = boundaries[1..].partition_point(|boundary| *boundary <= s);
    index = index.min(chain.bodies.len() - 1);
    let start = boundaries[index];
    let end = boundaries[index + 1];
    let fraction = ((s - start) / (end - start)).clamp(0.0, 1.0);
    validate_finite("cross_link_anchor_fraction", fraction)?;

    let body_id = chain.bodies[index];
    let body = bodies.get(body_id)?;
    let local = body.local_head.lerp(body.local_tail, fraction);
    let world = body.rest_pose.translation + body.rest_pose.rotation * local;
    validate_vec3("cross_link_local_anchor", local)?;
    validate_vec3("cross_link_world_anchor", world)?;
    Ok(AnchorSample {
        body: body_id,
        local,
        world,
    })
}

fn link_count_for_samples(sample_count: usize, diagonals: bool) -> Result<usize, SolverError> {
    let diagonal_count = if diagonals {
        sample_count
            .checked_sub(1)
            .and_then(|count| count.checked_mul(2))
            .ok_or(SolverError::invalid_value(
                "cross_link_count",
                f64::INFINITY,
            ))?
    } else {
        0
    };
    let link_count = sample_count
        .checked_add(diagonal_count)
        .ok_or(SolverError::invalid_value(
            "cross_link_count",
            f64::INFINITY,
        ))?;
    if link_count == 0 {
        return Err(SolverError::invalid_value("cross_link_count", 0.0));
    }
    Ok(link_count)
}

fn validate_constraint_id_range(
    first_constraint_id: ConstraintId,
    link_count: usize,
) -> Result<(), SolverError> {
    let final_offset = u32::try_from(link_count - 1)
        .map_err(|_| SolverError::invalid_value("constraint_id", f64::INFINITY))?;
    first_constraint_id
        .get()
        .checked_add(final_offset)
        .ok_or(SolverError::invalid_value("constraint_id", f64::INFINITY))?;
    Ok(())
}

fn link_candidates(
    anchors_a: &[AnchorSample],
    anchors_b: &[AnchorSample],
    samples: &[f32],
    diagonals: bool,
    link_count: usize,
) -> Vec<LinkCandidate> {
    let mut candidates = Vec::with_capacity(link_count);
    for (index, sample_s) in samples.iter().copied().enumerate() {
        candidates.push(LinkCandidate {
            anchor_a: anchors_a[index],
            anchor_b: anchors_b[index],
            sample_s,
        });
    }
    if diagonals {
        for index in 0..samples.len() - 1 {
            let sample_s = (samples[index] + samples[index + 1]) * 0.5;
            candidates.push(LinkCandidate {
                anchor_a: anchors_a[index],
                anchor_b: anchors_b[index + 1],
                sample_s,
            });
            candidates.push(LinkCandidate {
                anchor_a: anchors_a[index + 1],
                anchor_b: anchors_b[index],
                sample_s,
            });
        }
    }
    candidates
}

fn build_constraints(
    first_constraint_id: ConstraintId,
    candidates: &[LinkCandidate],
    compliance_curve: &ParameterCurve,
    mut graph: RigidGraph,
) -> Result<Vec<PersistentConstraint>, SolverError> {
    let mut seen = BTreeSet::new();
    let mut constraints = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let key_a = AnchorKey::new(candidate.anchor_a);
        let key_b = AnchorKey::new(candidate.anchor_b);
        if candidate.anchor_a.body == candidate.anchor_b.body || key_a == key_b {
            return Err(SolverError::invalid_value("cross_link_self_link", 0.0));
        }
        if !seen.insert(LinkKey::new(key_a, key_b)) {
            return Err(SolverError::invalid_value(
                "duplicate_cross_link_geometry",
                0.0,
            ));
        }

        let compliance = compliance_curve.sample(candidate.sample_s)?;
        validate_nonnegative("cross_link_compliance", compliance)?;
        if compliance == 0.0 {
            if graph.connected(key_a, key_b) {
                return Err(SolverError::invalid_value(
                    "cyclic_cross_link_compliance",
                    compliance,
                ));
            }
            graph.union(key_a, key_b);
        }

        let rest_length = candidate.anchor_a.world.distance(candidate.anchor_b.world);
        validate_nonnegative("cross_link_rest_length", rest_length)?;
        let offset = u32::try_from(constraints.len())
            .map_err(|_| SolverError::invalid_value("constraint_id", f64::INFINITY))?;
        let raw_id = first_constraint_id
            .get()
            .checked_add(offset)
            .ok_or(SolverError::invalid_value("constraint_id", f64::INFINITY))?;
        constraints.push(PersistentConstraint::distance(
            ConstraintId::new(raw_id),
            candidate.anchor_a.body,
            candidate.anchor_b.body,
            candidate.anchor_a.local,
            candidate.anchor_b.local,
            rest_length,
            compliance,
            0.0,
        ));
    }
    Ok(constraints)
}

fn canonical_float_bits(value: f32) -> u32 {
    if value == 0.0 {
        0.0_f32.to_bits()
    } else {
        value.to_bits()
    }
}
