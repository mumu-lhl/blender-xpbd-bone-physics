use bone_physics_core::{
    AnalyticProxy, BodyId, ColliderId, DeformingMesh, FixedClock, MeshId, MeshUpdate,
    QualityPolicy, SolverError, SolverWorld, StepDiagnostics, StepOptions, StepPhaseTimings,
    WorldFingerprint,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct AdvanceSummary {
    pub steps: u32,
    pub dropped: bool,
    pub alpha: f32,
    pub generated_contacts: u32,
    pub active_contacts: u32,
    pub deepest_penetration: f32,
    pub max_constraint_error: f32,
    pub broad_phase_candidates: u32,
    pub ccd_fallbacks: u32,
    pub ccd_failures: u32,
    pub phase_timings: StepPhaseTimings,
    pub substep_profiled: bool,
    pub rolled_back: bool,
}

pub struct NativeWorld {
    solver: SolverWorld,
    clock: FixedClock,
    body_ids: Vec<BodyId>,
    body_collider_ids: Vec<ColliderId>,
    last_summary: AdvanceSummary,
}

impl NativeWorld {
    pub fn new(
        solver: SolverWorld,
        mut body_ids: Vec<BodyId>,
        mut body_collider_ids: Vec<ColliderId>,
        fixed_dt: f32,
    ) -> Result<Self, SolverError> {
        body_ids.sort_unstable();
        body_collider_ids.sort_unstable();
        let clock = FixedClock::new(fixed_dt)?;
        Ok(Self {
            solver,
            clock,
            body_ids,
            body_collider_ids,
            last_summary: AdvanceSummary::default(),
        })
    }

    pub fn advance(
        &mut self,
        frame_seconds: f32,
        quality: QualityPolicy,
    ) -> Result<AdvanceSummary, SolverError> {
        let batch = self.clock.try_accumulate(frame_seconds, quality)?;
        let mut summary = AdvanceSummary {
            steps: batch.steps,
            dropped: batch.dropped,
            alpha: batch.alpha,
            ..AdvanceSummary::default()
        };
        let options = StepOptions {
            position_iterations: 8,
            contact_iterations: 4,
            quality,
        };
        for step in 0..batch.steps {
            let target_alpha = (step + 1) as f32 / batch.steps as f32;
            let (diagnostics, timings) =
                self.solver
                    .substep_profiled(batch.dt, target_alpha, options)?;
            accumulate_timings(&mut summary.phase_timings, timings);
            summary.substep_profiled = true;
            accumulate_diagnostics(&mut summary, diagnostics);
            if diagnostics.rolled_back {
                break;
            }
        }
        self.last_summary = summary;
        Ok(summary)
    }

    pub fn world_fingerprint(&self) -> [u8; 32] {
        self.solver.world_fingerprint().bytes()
    }

    pub fn encode_checkpoint(&self, quality: QualityPolicy) -> Result<Vec<u8>, SolverError> {
        if matches!(quality, QualityPolicy::ApproximatePreview { .. }) {
            return Err(SolverError::invalid_value("approximate_checkpoint", 0));
        }
        let solver_checkpoint = self.solver.encode_checkpoint()?;
        let mut bytes = Vec::with_capacity(20 + solver_checkpoint.len());
        bytes.extend_from_slice(b"BPWN001\0");
        bytes.extend_from_slice(&self.clock.remainder().to_le_bytes());
        bytes.extend_from_slice(&(solver_checkpoint.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&solver_checkpoint);
        Ok(bytes)
    }

    pub fn restore_checkpoint(
        &mut self,
        bytes: &[u8],
        expected_fingerprint: [u8; 32],
    ) -> Result<(), SolverError> {
        if bytes.len() < 20 || &bytes[..8] != b"BPWN001\0" {
            return Err(SolverError::invalid_value("native_checkpoint_header", 0));
        }
        let remainder = f64::from_le_bytes(bytes[8..16].try_into().expect("fixed width"));
        let length = u32::from_le_bytes(bytes[16..20].try_into().expect("fixed width")) as usize;
        let end = 20usize
            .checked_add(length)
            .ok_or_else(|| SolverError::invalid_value("native_checkpoint_length", length as f64))?;
        if end != bytes.len() {
            return Err(SolverError::invalid_value(
                "native_checkpoint_length",
                length as f64,
            ));
        }
        if self.world_fingerprint() != expected_fingerprint {
            return Err(SolverError::invalid_value("world_fingerprint", 0));
        }
        let mut candidate_clock = self.clock.clone();
        candidate_clock.restore_remainder(remainder)?;
        self.solver.restore_checkpoint(
            &bytes[20..end],
            WorldFingerprint::from_bytes(expected_fingerprint),
        )?;
        self.clock = candidate_clock;
        Ok(())
    }

    pub fn set_animation_targets(
        &mut self,
        targets: &[(BodyId, bone_physics_core::AnimationTarget)],
    ) -> Result<(), SolverError> {
        self.solver.set_animation_targets(targets)
    }

    pub fn resume(&mut self) -> Result<(), SolverError> {
        self.solver.resume()
    }

    pub fn set_analytic_colliders(
        &mut self,
        colliders: Vec<AnalyticProxy>,
    ) -> Result<(), SolverError> {
        if let Some(collider) = colliders
            .iter()
            .find(|collider| self.body_collider_ids.binary_search(&collider.id).is_ok())
        {
            return Err(SolverError::DuplicateId {
                kind: "collider",
                id: collider.id.get(),
            });
        }
        self.solver.set_analytic_proxies(colliders)
    }

    pub fn set_mesh(&mut self, mesh: DeformingMesh) -> Result<u64, SolverError> {
        let topology_hash = mesh.validation_report().topology_hash;
        self.solver.set_mesh(mesh)?;
        Ok(topology_hash)
    }

    pub fn update_mesh_vertices(
        &mut self,
        id: MeshId,
        previous: Vec<glam::Vec3>,
        current: Vec<glam::Vec3>,
        expected_hash: u64,
    ) -> Result<MeshUpdate, SolverError> {
        let mesh = self
            .solver
            .mesh(id)
            .ok_or_else(|| SolverError::invalid_value("mesh_id", id.get()))?;
        if mesh.validation_report().topology_hash != expected_hash {
            return Err(SolverError::invalid_value(
                "mesh_topology_hash",
                expected_hash as f64,
            ));
        }
        self.solver.update_mesh_vertices(id, previous, current)
    }

    pub fn remove_mesh(&mut self, id: MeshId) -> Result<(), SolverError> {
        self.solver.remove_mesh(id)
    }

    #[must_use]
    pub fn transform_values(&self) -> Vec<f32> {
        let mut output = Vec::with_capacity(self.body_ids.len() * 7);
        for id in &self.body_ids {
            if let Some(body) = self.solver.body(*id) {
                output.extend_from_slice(&body.pose.translation.to_array());
                output.extend_from_slice(&body.pose.rotation.to_array());
            }
        }
        output
    }

    #[must_use]
    pub fn body_count(&self) -> usize {
        self.body_ids.len()
    }

    #[must_use]
    pub fn analytic_collider_count(&self) -> usize {
        self.solver.analytic_proxy_count()
    }

    #[must_use]
    pub fn mesh_count(&self) -> usize {
        self.solver.mesh_count()
    }

    #[must_use]
    pub fn mesh_vertex_count(&self, id: MeshId) -> Option<usize> {
        self.solver
            .mesh(id)
            .map(|mesh| mesh.current_positions().len())
    }

    #[must_use]
    pub fn mesh_topology_hash(&self, id: MeshId) -> Option<u64> {
        self.solver
            .mesh(id)
            .map(|mesh| mesh.validation_report().topology_hash)
    }

    #[must_use]
    pub const fn last_summary(&self) -> AdvanceSummary {
        self.last_summary
    }
}

fn accumulate_diagnostics(summary: &mut AdvanceSummary, diagnostics: StepDiagnostics) {
    summary.generated_contacts = summary
        .generated_contacts
        .saturating_add(diagnostics.generated_contacts);
    summary.active_contacts = diagnostics.active_contacts;
    summary.deepest_penetration = summary
        .deepest_penetration
        .max(diagnostics.deepest_penetration);
    summary.max_constraint_error = summary
        .max_constraint_error
        .max(diagnostics.max_constraint_error);
    summary.broad_phase_candidates = summary
        .broad_phase_candidates
        .saturating_add(diagnostics.broad_phase_candidates);
    summary.ccd_fallbacks = summary
        .ccd_fallbacks
        .saturating_add(diagnostics.ccd_fallbacks);
    summary.ccd_failures = summary
        .ccd_failures
        .saturating_add(diagnostics.ccd_failures);
    summary.rolled_back |= diagnostics.rolled_back;
}

fn accumulate_timings(total: &mut StepPhaseTimings, step: StepPhaseTimings) {
    total.prediction_seconds += step.prediction_seconds;
    total.animation_inertia_seconds += step.animation_inertia_seconds;
    total.contact_generation_seconds += step.contact_generation_seconds;
    total.position_solve_seconds += step.position_solve_seconds;
    total.diagnostics_seconds += step.diagnostics_seconds;
    total.velocity_seconds += step.velocity_seconds;
}
