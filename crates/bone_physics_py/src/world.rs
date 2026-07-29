use bone_physics_core::{
    AnalyticProxy, BodyId, ColliderId, FixedClock, QualityPolicy, SolverError, SolverWorld,
    StepDiagnostics, StepOptions,
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
    pub rolled_back: bool,
}

pub struct NativeWorld {
    solver: SolverWorld,
    clock: FixedClock,
    body_ids: Vec<BodyId>,
    body_collider_ids: Vec<ColliderId>,
    analytic_colliders: Vec<AnalyticProxy>,
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
            analytic_colliders: Vec::new(),
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
            let diagnostics = self.solver.substep(batch.dt, target_alpha, options)?;
            accumulate_diagnostics(&mut summary, diagnostics);
            if diagnostics.rolled_back {
                break;
            }
        }
        self.last_summary = summary;
        Ok(summary)
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
        self.analytic_colliders = colliders;
        Ok(())
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
        self.analytic_colliders.len()
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
    summary.rolled_back |= diagnostics.rolled_back;
}
