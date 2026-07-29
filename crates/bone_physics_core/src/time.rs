use crate::SolverError;
use crate::error::validate_finite;

const STEP_RATIO_EPSILON: f64 = 1.0e-5;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QualityPolicy {
    DeterministicPreview,
    ApproximatePreview { max_steps: u32 },
    AuthoritativeBake,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepBatch {
    pub steps: u32,
    pub dt: f32,
    pub alpha: f32,
    pub dropped: bool,
}

#[derive(Clone, Debug)]
pub struct FixedClock {
    step_seconds: f32,
    step_seconds_f64: f64,
    accumulator: f64,
}

impl FixedClock {
    pub fn new(step_seconds: f32) -> Result<Self, SolverError> {
        validate_finite("step_seconds", step_seconds)?;
        if step_seconds <= 0.0 {
            return Err(SolverError::invalid_value("step_seconds", step_seconds));
        }

        Ok(Self {
            step_seconds,
            step_seconds_f64: f64::from(step_seconds),
            accumulator: 0.0,
        })
    }

    pub fn try_accumulate(
        &mut self,
        frame_seconds: f32,
        quality: QualityPolicy,
    ) -> Result<StepBatch, SolverError> {
        validate_finite("frame_seconds", frame_seconds)?;
        if frame_seconds < 0.0 {
            return Err(SolverError::invalid_value("frame_seconds", frame_seconds));
        }

        self.accumulator += f64::from(frame_seconds);
        let available_f64 = (self.accumulator / self.step_seconds_f64 + STEP_RATIO_EPSILON).floor();
        if available_f64 > f64::from(u32::MAX) {
            return Err(SolverError::invalid_value(
                "accumulated_step_count",
                f32::MAX,
            ));
        }

        let available = available_f64 as u32;
        self.accumulator -= f64::from(available) * self.step_seconds_f64;
        if self.accumulator < 0.0
            && self.accumulator.abs() <= self.step_seconds_f64 * STEP_RATIO_EPSILON
        {
            self.accumulator = 0.0;
        }

        let (steps, dropped) = match quality {
            QualityPolicy::ApproximatePreview { max_steps } => {
                (available.min(max_steps), available > max_steps)
            }
            QualityPolicy::DeterministicPreview | QualityPolicy::AuthoritativeBake => {
                (available, false)
            }
        };

        Ok(StepBatch {
            steps,
            dt: self.step_seconds,
            alpha: (self.accumulator / self.step_seconds_f64) as f32,
            dropped,
        })
    }

    pub fn reset(&mut self) {
        self.accumulator = 0.0;
    }
}
