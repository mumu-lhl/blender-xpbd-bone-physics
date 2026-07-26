use crate::error::validate_finite;
use crate::SolverError;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurvePoint {
    pub s: f32,
    pub value: f32,
    pub tangent: f32,
}

impl CurvePoint {
    #[must_use]
    pub const fn new(s: f32, value: f32, tangent: f32) -> Self {
        Self { s, value, tangent }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveInterpolation {
    Constant,
    Linear,
    CubicHermite,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParameterCurve {
    pub points: Vec<CurvePoint>,
    pub interpolation: CurveInterpolation,
}

impl ParameterCurve {
    pub fn try_new(
        points: Vec<CurvePoint>,
        interpolation: CurveInterpolation,
    ) -> Result<Self, SolverError> {
        if points.is_empty() {
            return Err(SolverError::invalid_value("curve_point_count", 0.0));
        }

        for point in &points {
            validate_finite("curve_s", point.s)?;
            validate_finite("curve_value", point.value)?;
            validate_finite("curve_tangent", point.tangent)?;
            if !(0.0..=1.0).contains(&point.s) {
                return Err(SolverError::invalid_value("curve_s", point.s));
            }
        }
        for pair in points.windows(2) {
            if pair[0].s >= pair[1].s {
                return Err(SolverError::invalid_value(
                    "curve_point_order",
                    pair[1].s - pair[0].s,
                ));
            }
        }

        Ok(Self {
            points,
            interpolation,
        })
    }

    pub fn sample(&self, s: f32) -> Result<f32, SolverError> {
        validate_finite("curve_sample_s", s)?;
        let first = self
            .points
            .first()
            .ok_or(SolverError::invalid_value("curve_point_count", 0.0))?;
        if s <= first.s {
            return Ok(first.value);
        }
        let last = self
            .points
            .last()
            .ok_or(SolverError::invalid_value("curve_point_count", 0.0))?;
        if s >= last.s {
            return Ok(last.value);
        }

        let upper = self.points.partition_point(|point| point.s <= s);
        let left = self.points[upper - 1];
        let right = self.points[upper];
        let span = right.s - left.s;
        let t = (s - left.s) / span;
        let value = match self.interpolation {
            CurveInterpolation::Constant => left.value,
            CurveInterpolation::Linear => left.value + (right.value - left.value) * t,
            CurveInterpolation::CubicHermite => {
                let t2 = t * t;
                let t3 = t2 * t;
                let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
                let h10 = t3 - 2.0 * t2 + t;
                let h01 = -2.0 * t3 + 3.0 * t2;
                let h11 = t3 - t2;
                h00 * left.value
                    + h10 * left.tangent * span
                    + h01 * right.value
                    + h11 * right.tangent * span
            }
        };
        validate_finite("curve_sample_value", value)?;
        Ok(value)
    }
}
