use bone_physics_core::{
    BodyId, ChainDesc, CurveInterpolation, CurvePoint, ParameterCurve, sample_chain_s,
};

fn assert_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-6,
        "actual={actual}, expected={expected}"
    );
}

#[test]
fn chain_coordinates_follow_accumulated_rest_length() {
    let chain = ChainDesc {
        bodies: vec![BodyId::new(1), BodyId::new(2), BodyId::new(3)],
        rest_lengths: vec![1.0, 3.0, 2.0],
        closed: false,
    };

    let coordinates = sample_chain_s(&chain).unwrap();

    assert_eq!(coordinates.len(), 4);
    assert_close(coordinates[0], 0.0);
    assert_close(coordinates[1], 1.0 / 6.0);
    assert_close(coordinates[2], 4.0 / 6.0);
    assert_close(coordinates[3], 1.0);
}

#[test]
fn parameter_curves_sample_constant_linear_and_cubic_hermite_values() {
    let constant = ParameterCurve::try_new(
        vec![
            CurvePoint::new(0.0, 2.0, 0.0),
            CurvePoint::new(1.0, 6.0, 0.0),
        ],
        CurveInterpolation::Constant,
    )
    .unwrap();
    let linear = ParameterCurve::try_new(
        vec![
            CurvePoint::new(0.0, 2.0, 0.0),
            CurvePoint::new(1.0, 6.0, 0.0),
        ],
        CurveInterpolation::Linear,
    )
    .unwrap();
    let cubic = ParameterCurve::try_new(
        vec![
            CurvePoint::new(0.0, 0.0, 0.0),
            CurvePoint::new(1.0, 1.0, 0.0),
        ],
        CurveInterpolation::CubicHermite,
    )
    .unwrap();

    assert_close(constant.sample(0.25).unwrap(), 2.0);
    assert_close(linear.sample(0.25).unwrap(), 3.0);
    assert_close(cubic.sample(0.25).unwrap(), 0.156_25);
}

#[test]
fn parameter_curves_reject_invalid_points_and_samples() {
    let decreasing = ParameterCurve::try_new(
        vec![
            CurvePoint::new(0.75, 1.0, 0.0),
            CurvePoint::new(0.25, 2.0, 0.0),
        ],
        CurveInterpolation::Linear,
    );
    let non_finite = ParameterCurve::try_new(
        vec![CurvePoint::new(0.0, f32::NAN, 0.0)],
        CurveInterpolation::Constant,
    );
    let valid = ParameterCurve::try_new(
        vec![CurvePoint::new(0.0, 1.0, 0.0)],
        CurveInterpolation::Constant,
    )
    .unwrap();

    assert!(decreasing.is_err());
    assert!(non_finite.is_err());
    assert!(valid.sample(f32::INFINITY).is_err());
}

#[test]
fn parameter_curve_sampling_revalidates_publicly_mutated_points() {
    let mut curve = ParameterCurve::try_new(
        vec![
            CurvePoint::new(0.0, 1.0, 0.0),
            CurvePoint::new(1.0, 2.0, 0.0),
        ],
        CurveInterpolation::Linear,
    )
    .unwrap();

    curve.points[0].value = f32::NAN;
    assert!(curve.sample(0.0).is_err());

    curve.points[0] = CurvePoint::new(0.75, 1.0, 0.0);
    curve.points[1] = CurvePoint::new(0.25, 2.0, 0.0);
    assert!(curve.sample(0.5).is_err());
}

#[test]
fn chains_reject_empty_zero_and_non_finite_geometry() {
    let empty = ChainDesc {
        bodies: Vec::new(),
        rest_lengths: Vec::new(),
        closed: false,
    };
    let zero = ChainDesc {
        bodies: vec![BodyId::new(1)],
        rest_lengths: vec![0.0],
        closed: false,
    };
    let non_finite = ChainDesc {
        bodies: vec![BodyId::new(1)],
        rest_lengths: vec![f32::INFINITY],
        closed: false,
    };

    assert!(sample_chain_s(&empty).is_err());
    assert!(sample_chain_s(&zero).is_err());
    assert!(sample_chain_s(&non_finite).is_err());
}
