use bone_physics_core::{FixedClock, QualityPolicy, SolverError};

#[test]
fn one_second_produces_the_same_step_count_at_common_frame_rates() {
    for fps in [24_u32, 30, 60] {
        let mut clock = FixedClock::new(1.0 / 120.0).unwrap();
        let steps: u32 = (0..fps)
            .map(|_| {
                clock
                    .try_accumulate(1.0 / fps as f32, QualityPolicy::AuthoritativeBake)
                    .unwrap()
                    .steps
            })
            .sum();

        assert_eq!(steps, 120, "unexpected step count at {fps} FPS");
    }
}

#[test]
fn invalid_step_or_frame_duration_is_rejected() {
    assert!(matches!(
        FixedClock::new(0.0),
        Err(SolverError::InvalidValue {
            field: "step_seconds",
            ..
        })
    ));
    assert!(FixedClock::new(f32::NAN).is_err());

    let mut clock = FixedClock::new(1.0 / 120.0).unwrap();
    assert!(
        clock
            .try_accumulate(-0.1, QualityPolicy::DeterministicPreview)
            .is_err()
    );
    assert!(
        clock
            .try_accumulate(f32::NAN, QualityPolicy::DeterministicPreview)
            .is_err()
    );
}

#[test]
fn deterministic_modes_never_drop_accumulated_time() {
    for quality in [
        QualityPolicy::DeterministicPreview,
        QualityPolicy::AuthoritativeBake,
    ] {
        let mut clock = FixedClock::new(1.0 / 120.0).unwrap();

        let batch = clock.try_accumulate(1.0, quality).unwrap();

        assert_eq!(batch.steps, 120);
        assert!(!batch.dropped);
        assert_eq!(batch.dt, 1.0 / 120.0);
        assert!(batch.alpha >= 0.0 && batch.alpha < 1.0);
    }
}

#[test]
fn approximate_preview_caps_and_discards_only_whole_excess_steps() {
    let mut clock = FixedClock::new(1.0 / 120.0).unwrap();

    let capped = clock
        .try_accumulate(1.0, QualityPolicy::ApproximatePreview { max_steps: 8 })
        .unwrap();
    let following = clock
        .try_accumulate(0.0, QualityPolicy::ApproximatePreview { max_steps: 8 })
        .unwrap();

    assert_eq!(capped.steps, 8);
    assert!(capped.dropped);
    assert_eq!(following.steps, 0);
    assert!(!following.dropped);
}

#[test]
fn reset_clears_the_fractional_remainder() {
    let mut clock = FixedClock::new(1.0 / 120.0).unwrap();
    let half_step = clock
        .try_accumulate(1.0 / 240.0, QualityPolicy::DeterministicPreview)
        .unwrap();
    assert_eq!(half_step.steps, 0);
    assert!((half_step.alpha - 0.5).abs() < 1.0e-6);

    clock.reset();
    let empty = clock
        .try_accumulate(0.0, QualityPolicy::DeterministicPreview)
        .unwrap();

    assert_eq!(empty.steps, 0);
    assert_eq!(empty.alpha, 0.0);
}
