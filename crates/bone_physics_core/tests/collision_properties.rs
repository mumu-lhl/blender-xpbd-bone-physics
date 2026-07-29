use bone_physics_core::{
    BodyId, Capsule, CapsuleCollider, ColliderId, CollisionFilter, Pose, capsule_manifold,
};
use glam::{Quat, Vec3};

fn next_scalar(state: &mut u32) -> f32 {
    *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    let unit = (*state >> 8) as f32 / 16_777_215.0;
    unit * 4.0 - 2.0
}

fn next_vec3(state: &mut u32) -> Vec3 {
    Vec3::new(next_scalar(state), next_scalar(state), next_scalar(state))
}

fn collider(
    id: u32,
    local_a: Vec3,
    local_b: Vec3,
    radius: f32,
    translation: Vec3,
    velocity: Vec3,
) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(id),
        shape: Capsule::try_new(local_a, local_b, radius, 0.02).unwrap(),
        pose: Pose::try_new(translation, Quat::IDENTITY).unwrap(),
        linear_velocity: velocity,
        filter: CollisionFilter::default(),
    }
}

#[test]
fn deterministic_degenerate_capsule_cases_are_symmetric_and_never_non_finite() {
    let mut state = 0x5eed_cafe;
    for case in 0..512 {
        let mut local_a = next_vec3(&mut state);
        let mut local_b = next_vec3(&mut state);
        if case % 5 == 0 {
            local_b = local_a;
        }
        if case % 7 == 0 {
            local_a = Vec3::ZERO;
            local_b = Vec3::X;
        }
        let translation_a = next_vec3(&mut state);
        let translation_b = if case % 11 == 0 {
            translation_a
        } else {
            next_vec3(&mut state)
        };
        let a = collider(
            1,
            local_a,
            local_b,
            0.05 + next_scalar(&mut state).abs() * 0.2,
            translation_a,
            next_vec3(&mut state),
        );
        let b = collider(
            2,
            if case % 13 == 0 {
                local_a
            } else {
                next_vec3(&mut state)
            },
            if case % 13 == 0 {
                local_b
            } else {
                next_vec3(&mut state)
            },
            0.05 + next_scalar(&mut state).abs() * 0.2,
            translation_b,
            next_vec3(&mut state),
        );

        let forward = capsule_manifold(&a, &b, None).unwrap();
        let reverse = capsule_manifold(&b, &a, None).unwrap();

        assert_eq!(forward, reverse, "case={case}");
        if let Some(manifold) = forward {
            assert!(!manifold.points.is_empty(), "case={case}");
            assert!(manifold.points.len() <= 2, "case={case}");
            for point in manifold.points {
                assert!(point.point_a.is_finite(), "case={case}");
                assert!(point.point_b.is_finite(), "case={case}");
                assert!(point.normal.is_finite(), "case={case}");
                assert!((point.normal.length() - 1.0).abs() < 1.0e-4, "case={case}");
                assert!(point.penetration.is_finite(), "case={case}");
                assert!(point.penetration >= 0.0, "case={case}");
            }
        }
    }
}
