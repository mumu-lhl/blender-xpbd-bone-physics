use bone_physics_core::{
    AnalyticContact, AnalyticProxy, AnalyticShape, BodyId, Capsule, CapsuleCollider, ColliderId,
    CollisionFilter, Pose, SolverError, capsule_analytic_contact,
};
use glam::{Quat, Vec3};

const GEOMETRY_TOLERANCE: f32 = 2.0e-5;

fn capsule(
    id: u32,
    local_a: Vec3,
    local_b: Vec3,
    radius: f32,
    translation: Vec3,
    linear_velocity: Vec3,
) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(id),
        shape: Capsule::try_new(local_a, local_b, radius, 0.0).unwrap(),
        pose: Pose::try_new(translation, Quat::IDENTITY).unwrap(),
        linear_velocity,
        filter: CollisionFilter::default(),
    }
}

fn capsule_sphere(id: u32, center: Vec3, radius: f32) -> CapsuleCollider {
    capsule(id, Vec3::ZERO, Vec3::ZERO, radius, center, Vec3::ZERO)
}

fn proxy(id: u32, pose: Pose, linear_velocity: Vec3, shape: AnalyticShape) -> AnalyticProxy {
    AnalyticProxy {
        id: ColliderId::new(id),
        pose,
        linear_velocity,
        shape,
        collision_enabled: true,
    }
}

fn sphere_proxy(id: u32, center: Vec3, radius: f32) -> AnalyticProxy {
    proxy(
        id,
        Pose::try_new(center, Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::Sphere { radius },
    )
}

fn assert_scalar_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() <= GEOMETRY_TOLERANCE,
        "actual={actual}, expected={expected}"
    );
}

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    assert!(
        actual.distance(expected) <= GEOMETRY_TOLERANCE,
        "actual={actual:?}, expected={expected:?}"
    );
}

fn assert_finite_canonical_contact(contact: &AnalyticContact) {
    assert!(contact.point_capsule.is_finite());
    assert!(contact.point_proxy.is_finite());
    assert!(contact.normal.is_finite());
    assert_scalar_close(contact.normal.length(), 1.0);
    assert!(contact.separation.is_finite());
    assert_scalar_close(
        (contact.point_proxy - contact.point_capsule).dot(contact.normal),
        contact.separation,
    );
}

fn assert_contact_witness(
    contact: &AnalyticContact,
    point_capsule: Vec3,
    point_proxy: Vec3,
    normal: Vec3,
    separation: f32,
    speculative: bool,
) {
    assert_finite_canonical_contact(contact);
    assert_vec3_close(contact.point_capsule, point_capsule);
    assert_vec3_close(contact.point_proxy, point_proxy);
    assert_vec3_close(contact.normal, normal);
    assert_scalar_close(contact.separation, separation);
    assert_eq!(contact.speculative, speculative);
}

fn contact_bits(contact: &AnalyticContact) -> ([u32; 3], [u32; 3], [u32; 3], u32, bool) {
    (
        contact.point_capsule.to_array().map(f32::to_bits),
        contact.point_proxy.to_array().map(f32::to_bits),
        contact.normal.to_array().map(f32::to_bits),
        contact.separation.to_bits(),
        contact.speculative,
    )
}

fn assert_invalid_value(
    capsule: &CapsuleCollider,
    proxy: &AnalyticProxy,
    dt: f32,
    expected_field: &'static str,
    expected_value: f64,
) {
    let Err(error) = capsule_analytic_contact(capsule, proxy, dt) else {
        panic!("expected invalid value error");
    };
    let SolverError::InvalidValue { field, value } = error else {
        panic!("expected SolverError::InvalidValue");
    };
    assert_eq!(field, expected_field);
    if expected_value.is_nan() {
        assert!(value.is_nan());
    } else {
        assert_eq!(value.to_bits(), expected_value.to_bits());
    }
}

#[test]
fn stationary_separated_sphere_has_no_contact() {
    let capsule = capsule_sphere(1, Vec3::ZERO, 0.5);
    let proxy = sphere_proxy(2, Vec3::new(1.25, 0.0, 0.0), 0.5);

    assert!(
        capsule_analytic_contact(&capsule, &proxy, 0.125)
            .unwrap()
            .is_none()
    );
}

#[test]
fn touching_sphere_has_zero_canonical_separation() {
    let capsule = capsule_sphere(1, Vec3::ZERO, 0.5);
    let proxy = sphere_proxy(2, Vec3::X, 0.5);

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_finite_canonical_contact(&contact);
    assert_vec3_close(contact.normal, Vec3::X);
    assert_vec3_close(contact.point_capsule, Vec3::new(0.5, 0.0, 0.0));
    assert_vec3_close(contact.point_proxy, Vec3::new(0.5, 0.0, 0.0));
    assert_scalar_close(contact.separation, 0.0);
    assert!(!contact.speculative);
}

#[test]
fn penetrating_sphere_has_negative_canonical_separation() {
    let capsule = capsule_sphere(1, Vec3::ZERO, 0.5);
    let proxy = sphere_proxy(2, Vec3::new(0.75, 0.0, 0.0), 0.5);

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_finite_canonical_contact(&contact);
    assert_vec3_close(contact.normal, Vec3::X);
    assert_vec3_close(contact.point_capsule, Vec3::new(0.5, 0.0, 0.0));
    assert_vec3_close(contact.point_proxy, Vec3::new(0.25, 0.0, 0.0));
    assert_scalar_close(contact.separation, -0.25);
    assert!(!contact.speculative);
}

#[test]
fn analytic_capsule_endpoint_permutations_are_bit_deterministic() {
    let forward_capsule = capsule(10, -Vec3::X, Vec3::X, 0.25, Vec3::ZERO, Vec3::ZERO);
    let reversed_capsule = capsule(10, Vec3::X, -Vec3::X, 0.25, Vec3::ZERO, Vec3::ZERO);
    let forward_proxy = proxy(
        11,
        Pose::try_new(Vec3::new(0.0, 0.4, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::Capsule {
            local_a: -Vec3::Z,
            local_b: Vec3::Z,
            radius: 0.25,
        },
    );
    let reversed_proxy = proxy(
        11,
        Pose::try_new(Vec3::new(0.0, 0.4, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::Capsule {
            local_a: Vec3::Z,
            local_b: -Vec3::Z,
            radius: 0.25,
        },
    );

    let mut expected_bits = None;
    for capsule in [&forward_capsule, &reversed_capsule] {
        for proxy in [&forward_proxy, &reversed_proxy] {
            let contact = capsule_analytic_contact(capsule, proxy, 0.125)
                .unwrap()
                .unwrap();
            assert_contact_witness(
                &contact,
                Vec3::new(0.0, 0.25, 0.0),
                Vec3::new(0.0, 0.15, 0.0),
                Vec3::Y,
                -0.1,
                false,
            );

            let bits = contact_bits(&contact);
            if let Some(expected) = expected_bits {
                assert_eq!(bits, expected);
            } else {
                expected_bits = Some(bits);
            }
        }
    }
}

#[test]
fn zero_length_analytic_capsule_behaves_like_a_sphere() {
    let capsule = capsule_sphere(20, Vec3::ZERO, 0.25);
    let analytic_capsule = proxy(
        21,
        Pose::try_new(Vec3::new(0.4, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::Capsule {
            local_a: Vec3::ZERO,
            local_b: Vec3::ZERO,
            radius: 0.25,
        },
    );
    let analytic_sphere = sphere_proxy(21, Vec3::new(0.4, 0.0, 0.0), 0.25);

    let capsule_contact = capsule_analytic_contact(&capsule, &analytic_capsule, 0.125)
        .unwrap()
        .unwrap();
    let sphere_contact = capsule_analytic_contact(&capsule, &analytic_sphere, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &capsule_contact,
        Vec3::new(0.25, 0.0, 0.0),
        Vec3::new(0.15, 0.0, 0.0),
        Vec3::X,
        -0.1,
        false,
    );
    assert_contact_witness(
        &sphere_contact,
        Vec3::new(0.25, 0.0, 0.0),
        Vec3::new(0.15, 0.0, 0.0),
        Vec3::X,
        -0.1,
        false,
    );
    assert_eq!(
        contact_bits(&capsule_contact),
        contact_bits(&sphere_contact)
    );
}

fn rotated_rounded_box(sample_in_proxy: Vec3) -> AnalyticProxy {
    let rotation = Quat::from_rotation_z(core::f32::consts::FRAC_PI_2);
    proxy(
        31,
        Pose::try_new(-(rotation * sample_in_proxy), rotation).unwrap(),
        Vec3::ZERO,
        AnalyticShape::RoundedBox {
            half_extents: Vec3::new(1.0, 0.5, 0.5),
            radius: 0.25,
        },
    )
}

#[test]
fn rotated_rounded_box_face_has_hand_computable_contact() {
    let capsule = capsule_sphere(30, Vec3::ZERO, 0.25);
    let proxy = rotated_rounded_box(Vec3::new(1.4, 0.0, 0.0));

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &contact,
        Vec3::new(0.0, -0.25, 0.0),
        Vec3::new(0.0, -0.15, 0.0),
        -Vec3::Y,
        -0.1,
        false,
    );
}

#[test]
fn rotated_rounded_box_edge_has_hand_computable_contact() {
    let capsule = capsule_sphere(30, Vec3::ZERO, 0.25);
    let proxy = rotated_rounded_box(Vec3::new(1.3, 0.8, 0.0));

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    let normal = Vec3::new(1.0, -1.0, 0.0).normalize();
    assert_contact_witness(
        &contact,
        normal * 0.25,
        Vec3::new(0.3, -0.3, 0.0) - normal * 0.25,
        normal,
        0.18_f32.sqrt() - 0.5,
        false,
    );
}

#[test]
fn rotated_rounded_box_corner_has_hand_computable_contact() {
    let capsule = capsule_sphere(30, Vec3::ZERO, 0.25);
    let proxy = rotated_rounded_box(Vec3::new(1.2, 0.7, 0.7));

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    let normal = Vec3::new(1.0, -1.0, -1.0).normalize();
    assert_contact_witness(
        &contact,
        normal * 0.25,
        Vec3::new(0.2, -0.2, -0.2) - normal * 0.25,
        normal,
        0.12_f32.sqrt() - 0.5,
        false,
    );
}

#[test]
fn rounded_box_containing_capsule_uses_minimum_translation_axis() {
    let capsule = capsule(
        2,
        -0.9 * Vec3::X,
        0.9 * Vec3::X,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let proxy = proxy(
        8,
        Pose::IDENTITY,
        Vec3::ZERO,
        AnalyticShape::RoundedBox {
            half_extents: Vec3::ONE,
            radius: 0.1,
        },
    );

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &contact,
        Vec3::new(-0.9, 0.1, 0.0),
        Vec3::new(-0.9, -1.1, 0.0),
        Vec3::Y,
        -1.2,
        false,
    );
}

#[test]
fn contained_capsule_endpoint_permutation_is_bit_deterministic() {
    let forward = capsule(
        2,
        -0.9 * Vec3::X,
        0.9 * Vec3::X,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let reversed = capsule(
        2,
        0.9 * Vec3::X,
        -0.9 * Vec3::X,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let proxy = proxy(
        8,
        Pose::IDENTITY,
        Vec3::ZERO,
        AnalyticShape::RoundedBox {
            half_extents: Vec3::ONE,
            radius: 0.1,
        },
    );

    let forward_contact = capsule_analytic_contact(&forward, &proxy, 0.125)
        .unwrap()
        .unwrap();
    let reversed_contact = capsule_analytic_contact(&reversed, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &reversed_contact,
        Vec3::new(-0.9, 0.1, 0.0),
        Vec3::new(-0.9, -1.1, 0.0),
        Vec3::Y,
        -1.2,
        false,
    );
    assert_eq!(
        contact_bits(&forward_contact),
        contact_bits(&reversed_contact)
    );
}

#[test]
fn asymmetric_contained_capsule_selects_the_shorter_exit() {
    let forward = capsule(
        2,
        -0.2 * Vec3::X,
        0.9 * Vec3::X,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let reversed = capsule(
        2,
        0.9 * Vec3::X,
        -0.2 * Vec3::X,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let proxy = proxy(
        8,
        Pose::IDENTITY,
        Vec3::ZERO,
        AnalyticShape::RoundedBox {
            half_extents: Vec3::new(1.0, 3.0, 3.0),
            radius: 0.1,
        },
    );

    let forward_contact = capsule_analytic_contact(&forward, &proxy, 0.125)
        .unwrap()
        .unwrap();
    let reversed_contact = capsule_analytic_contact(&reversed, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &forward_contact,
        -0.3 * Vec3::X,
        1.1 * Vec3::X,
        -Vec3::X,
        -1.4,
        false,
    );
    assert_eq!(
        contact_bits(&forward_contact),
        contact_bits(&reversed_contact)
    );
}

#[test]
fn body_diagonal_capsule_uses_cross_axis_minimum_translation() {
    let forward = capsule(
        1,
        -0.9 * Vec3::ONE,
        0.9 * Vec3::ONE,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let reversed = capsule(
        1,
        0.9 * Vec3::ONE,
        -0.9 * Vec3::ONE,
        0.1,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let proxy = proxy(
        4,
        Pose::IDENTITY,
        Vec3::ZERO,
        AnalyticShape::RoundedBox {
            half_extents: Vec3::ONE,
            radius: 0.1,
        },
    );
    let motion = Vec3::new(0.0, 1.0, -1.0).normalize();
    let normal = -motion;
    let center_capsule = -0.9 * Vec3::ONE;
    let center_proxy = Vec3::new(-0.9, 1.0, -1.0);

    let forward_contact = capsule_analytic_contact(&forward, &proxy, 0.125)
        .unwrap()
        .unwrap();
    let reversed_contact = capsule_analytic_contact(&reversed, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &forward_contact,
        center_capsule + normal * 0.1,
        center_proxy - normal * 0.1,
        normal,
        -(2.0_f32.sqrt() + 0.2),
        false,
    );
    assert_eq!(
        contact_bits(&forward_contact),
        contact_bits(&reversed_contact)
    );
}

#[test]
fn baked_nonuniform_dimensions_are_explicit_geometry() {
    let capsule = capsule_sphere(40, Vec3::ZERO, 0.25);
    let elongated_capsule = proxy(
        41,
        Pose::try_new(Vec3::new(0.0, 0.4, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::Capsule {
            local_a: Vec3::new(-1.5, 0.0, 0.0),
            local_b: Vec3::new(1.5, 0.0, 0.0),
            radius: 0.2,
        },
    );
    let nonuniform_box = proxy(
        42,
        Pose::try_new(Vec3::new(1.8, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::RoundedBox {
            half_extents: Vec3::new(1.5, 0.35, 0.2),
            radius: 0.1,
        },
    );

    let capsule_contact = capsule_analytic_contact(&capsule, &elongated_capsule, 0.125)
        .unwrap()
        .unwrap();
    let box_contact = capsule_analytic_contact(&capsule, &nonuniform_box, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &capsule_contact,
        Vec3::new(0.0, 0.25, 0.0),
        Vec3::new(0.0, 0.2, 0.0),
        Vec3::Y,
        -0.05,
        false,
    );
    assert_contact_witness(
        &box_contact,
        Vec3::new(0.25, 0.0, 0.0),
        Vec3::new(0.2, 0.0, 0.0),
        Vec3::X,
        -0.05,
        false,
    );
}

#[test]
fn reachable_positive_gap_returns_a_speculative_contact() {
    let capsule = capsule_sphere(50, Vec3::ZERO, 0.25);
    let mut proxy = sphere_proxy(51, Vec3::X, 0.25);
    proxy.linear_velocity = -4.0 * Vec3::X;

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &contact,
        Vec3::new(0.25, 0.0, 0.0),
        Vec3::new(0.75, 0.0, 0.0),
        Vec3::X,
        0.5,
        true,
    );
}

#[test]
fn moving_capsule_reaches_a_stationary_proxy_by_relative_velocity() {
    let capsule = capsule(52, Vec3::ZERO, Vec3::ZERO, 0.25, Vec3::ZERO, 4.0 * Vec3::X);
    let proxy = sphere_proxy(53, Vec3::X, 0.25);

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_contact_witness(
        &contact,
        Vec3::new(0.25, 0.0, 0.0),
        Vec3::new(0.75, 0.0, 0.0),
        Vec3::X,
        0.5,
        true,
    );
}

#[test]
fn co_moving_separated_geometry_has_no_speculative_contact() {
    let capsule = capsule(52, Vec3::ZERO, Vec3::ZERO, 0.25, Vec3::ZERO, 4.0 * Vec3::X);
    let mut proxy = sphere_proxy(53, Vec3::X, 0.25);
    proxy.linear_velocity = 4.0 * Vec3::X;

    assert!(
        capsule_analytic_contact(&capsule, &proxy, 0.125)
            .unwrap()
            .is_none()
    );
}

#[test]
fn fast_tangential_motion_has_no_speculative_contact() {
    let capsule = capsule_sphere(52, Vec3::ZERO, 0.25);
    let mut proxy = sphere_proxy(53, Vec3::X, 0.25);
    proxy.linear_velocity = 100.0 * Vec3::Y;

    assert!(
        capsule_analytic_contact(&capsule, &proxy, 0.125)
            .unwrap()
            .is_none()
    );
}

#[test]
fn unreachable_or_opening_positive_gap_has_no_contact() {
    let capsule = capsule_sphere(50, Vec3::ZERO, 0.25);
    for velocity in [Vec3::X, -3.0 * Vec3::X] {
        let mut proxy = sphere_proxy(51, Vec3::X, 0.25);
        proxy.linear_velocity = velocity;

        assert!(
            capsule_analytic_contact(&capsule, &proxy, 0.125)
                .unwrap()
                .is_none(),
            "velocity={velocity:?}"
        );
    }
}

#[test]
fn intentionally_embedded_disabled_root_has_no_contact() {
    let capsule = capsule_sphere(60, Vec3::ZERO, 0.5);
    let mut proxy = sphere_proxy(61, Vec3::ZERO, 0.25);
    proxy.collision_enabled = false;

    assert!(
        capsule_analytic_contact(&capsule, &proxy, 0.125)
            .unwrap()
            .is_none()
    );

    proxy.collision_enabled = true;
    let enabled = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();
    let repeated = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();
    assert_finite_canonical_contact(&enabled);
    assert_scalar_close(enabled.separation, -0.75);
    assert_eq!(contact_bits(&enabled), contact_bits(&repeated));
}

#[test]
fn analytic_dimensions_reject_negative_and_non_finite_values() {
    let capsule = capsule_sphere(70, Vec3::ZERO, 0.25);
    let cases = [
        (
            AnalyticShape::Sphere { radius: -0.1 },
            "analytic_sphere_radius",
            f64::from(-0.1_f32),
        ),
        (
            AnalyticShape::Sphere {
                radius: f32::INFINITY,
            },
            "analytic_sphere_radius",
            f64::INFINITY,
        ),
        (
            AnalyticShape::Capsule {
                local_a: Vec3::new(f32::NAN, 0.0, 0.0),
                local_b: Vec3::ZERO,
                radius: 0.25,
            },
            "analytic_capsule_local_a",
            f64::NAN,
        ),
        (
            AnalyticShape::Capsule {
                local_a: Vec3::ZERO,
                local_b: Vec3::new(0.0, f32::INFINITY, 0.0),
                radius: 0.25,
            },
            "analytic_capsule_local_b",
            f64::INFINITY,
        ),
        (
            AnalyticShape::Capsule {
                local_a: Vec3::ZERO,
                local_b: Vec3::X,
                radius: -0.25,
            },
            "analytic_capsule_radius",
            -0.25,
        ),
        (
            AnalyticShape::Capsule {
                local_a: Vec3::ZERO,
                local_b: Vec3::X,
                radius: f32::NAN,
            },
            "analytic_capsule_radius",
            f64::NAN,
        ),
        (
            AnalyticShape::RoundedBox {
                half_extents: Vec3::new(1.0, -0.5, 0.25),
                radius: 0.1,
            },
            "analytic_rounded_box_half_extents",
            -0.5,
        ),
        (
            AnalyticShape::RoundedBox {
                half_extents: Vec3::new(1.0, f32::INFINITY, 0.25),
                radius: 0.1,
            },
            "analytic_rounded_box_half_extents",
            f64::INFINITY,
        ),
        (
            AnalyticShape::RoundedBox {
                half_extents: Vec3::ONE,
                radius: -0.1,
            },
            "analytic_rounded_box_radius",
            f64::from(-0.1_f32),
        ),
        (
            AnalyticShape::RoundedBox {
                half_extents: Vec3::ONE,
                radius: f32::NAN,
            },
            "analytic_rounded_box_radius",
            f64::NAN,
        ),
    ];

    for (shape, expected_field, expected_value) in cases {
        let proxy = proxy(71, Pose::IDENTITY, Vec3::ZERO, shape);
        assert_invalid_value(&capsule, &proxy, 0.125, expected_field, expected_value);
    }
}

#[test]
fn non_finite_proxy_pose_and_velocity_are_rejected_precisely() {
    let capsule = capsule_sphere(80, Vec3::ZERO, 0.25);
    let invalid_pose = proxy(
        81,
        Pose {
            translation: Vec3::new(f32::INFINITY, 0.0, 0.0),
            rotation: Quat::IDENTITY,
        },
        Vec3::ZERO,
        AnalyticShape::Sphere { radius: 0.25 },
    );
    assert_invalid_value(&capsule, &invalid_pose, 0.125, "translation", f64::INFINITY);

    let invalid_rotation = proxy(
        81,
        Pose {
            translation: Vec3::ZERO,
            rotation: Quat::from_xyzw(0.0, f32::NAN, 0.0, 1.0),
        },
        Vec3::ZERO,
        AnalyticShape::Sphere { radius: 0.25 },
    );
    assert_invalid_value(&capsule, &invalid_rotation, 0.125, "rotation", f64::NAN);

    let invalid_velocity = proxy(
        81,
        Pose::IDENTITY,
        Vec3::new(0.0, f32::NAN, 0.0),
        AnalyticShape::Sphere { radius: 0.25 },
    );
    assert_invalid_value(
        &capsule,
        &invalid_velocity,
        0.125,
        "analytic_proxy_linear_velocity",
        f64::NAN,
    );
}

#[test]
fn malformed_capsule_collider_is_rejected_at_the_query_boundary() {
    let proxy = sphere_proxy(83, Vec3::X, 0.25);

    let mut invalid_pose = capsule_sphere(82, Vec3::ZERO, 0.25);
    invalid_pose.pose = Pose {
        translation: Vec3::new(0.0, f32::INFINITY, 0.0),
        rotation: Quat::IDENTITY,
    };
    assert_invalid_value(&invalid_pose, &proxy, 0.125, "translation", f64::INFINITY);

    let mut invalid_rotation = capsule_sphere(82, Vec3::ZERO, 0.25);
    invalid_rotation.pose = Pose {
        translation: Vec3::ZERO,
        rotation: Quat::from_xyzw(0.0, 0.0, 0.0, 2.0),
    };
    assert_invalid_value(
        &invalid_rotation,
        &proxy,
        0.125,
        "rotation_normalization",
        3.0,
    );

    let mut invalid_velocity = capsule_sphere(82, Vec3::ZERO, 0.25);
    invalid_velocity.linear_velocity = Vec3::new(f32::NAN, 0.0, 0.0);
    assert_invalid_value(
        &invalid_velocity,
        &proxy,
        0.125,
        "collider_linear_velocity",
        f64::NAN,
    );

    let mut invalid_shape = capsule_sphere(82, Vec3::ZERO, 0.25);
    invalid_shape.shape = Capsule {
        local_a: Vec3::ZERO,
        local_b: Vec3::ZERO,
        radius: 0.0,
        margin: 0.0,
    };
    assert_invalid_value(&invalid_shape, &proxy, 0.125, "capsule_radius", 0.0);

    let mut invalid_endpoint = capsule_sphere(82, Vec3::ZERO, 0.25);
    invalid_endpoint.shape.local_b = Vec3::new(0.0, 0.0, f32::INFINITY);
    assert_invalid_value(
        &invalid_endpoint,
        &proxy,
        0.125,
        "capsule_local_b",
        f64::INFINITY,
    );

    let mut negative_margin = capsule_sphere(82, Vec3::ZERO, 0.25);
    negative_margin.shape.margin = -0.1;
    assert_invalid_value(
        &negative_margin,
        &proxy,
        0.125,
        "capsule_margin",
        f64::from(-0.1_f32),
    );

    let mut non_finite_margin = capsule_sphere(82, Vec3::ZERO, 0.25);
    non_finite_margin.shape.margin = f32::NAN;
    assert_invalid_value(
        &non_finite_margin,
        &proxy,
        0.125,
        "capsule_margin",
        f64::NAN,
    );

    let mut overflowed_effective_radius = capsule_sphere(82, Vec3::ZERO, 0.25);
    overflowed_effective_radius.shape.radius = f32::MAX;
    overflowed_effective_radius.shape.margin = f32::MAX;
    assert_invalid_value(
        &overflowed_effective_radius,
        &proxy,
        0.125,
        "capsule_effective_radius",
        f64::INFINITY,
    );
}

#[test]
fn duplicate_collider_id_is_rejected_precisely() {
    let capsule = capsule_sphere(80, Vec3::ZERO, 0.25);
    let duplicate_id = sphere_proxy(80, Vec3::X, 0.25);

    assert_invalid_value(&capsule, &duplicate_id, 0.125, "collider_pair", 80.0);
}

#[test]
fn non_positive_or_non_finite_dt_is_rejected_precisely() {
    let capsule = capsule_sphere(80, Vec3::ZERO, 0.25);
    let valid = sphere_proxy(81, Vec3::X, 0.25);

    for dt in [0.0, -0.125, f32::NAN] {
        assert_invalid_value(&capsule, &valid, dt, "dt", f64::from(dt));
    }
}

#[test]
fn repeated_and_equivalent_queries_are_bit_deterministic() {
    let capsule_a = capsule_sphere(90, Vec3::ZERO, 0.5);
    let proxy_a = sphere_proxy(91, Vec3::new(0.75, 0.0, 0.0), 0.5);
    let capsule_b = capsule_sphere(90, Vec3::ZERO, 0.5);
    let proxy_b = proxy(
        91,
        Pose::try_new(Vec3::new(0.75, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        AnalyticShape::Sphere { radius: 0.5 },
    );

    let first = capsule_analytic_contact(&capsule_a, &proxy_a, 0.125)
        .unwrap()
        .unwrap();
    let repeated = capsule_analytic_contact(&capsule_a, &proxy_a, 0.125)
        .unwrap()
        .unwrap();
    let equivalent = capsule_analytic_contact(&capsule_b, &proxy_b, 0.125)
        .unwrap()
        .unwrap();

    assert_eq!(contact_bits(&first), contact_bits(&repeated));
    assert_eq!(contact_bits(&first), contact_bits(&equivalent));
}

#[test]
fn exact_coincidence_scale_normalizes_extreme_finite_relative_velocity() {
    let capsule = capsule_sphere(100, Vec3::ZERO, 0.5);
    let proxy = proxy(
        101,
        Pose::IDENTITY,
        Vec3::splat(0.5 * f32::MAX),
        AnalyticShape::Sphere { radius: 0.5 },
    );

    let contact = capsule_analytic_contact(&capsule, &proxy, 0.125)
        .unwrap()
        .unwrap();

    assert_finite_canonical_contact(&contact);
    assert_vec3_close(contact.normal, Vec3::ONE.normalize());
}
