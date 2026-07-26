use std::collections::BTreeSet;

use bone_physics_core::{
    build_cross_link_lattice, build_cross_links, BodyId, BodyLookup, BoneBodyDesc, ChainDesc,
    ConstraintId, ConstraintKind, CrossLinkLatticeSpec, CrossLinkSpec, CurveInterpolation,
    CurvePoint, ParameterCurve, Pose,
};
use glam::{Quat, Vec3};

fn body(id: u32, x: f32, start_y: f32, length: f32) -> BoneBodyDesc {
    BoneBodyDesc {
        id: BodyId::new(id),
        rest_pose: Pose::try_new(Vec3::new(x, start_y, 0.0), Quat::IDENTITY).unwrap(),
        local_center_of_mass: Vec3::new(0.0, length * 0.5, 0.0),
        local_head: Vec3::ZERO,
        local_tail: Vec3::new(0.0, length, 0.0),
        inverse_mass: 1.0,
        inverse_inertia: Vec3::ONE,
        gravity_scale: 1.0,
        linear_damping: 0.0,
        angular_damping: 0.0,
    }
}

fn curve(value_a: f32, value_b: f32) -> ParameterCurve {
    ParameterCurve::try_new(
        vec![
            CurvePoint::new(0.0, value_a, 0.0),
            CurvePoint::new(1.0, value_b, 0.0),
        ],
        CurveInterpolation::Linear,
    )
    .unwrap()
}

fn lookup() -> BodyLookup {
    BodyLookup::try_new(vec![
        body(1, 0.0, 0.0, 1.0),
        body(2, 0.0, 1.0, 3.0),
        body(3, 0.0, 4.0, 2.0),
        body(4, 2.0, 0.0, 2.0),
        body(5, 2.0, 2.0, 4.0),
    ])
    .unwrap()
}

fn uneven_spec(compliance: ParameterCurve) -> CrossLinkSpec {
    CrossLinkSpec {
        first_constraint_id: ConstraintId::new(100),
        chain_a: ChainDesc {
            bodies: vec![BodyId::new(1), BodyId::new(2), BodyId::new(3)],
            rest_lengths: vec![1.0, 3.0, 2.0],
            closed: false,
        },
        chain_b: ChainDesc {
            bodies: vec![BodyId::new(4), BodyId::new(5)],
            rest_lengths: vec![2.0, 4.0],
            closed: false,
        },
        samples: vec![0.0, 0.5, 1.0],
        diagonals: true,
        compliance_curve: compliance,
    }
}

#[test]
fn unequal_chains_connect_at_matched_arc_length_not_body_index() {
    let constraints = build_cross_links(&uneven_spec(curve(1.0e-4, 3.0e-4)), &lookup()).unwrap();

    assert_eq!(constraints.len(), 7);
    assert_eq!(
        constraints
            .iter()
            .map(|constraint| constraint.id.get())
            .collect::<Vec<_>>(),
        vec![100, 101, 102, 103, 104, 105, 106]
    );

    let middle = &constraints[1];
    assert_eq!(middle.body_a, BodyId::new(2));
    assert_eq!(middle.body_b, Some(BodyId::new(5)));
    assert!((middle.compliance - 2.0e-4).abs() < 1.0e-8);
    match middle.kind.clone() {
        ConstraintKind::Distance {
            local_a,
            local_b,
            rest_length,
        } => {
            assert!((local_a - Vec3::new(0.0, 2.0, 0.0)).length() < 1.0e-6);
            assert!((local_b - Vec3::new(0.0, 1.0, 0.0)).length() < 1.0e-6);
            assert!((rest_length - 2.0).abs() < 1.0e-6);
        }
        _ => panic!("expected a concrete distance constraint"),
    }

    match constraints[3].kind.clone() {
        ConstraintKind::Distance { rest_length, .. } => {
            assert!((rest_length - 13.0_f32.sqrt()).abs() < 1.0e-6);
        }
        _ => panic!("expected a diagonal distance constraint"),
    }
}

#[test]
fn generated_links_are_unique_and_all_cycle_forming_links_are_compliant() {
    let constraints = build_cross_links(&uneven_spec(curve(1.0e-4, 3.0e-4)), &lookup()).unwrap();
    let mut keys = BTreeSet::new();

    for constraint in &constraints {
        let body_b = constraint.body_b.unwrap();
        let (local_a, local_b) = match constraint.kind.clone() {
            ConstraintKind::Distance {
                local_a, local_b, ..
            } => (local_a, local_b),
            _ => panic!("expected a distance constraint"),
        };
        let key = (
            constraint.body_a,
            body_b,
            local_a.x.to_bits(),
            local_a.y.to_bits(),
            local_a.z.to_bits(),
            local_b.x.to_bits(),
            local_b.y.to_bits(),
            local_b.z.to_bits(),
        );
        assert!(keys.insert(key), "duplicate cross-link geometry");
        assert!(constraint.compliance.is_finite());
        assert!(constraint.compliance > 0.0);
    }
}

#[test]
fn exact_internal_boundary_selects_the_following_bone() {
    let bodies = BodyLookup::try_new(vec![
        body(30, 0.0, 0.0, 0.01),
        body(31, 0.0, 0.01, 5.0),
        body(32, 2.0, 0.0, 5.01),
    ])
    .unwrap();
    let spec = CrossLinkSpec {
        first_constraint_id: ConstraintId::new(90),
        chain_a: ChainDesc {
            bodies: vec![BodyId::new(30), BodyId::new(31)],
            rest_lengths: vec![0.01, 5.0],
            closed: false,
        },
        chain_b: ChainDesc {
            bodies: vec![BodyId::new(32)],
            rest_lengths: vec![5.01],
            closed: false,
        },
        samples: vec![0.01_f32 / 5.01_f32],
        diagonals: false,
        compliance_curve: curve(0.0, 0.0),
    };

    let constraints = build_cross_links(&spec, &bodies).unwrap();

    assert_eq!(constraints.len(), 1);
    assert_eq!(constraints[0].body_a, BodyId::new(31));
    match constraints[0].kind.clone() {
        ConstraintKind::Distance { local_a, .. } => {
            assert_eq!(local_a, Vec3::ZERO);
        }
        _ => panic!("expected a distance constraint"),
    }
}

#[test]
fn mixed_compliance_breaks_a_rigid_ladder_cycle() {
    let mut spec = uneven_spec(curve(0.0, 1.0e-4));
    spec.samples = vec![0.0, 1.0];
    spec.diagonals = false;

    let constraints = build_cross_links(&spec, &lookup()).unwrap();

    assert_eq!(constraints.len(), 2);
    assert_eq!(constraints[0].compliance, 0.0);
    assert!(constraints[1].compliance > 0.0);
}

#[test]
fn cross_links_reject_self_links_and_non_finite_derived_geometry() {
    let bodies = lookup();
    let mut self_link = uneven_spec(curve(0.0, 0.0));
    self_link.chain_b = self_link.chain_a.clone();
    self_link.samples = vec![0.5];
    self_link.diagonals = false;
    assert!(build_cross_links(&self_link, &bodies).is_err());

    let extreme = BodyLookup::try_new(vec![
        body(10, f32::MAX, 0.0, f32::MAX),
        body(11, 0.0, 0.0, 1.0),
    ])
    .unwrap();
    let overflow = CrossLinkSpec {
        first_constraint_id: ConstraintId::new(500),
        chain_a: ChainDesc {
            bodies: vec![BodyId::new(10)],
            rest_lengths: vec![1.0],
            closed: false,
        },
        chain_b: ChainDesc {
            bodies: vec![BodyId::new(11)],
            rest_lengths: vec![1.0],
            closed: false,
        },
        samples: vec![1.0],
        diagonals: false,
        compliance_curve: curve(0.0, 0.0),
    };
    assert!(build_cross_links(&overflow, &extreme).is_err());
}

#[test]
fn closed_lattice_builds_one_seam_and_rejects_an_all_rigid_ring() {
    let bodies = BodyLookup::try_new(vec![
        body(20, 0.0, 0.0, 1.0),
        body(21, 1.0, 0.0, 2.0),
        body(22, 2.0, 0.0, 3.0),
        body(23, 3.0, 0.0, 4.0),
    ])
    .unwrap();
    let chains = vec![
        ChainDesc {
            bodies: vec![BodyId::new(20)],
            rest_lengths: vec![1.0],
            closed: false,
        },
        ChainDesc {
            bodies: vec![BodyId::new(21)],
            rest_lengths: vec![2.0],
            closed: false,
        },
        ChainDesc {
            bodies: vec![BodyId::new(22)],
            rest_lengths: vec![3.0],
            closed: false,
        },
        ChainDesc {
            bodies: vec![BodyId::new(23)],
            rest_lengths: vec![4.0],
            closed: false,
        },
    ];
    let lattice = CrossLinkLatticeSpec {
        first_constraint_id: ConstraintId::new(600),
        chains: chains.clone(),
        samples: vec![0.0, 1.0],
        diagonals: true,
        closed: true,
        compliance_curve: curve(1.0e-4, 2.0e-4),
    };

    let constraints = build_cross_link_lattice(&lattice, &bodies).unwrap();

    assert_eq!(constraints.len(), 16);
    assert_eq!(
        constraints
            .iter()
            .map(|constraint| constraint.id.get())
            .collect::<Vec<_>>(),
        (600..616).collect::<Vec<_>>()
    );
    let pairs = constraints
        .iter()
        .map(|constraint| {
            let a = constraint.body_a.get();
            let b = constraint.body_b.unwrap().get();
            (a.min(b), a.max(b))
        })
        .collect::<Vec<_>>();
    for expected in [(20, 21), (21, 22), (22, 23), (20, 23)] {
        assert_eq!(
            pairs.iter().filter(|pair| **pair == expected).count(),
            4,
            "pair={expected:?}"
        );
    }

    let rigid_ring = CrossLinkLatticeSpec {
        first_constraint_id: ConstraintId::new(700),
        chains,
        samples: vec![0.5],
        diagonals: false,
        closed: true,
        compliance_curve: curve(0.0, 0.0),
    };
    assert!(build_cross_link_lattice(&rigid_ring, &bodies).is_err());
}

#[test]
fn cross_link_build_rejects_invalid_references_samples_ids_and_rigid_cycles() {
    let bodies = lookup();

    let mut missing = uneven_spec(curve(1.0e-4, 1.0e-4));
    missing.chain_b.bodies[0] = BodyId::new(999);
    assert!(build_cross_links(&missing, &bodies).is_err());

    let mut non_finite = uneven_spec(curve(1.0e-4, 1.0e-4));
    non_finite.samples[1] = f32::NAN;
    assert!(build_cross_links(&non_finite, &bodies).is_err());

    let mut overflowing = uneven_spec(curve(1.0e-4, 1.0e-4));
    overflowing.first_constraint_id = ConstraintId::new(u32::MAX - 2);
    assert!(build_cross_links(&overflowing, &bodies).is_err());

    let mut rigid_cycle = uneven_spec(curve(0.0, 0.0));
    rigid_cycle.diagonals = false;
    assert!(build_cross_links(&rigid_cycle, &bodies).is_err());
}
