use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;

use bone_physics_core::{
    BodyId, BodyInit, BodyLookup, BoneBodyDesc, Capsule, CapsuleColliderDesc, ChainDesc,
    ColliderId, CollisionFilter, ConstraintId, ContactMaterial, CrossLinkLatticeSpec,
    CurveInterpolation, CurvePoint, ParameterCurve, PersistentConstraint, Pose, QualityPolicy,
    SolverConfig, SolverWorld, StepDiagnostics, StepOptions, StepPhaseTimings, Velocity,
    build_cross_link_lattice,
};
use glam::{Quat, Vec3};

const DT: f32 = 1.0 / 120.0;
const STEPS: usize = 240;
const BODY_LENGTH: f32 = 0.5;
const COLLIDER_RADIUS: f32 = 0.12;
const HARD_ANCHOR_TOLERANCE: f32 = 5.0e-5;
const PENETRATION_TOLERANCE: f32 = 0.01 * COLLIDER_RADIUS;
const COMPLIANT_JOINT_STABILITY_TOLERANCE: f32 = 0.25;

#[derive(Clone, Debug, PartialEq)]
struct ReferenceResult {
    scene: &'static str,
    max_bone_length_error: f32,
    max_joint_separation: f32,
    max_hard_anchor_separation: f32,
    max_residual_penetration: f32,
    peak_velocity: f32,
    peak_angular_velocity: f32,
    generated_contacts: u64,
    active_contacts: u64,
    final_poses: Vec<Pose>,
    phase_timings: StepPhaseTimings,
}

fn body(id: u32, translation: Vec3, dynamic: bool, velocity: Vec3) -> BodyInit {
    let inverse = if dynamic { 1.0 } else { 0.0 };
    let pose = Pose::try_new(translation, Quat::IDENTITY).unwrap();
    BodyInit {
        desc: BoneBodyDesc {
            id: BodyId::new(id),
            rest_pose: pose,
            local_center_of_mass: Vec3::ZERO,
            local_head: Vec3::Y * (-0.5 * BODY_LENGTH),
            local_tail: Vec3::Y * (0.5 * BODY_LENGTH),
            inverse_mass: inverse,
            inverse_inertia: Vec3::splat(inverse),
            gravity_scale: inverse,
            linear_damping: if dynamic { 0.25 } else { 0.0 },
            angular_damping: if dynamic { 0.25 } else { 0.0 },
        },
        pose,
        velocity: Velocity {
            linear: velocity,
            angular: Vec3::ZERO,
        },
    }
}

fn options() -> StepOptions {
    StepOptions {
        position_iterations: 64,
        contact_iterations: 64,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

fn collider(id: u32, body: u32) -> CapsuleColliderDesc {
    CapsuleColliderDesc {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(
            Vec3::Y * (-0.4 * BODY_LENGTH),
            Vec3::Y * (0.4 * BODY_LENGTH),
            COLLIDER_RADIUS,
            0.0,
        )
        .unwrap(),
        material: ContactMaterial::try_new(0.35, 0.0).unwrap(),
        filter: CollisionFilter::default(),
    }
}

fn anchor(id: u32, upper: u32, lower: u32, compliance: f32) -> PersistentConstraint {
    PersistentConstraint::anchor(
        ConstraintId::new(id),
        BodyId::new(upper),
        BodyId::new(lower),
        Vec3::Y * (-0.5 * BODY_LENGTH),
        Vec3::Y * (0.5 * BODY_LENGTH),
        compliance,
        0.05,
    )
}

fn simulate(
    scene: &'static str,
    mut world: SolverWorld,
    body_ids: &[u32],
    constraint_ids: &[u32],
    hard_constraints: &[u32],
) -> ReferenceResult {
    let mut max_bone_length_error: f32 = 0.0;
    let mut max_joint_separation: f32 = 0.0;
    let mut max_hard_anchor_separation: f32 = 0.0;
    let mut max_residual_penetration: f32 = 0.0;
    let mut peak_velocity: f32 = 0.0;
    let mut peak_angular_velocity: f32 = 0.0;
    let mut generated_contacts = 0_u64;
    let mut active_contacts = 0_u64;
    let mut phase_timings = StepPhaseTimings::default();

    for step in 0..STEPS {
        write_replay_checkpoint(scene, step);
        let (diagnostics, step_timings) = world.substep_profiled(DT, 1.0, options()).unwrap();
        accumulate_timings(&mut phase_timings, step_timings);
        assert!(!diagnostics.rolled_back, "{scene} rolled back");
        accumulate_diagnostics(
            diagnostics,
            &mut max_residual_penetration,
            &mut generated_contacts,
            &mut active_contacts,
        );

        for id in body_ids {
            let state = world.body(BodyId::new(*id)).unwrap();
            let head =
                state.pose.translation + state.pose.rotation * Vec3::Y * (-0.5 * BODY_LENGTH);
            let tail = state.pose.translation + state.pose.rotation * Vec3::Y * (0.5 * BODY_LENGTH);
            max_bone_length_error =
                max_bone_length_error.max((head.distance(tail) - BODY_LENGTH).abs());
            peak_velocity = peak_velocity.max(state.velocity.linear.length());
            peak_angular_velocity = peak_angular_velocity.max(state.velocity.angular.length());
        }
        for constraint in constraint_ids {
            let error = world.anchor_error(ConstraintId::new(*constraint)).unwrap();
            max_joint_separation = max_joint_separation.max(error);
        }
        for constraint in hard_constraints {
            let error = world.anchor_error(ConstraintId::new(*constraint)).unwrap();
            max_hard_anchor_separation = max_hard_anchor_separation.max(error);
        }
    }

    let final_poses = body_ids
        .iter()
        .map(|id| world.body(BodyId::new(*id)).unwrap().pose)
        .collect();
    ReferenceResult {
        scene,
        max_bone_length_error,
        max_joint_separation,
        max_hard_anchor_separation,
        max_residual_penetration,
        peak_velocity,
        peak_angular_velocity,
        generated_contacts,
        active_contacts,
        final_poses,
        phase_timings,
    }
}

fn accumulate_timings(total: &mut StepPhaseTimings, step: StepPhaseTimings) {
    total.prediction_seconds += step.prediction_seconds;
    total.animation_inertia_seconds += step.animation_inertia_seconds;
    total.contact_generation_seconds += step.contact_generation_seconds;
    total.position_solve_seconds += step.position_solve_seconds;
    total.diagnostics_seconds += step.diagnostics_seconds;
    total.velocity_seconds += step.velocity_seconds;
}

fn accumulate_diagnostics(
    diagnostics: StepDiagnostics,
    max_penetration: &mut f32,
    generated_contacts: &mut u64,
    active_contacts: &mut u64,
) {
    assert!(diagnostics.deepest_penetration.is_finite());
    assert!(diagnostics.max_constraint_error.is_finite());
    *max_penetration = max_penetration.max(diagnostics.deepest_penetration);
    *generated_contacts += u64::from(diagnostics.generated_contacts);
    *active_contacts += u64::from(diagnostics.active_contacts);
}

fn pendulum() -> ReferenceResult {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: DT,
        },
        vec![
            body(1, Vec3::ZERO, false, Vec3::ZERO),
            body(2, Vec3::new(0.0, -BODY_LENGTH, 0.0), true, Vec3::X * 1.5),
        ],
    )
    .unwrap();
    world.set_constraints(vec![anchor(1, 1, 2, 0.0)]).unwrap();
    simulate("single_pendulum", world, &[1, 2], &[1], &[1])
}

fn colliding_hair_chains() -> ReferenceResult {
    let mut bodies = Vec::new();
    let mut constraints = Vec::new();
    let mut colliders = Vec::new();
    let mut body_ids = Vec::new();
    let mut hard_constraints = Vec::new();

    for (chain, x, velocity) in [(0_u32, -0.13, Vec3::X * 0.6), (1_u32, 0.13, Vec3::X * -0.6)] {
        let root = 10 + chain * 10;
        bodies.push(body(root, Vec3::new(x, 0.0, 0.0), false, Vec3::ZERO));
        body_ids.push(root);
        for segment in 1..=3 {
            let id = root + segment;
            bodies.push(body(
                id,
                Vec3::new(x, -(segment as f32) * BODY_LENGTH, 0.0),
                true,
                velocity,
            ));
            body_ids.push(id);
            colliders.push(collider(100 + id, id));
            let constraint = 1000 + chain * 10 + segment;
            constraints.push(anchor(constraint, id - 1, id, 0.0));
            hard_constraints.push(constraint);
        }
    }

    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: DT,
        },
        bodies,
    )
    .unwrap();
    world.set_constraints(constraints).unwrap();
    world.set_colliders(colliders).unwrap();
    simulate(
        "colliding_hair_chains",
        world,
        &body_ids,
        &hard_constraints,
        &hard_constraints,
    )
}

fn root_stiff_tip_soft_tail() -> ReferenceResult {
    let mut bodies = vec![body(30, Vec3::ZERO, false, Vec3::ZERO)];
    let mut constraints = Vec::new();
    let mut body_ids = vec![30];
    for segment in 1..=5 {
        let id = 30 + segment;
        bodies.push(body(
            id,
            Vec3::new(0.0, -(segment as f32) * BODY_LENGTH, 0.0),
            true,
            if segment == 5 {
                Vec3::X * 1.5
            } else {
                Vec3::ZERO
            },
        ));
        body_ids.push(id);
        let compliance = match segment {
            1 => 0.0,
            2 => 1.0e-7,
            3 => 1.0e-6,
            4 => 1.0e-5,
            _ => 1.0e-4,
        };
        constraints.push(anchor(2000 + segment, id - 1, id, compliance));
    }
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: DT,
        },
        bodies,
    )
    .unwrap();
    world.set_constraints(constraints).unwrap();
    let constraint_ids = (2001..=2005).collect::<Vec<_>>();
    simulate(
        "root_stiff_tip_soft_tail",
        world,
        &body_ids,
        &constraint_ids,
        &[2001],
    )
}

fn closed_skirt_lattice() -> ReferenceResult {
    let corners = [
        Vec3::new(-0.45, 0.0, -0.45),
        Vec3::new(0.45, 0.0, -0.45),
        Vec3::new(0.45, 0.0, 0.45),
        Vec3::new(-0.45, 0.0, 0.45),
    ];
    let mut bodies = Vec::new();
    let mut constraints = Vec::new();
    let mut colliders = Vec::new();
    let mut body_ids = Vec::new();
    let mut hard_constraints = Vec::new();

    for (index, corner) in corners.into_iter().enumerate() {
        let root = 50 + (index as u32) * 10;
        let tip = root + 1;
        bodies.push(body(root, corner, false, Vec3::ZERO));
        bodies.push(body(
            tip,
            corner - Vec3::Y * BODY_LENGTH,
            true,
            Vec3::new(-corner.z, 0.0, corner.x) * 0.75,
        ));
        constraints.push(anchor(3000 + index as u32, root, tip, 0.0));
        colliders.push(collider(4000 + index as u32, tip));
        body_ids.extend([root, tip]);
        hard_constraints.push(3000 + index as u32);
    }

    let body_lookup =
        BodyLookup::try_new(bodies.iter().map(|init| init.desc.clone()).collect()).unwrap();
    let chains = (0..4_u32)
        .map(|index| ChainDesc {
            bodies: vec![BodyId::new(50 + index * 10), BodyId::new(51 + index * 10)],
            rest_lengths: vec![BODY_LENGTH, BODY_LENGTH],
            closed: false,
        })
        .collect();
    let compliance_curve = ParameterCurve::try_new(
        vec![CurvePoint::new(0.0, 2.0e-5, 0.0)],
        CurveInterpolation::Constant,
    )
    .unwrap();
    constraints.extend(
        build_cross_link_lattice(
            &CrossLinkLatticeSpec {
                first_constraint_id: ConstraintId::new(3100),
                chains,
                samples: vec![0.0, 0.5, 1.0],
                diagonals: true,
                closed: true,
                compliance_curve,
            },
            &body_lookup,
        )
        .unwrap(),
    );
    let constraint_ids = constraints
        .iter()
        .map(|constraint| constraint.id.get())
        .collect::<Vec<_>>();

    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: DT,
        },
        bodies,
    )
    .unwrap();
    world.set_constraints(constraints).unwrap();
    world.set_colliders(colliders).unwrap();
    simulate(
        "closed_skirt_lattice",
        world,
        &body_ids,
        &constraint_ids,
        &hard_constraints,
    )
}

fn run_references() -> Vec<ReferenceResult> {
    vec![
        pendulum(),
        colliding_hair_chains(),
        root_stiff_tip_soft_tail(),
        closed_skirt_lattice(),
    ]
}

fn report_path() -> PathBuf {
    std::env::var_os("BONE_PHYSICS_REFERENCE_OUTPUT").map_or_else(
        || {
            std::env::temp_dir().join(format!(
                "bone-physics-reference-{}.json",
                std::process::id()
            ))
        },
        PathBuf::from,
    )
}

fn write_replay_checkpoint(scene: &str, next_step: usize) {
    let Some(report_path) = std::env::var_os("BONE_PHYSICS_REFERENCE_OUTPUT") else {
        return;
    };
    let report_path = PathBuf::from(report_path);
    let path = report_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join("rust-reference-replay.json");
    let checkpoint = format!(
        concat!(
            "{{\"fixture_version\":1,\"scene\":\"{}\",\"next_step\":{},",
            "\"fixed_dt\":{},\"target_alpha\":1.0,",
            "\"position_iterations\":64,\"contact_iterations\":64}}\n"
        ),
        scene, next_step, DT,
    );
    fs::write(path, checkpoint).unwrap();
}

fn write_report(results: &[ReferenceResult]) -> PathBuf {
    let mut json = String::from("{\"status\":\"passed\",\"scenes\":[\n");
    for (index, result) in results.iter().enumerate() {
        let comma = if index + 1 == results.len() { "" } else { "," };
        writeln!(
            json,
            concat!(
                "  {{\"scene\":\"{}\",\"max_bone_length_error\":{},",
                "\"max_joint_separation\":{},\"max_hard_anchor_separation\":{},",
                "\"max_residual_penetration\":{},",
                "\"peak_velocity\":{},\"peak_angular_velocity\":{},",
                "\"generated_contacts\":{},\"active_contacts\":{},",
                "\"phase_timings\":{{\"prediction_seconds\":{},",
                "\"animation_inertia_seconds\":{},\"contact_generation_seconds\":{},",
                "\"position_solve_seconds\":{},\"diagnostics_seconds\":{},",
                "\"velocity_seconds\":{}}}}}{}"
            ),
            result.scene,
            result.max_bone_length_error,
            result.max_joint_separation,
            result.max_hard_anchor_separation,
            result.max_residual_penetration,
            result.peak_velocity,
            result.peak_angular_velocity,
            result.generated_contacts,
            result.active_contacts,
            result.phase_timings.prediction_seconds,
            result.phase_timings.animation_inertia_seconds,
            result.phase_timings.contact_generation_seconds,
            result.phase_timings.position_solve_seconds,
            result.phase_timings.diagnostics_seconds,
            result.phase_timings.velocity_seconds,
            comma,
        )
        .unwrap();
    }
    json.push_str(
        "],\"failure\":null,\"replay\":{\"fixture_version\":1,\
         \"fixed_dt\":0.008333334,\"steps\":240}}\n",
    );

    let path = report_path();
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).unwrap();
    }
    fs::write(&path, json).unwrap();
    path
}

#[test]
fn authoritative_reference_scenes_are_stable_and_within_tolerance() {
    let first = run_references();
    let second = run_references();

    for (left, right) in first.iter().zip(&second) {
        assert_eq!(left.scene, right.scene);
        assert_eq!(
            left.final_poses, right.final_poses,
            "{} changed",
            left.scene
        );
        assert!(
            left.max_bone_length_error <= 1.0e-5 * BODY_LENGTH,
            "{} bone length error {} exceeded {}",
            left.scene,
            left.max_bone_length_error,
            1.0e-5 * BODY_LENGTH,
        );
        assert!(
            left.max_hard_anchor_separation <= HARD_ANCHOR_TOLERANCE,
            "{} hard anchor separation {} exceeded {}",
            left.scene,
            left.max_hard_anchor_separation,
            HARD_ANCHOR_TOLERANCE,
        );
        assert!(
            left.max_joint_separation <= COMPLIANT_JOINT_STABILITY_TOLERANCE,
            "{} compliant joint separation {} exceeded {}",
            left.scene,
            left.max_joint_separation,
            COMPLIANT_JOINT_STABILITY_TOLERANCE,
        );
        assert!(
            left.max_residual_penetration <= PENETRATION_TOLERANCE,
            "{} penetration {} exceeded {}",
            left.scene,
            left.max_residual_penetration,
            PENETRATION_TOLERANCE,
        );
        assert!(left.peak_velocity.is_finite());
        assert!(left.peak_angular_velocity.is_finite());
    }
    assert!(
        first
            .iter()
            .find(|result| result.scene == "colliding_hair_chains")
            .unwrap()
            .generated_contacts
            > 0
    );

    let path = write_report(&first);
    assert!(fs::metadata(&path).unwrap().len() > 2);
    println!("reference report: {}", path.display());
}
