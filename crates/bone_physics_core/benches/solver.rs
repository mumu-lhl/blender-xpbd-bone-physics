use std::hint::black_box;

use bone_physics_core::{
    AnimationTarget, BodyId, BodyInit, BodyLookup, BoneBodyDesc, Capsule, CapsuleColliderDesc,
    ChainDesc, ColliderId, CollisionFilter, ConstraintId, ContactMaterial, CrossLinkLatticeSpec,
    CurveInterpolation, CurvePoint, ParameterCurve, PersistentConstraint, Pose, QualityPolicy,
    SolverConfig, SolverWorld, StepOptions, Velocity, build_cross_link_lattice,
};
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use glam::{Quat, Vec3};

const DT: f32 = 1.0 / 120.0;
const LENGTH: f32 = 0.25;

fn body(id: u32, translation: Vec3, dynamic: bool) -> BodyInit {
    let inverse = if dynamic { 1.0 } else { 0.0 };
    let pose = Pose::try_new(translation, Quat::IDENTITY).unwrap();
    BodyInit {
        desc: BoneBodyDesc {
            id: BodyId::new(id),
            rest_pose: pose,
            local_center_of_mass: Vec3::ZERO,
            local_head: Vec3::Y * (-0.5 * LENGTH),
            local_tail: Vec3::Y * (0.5 * LENGTH),
            inverse_mass: inverse,
            inverse_inertia: Vec3::splat(inverse),
            gravity_scale: inverse,
            linear_damping: 0.1,
            angular_damping: 0.1,
        },
        pose,
        velocity: Velocity::ZERO,
    }
}

fn anchor(id: u32, upper: u32, lower: u32) -> PersistentConstraint {
    PersistentConstraint::anchor(
        ConstraintId::new(id),
        BodyId::new(upper),
        BodyId::new(lower),
        Vec3::Y * (-0.5 * LENGTH),
        Vec3::Y * (0.5 * LENGTH),
        0.0,
        0.05,
    )
}

fn options(iterations: u32) -> StepOptions {
    StepOptions {
        position_iterations: iterations,
        contact_iterations: iterations,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

fn world(bodies: Vec<BodyInit>) -> SolverWorld {
    SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: DT,
        },
        bodies,
    )
    .unwrap()
}

fn install_fixed_animation(world: &mut SolverWorld, body_ids: impl IntoIterator<Item = u32>) {
    let targets = body_ids
        .into_iter()
        .map(|raw_id| {
            let id = BodyId::new(raw_id);
            let previous = world.body(id).unwrap().pose;
            let phase = (raw_id % 7) as f32 - 3.0;
            let current = Pose::try_new(
                previous.translation + Vec3::new(0.01 * phase, 0.0, -0.005 * phase),
                Quat::from_rotation_z(0.02 * phase) * previous.rotation,
            )
            .unwrap();
            (
                id,
                AnimationTarget {
                    previous,
                    current,
                    translation_inertia: 1.0,
                    rotation_inertia: 1.0,
                },
            )
        })
        .collect::<Vec<_>>();
    world.set_animation_targets(&targets).unwrap();
}

fn hair_64() -> SolverWorld {
    let mut bodies = vec![body(1, Vec3::ZERO, false)];
    let mut constraints = Vec::with_capacity(64);
    for segment in 1..=64_u32 {
        let id = segment + 1;
        bodies.push(body(
            id,
            Vec3::new(0.0, -(segment as f32) * LENGTH, 0.0),
            true,
        ));
        constraints.push(anchor(1000 + segment, id - 1, id));
    }
    let mut world = world(bodies);
    world.set_constraints(constraints).unwrap();
    install_fixed_animation(&mut world, 1..=65);
    world
}

fn skirt_12x8() -> SolverWorld {
    const CHAINS: u32 = 12;
    const ROWS: u32 = 8;
    let mut bodies = Vec::with_capacity((CHAINS * ROWS) as usize);
    let mut constraints = Vec::new();
    let mut chains = Vec::new();

    for chain in 0..CHAINS {
        let angle = chain as f32 * core::f32::consts::TAU / CHAINS as f32;
        let radial = Vec3::new(angle.cos(), 0.0, angle.sin()) * 0.75;
        let mut chain_bodies = Vec::new();
        for row in 0..ROWS {
            let id = 1 + chain * ROWS + row;
            chain_bodies.push(BodyId::new(id));
            bodies.push(body(id, radial - Vec3::Y * (row as f32) * LENGTH, row != 0));
            if row > 0 {
                constraints.push(anchor(1000 + id, id - 1, id));
            }
        }
        chains.push(ChainDesc {
            bodies: chain_bodies,
            rest_lengths: vec![LENGTH; ROWS as usize],
            closed: false,
        });
    }

    let lookup =
        BodyLookup::try_new(bodies.iter().map(|init| init.desc.clone()).collect()).unwrap();
    let compliance_curve = ParameterCurve::try_new(
        vec![CurvePoint::new(0.0, 1.0e-5, 0.0)],
        CurveInterpolation::Constant,
    )
    .unwrap();
    constraints.extend(
        build_cross_link_lattice(
            &CrossLinkLatticeSpec {
                first_constraint_id: ConstraintId::new(10_000),
                chains,
                samples: (0..ROWS)
                    .map(|row| row as f32 / (ROWS - 1) as f32)
                    .collect(),
                diagonals: true,
                closed: true,
                compliance_curve,
            },
            &lookup,
        )
        .unwrap(),
    );

    let mut world = world(bodies);
    world.set_constraints(constraints).unwrap();
    install_fixed_animation(&mut world, 1..=(CHAINS * ROWS));
    world
}

fn contacts_256() -> SolverWorld {
    let mut bodies = vec![body(1, Vec3::ZERO, false)];
    let mut colliders = vec![CapsuleColliderDesc {
        id: ColliderId::new(1),
        body: BodyId::new(1),
        shape: Capsule::try_new(
            Vec3::new(-40.0, 0.0, 0.0),
            Vec3::new(40.0, 0.0, 0.0),
            0.1,
            0.0,
        )
        .unwrap(),
        material: ContactMaterial::try_new(0.5, 0.0).unwrap(),
        filter: CollisionFilter::default(),
    }];

    for index in 0..256_u32 {
        let id = index + 2;
        bodies.push(body(
            id,
            Vec3::new(-38.25 + index as f32 * 0.3, 0.19, 0.0),
            true,
        ));
        colliders.push(CapsuleColliderDesc {
            id: ColliderId::new(id),
            body: BodyId::new(id),
            shape: Capsule::try_new(Vec3::ZERO, Vec3::ZERO, 0.1, 0.0).unwrap(),
            material: ContactMaterial::try_new(0.5, 0.0).unwrap(),
            filter: CollisionFilter::default(),
        });
    }

    let mut world = world(bodies);
    world.set_colliders(colliders).unwrap();
    install_fixed_animation(&mut world, 1..=257);
    world
}

fn run_steps(mut world: SolverWorld, iterations: u32) -> SolverWorld {
    for step in 1..=10 {
        let target_alpha = step as f32 / 10.0;
        black_box(
            world
                .substep(DT, target_alpha, options(iterations))
                .unwrap(),
        );
    }
    world
}

fn solver_benchmarks(criterion: &mut Criterion) {
    criterion.bench_function("hair_64", |bencher| {
        bencher.iter_batched(
            hair_64,
            |world| black_box(run_steps(world, 16)),
            BatchSize::SmallInput,
        );
    });
    criterion.bench_function("skirt_12x8", |bencher| {
        bencher.iter_batched(
            skirt_12x8,
            |world| black_box(run_steps(world, 16)),
            BatchSize::SmallInput,
        );
    });
    criterion.bench_function("contacts_256", |bencher| {
        bencher.iter_batched(
            contacts_256,
            |world| black_box(run_steps(world, 8)),
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, solver_benchmarks);
criterion_main!(benches);
