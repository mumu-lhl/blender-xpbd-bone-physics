use bone_physics_core::{
    BodyId, Capsule, CapsuleCollider, ColliderId, CollisionFilter, CollisionPipeline, ConstraintId,
    PersistentConstraint, Pose,
};
use glam::{Quat, Vec3};

fn collider(id: u32, body: u32) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(Vec3::ZERO, Vec3::Y, 0.25, 0.0).unwrap(),
        pose: Pose::try_new(Vec3::ZERO, Quat::IDENTITY).unwrap(),
        linear_velocity: Vec3::ZERO,
        filter: CollisionFilter::default(),
    }
}

fn hard_anchor(id: u32, body_a: u32, body_b: u32) -> PersistentConstraint {
    PersistentConstraint::anchor(
        ConstraintId::new(id),
        BodyId::new(body_a),
        BodyId::new(body_b),
        Vec3::ZERO,
        Vec3::ZERO,
        0.0,
        0.0,
    )
}

#[test]
fn self_pairs_same_body_layers_and_masks_are_filtered_symmetrically() {
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();
    let mut a = collider(1, 1);
    let mut b = collider(2, 2);
    let same_body = collider(3, 1);

    assert!(!pipeline.allows_pair(&a, &a).unwrap());
    assert!(!pipeline.allows_pair(&a, &same_body).unwrap());

    a.filter.layer = 0b0001;
    a.filter.mask = 0b0010;
    b.filter.layer = 0b0010;
    b.filter.mask = 0b0001;
    assert!(pipeline.allows_pair(&a, &b).unwrap());

    b.filter.mask = 0b0100;
    assert!(!pipeline.allows_pair(&a, &b).unwrap());
    assert!(!pipeline.allows_pair(&b, &a).unwrap());
}

#[test]
fn explicit_parent_and_structural_neighbor_exclusions_are_bidirectional() {
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();
    let mut parent = collider(10, 10);
    let child = collider(11, 11);
    parent.filter.excluded_bodies.insert(child.body);

    assert!(!pipeline.allows_pair(&parent, &child).unwrap());
    assert!(!pipeline.allows_pair(&child, &parent).unwrap());

    parent.filter.excluded_bodies.clear();
    assert!(pipeline.allows_pair(&parent, &child).unwrap());
}

#[test]
fn hard_joint_exclusion_is_direct_not_transitive_along_a_chain() {
    let pipeline =
        CollisionPipeline::try_new(&[hard_anchor(20, 1, 2), hard_anchor(21, 2, 3)]).unwrap();
    let first = collider(20, 1);
    let middle = collider(21, 2);
    let last = collider(22, 3);

    assert!(!pipeline.allows_pair(&first, &middle).unwrap());
    assert!(!pipeline.allows_pair(&middle, &last).unwrap());
    assert!(pipeline.allows_pair(&first, &last).unwrap());
}

#[test]
fn configured_constraint_exclusion_can_filter_a_compliant_link() {
    let compliant = PersistentConstraint::distance(
        ConstraintId::new(30),
        BodyId::new(1),
        BodyId::new(2),
        Vec3::ZERO,
        Vec3::ZERO,
        1.0,
        1.0e-4,
        0.0,
    );
    let pipeline = CollisionPipeline::try_new(&[compliant]).unwrap();
    let mut a = collider(30, 1);
    let b = collider(31, 2);

    assert!(pipeline.allows_pair(&a, &b).unwrap());
    a.filter.excluded_constraints.insert(ConstraintId::new(30));
    assert!(!pipeline.allows_pair(&a, &b).unwrap());
    assert!(!pipeline.allows_pair(&b, &a).unwrap());
}

#[test]
fn filter_pipeline_rejects_duplicate_constraint_ids() {
    let duplicate = vec![hard_anchor(40, 1, 2), hard_anchor(40, 3, 4)];
    assert!(CollisionPipeline::try_new(&duplicate).is_err());
}
