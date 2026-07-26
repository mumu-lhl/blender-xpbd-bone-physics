use bone_physics_core::{BodyId, ColliderId, ConstraintId, SolverError};

#[test]
fn body_ids_sort_by_their_serialized_value() {
    let mut ids = [BodyId::new(7), BodyId::new(2), BodyId::new(5)];

    ids.sort();

    assert_eq!(ids, [BodyId::new(2), BodyId::new(5), BodyId::new(7)]);
    assert_eq!(ids[0].get(), 2);
}

#[test]
fn entity_id_types_preserve_their_own_serialized_values() {
    assert_eq!(BodyId::new(3).get(), 3);
    assert_eq!(ConstraintId::new(4).get(), 4);
    assert_eq!(ColliderId::new(5).get(), 5);
}

#[test]
fn invalid_value_error_identifies_the_rejected_field_and_value() {
    let error = SolverError::invalid_value("radius", -1.0);
    let message = error.to_string();

    assert!(message.contains("radius"));
    assert!(message.contains("-1"));
}
