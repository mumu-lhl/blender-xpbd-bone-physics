use bone_physics_native::{
    BODY_COLUMNS, BatchValidationError, reject_duplicates, validate_offsets, validate_rows,
};

#[test]
fn row_layout_rejects_missing_columns_and_ragged_values() {
    assert_eq!(
        validate_rows(BODY_COLUMNS, BODY_COLUMNS - 1, BODY_COLUMNS, "bodies"),
        Err(BatchValidationError::Columns {
            name: "bodies".to_owned(),
            expected: BODY_COLUMNS,
            actual: BODY_COLUMNS - 1,
        })
    );
    assert_eq!(
        validate_rows(BODY_COLUMNS + 1, BODY_COLUMNS, BODY_COLUMNS, "bodies"),
        Err(BatchValidationError::Ragged {
            name: "bodies".to_owned(),
            values: BODY_COLUMNS + 1,
            columns: BODY_COLUMNS,
        })
    );
}

#[test]
fn valid_row_layout_reports_the_owned_copy_row_count() {
    assert_eq!(
        validate_rows(BODY_COLUMNS * 2, BODY_COLUMNS, BODY_COLUMNS, "bodies"),
        Ok(2)
    );
}

#[test]
fn compressed_offsets_must_cover_the_exact_item_range() {
    assert_eq!(validate_offsets("offsets", &[0, 1, 1, 3], 3), Ok(()));
    assert!(matches!(
        validate_offsets("offsets", &[0, 2, 1, 3], 3),
        Err(BatchValidationError::Offsets { .. })
    ));
    assert!(matches!(
        validate_offsets("offsets", &[0, 1, 2], 3),
        Err(BatchValidationError::Offsets { .. })
    ));
}

#[test]
fn duplicate_ids_are_rejected_independent_of_input_order() {
    assert_eq!(reject_duplicates("body", &[7, 1, 9]), Ok(()));
    assert_eq!(
        reject_duplicates("body", &[7, 1, 7]),
        Err(BatchValidationError::DuplicateId {
            kind: "body".to_owned(),
            id: 7,
        })
    );
}
