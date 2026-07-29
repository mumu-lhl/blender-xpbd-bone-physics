use bone_physics_core::{SolverConfig, SolverWorld};
use bone_physics_py::{RegistryError, WorldRegistry};
use glam::Vec3;

fn empty_world() -> SolverWorld {
    SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: 1.0 / 120.0,
        },
        Vec::new(),
    )
    .unwrap()
}

#[test]
fn destroyed_handle_never_aliases_a_reused_slot() {
    let mut registry = WorldRegistry::new();
    let stale = registry.insert(empty_world()).unwrap();

    registry.remove(stale).unwrap();
    let replacement = registry.insert(empty_world()).unwrap();

    assert_eq!(stale.get(), 1_u64 << 32);
    assert_eq!(replacement.get(), 2_u64 << 32);
    assert_ne!(stale, replacement);
    assert_eq!(
        registry.remove(stale).unwrap_err(),
        RegistryError::StaleHandle(stale)
    );
    registry.remove(replacement).unwrap();
}

#[test]
fn destroying_the_same_handle_twice_is_rejected() {
    let mut registry = WorldRegistry::new();
    let handle = registry.insert(empty_world()).unwrap();

    registry.remove(handle).unwrap();

    assert_eq!(
        registry.remove(handle).unwrap_err(),
        RegistryError::StaleHandle(handle)
    );
}

#[test]
fn a_live_handle_resolves_only_its_own_world() {
    let mut registry = WorldRegistry::new();
    let first = registry.insert(empty_world()).unwrap();
    let second = registry.insert(empty_world()).unwrap();

    registry
        .with_world_mut(first, |world| {
            assert!(!world.is_paused());
        })
        .unwrap();
    registry.remove(first).unwrap();

    assert!(matches!(
        registry.with_world_mut(first, |_| ()),
        Err(RegistryError::StaleHandle(handle)) if handle == first
    ));
    registry.remove(second).unwrap();
}
