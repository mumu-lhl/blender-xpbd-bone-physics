use bone_physics_core::SolverWorld;

const FIRST_GENERATION: u32 = 1;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct WorldHandle(u64);

impl WorldHandle {
    #[must_use]
    pub const fn from_raw(raw: u64) -> Self {
        Self(raw)
    }

    #[must_use]
    pub const fn get(self) -> u64 {
        self.0
    }

    const fn from_parts(slot: u32, generation: u32) -> Self {
        Self(((generation as u64) << 32) | slot as u64)
    }

    const fn slot(self) -> u32 {
        self.0 as u32
    }

    const fn generation(self) -> u32 {
        (self.0 >> 32) as u32
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, thiserror::Error)]
pub enum RegistryError {
    #[error("world handle {0:?} is stale or does not exist")]
    StaleHandle(WorldHandle),

    #[error("world registry exhausted its stable handle space")]
    CapacityExhausted,
}

struct Slot {
    generation: u32,
    reusable: bool,
    world: Option<SolverWorld>,
}

pub struct WorldRegistry {
    slots: Vec<Slot>,
}

impl WorldRegistry {
    #[must_use]
    pub const fn new() -> Self {
        Self { slots: Vec::new() }
    }

    pub fn insert(&mut self, world: SolverWorld) -> Result<WorldHandle, RegistryError> {
        if let Some((index, slot)) = self
            .slots
            .iter_mut()
            .enumerate()
            .find(|(_, slot)| slot.reusable && slot.world.is_none())
        {
            slot.world = Some(world);
            return Ok(WorldHandle::from_parts(
                u32::try_from(index).map_err(|_| RegistryError::CapacityExhausted)?,
                slot.generation,
            ));
        }

        let index =
            u32::try_from(self.slots.len()).map_err(|_| RegistryError::CapacityExhausted)?;
        self.slots.push(Slot {
            generation: FIRST_GENERATION,
            reusable: true,
            world: Some(world),
        });
        Ok(WorldHandle::from_parts(index, FIRST_GENERATION))
    }

    pub fn remove(&mut self, handle: WorldHandle) -> Result<SolverWorld, RegistryError> {
        let slot = self.live_slot_mut(handle)?;
        let next_generation = slot.generation.checked_add(1);
        let world = slot
            .world
            .take()
            .ok_or(RegistryError::StaleHandle(handle))?;
        if let Some(generation) = next_generation {
            slot.generation = generation;
        } else {
            slot.reusable = false;
        }
        Ok(world)
    }

    pub fn with_world_mut<T>(
        &mut self,
        handle: WorldHandle,
        operation: impl FnOnce(&mut SolverWorld) -> T,
    ) -> Result<T, RegistryError> {
        let slot = self.live_slot_mut(handle)?;
        let world = slot
            .world
            .as_mut()
            .ok_or(RegistryError::StaleHandle(handle))?;
        Ok(operation(world))
    }

    fn live_slot_mut(&mut self, handle: WorldHandle) -> Result<&mut Slot, RegistryError> {
        let slot = self
            .slots
            .get_mut(handle.slot() as usize)
            .ok_or(RegistryError::StaleHandle(handle))?;
        if slot.generation != handle.generation() || slot.world.is_none() {
            return Err(RegistryError::StaleHandle(handle));
        }
        Ok(slot)
    }
}

impl Default for WorldRegistry {
    fn default() -> Self {
        Self::new()
    }
}
