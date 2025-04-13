#[derive(Clone)]
pub struct SpacecraftState {
    pub truth_ephemeris: u16,
    pub truth_attitude: u16,
    pub truth_dynamics: u16,
}

impl Default for SpacecraftState {
    fn default() -> Self {
        Self {
            truth_attitude: 0,
            truth_dynamics: 0,
            truth_ephemeris: 0,
        }
    }
}

pub struct TruthBus {}

impl TruthBus {
    pub fn initialize() -> Self {
        Self {}
    }
}

pub struct SimBus {}
