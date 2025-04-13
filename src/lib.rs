pub mod attitude;
pub mod ephemeris;

pub mod ode;
pub mod types;

use polaris_fsw::actuators::types::ActuatorBus;
use polaris_fsw::sensors::types::RawSensorBus;
use polaris_log::types::PolarisLogger;

pub struct Spacecraft {
    logger: PolarisLogger,
}
impl Spacecraft {
    pub fn initialize(logger: PolarisLogger) -> Self {
        Self { logger }
    }

    pub fn initial_state(&self) -> RawSensorBus {
        RawSensorBus {}
    }

    pub fn simulate_plant(&mut self, actuator_commands: &ActuatorBus) -> RawSensorBus {
        RawSensorBus {}
    }
}
