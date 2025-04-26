use crate::actuators::types::TruthActuatorBus;
use crate::attitude::types::{TruthAttitudeBus, TruthMultibodyBus};
use crate::ephemeris::types::TruthEphemerisBus;
use polaris_fsw::actuators::types::ActuatorBus;
use polaris_fsw::sensors::types::RawSensorBus;

#[derive(Clone, Debug, Default)]
pub struct TruthSensorBus {}
impl TruthSensorBus {
    pub fn process(
        actuator_cmd: &ActuatorBus,
        actuator_dynamics: &TruthActuatorBus,
        ephemeris_bus: &TruthEphemerisBus,
        attitude_bus: &TruthAttitudeBus,
        multibody_bud: &TruthMultibodyBus,
        prev_sensor: &Self,
    ) -> Self {
        Self {}
    }

    pub fn to_raw_bus(&self) -> RawSensorBus {
        RawSensorBus::default()
    }
}
