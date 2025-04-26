use altai_rs::meta::types::Vector3;
use ndarray::array;
use polaris_fsw::actuators::types::ActuatorBus;

#[derive(Clone, Debug)]
pub struct TruthActuatorBus {
    pub net_forces: Vector3,
    pub net_torques: Vector3,
}

impl Default for TruthActuatorBus {
    fn default() -> Self {
        Self {
            net_forces: array![0., 0., 0.].into_shape_with_order((3, 1)).unwrap(),
            net_torques: array![0., 0., 0.].into_shape_with_order((3, 1)).unwrap(),
        }
    }
}

impl TruthActuatorBus {
    pub fn process(actuator_cmd: &ActuatorBus, prev_actuator: &Self) -> Self {
        Self::default()
    }
}
