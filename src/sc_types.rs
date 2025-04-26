use crate::{
    actuators::types::TruthActuatorBus,
    attitude::types::{TruthAttitudeBus, TruthMultibodyBus},
    ephemeris::{consts, types::TruthEphemerisBus},
    sensors::types::TruthSensorBus,
};
use ndarray::array;

use altai_rs::meta::types::{Quaternion4, Vector3};

#[derive(Clone, Debug)]
pub struct SpacecraftState {
    pub truth_actuator_bus: TruthActuatorBus,
    pub truth_ephemeris: TruthEphemerisBus,
    pub truth_attitude: TruthAttitudeBus,
    pub truth_multibody: TruthMultibodyBus,
    pub truth_sensor_bus: TruthSensorBus,
}
impl SpacecraftState {
    pub fn initialize(
        _SC_Ts: f64,
        initial_actuator: Option<TruthActuatorBus>,
        initial_ephem: Option<TruthEphemerisBus>,
        initial_attitude: Option<TruthAttitudeBus>,
        initial_multibody: Option<TruthMultibodyBus>,
        initial_sensor: Option<TruthSensorBus>,
    ) -> Self {
        Self {
            truth_actuator_bus: initial_actuator.unwrap_or(TruthActuatorBus::default()),
            truth_ephemeris: initial_ephem.unwrap_or(TruthEphemerisBus::default()),
            truth_attitude: initial_attitude.unwrap_or(TruthAttitudeBus::default()),
            truth_multibody: initial_multibody.unwrap_or(TruthMultibodyBus::default()),
            truth_sensor_bus: initial_sensor.unwrap_or(TruthSensorBus::default()),
        }
    }
}

impl Default for SpacecraftState {
    fn default() -> Self {
        Self {
            truth_actuator_bus: TruthActuatorBus::default(),
            truth_ephemeris: TruthEphemerisBus::default(),
            truth_attitude: TruthAttitudeBus::default(),
            truth_multibody: TruthMultibodyBus::default(),
            truth_sensor_bus: TruthSensorBus::default(),
        }
    }
}

pub trait SpacecraftParam {}
#[derive(Clone, Default, Debug)]
pub struct SpacecraftParamBus {
    pub sc_actuators: SpacecraftActuatorArchitecture,
    pub sc_ephemeris: SpacecraftEphemerisArchitecture,
    pub sc_attitude: SpacecraftAttitudeArchitecture,
    pub sc_multibody: SpacecraftMultibodyArchitecture,
    pub sc_sensors: SpacecraftSensorArchitecture,
}

impl SpacecraftParamBus {
    pub fn initialize(
        sc_actuators: SpacecraftActuatorArchitecture,
        sc_ephemeris: SpacecraftEphemerisArchitecture,
        sc_attitude: SpacecraftAttitudeArchitecture,
        sc_multibody: SpacecraftMultibodyArchitecture,
        sc_sensors: SpacecraftSensorArchitecture,
    ) -> Self {
        Self {
            sc_actuators,
            sc_ephemeris,
            sc_attitude,
            sc_multibody,
            sc_sensors,
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct SpacecraftActuatorArchitecture {}
impl SpacecraftParam for SpacecraftActuatorArchitecture {}

#[derive(Clone, Debug)]
pub struct SpacecraftEphemerisArchitecture {
    pub r_sc_eci: Vector3,
    pub v_sc_eci: Vector3,
}
impl SpacecraftParam for SpacecraftEphemerisArchitecture {}
impl SpacecraftEphemerisArchitecture {
    pub fn initialize(r_sc: Vector3, v_sc: Vector3) -> Self {
        Self {
            r_sc_eci: r_sc,
            v_sc_eci: v_sc,
        }
    }
}
impl Default for SpacecraftEphemerisArchitecture {
    fn default() -> Self {
        let a_sc = consts::RE + 500e3;
        Self {
            r_sc_eci: array![[a_sc], [0.], [0.]],
            v_sc_eci: array![[0.], [(consts::MU / a_sc).sqrt()], [0.]],
        }
    }
}

#[derive(Clone, Debug)]
pub struct SpacecraftAttitudeArchitecture {
    pub q_sc_eci: Quaternion4,
    pub omega_sc: Vector3,
    pub alpha_sc: Vector3,
}
impl SpacecraftParam for SpacecraftAttitudeArchitecture {}
impl SpacecraftAttitudeArchitecture {
    pub fn initialize(q_sc_eci0: Quaternion4, omega_sc0: Vector3) -> Self {
        Self {
            q_sc_eci: q_sc_eci0,
            omega_sc: omega_sc0,
            ..Default::default()
        }
    }
}
impl Default for SpacecraftAttitudeArchitecture {
    fn default() -> Self {
        Self {
            q_sc_eci: array![[0.], [0.], [0.], [1.]],
            omega_sc: array![[0.], [0.], [0.]],
            alpha_sc: array![[0.], [0.], [0.]],
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct SpacecraftMultibodyArchitecture {
    pub j_multibody: Vector3,
}
impl SpacecraftParam for SpacecraftMultibodyArchitecture {}

impl SpacecraftMultibodyArchitecture {
    pub fn initialize(j_multibody: Vector3) -> Self {
        Self { j_multibody }
    }
}

#[derive(Clone, Default, Debug)]
pub struct SpacecraftSensorArchitecture {}
impl SpacecraftParam for SpacecraftSensorArchitecture {}
