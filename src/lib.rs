pub mod attitude;
pub mod ephemeris;

pub mod actuators;
pub mod ode;
pub mod sc_types;
pub mod sensors;

use actuators::types::TruthActuatorBus;
use attitude::types::{TruthAttitudeBus, TruthAttitudeSignal, TruthMultibodyBus};
use ephemeris::types::TruthEphemerisBus;
use polaris_fsw::actuators::types::ActuatorBus;
use polaris_fsw::sensors::types::RawSensorBus;
use sc_types::{SpacecraftParamBus, SpacecraftState};

use log;
use sensors::types::TruthSensorBus;

#[derive(Clone, Debug)]
pub struct Spacecraft {
    pub sim_time: f64,
    pub ts: f64,
    pub sc_param_bus: SpacecraftParamBus,
    pub prev_sc_state: SpacecraftState,
    pub curr_sc_state: SpacecraftState,
}
impl Spacecraft {
    pub fn initialize(SC_Ts: f64, param_bus: SpacecraftParamBus) -> Self {
        log::trace!("Initializing Plant");

        log::trace!("Initializing Attitude Bus");
        let att_bus = TruthAttitudeBus::initialize(SC_Ts, param_bus.sc_attitude.clone());

        // Initialize Params
        Self {
            sim_time: 0.,
            ts: SC_Ts,
            sc_param_bus: param_bus,
            prev_sc_state: SpacecraftState::initialize(SC_Ts, None, None, None, None, None),
            curr_sc_state: SpacecraftState::initialize(
                SC_Ts,
                None,
                None,
                Some(att_bus),
                None,
                None,
            ),
        }
    }

    pub fn initial_state(&self) -> RawSensorBus {
        RawSensorBus {}
    }

    pub fn simulate_plant(&mut self, actuator_commands: &ActuatorBus) -> RawSensorBus {
        log::trace!("Running GNC Plant Loop");

        // Update prev/curr
        std::mem::swap(&mut self.curr_sc_state, &mut self.prev_sc_state);

        // Read Actuators
        self.curr_sc_state.truth_actuator_bus = TruthActuatorBus::process(
            // Curr State
            actuator_commands,
            // Prev State
            &self.prev_sc_state.truth_actuator_bus,
        );

        // Update Dynamics
        // // Update Ephemeris Dynamics
        self.curr_sc_state.truth_ephemeris.process(
            // Curr State
            &self.curr_sc_state.truth_actuator_bus,
            // Prev State
            &self.prev_sc_state.truth_ephemeris,
        );

        // // Update Attitude Dynamics
        self.curr_sc_state.truth_attitude.process(
            // Current State
            &self.curr_sc_state.truth_actuator_bus,
            // Prev State
            &self.prev_sc_state.truth_attitude,
        );

        // // Update Multibody Dynamics
        self.curr_sc_state.truth_multibody = TruthMultibodyBus::process(
            // Curr State
            &self.curr_sc_state.truth_actuator_bus,
            // Prev State
            &self.prev_sc_state.truth_multibody,
        );

        // Simulate Sensor Data
        self.curr_sc_state.truth_sensor_bus = TruthSensorBus::process(
            // Curr State
            actuator_commands,
            &self.curr_sc_state.truth_actuator_bus,
            &self.curr_sc_state.truth_ephemeris,
            &self.curr_sc_state.truth_attitude,
            &self.curr_sc_state.truth_multibody,
            // Prev State
            &self.prev_sc_state.truth_sensor_bus,
        );

        self.sim_time += self.ts;

        // Send RawSensorBus
        self.curr_sc_state.truth_sensor_bus.to_raw_bus()
    }
}
