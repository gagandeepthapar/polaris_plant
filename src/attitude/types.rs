use std::f64::consts::PI;

use crate::attitude::kinedynamics;
use crate::ode::{self, RK2, RK5};
use crate::sc_types::SpacecraftAttitudeArchitecture;
use crate::{actuators::types::TruthActuatorBus, ode::Integrator};

use altai_rs::meta::types::{Generic1D, Generic2D, Quaternion4, Vector3};
use ndarray::{array, concatenate, s, Axis};

#[derive(Clone, Debug)]
pub struct TruthAttitudeSignal {
    pub q_sc_eci: Quaternion4,
    pub omega_sc: Vector3,
}
impl Default for TruthAttitudeSignal {
    fn default() -> Self {
        Self {
            q_sc_eci: array![0., 0., 0., 1.,]
                .into_shape_with_order((4, 1))
                .unwrap(),
            omega_sc: (array![0., 0., 0.,]).into_shape_with_order((3, 1)).unwrap(),
        }
    }
}

impl TruthAttitudeSignal {
    pub fn initialize(q_sc_eci0: Quaternion4, omega_sc0: Vector3) -> Self {
        Self {
            q_sc_eci: q_sc_eci0,
            omega_sc: omega_sc0,
        }
    }

    pub fn to_state_vector(&self) -> Generic1D {
        Generic1D::from_iter(
            self.q_sc_eci
                .to_owned()
                .remove_axis(Axis(1))
                .iter()
                .chain(self.omega_sc.to_owned().remove_axis(Axis(1)).iter())
                .map(|&x| x),
        )
    }

    pub fn from_state_vector(&mut self, state_vec: Generic1D) {
        state_vec
            .slice(s![0..4])
            .assign_to(self.q_sc_eci.slice_mut(s![0..4, 0]));
        state_vec
            .slice(s![4..7])
            .assign_to(self.omega_sc.slice_mut(s![0..3, 0]));
    }
}

#[derive(Clone, Debug)]
pub struct TruthAttitudeBus {
    pub signal: TruthAttitudeSignal,
    integrator: ode::RK5,
}

impl Default for TruthAttitudeBus {
    fn default() -> Self {
        Self {
            signal: TruthAttitudeSignal::default(),
            integrator: RK5(0.1),
        }
    }
}

impl TruthAttitudeBus {
    pub fn initialize(SC_Ts: f64, attitude_params: SpacecraftAttitudeArchitecture) -> Self {
        Self {
            signal: TruthAttitudeSignal::initialize(
                attitude_params.q_sc_eci,
                attitude_params.omega_sc,
            ),
            integrator: ode::RK5(SC_Ts),
        }
    }
    pub fn process(&mut self, actuator_dynamics: &TruthActuatorBus, prev_attitude: &Self) {
        self.propagate(actuator_dynamics, &prev_attitude.signal);
    }

    fn propagate(
        &mut self,
        actuator_dynamics: &TruthActuatorBus,
        prev_attitude: &TruthAttitudeSignal,
    ) {
        let state0 = prev_attitude.to_state_vector();
        let Jsc =
            Generic2D::from_shape_vec((3, 3), vec![10., 0., 0., 0., 20., 0., 0., 0., 30.]).unwrap();
        let inpts = concatenate![Axis(1), actuator_dynamics.net_torques.to_owned(), Jsc];
        let nstate =
            self.integrator
                .integrate(&kinedynamics::rigid_body_dynamics, 0., &state0, &inpts);
        self.signal.from_state_vector(nstate);
    }
}

#[derive(Default, Clone, Debug)]
pub struct TruthMultibodyBus {}
impl TruthMultibodyBus {
    pub fn process(actuator_dynamics: &TruthActuatorBus, prev_multibody: &Self) -> Self {
        Self::default()
    }
}
