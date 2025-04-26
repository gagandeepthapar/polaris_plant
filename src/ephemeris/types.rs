use crate::{
    ephemeris::consts::{MU, RE},
    ode::{self, Integrator},
};
use altai_rs::meta::types::{Generic1D, Generic2D, Vector3};
use ndarray::{array, s, Axis};

use crate::actuators::types::TruthActuatorBus;

use super::kinedynamics;

#[derive(Debug, Clone)]
pub struct TruthEphemerisSignal {
    pub r_sc_eci: Vector3,
    pub v_sc_eci: Vector3,
}
impl Default for TruthEphemerisSignal {
    fn default() -> Self {
        // z_sv: 500km
        // e_sv: 0
        // i_sv: 0 deg
        // O_sv: 0 deg
        // w_sv: 0 deg
        // M_sv: 0 deg
        let a_sv = RE + 900e3;
        Self {
            r_sc_eci: array![a_sv, 0., 0.].into_shape_with_order((3, 1)).unwrap(),
            v_sc_eci: array![0., (MU / a_sv).sqrt(), 0.]
                .into_shape_with_order((3, 1))
                .unwrap(),
        }
    }
}

impl TruthEphemerisSignal {
    pub fn to_state_vector(&self) -> Generic1D {
        Generic1D::from_iter(
            self.r_sc_eci
                .to_owned()
                .remove_axis(Axis(1))
                .iter()
                .chain(self.v_sc_eci.to_owned().remove_axis(Axis(1)).iter())
                .map(|&x| x),
        )
    }
    pub fn from_state_vector(&mut self, state_vec: Generic1D) {
        state_vec
            .slice(s![0..3])
            .assign_to(self.r_sc_eci.slice_mut(s![0..3, 0]));
        state_vec
            .slice(s![3..6])
            .assign_to(self.v_sc_eci.slice_mut(s![0..3, 0]));
    }
}

#[derive(Debug, Clone)]
pub struct TruthEphemerisBus {
    pub signal: TruthEphemerisSignal,
    integrator: ode::RK5,
}

impl Default for TruthEphemerisBus {
    fn default() -> Self {
        Self {
            signal: TruthEphemerisSignal::default(),
            integrator: ode::RK5(0.1),
        }
    }
}

impl TruthEphemerisBus {
    pub fn initialize(SC_Ts: f64) -> Self {
        Self {
            signal: TruthEphemerisSignal::default(),
            integrator: ode::RK5(SC_Ts),
        }
    }

    pub fn process(&mut self, actuator_dynamics: &TruthActuatorBus, prev_ephem: &Self) {
        self.propagate(actuator_dynamics, &prev_ephem.signal);
    }

    fn propagate(
        &mut self,
        actuator_dynamics: &TruthActuatorBus,
        prev_ephem: &TruthEphemerisSignal,
    ) {
        let state0 = prev_ephem.to_state_vector();
        let nstate = self.integrator.integrate(
            &kinedynamics::orbital_twobody,
            0.,
            &state0,
            &actuator_dynamics.net_forces.to_owned(),
        );
        self.signal.from_state_vector(nstate);
    }
}
