use crate::ephemeris::consts;
use altai_rs::meta::types::Generic1D;
use altai_rs::veclib::unit;
use ndarray::{concatenate, s, Axis};

pub fn orbital_twobody(_t: f64, state0: &Generic1D, _inpt: &Generic1D) -> Generic1D {
    /*
    Inputs:
    0-2: R-vector (ECI) at Time [m]
    3-5: V-vector (ECI) at Time [m/s]

    Outputs:
    0-2: V-Vector (ECI) at time [m/s]
    3-5: A-Vector (ECI) at time [m/s2]
    */

    // Unpack
    let rsc = state0.slice(s![0..3]);
    let vsc = state0.slice(s![3..6]);

    // Compute Accel
    let (ursc, mrsc) = unit(rsc.to_owned().insert_axis(Axis(1)));
    let asc = -1. * consts::MU * ursc.remove_axis(Axis(1)) / (mrsc.powi(3));

    concatenate![Axis(0), vsc, asc]
}
