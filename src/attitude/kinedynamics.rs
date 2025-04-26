use altai_rs as lib;
use altai_rs::meta::types::{Generic1D, Generic2D};
use altai_rs::quatlib::psi_q;
use altai_rs::veclib::{fcross, fdot};
use ndarray::{array, concatenate, s, Axis};
// use ndarray_linalg::Inverse::inverse;

fn inv_3x3(j: &Generic2D) -> Generic2D {
    let jx = j.slice(s![0..3, 0]).to_owned().insert_axis(Axis(1));
    let jy = j.slice(s![0..3, 1]).to_owned().insert_axis(Axis(1));
    let jz = j.slice(s![0..3, 2]).to_owned().insert_axis(Axis(1));

    let ijx = fcross(&jy, &jz); // x column of inv
    let ijy = fcross(&jz, &jx); // y column of inv
    let ijz = fcross(&jx, &jy); // z column of inv
    let detj = fdot(jx, ijx.to_owned()); // Determinant of J

    concatenate![Axis(1), ijx, ijy, ijz].t().to_owned() / detj[0]
}

pub fn rigid_body_dynamics(_t: f64, state0: &Generic1D, inpt: &Generic2D) -> Generic1D {
    /*
    Inputs:
    0-3: Quaternion at Time
    4-6: Angular Rate at Time

    Outputs:
    0-3: dQuaternion at Time
    4-6: dOmega at Time
    */

    // unpack state vector
    let q = state0.slice(s![0..4]);
    let w = state0.slice(s![4..7]);
    let tq = inpt.slice(s![0..3, 0]);
    let j_mat = inpt.slice(s![0..3, 1..4]).to_owned();
    let inv_j = inv_3x3(&j_mat);

    // quaternion dot; Markley 3.79
    // 0.5 * w \otimes q
    let wquat = concatenate![Axis(0), w.to_owned(), array![0.]]
        .into_shape_with_order((4, 1))
        .unwrap();
    let wpsi = psi_q(wquat.to_owned()).remove_axis(Axis(2));
    let wcross = concatenate![Axis(1), wpsi, wquat];
    let qdot = 0.5 * wcross.dot(&q);

    // angular rate dot; Markley 3.81
    // wdot = inv(J) * (T - w cross J * w)
    let jw = j_mat.dot(&w);
    let wxjw = lib::veclib::mfcross(&w, &jw.view());
    let wdot = inv_j.dot(&(-1. * wxjw + tq));

    concatenate![Axis(0), qdot, wdot]
}
