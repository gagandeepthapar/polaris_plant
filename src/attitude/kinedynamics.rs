use altai_rs as lib;
use altai_rs::meta::types::Generic1D;
use altai_rs::quatlib::psi_q;
use ndarray::{array, concatenate, s, Axis};
// use ndarray_linalg::Inverse;

pub fn rigid_body_dynamics(_t: f64, state0: &Generic1D, inpt: &Generic1D) -> Generic1D {
    /*
    Inputs:
    0-3: Quaternion at Time
    4-6: Angular Rate at Time

    Outputs:
    0-3: dQuaternion at Time
    4-6: dOmega at Time
    */
    let j_mat = array![[10., 0., 0.], [0., 20., 0.], [0., 0., 30.]];
    // let inv_j = J_MAT.inv().unwrap();
    let inv_j = array![[1. / 10., 0., 0.], [0., 1. / 20., 0.], [0., 0., 1. / 30.]];

    // unpack state vector
    let q = state0.slice(s![0..4]);
    let w = state0.slice(s![4..7]);
    let tq = inpt.slice(s![0..3]);

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
