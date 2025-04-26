use altai_rs::meta::types::{Generic1D, Generic2D};

#[derive(Debug, Clone)]
pub struct RK2(pub f64);

#[derive(Debug, Clone)]
pub struct RK5(pub f64);

pub trait Integrator {
    fn integrate<F>(
        &self,
        d_func: &F,
        time: f64,
        state0: &Generic1D,
        inputs: &Generic2D,
    ) -> Generic1D
    where
        F: Fn(f64, &Generic1D, &Generic2D) -> Generic1D;
}

impl Integrator for RK2 {
    fn integrate<F>(
        &self,
        d_func: &F,
        time: f64,
        state0: &Generic1D,
        inputs: &Generic2D,
    ) -> Generic1D
    where
        F: Fn(f64, &Generic1D, &Generic2D) -> Generic1D,
    {
        let k1 = d_func(time, state0, inputs) * self.0;
        let kn = Generic1D::from_iter(state0.iter().zip(k1.iter()).map(|(&s, &a)| s + a / 2.));
        let k2 = d_func(time + self.0 / 2., &kn, inputs) * self.0;

        state0 + k2
    }
}

impl Integrator for RK5 {
    fn integrate<F>(
        &self,
        d_func: &F,
        time: f64,
        state0: &Generic1D,
        inputs: &Generic2D,
    ) -> Generic1D
    where
        F: Fn(f64, &Generic1D, &Generic2D) -> Generic1D,
    {
        let mut kn = Generic1D::from_elem(state0.len(), 0.);
        let k1 = d_func(time, &(state0 + self.0 * kn), inputs);

        kn = Generic1D::from_iter(k1.iter().map(|&a| 1. / 3. * a));
        let k2 = d_func(time + self.0 / 3., &(state0 + self.0 * kn), inputs);

        kn = Generic1D::from_iter(
            k1.iter()
                .zip(k2.iter())
                .map(|(&a, &b)| 4. / 25. * a + 6. / 25. * b),
        );
        let k3 = d_func(time + 2. * self.0 / 25., &(state0 + self.0 * kn), inputs);

        kn = Generic1D::from_iter(
            k1.iter()
                .zip(k2.iter())
                .zip(k3.iter())
                .map(|((&a, &b), &c)| 1. / 4. * a - 3. * b + 15. / 4. * c),
        );
        let k4 = d_func(time + self.0, &(state0 + self.0 * kn), inputs);

        kn = Generic1D::from_iter(k1.iter().zip(k2.iter()).zip(k3.iter()).zip(k4.iter()).map(
            |(((&a, &b), &c), &d)| 2. / 27. * a + 10. / 9. * b - 50. / 81. * c + 8. / 81. * d,
        ));
        let k5 = d_func(time + 2. / 3. * self.0, &(state0 + self.0 * kn), inputs);

        kn = Generic1D::from_iter(k1.iter().zip(k2.iter()).zip(k3.iter()).zip(k4.iter()).map(
            |(((&a, &b), &c), &d)| 2. / 25. * a + 12. / 25. * b + 2. / 15. * c + 8. / 75. * d,
        ));
        let k6 = d_func(time + 4. / 5. * self.0, &(state0 + self.0 * kn), inputs);

        state0 + self.0 * (23. / 192. * k1 + 125. / 192. * k3 - 27. / 64. * k5 + 125. / 192. * k6)
    }
}
