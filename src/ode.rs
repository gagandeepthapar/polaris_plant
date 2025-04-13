use altai_rs::meta::types::Generic1D;

pub struct RK2(pub f64);
pub struct RK5(pub f64);

pub trait Integrator {
    fn integrate<F>(
        &self,
        d_func: &F,
        time: f64,
        state0: &Generic1D,
        inputs: &Generic1D,
    ) -> Generic1D
    where
        F: Fn(f64, &Generic1D, &Generic1D) -> Generic1D;
}

impl Integrator for RK2 {
    fn integrate<F>(
        &self,
        d_func: &F,
        time: f64,
        state0: &Generic1D,
        inputs: &Generic1D,
    ) -> Generic1D
    where
        F: Fn(f64, &Generic1D, &Generic1D) -> Generic1D,
    {
        let k1 = d_func(time, state0, inputs) * self.0;
        let kn = Generic1D::from_iter(state0.iter().zip(k1.iter()).map(|(&s, &a)| s + a / 2.));
        let k2 = d_func(time + self.0 / 2., &kn, &inputs) * self.0;

        state0 + k2
    }
}

impl Integrator for RK5 {
    fn integrate<F>(
        &self,
        d_func: &F,
        time: f64,
        state0: &Generic1D,
        inputs: &Generic1D,
    ) -> Generic1D
    where
        F: Fn(f64, &Generic1D, &Generic1D) -> Generic1D,
    {
        let k1 = d_func(time, &state0, &inputs) * self.0;

        let mut kn = Generic1D::from_iter(
            state0
                .iter()
                .zip(k1.iter())
                .map(|(&s, &a)| s / self.0 + a / 3.),
        ) * self.0;
        let k2 = d_func(time + self.0 / 3., &kn, &inputs);

        kn = Generic1D::from_iter(
            state0
                .iter()
                .zip(k1.iter().zip(k2.iter()))
                .map(|(&s, (&a, &b))| s / self.0 + a * 4. / 25. + b * 6. / 25.),
        ) * self.0;
        let k3 = d_func(time + self.0 * 2. / 25., &kn, &inputs);

        kn = Generic1D::from_iter(
            state0
                .iter()
                .zip(k1.iter().zip(k2.iter().zip(k3.iter())))
                .map(|(&s, (&a, (&b, &c)))| s / self.0 + a / 4. + b * -3. + c * 15. / 4.),
        ) * self.0;
        let k4 = d_func(time + self.0, &kn, &inputs);

        kn = Generic1D::from_iter(
            state0
                .iter()
                .zip(k1.iter().zip(k2.iter().zip(k3.iter().zip(k4.iter()))))
                .map(|(&s, (&a, (&b, (&c, &d))))| {
                    s / self.0 + a * 2. / 27. + b * 10. / 9. + c * -50. / 81. + d * 8. / 81.
                }),
        ) * self.0;
        let k5 = d_func(time + self.0 * 2. / 3., &kn, &inputs);

        kn = Generic1D::from_iter(
            state0
                .iter()
                .zip(k1.iter().zip(k2.iter().zip(k3.iter().zip(k4.iter()))))
                .map(|(&s, (&a, (&b, (&c, &d))))| {
                    s / self.0 + a * 2. / 25. + b * 12. / 15. + c * 2. / 15. + d * 8. / 75.
                }),
        ) * self.0;
        let k6 = d_func(time + self.0 * 4. / 5., &kn, &inputs);

        kn = Generic1D::from_iter(
            state0
                .iter()
                .zip(
                    k1.iter().zip(
                        k2.iter()
                            .zip(k3.iter().zip(k4.iter().zip(k5.iter().zip(k6.iter())))),
                    ),
                )
                .map(|(&s, (&a, (&b, (&c, (&d, (&e, &f))))))| {
                    s / self.0
                        + a * 23. / 192.
                        + b * 0.
                        + c * 125. / 192.
                        + d * 0.
                        + e * -27. / 64.
                        + f * 125. / 192.
                }),
        ) * self.0;

        kn
    }
}
