use frugal_ml::linalg::decomposition::inverse;
use frugal_ml::linalg::{Tensor, Vector};

/// Same 9-state/3-measurement Kalman filter as [`crate::KalmanFilter`],
/// rewritten on `frugal_ml::linalg::Tensor` instead of `nalgebra::SMatrix`,
/// to compare the two side by side (API and benchmark).
pub struct KalmanFilterFrugal<const N: usize, const M: usize> {
    pub x: Vector<N>,
    pub p: Tensor<N, N>,

    pub f: Tensor<N, N>,
    pub q: Tensor<N, N>,

    pub h: Tensor<M, N>,
    pub r: Tensor<M, M>,
}

impl KalmanFilterFrugal<9, 3> {
    pub fn new(dt: f32, process_noise: f32, sensor_noise: f32) -> Self {
        let mut f = Tensor::<9, 9>::identity();

        let half_dt_square = 0.5 * dt * dt;

        f[(0, 1)] = dt;
        f[(0, 2)] = half_dt_square;
        f[(1, 2)] = dt;

        f[(3, 4)] = dt;
        f[(3, 5)] = half_dt_square;
        f[(4, 5)] = dt;

        f[(6, 7)] = dt;
        f[(6, 8)] = half_dt_square;
        f[(7, 8)] = dt;

        let mut h = Tensor::<3, 9>::new([[0.0; 9]; 3]);
        h[(0, 0)] = 1.0;
        h[(1, 3)] = 1.0;
        h[(2, 6)] = 1.0;

        let q = Tensor::<9, 9>::identity() * process_noise;
        let r = Tensor::<3, 3>::identity() * sensor_noise;

        let p = Tensor::<9, 9>::identity() * 1000.0;
        let x = Vector::<9>::from_data([0.0; 9]);

        Self { x, p, f, q, r, h }
    }

    pub fn predict(&mut self) {
        self.x = self.f.multiply(&self.x);
        self.p = self.f.multiply(&self.p).multiply(&self.f.transposed()) + self.q;
    }

    pub fn update(&mut self, measurement: Vector<3>) {
        let z = measurement;
        let y = z - self.h.multiply(&self.x);

        let s = self.h.multiply(&self.p).multiply(&self.h.transposed()) + self.r;

        let s_inv = inverse(&s).expect("Matrice S non inversible !");

        let k = self.p.multiply(&self.h.transposed()).multiply(&s_inv);

        self.x = self.x + k.multiply(&y);

        let i = Tensor::<9, 9>::identity();
        self.p = (i - k.multiply(&self.h)).multiply(&self.p);
    }
}
