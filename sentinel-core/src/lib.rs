#![no_std]

extern crate nalgebra as na;
use na::{SMatrix, SVector};

pub mod kalman_frugal;
pub use kalman_frugal::KalmanFilterFrugal;

pub struct KalmanFilter<const N: usize, const M: usize> {
    // --- 1. L'ÉTAT DU SYSTÈME ---
    pub x: SVector<f32, N>,
    pub p: SMatrix<f32, N, N>,

    // --- 2. LE MODÈLE PHYSIQUE ---
    pub f: SMatrix<f32, N, N>,
    pub q: SMatrix<f32, N, N>,

    // --- 3. LE MODÈLE CAPTEUR ---
    pub h: SMatrix<f32, M, N>,
    pub r: SMatrix<f32, M, M>,
}

impl KalmanFilter<9, 3> {
    // AJOUT DE 'pub' ICI
    pub fn new(dt: f32, process_noise: f32, sensor_noise: f32) -> Self {
        let mut f = SMatrix::<f32, 9, 9>::identity();

        // Physique : x = x0 + v*t + 0.5*a*t^2
        let half_dt_square = 0.5 * dt * dt;

        // Axe X
        f[(0, 1)] = dt;
        f[(0, 2)] = half_dt_square;
        f[(1, 2)] = dt;

        // Axe Y
        f[(3, 4)] = dt;
        f[(3, 5)] = half_dt_square;
        f[(4, 5)] = dt;

        // Axe Z
        f[(6, 7)] = dt;
        f[(6, 8)] = half_dt_square;
        f[(7, 8)] = dt;

        // Matrice de Mesure H
        let mut h = SMatrix::<f32, 3, 9>::zeros();
        h[(0, 0)] = 1.0; // Mesure X -> Etat 0
        h[(1, 3)] = 1.0; // Mesure Y -> Etat 3
        h[(2, 6)] = 1.0; // Mesure Z -> Etat 6

        // Matrices de bruit
        let q = SMatrix::<f32, 9, 9>::identity() * process_noise;
        let r = SMatrix::<f32, 3, 3>::identity() * sensor_noise;

        // État Initial (Incertitude maximale au début)
        let p = SMatrix::<f32, 9, 9>::identity() * 1000.0;
        let x = SVector::<f32, 9>::zeros();

        Self { x, p, f, q, r, h }
    }

    pub fn predict(&mut self) {
        // CORRECTION : Ajout de self. partout !

        // 1. Prédire l'ÉTAT
        self.x = self.f * self.x;

        // 2. Prédire l'INCERTITUDE
        self.p = self.f * self.p * self.f.transpose() + self.q;
    }

    pub fn update(&mut self, measurement: SVector<f32, 3>) {
        // CORRECTION : Ajout de self. pour H et x
        let z = measurement;
        let y = z - self.h * self.x; // Innovation

        // Covariance de l'innovation S
        let s = self.h * self.p * self.h.transpose() + self.r;

        // Inversion (On panic si impossible pour la simu)
        let s_inv = s.try_inverse().expect("Matrice S non inversible !");

        // Gain de Kalman K
        let k = self.p * self.h.transpose() * s_inv;

        // Mise à jour finale
        self.x = self.x + k * y;

        // Mise à jour covariance
        let i = SMatrix::<f32, 9, 9>::identity();
        self.p = (i - k * self.h) * self.p;
    }
}

