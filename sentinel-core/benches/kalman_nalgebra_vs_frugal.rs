use criterion::{criterion_group, criterion_main, Criterion};
use nalgebra::SVector;
use sentinel_core::{KalmanFilter, KalmanFilterFrugal};
use std::hint::black_box;

fn bench_nalgebra(c: &mut Criterion) {
    c.bench_function("KalmanFilter (nalgebra) predict+update", |b| {
        let mut kf = KalmanFilter::<9, 3>::new(0.01, 0.1, 0.5);
        b.iter(|| {
            kf.predict();
            kf.update(black_box(SVector::<f32, 3>::new(1.0, 2.0, 3.0)));
            black_box(&kf.x);
        })
    });
}

fn bench_frugal(c: &mut Criterion) {
    c.bench_function("KalmanFilterFrugal (frugal_ml) predict+update", |b| {
        let mut kf = KalmanFilterFrugal::<9, 3>::new(0.01, 0.1, 0.5);
        b.iter(|| {
            kf.predict();
            kf.update(black_box(frugal_ml::linalg::Vector::<3>::from_data([
                1.0, 2.0, 3.0,
            ])));
            black_box(&kf.x);
        })
    });
}

criterion_group!(benches, bench_nalgebra, bench_frugal);
criterion_main!(benches);
