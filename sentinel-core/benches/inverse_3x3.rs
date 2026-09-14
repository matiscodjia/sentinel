use criterion::{criterion_group, criterion_main, Criterion};
use frugal_ml::linalg::decomposition::inverse;
use frugal_ml::linalg::Tensor;
use nalgebra::SMatrix;
use std::hint::black_box;

// Same symmetric positive-definite 3x3 matrix on both sides (shape of a
// Kalman innovation covariance S), so both benches invert the same numbers.
fn bench_nalgebra(c: &mut Criterion) {
    let m = SMatrix::<f32, 3, 3>::new(4.0, 1.0, 2.0, 1.0, 3.0, 0.5, 2.0, 0.5, 5.0);
    c.bench_function("3x3 inverse (nalgebra try_inverse)", |b| {
        b.iter(|| black_box(black_box(m).try_inverse().unwrap()))
    });
}

fn bench_frugal(c: &mut Criterion) {
    let m = Tensor::<3, 3>::new([[4.0, 1.0, 2.0], [1.0, 3.0, 0.5], [2.0, 0.5, 5.0]]);
    c.bench_function("3x3 inverse (frugal_ml decomposition::inverse)", |b| {
        b.iter(|| black_box(inverse(black_box(&m)).unwrap()))
    });
}

criterion_group!(benches, bench_nalgebra, bench_frugal);
criterion_main!(benches);
