//! Digital twin of the VL53L5CX time-of-flight sensor on the pan-tilt arm.
//!
//! The real sensor returns an 8×8 grid of distances over a 45°×45° field of
//! view. Each zone is a ~5.6° cone, not a thin ray: the distance it reports
//! comes from all the light coming back inside that cone. The twin models a
//! zone as `subrays × subrays` rays cast through the ray-caster, and reports
//! the median distance of those that hit something in range. Edges between a
//! near and a far object therefore blur over a whole zone, like on the real
//! sensor (see PLAN.md, "résolution vs portée").
//!
//! Frames: world x forward, y left, z up. Pan turns around z (positive =
//! left), tilt turns around the arm's left axis (positive = up).

use crate::intersectable::Scene;
use frugal_ml::{Scalar, Tensor, Vector};
use rand::Rng;
use rand_distr::StandardNormal;

pub const ZONES: usize = 8;

/// Orientation of the pan-tilt arm, in radians.
#[derive(Clone, Copy, Debug)]
pub struct PanTilt {
    pub pan: Scalar,
    pub tilt: Scalar,
}

impl PanTilt {
    pub fn degrees(pan: Scalar, tilt: Scalar) -> Self {
        Self {
            pan: pan.to_radians(),
            tilt: tilt.to_radians(),
        }
    }

    /// Rotation from the sensor frame to the world frame. Its columns are
    /// the sensor's forward, left and up axes expressed in the world.
    pub fn rotation(&self) -> Tensor<3, 3> {
        let (sp, cp) = self.pan.sin_cos();
        let (st, ct) = self.tilt.sin_cos();
        let forward = [ct * cp, ct * sp, st];
        let left = [-sp, cp, 0.0];
        let up = [-st * cp, -st * sp, ct];
        Tensor::<3, 3>::new([
            [forward[0], left[0], up[0]],
            [forward[1], left[1], up[1]],
            [forward[2], left[2], up[2]],
        ])
    }
}

/// One 8×8 measurement. `None` means no target in range for that zone.
/// Row 0 is the top of the field of view, column 0 its left side.
#[derive(Clone, Debug)]
pub struct Frame {
    pub pose: PanTilt,
    pub zones: [[Option<Scalar>; ZONES]; ZONES],
    /// Each valid zone turned into a 3-D point: sensor position + the
    /// zone's centre direction × measured distance (PLAN.md, step 3).
    pub points: Vec<Vector<3>>,
}

pub struct TofSensor {
    /// Where the sensor sits in the world (the pan-tilt pivot).
    pub position: Vector<3>,
    /// Full field of view, horizontal = vertical, in radians.
    pub fov: Scalar,
    pub max_range: Scalar,
    /// Rays per zone side: `subrays²` rays are cast per zone.
    pub subrays: usize,
    /// Standard deviation of the ranging noise, in metres.
    pub noise_sigma: Scalar,
}

impl TofSensor {
    /// VL53L5CX in 8×8 mode: 45° field of view, up to 4 m, about ±1.5 cm noise.
    pub fn vl53l5cx(position: Vector<3>) -> Self {
        Self {
            position,
            fov: 45f32.to_radians(),
            max_range: 4.0,
            subrays: 4,
            noise_sigma: 0.015,
        }
    }

    /// World direction through the point (`u`, `v`) of the field of view,
    /// both in [-1, 1]: `u = 1` is the left edge, `v = 1` the top edge.
    pub fn ray(&self, pose: &PanTilt, u: Scalar, v: Scalar) -> Vector<3> {
        let half = (self.fov / 2.0).tan();
        let in_sensor = Vector::<3>::from_data([1.0, u * half, v * half]).normalize();
        pose.rotation().multiply(&in_sensor)
    }

    /// Centre of zone (`row`, `col`) in the [-1, 1] field-of-view coordinates.
    fn zone_uv(row: usize, col: usize, sub_r: Scalar, sub_c: Scalar) -> (Scalar, Scalar) {
        let n = ZONES as Scalar;
        let u = 1.0 - 2.0 * (col as Scalar + sub_c) / n;
        let v = 1.0 - 2.0 * (row as Scalar + sub_r) / n;
        (u, v)
    }

    pub fn measure(&self, scene: &Scene, pose: PanTilt, rng: &mut impl Rng) -> Frame {
        let mut zones = [[None; ZONES]; ZONES];
        let mut points = Vec::new();
        let k = self.subrays.max(1);
        let mut hits = Vec::with_capacity(k * k);

        for (row, zone_row) in zones.iter_mut().enumerate() {
            for (col, zone) in zone_row.iter_mut().enumerate() {
                hits.clear();
                for i in 0..k {
                    for j in 0..k {
                        let sub_r = (i as Scalar + 0.5) / k as Scalar;
                        let sub_c = (j as Scalar + 0.5) / k as Scalar;
                        let (u, v) = Self::zone_uv(row, col, sub_r, sub_c);
                        let dir = self.ray(&pose, u, v);
                        if let Some(hit) = scene.cast(self.position, dir)
                            && hit.distance <= self.max_range
                        {
                            hits.push(hit.distance);
                        }
                    }
                }
                // Not enough light back: the zone reports no target.
                if hits.len() * 2 < k * k {
                    continue;
                }
                hits.sort_by(|a, b| a.total_cmp(b));
                let median = hits[hits.len() / 2];
                let noise: Scalar = rng.sample(StandardNormal);
                let distance = (median + noise * self.noise_sigma).max(0.0);
                *zone = Some(distance);

                let (u, v) = Self::zone_uv(row, col, 0.5, 0.5);
                points.push(self.position + self.ray(&pose, u, v) * distance);
            }
        }
        Frame {
            pose,
            zones,
            points,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::intersectable::Shape;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    fn v(x: Scalar, y: Scalar, z: Scalar) -> Vector<3> {
        Vector::<3>::from_data([x, y, z])
    }

    #[test]
    fn rotation_points_forward_left_and_up() {
        let left = PanTilt::degrees(90.0, 0.0)
            .rotation()
            .multiply(&v(1.0, 0.0, 0.0));
        let up = PanTilt::degrees(0.0, 90.0)
            .rotation()
            .multiply(&v(1.0, 0.0, 0.0));
        assert!((left[1] - 1.0).abs() < 1e-5);
        assert!((up[2] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn wall_in_front_reads_its_distance_in_every_zone() {
        let mut scene = Scene::default();
        scene.add(Shape::Plane {
            point: v(2.0, 0.0, 0.0),
            normal: v(-1.0, 0.0, 0.0),
        });
        let mut sensor = TofSensor::vl53l5cx(v(0.0, 0.0, 0.0));
        sensor.noise_sigma = 0.0;
        let frame = sensor.measure(
            &scene,
            PanTilt::degrees(0.0, 0.0),
            &mut StdRng::seed_from_u64(0),
        );
        assert_eq!(frame.points.len(), ZONES * ZONES);
        // Zones look along slanted rays: the distance grows towards the corners.
        let centre = frame.zones[3][3].unwrap();
        let corner = frame.zones[0][0].unwrap();
        assert!((centre - 2.0).abs() < 0.05);
        assert!(corner > centre);
        // Points land back on the wall, up to the zone's spread: the point is
        // rebuilt along the zone's centre ray, with the median sub-ray distance.
        assert!(frame.points.iter().all(|p| (p[0] - 2.0).abs() < 0.03));
    }

    #[test]
    fn nothing_in_range_gives_empty_zones() {
        let mut scene = Scene::default();
        scene.add(Shape::Plane {
            point: v(10.0, 0.0, 0.0),
            normal: v(-1.0, 0.0, 0.0),
        });
        let sensor = TofSensor::vl53l5cx(v(0.0, 0.0, 0.0));
        let frame = sensor.measure(
            &scene,
            PanTilt::degrees(0.0, 0.0),
            &mut StdRng::seed_from_u64(0),
        );
        assert!(frame.points.is_empty());
    }

    #[test]
    fn panning_left_sees_an_object_on_the_left() {
        let mut scene = Scene::default();
        scene.add(Shape::Sphere {
            center: v(0.0, 2.0, 0.0),
            radius: 0.5,
        });
        let sensor = TofSensor::vl53l5cx(v(0.0, 0.0, 0.0));
        let mut rng = StdRng::seed_from_u64(0);
        assert!(
            sensor
                .measure(&scene, PanTilt::degrees(0.0, 0.0), &mut rng)
                .points
                .is_empty()
        );
        let frame = sensor.measure(&scene, PanTilt::degrees(90.0, 0.0), &mut rng);
        assert!(!frame.points.is_empty());
        assert!(frame.points.iter().all(|p| p[1] > 1.3));
    }
}
