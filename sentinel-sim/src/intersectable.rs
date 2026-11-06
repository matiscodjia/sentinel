use frugal_ml::{self as fgml, Scalar, Vector};
//Perform a ray-casting to some basics shape
//
// Convention: `direction` is a unit vector, so `distance` is in the same unit
// as the scene (metres in the digital twin) and `impact_point = origin +
// direction * distance`.
#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub impact_point: fgml::Vector<3>,
    pub distance: fgml::Scalar,
    pub normal: fgml::Vector<3>,
}
#[derive(Clone, Copy, Debug)]
pub enum Shape {
    Plane {
        point: fgml::Vector<3>,
        normal: fgml::Vector<3>,
    },
    Sphere {
        center: fgml::Vector<3>,
        radius: Scalar,
    },
    Triangle {
        a: Vector<3>,
        b: Vector<3>,
        c: Vector<3>,
    },
}
pub trait Intersectable {
    fn intersect(&self, origin: fgml::Vector<3>, direction: fgml::Vector<3>) -> Option<Hit>;
}
enum Roots {
    None,
    One(Scalar),
    Two(Scalar, Scalar),
}

const EPSILON: Scalar = 1e-6;
fn solve_quadratic(a: Scalar, b: Scalar, c: Scalar) -> Roots {
    let discriminant = b * b - 4.0 * a * c;

    if discriminant < 0.0 {
        Roots::None
    } else if discriminant.abs() < EPSILON {
        Roots::One(-b / (2.0 * a))
    } else {
        let sqrt_d = discriminant.sqrt();
        let x1 = (-b - sqrt_d) / (2.0 * a);
        let x2 = (-b + sqrt_d) / (2.0 * a);
        Roots::Two(x1, x2)
    }
}

impl Intersectable for Shape {
    fn intersect(&self, origin: fgml::Vector<3>, direction: fgml::Vector<3>) -> Option<Hit> {
        match self {
            Shape::Plane { point, normal } => {
                let alignement = normal.dot(&direction);
                if alignement.abs() < EPSILON {
                    None
                } else {
                    let to_plane = *point - origin;
                    let distance = to_plane.dot(&normal) / alignement;
                    if distance < 0.0 {
                        None
                    } else {
                        Some(Hit {
                            distance,
                            normal: *normal,
                            impact_point: origin + direction * distance,
                        })
                    }
                }
            }
            Shape::Sphere { center, radius } => {
                let diff = origin - *center;
                let b = 2.0 * (direction.dot(&diff));
                let c = diff.dot(&diff) - radius * radius;
                let result = solve_quadratic(1.0, b, c);
                match result {
                    Roots::None => return None,
                    Roots::One(distance) => {
                        if distance < -EPSILON {
                            return None;
                        }
                        let impact_point = origin + direction * distance;
                        let normal = (impact_point - *center).normalize();
                        return Some(Hit {
                            distance: distance,
                            normal: normal,
                            impact_point: impact_point,
                        });
                    }
                    Roots::Two(distance1, distance2) => {
                        let distance: Scalar;
                        if distance1 >= 0.0 {
                            distance = distance1;
                        } else {
                            distance = distance2;
                        }

                        if distance < 0.0 {
                            return None;
                        }
                        let impact_point = origin + direction * distance;
                        let normal = (impact_point - *center).normalize();

                        return Some(Hit {
                            distance: distance,
                            normal: normal,
                            impact_point: impact_point,
                        });
                    }
                }
            }
            //Algorithm Möller-Trumbore
            //
            // Write the impact point in barycentric coordinates:
            //   origin + t·direction = a + u·(b − a) + v·(c − a)
            // which is a 3×3 linear system in (t, u, v). Cramer's rule solves
            // it with cross and dot products only, no matrix inverse. The ray
            // hits the triangle iff u ≥ 0, v ≥ 0, u + v ≤ 1 and t > 0.
            Shape::Triangle { a, b, c } => {
                let edge1 = *b - *a;
                let edge2 = *c - *a;
                let p = direction.cross(&edge2);
                let det = edge1.dot(&p);
                // det ≈ 0: the ray is parallel to the triangle's plane.
                if det.abs() < EPSILON {
                    return None;
                }
                let inv_det = 1.0 / det;

                let s = origin - *a;
                let u = s.dot(&p) * inv_det;
                if !(0.0..=1.0).contains(&u) {
                    return None;
                }

                let q = s.cross(&edge1);
                let v = direction.dot(&q) * inv_det;
                if v < 0.0 || u + v > 1.0 {
                    return None;
                }

                let distance = edge2.dot(&q) * inv_det;
                if distance <= EPSILON {
                    return None; // the triangle is behind the origin
                }

                // Geometric normal, flipped to face the incoming ray, so a
                // triangle is visible from both sides.
                let mut normal = edge1.cross(&edge2).normalize();
                if normal.dot(&direction) > 0.0 {
                    normal = -normal;
                }
                Some(Hit {
                    distance,
                    normal,
                    impact_point: origin + direction * distance,
                })
            }
        }
    }
}

impl Shape {
    /// Axis-aligned box from its two opposite corners, as 12 triangles (two
    /// per face), so boxes go through the same Möller-Trumbore path.
    pub fn cuboid(min: Vector<3>, max: Vector<3>) -> Vec<Shape> {
        let corner = |x: bool, y: bool, z: bool| {
            Vector::<3>::from_data([
                if x { max[0] } else { min[0] },
                if y { max[1] } else { min[1] },
                if z { max[2] } else { min[2] },
            ])
        };
        // Each face as 4 corners in order around its edge.
        let faces = [
            [
                (false, false, false),
                (true, false, false),
                (true, true, false),
                (false, true, false),
            ], // bottom
            [
                (false, false, true),
                (true, false, true),
                (true, true, true),
                (false, true, true),
            ], // top
            [
                (false, false, false),
                (true, false, false),
                (true, false, true),
                (false, false, true),
            ], // y = min
            [
                (false, true, false),
                (true, true, false),
                (true, true, true),
                (false, true, true),
            ], // y = max
            [
                (false, false, false),
                (false, true, false),
                (false, true, true),
                (false, false, true),
            ], // x = min
            [
                (true, false, false),
                (true, true, false),
                (true, true, true),
                (true, false, true),
            ], // x = max
        ];
        let mut triangles = Vec::with_capacity(12);
        for face in faces {
            let [p0, p1, p2, p3] = face.map(|(x, y, z)| corner(x, y, z));
            triangles.push(Shape::Triangle {
                a: p0,
                b: p1,
                c: p2,
            });
            triangles.push(Shape::Triangle {
                a: p0,
                b: p2,
                c: p3,
            });
        }
        triangles
    }
}

/// A set of shapes. Casting a ray returns the closest hit, which is what a
/// time-of-flight sensor sees: the first surface the light comes back from.
#[derive(Default)]
pub struct Scene {
    pub shapes: Vec<Shape>,
}

impl Scene {
    pub fn add(&mut self, shape: Shape) -> &mut Self {
        self.shapes.push(shape);
        self
    }

    pub fn extend(&mut self, shapes: impl IntoIterator<Item = Shape>) -> &mut Self {
        self.shapes.extend(shapes);
        self
    }

    pub fn cast(&self, origin: Vector<3>, direction: Vector<3>) -> Option<Hit> {
        self.shapes
            .iter()
            .filter_map(|shape| shape.intersect(origin, direction))
            .min_by(|h1, h2| h1.distance.total_cmp(&h2.distance))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: Scalar, y: Scalar, z: Scalar) -> Vector<3> {
        Vector::<3>::from_data([x, y, z])
    }

    fn close(a: Scalar, b: Scalar) -> bool {
        (a - b).abs() < 1e-4
    }

    const ORIGIN: [Scalar; 3] = [0.0, 0.0, 0.0];

    fn forward() -> Vector<3> {
        v(1.0, 0.0, 0.0)
    }

    #[test]
    fn plane_in_front_is_hit_at_its_distance() {
        let plane = Shape::Plane {
            point: v(3.0, 0.0, 0.0),
            normal: v(-1.0, 0.0, 0.0),
        };
        let hit = plane
            .intersect(Vector::from_data(ORIGIN), forward())
            .unwrap();
        assert!(close(hit.distance, 3.0));
        assert!(close(hit.impact_point[0], 3.0));
    }

    #[test]
    fn plane_behind_or_parallel_is_missed() {
        let behind = Shape::Plane {
            point: v(-3.0, 0.0, 0.0),
            normal: v(1.0, 0.0, 0.0),
        };
        let parallel = Shape::Plane {
            point: v(0.0, 0.0, -1.0),
            normal: v(0.0, 0.0, 1.0),
        };
        assert!(
            behind
                .intersect(Vector::from_data(ORIGIN), forward())
                .is_none()
        );
        assert!(
            parallel
                .intersect(Vector::from_data(ORIGIN), forward())
                .is_none()
        );
    }

    #[test]
    fn sphere_is_hit_on_its_near_side_with_outward_normal() {
        let sphere = Shape::Sphere {
            center: v(5.0, 0.0, 0.0),
            radius: 1.0,
        };
        let hit = sphere
            .intersect(Vector::from_data(ORIGIN), forward())
            .unwrap();
        assert!(close(hit.distance, 4.0));
        assert!(close(hit.normal[0], -1.0));
    }

    #[test]
    fn sphere_from_inside_hits_the_far_side() {
        let sphere = Shape::Sphere {
            center: v(0.0, 0.0, 0.0),
            radius: 2.0,
        };
        let hit = sphere
            .intersect(Vector::from_data(ORIGIN), forward())
            .unwrap();
        assert!(close(hit.distance, 2.0));
    }

    #[test]
    fn sphere_off_axis_is_missed() {
        let sphere = Shape::Sphere {
            center: v(5.0, 3.0, 0.0),
            radius: 1.0,
        };
        assert!(
            sphere
                .intersect(Vector::from_data(ORIGIN), forward())
                .is_none()
        );
    }

    fn triangle_at_x2() -> Shape {
        Shape::Triangle {
            a: v(2.0, -1.0, -1.0),
            b: v(2.0, 1.0, -1.0),
            c: v(2.0, 0.0, 1.0),
        }
    }

    #[test]
    fn triangle_hit_inside_with_normal_facing_the_ray() {
        let hit = triangle_at_x2()
            .intersect(Vector::from_data(ORIGIN), forward())
            .unwrap();
        assert!(close(hit.distance, 2.0));
        assert!(hit.normal.dot(&forward()) < 0.0);
    }

    #[test]
    fn triangle_missed_outside_its_edges_and_behind() {
        let tri = triangle_at_x2();
        let outside = v(1.0, 0.0, 0.8).normalize(); // above the apex at this distance
        assert!(tri.intersect(Vector::from_data(ORIGIN), outside).is_none());
        assert!(
            tri.intersect(Vector::from_data(ORIGIN), -forward())
                .is_none()
        );
    }

    #[test]
    fn cuboid_is_hit_on_its_near_face() {
        let mut scene = Scene::default();
        scene.extend(Shape::cuboid(v(2.0, -0.5, -0.5), v(3.0, 0.5, 0.5)));
        assert_eq!(scene.shapes.len(), 12);
        let hit = scene.cast(Vector::from_data(ORIGIN), forward()).unwrap();
        assert!(close(hit.distance, 2.0));
    }

    #[test]
    fn scene_returns_the_closest_hit() {
        let mut scene = Scene::default();
        scene
            .add(Shape::Plane {
                point: v(10.0, 0.0, 0.0),
                normal: v(-1.0, 0.0, 0.0),
            })
            .add(Shape::Sphere {
                center: v(4.0, 0.0, 0.0),
                radius: 0.5,
            })
            .add(triangle_at_x2());
        let hit = scene.cast(Vector::from_data(ORIGIN), forward()).unwrap();
        assert!(close(hit.distance, 2.0));
    }
}
