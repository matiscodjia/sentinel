use frugal_ml::{self as fgml, Scalar, Vector};

pub struct Hit {
    impact_point: fgml::Vector<3>,
    distance: fgml::Scalar,
    normal: fgml::Vector<3>,
}
pub enum Shape {
    Plan {
        point: fgml::Vector<3>,
        normal: fgml::Vector<3>,
    },
    Sphere {
        center: fgml::Vector<3>,
        radius: usize,
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
            Shape::Plan { point, normal } => {
                let alignement = normal.dot(&direction);
                if alignement.abs() < EPSILON {
                    return None;
                } else {
                    let to_plane = *point - origin;
                    let distance = to_plane.dot(&normal) / alignement;
                    if distance < 0.0 {
                        return None;
                    } else {
                        Some(Hit {
                            distance: distance,
                            normal: *normal,
                            impact_point: origin + direction * distance,
                        })
                    }
                }
            }
            Shape::Sphere { center, radius } => {
                let diff = origin - *center;
                let b = 2.0 * (direction.dot(&diff));
                let c = diff.dot(&diff) - (radius * radius) as Scalar;
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
            Shape::Triangle { a, b, c } => todo!(),
        }
    }
}
