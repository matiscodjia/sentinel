//! Runs the ray-caster and the VL53L5CX digital twin on a room-sized scene,
//! and writes two visuals to `sentinel-sim/renders/`:
//!
//! - `raycast_view.png`: what a camera at the sensor would see (one ray per
//!   pixel, Lambert shading) next to what the 8×8 sensor actually reports,
//!   for the same pose.
//! - `tof_scan.gif`: the pan-tilt arm sweeping the room while a target
//!   crosses it. On the left, the 3-D point cloud building up; on the right,
//!   the live 8×8 frame.
//!
//!     cargo run --release -p sentinel-sim --bin raycast_demo

use frugal_ml::{Scalar, Vector};
use plotters::coord::Shift;
use plotters::prelude::*;
use plotters::style::text_anchor;
use rand::SeedableRng;
use rand::rngs::StdRng;
use sentinel_sim::intersectable::{Scene, Shape};
use sentinel_sim::tof::{Frame, PanTilt, TofSensor, ZONES};
use std::error::Error;
use std::path::PathBuf;

type Res<T> = Result<T, Box<dyn Error>>;

fn v(x: Scalar, y: Scalar, z: Scalar) -> Vector<3> {
    Vector::<3>::from_data([x, y, z])
}

const SENSOR_HEIGHT: Scalar = 0.35;
const GIF_FRAMES: usize = 48;
const TARGET_RADIUS: Scalar = 0.15;

/// A 3.5 m × 3.6 m room: floor, back wall and side walls, a box and a pyramid
/// (both made of triangles), plus a spherical target added per frame.
fn room() -> Scene {
    let mut scene = Scene::default();
    scene
        .add(Shape::Plane {
            point: v(0.0, 0.0, 0.0),
            normal: v(0.0, 0.0, 1.0),
        })
        .add(Shape::Plane {
            point: v(3.5, 0.0, 0.0),
            normal: v(-1.0, 0.0, 0.0),
        })
        .add(Shape::Plane {
            point: v(0.0, 1.8, 0.0),
            normal: v(0.0, -1.0, 0.0),
        })
        .add(Shape::Plane {
            point: v(0.0, -1.8, 0.0),
            normal: v(0.0, 1.0, 0.0),
        })
        .extend(Shape::cuboid(v(1.4, -0.9, 0.0), v(1.8, -0.5, 0.45)));

    // Square pyramid: four triangular sides around an apex.
    let (cx, cy, half, height) = (2.3, 0.7, 0.3, 0.7);
    let apex = v(cx, cy, height);
    let base = [
        v(cx - half, cy - half, 0.0),
        v(cx + half, cy - half, 0.0),
        v(cx + half, cy + half, 0.0),
        v(cx - half, cy + half, 0.0),
    ];
    for i in 0..4 {
        scene.add(Shape::Triangle {
            a: base[i],
            b: base[(i + 1) % 4],
            c: apex,
        });
    }
    scene
}

/// The target crosses the room from left to right, bobbing up and down.
fn target_at(t: Scalar) -> Vector<3> {
    let angle = t * std::f32::consts::TAU;
    v(
        1.5 + 0.3 * angle.sin(),
        1.3 - 2.6 * t,
        0.55 + 0.15 * (2.0 * angle).sin(),
    )
}

fn with_target(base: &Scene, target: Vector<3>) -> Scene {
    let mut scene = Scene {
        shapes: base.shapes.clone(),
    };
    scene.add(Shape::Sphere {
        center: target,
        radius: TARGET_RADIUS,
    });
    scene
}

/// Near = warm, far = cool. Distances in metres, clamped to the sensor range.
fn distance_color(d: Scalar, max: Scalar) -> RGBColor {
    let stops: [(f32, [f32; 3]); 5] = [
        (0.0, [252.0, 74.0, 42.0]),
        (0.25, [253.0, 174.0, 60.0]),
        (0.5, [173.0, 221.0, 142.0]),
        (0.75, [65.0, 182.0, 196.0]),
        (1.0, [37.0, 52.0, 148.0]),
    ];
    let x = (d / max).clamp(0.0, 1.0);
    let i = stops
        .iter()
        .rposition(|(s, _)| *s <= x)
        .unwrap()
        .min(stops.len() - 2);
    let (s0, c0) = stops[i];
    let (s1, c1) = stops[i + 1];
    let f = (x - s0) / (s1 - s0);
    let mix = |k: usize| (c0[k] + (c1[k] - c0[k]) * f) as u8;
    RGBColor(mix(0), mix(1), mix(2))
}

const BACKGROUND: RGBColor = RGBColor(18, 20, 26);
const INK: RGBColor = RGBColor(225, 228, 235);
const MUTED: RGBColor = RGBColor(130, 136, 150);
/// Ground-truth target. Magenta is absent from the distance palette
/// (red-orange → yellow → green → blue), so it can't be read as a near point.
const TARGET: RGBColor = RGBColor(236, 64, 236);

/// One ray per pixel through the sensor's field of view: a pinhole camera
/// with the sensor's pose and FoV, shaded by the hit normal and tinted by
/// distance. The 8×8 zone grid is drawn on top.
fn draw_camera_view(
    area: &DrawingArea<BitMapBackend, Shift>,
    scene: &Scene,
    sensor: &TofSensor,
    pose: PanTilt,
) -> Res<()> {
    let (w, h) = area.dim_in_pixel();
    let light = v(-0.4, 0.5, 0.75).normalize();
    for py in 0..h {
        for px in 0..w {
            let u = 1.0 - 2.0 * (px as Scalar + 0.5) / w as Scalar;
            let vv = 1.0 - 2.0 * (py as Scalar + 0.5) / h as Scalar;
            let dir = sensor.ray(&pose, u, vv);
            let color = match scene.cast(sensor.position, dir) {
                Some(hit) => {
                    // Face the normal towards the camera (planes are one-sided
                    // otherwise), then plain Lambert: no light from behind.
                    let normal = if hit.normal.dot(&dir) > 0.0 {
                        -hit.normal
                    } else {
                        hit.normal
                    };
                    let lambert = 0.25 + 0.75 * normal.dot(&light).max(0.0);
                    let RGBColor(r, g, b) = distance_color(hit.distance, sensor.max_range);
                    let shade = |c: u8| (c as Scalar * lambert).min(255.0) as u8;
                    RGBColor(shade(r), shade(g), shade(b))
                }
                None => BACKGROUND,
            };
            area.draw_pixel((px as i32, py as i32), &color)?;
        }
    }
    let grid = RGBColor(255, 255, 255).mix(0.35);
    for i in 1..ZONES {
        let x = (i * w as usize / ZONES) as i32;
        let y = (i * h as usize / ZONES) as i32;
        area.draw(&PathElement::new(vec![(x, 0), (x, h as i32)], grid))?;
        area.draw(&PathElement::new(vec![(0, y), (w as i32, y)], grid))?;
    }
    Ok(())
}

/// The 8×8 frame as the sensor reports it: one colour per zone, distance in
/// centimetres, a cross for "no target".
fn draw_zones(
    area: &DrawingArea<BitMapBackend, Shift>,
    frame: &Frame,
    max_range: Scalar,
) -> Res<()> {
    let (w, h) = area.dim_in_pixel();
    let cell_w = w as i32 / ZONES as i32;
    let cell_h = h as i32 / ZONES as i32;
    let label = ("sans-serif", (cell_h / 3).max(10)).into_font();
    for (row, zone_row) in frame.zones.iter().enumerate() {
        for (col, zone) in zone_row.iter().enumerate() {
            let (x0, y0) = (col as i32 * cell_w, row as i32 * cell_h);
            let (x1, y1) = (x0 + cell_w - 2, y0 + cell_h - 2);
            match zone {
                Some(d) => {
                    area.draw(&Rectangle::new(
                        [(x0, y0), (x1, y1)],
                        distance_color(*d, max_range).filled(),
                    ))?;
                    let text = format!("{:.0}", d * 100.0);
                    let style = label.clone().color(&BLACK).pos(text_anchor::Pos::new(
                        text_anchor::HPos::Center,
                        text_anchor::VPos::Center,
                    ));
                    area.draw(&Text::new(text, ((x0 + x1) / 2, (y0 + y1) / 2), style))?;
                }
                None => {
                    area.draw(&Rectangle::new(
                        [(x0, y0), (x1, y1)],
                        RGBColor(40, 43, 52).filled(),
                    ))?;
                    area.draw(&PathElement::new(
                        vec![(x0 + 8, y0 + 8), (x1 - 8, y1 - 8)],
                        MUTED,
                    ))?;
                    area.draw(&PathElement::new(
                        vec![(x1 - 8, y0 + 8), (x0 + 8, y1 - 8)],
                        MUTED,
                    ))?;
                }
            }
        }
    }
    Ok(())
}

fn titled<'a>(
    area: &DrawingArea<BitMapBackend<'a>, Shift>,
    title: &str,
    subtitle: &str,
) -> Res<DrawingArea<BitMapBackend<'a>, Shift>> {
    area.fill(&BACKGROUND)?;
    area.draw_text(title, &("sans-serif", 22).into_font().color(&INK), (14, 10))?;
    area.draw_text(
        subtitle,
        &("sans-serif", 15).into_font().color(&MUTED),
        (14, 38),
    )?;
    Ok(area.margin(66, 14, 14, 14))
}

fn render_view(path: &PathBuf, base: &Scene, sensor: &TofSensor) -> Res<()> {
    let pose = PanTilt::degrees(-8.0, -8.0);
    let scene = with_target(base, target_at(0.42));
    let frame = sensor.measure(&scene, pose, &mut StdRng::seed_from_u64(7));

    let root = BitMapBackend::new(path, (1040, 560)).into_drawing_area();
    root.fill(&BACKGROUND)?;
    let (left, right) = root.split_horizontally(520);
    let left = titled(
        &left,
        "Ray-caster: one ray per pixel",
        "pinhole camera at the sensor, same 45° field of view",
    )?;
    draw_camera_view(&left, &scene, sensor, pose)?;
    let right = titled(
        &right,
        "VL53L5CX twin: 8×8 zones",
        "median of 4×4 sub-rays per zone, distances in cm",
    )?;
    draw_zones(&right, &frame, sensor.max_range)?;
    root.present()?;
    Ok(())
}

/// Pan sweep: −55° → +55° → −55°, tilt slightly down to see the floor objects.
fn pose_at(i: usize) -> PanTilt {
    let phase = i as Scalar / GIF_FRAMES as Scalar;
    let pan = -55.0 * (phase * std::f32::consts::TAU).cos();
    PanTilt::degrees(pan, -10.0)
}

fn render_scan(path: &PathBuf, base: &Scene, sensor: &TofSensor) -> Res<()> {
    let root = BitMapBackend::gif(path, (1040, 560), 120)?.into_drawing_area();
    let mut rng = StdRng::seed_from_u64(42);
    let mut cloud: Vec<Vector<3>> = Vec::new();
    let mut trail: Vec<Vector<3>> = Vec::new();

    for i in 0..GIF_FRAMES {
        let t = i as Scalar / (GIF_FRAMES - 1) as Scalar;
        let target = target_at(t);
        trail.push(target);
        let scene = with_target(base, target);
        let pose = pose_at(i);
        let frame = sensor.measure(&scene, pose, &mut rng);

        root.fill(&BACKGROUND)?;
        let (left, right) = root.split_horizontally(620);

        // Left: 3-D point cloud. Plotters' 3-D y axis is vertical, so world
        // (x, y, z) is drawn as (x, z, y).
        let left = titled(
            &left,
            "Pan-tilt scan: point cloud",
            &format!(
                "pan {:+.0}°  tilt {:+.0}°   grey: earlier frames   colour: this frame   magenta: true target",
                pose.pan.to_degrees(),
                pose.tilt.to_degrees()
            ),
        )?;
        let mut chart =
            ChartBuilder::on(&left).build_cartesian_3d(0.0f32..3.6, 0.0f32..1.4, -1.9f32..1.9)?;
        chart.with_projection(|mut p| {
            p.pitch = 0.5;
            p.yaw = 2.3; // from behind the sensor, looking along +x
            p.scale = 0.95;
            p.into_matrix()
        });
        chart
            .configure_axes()
            .light_grid_style(TRANSPARENT)
            .bold_grid_style(RGBColor(60, 64, 76))
            .axis_panel_style(RGBColor(28, 31, 38))
            .label_style(("sans-serif", 12).into_font().color(&MUTED))
            .max_light_lines(0)
            .draw()?;

        let to_chart = |p: &Vector<3>| (p[0], p[2], p[1]);
        chart.draw_series(
            cloud
                .iter()
                .map(|p| Circle::new(to_chart(p), 1, MUTED.mix(0.5).filled())),
        )?;
        chart.draw_series(frame.points.iter().map(|p| {
            let d = (*p - sensor.position).l2_norm();
            Circle::new(to_chart(p), 3, distance_color(d, sensor.max_range).filled())
        }))?;
        chart.draw_series(LineSeries::new(
            trail.iter().map(to_chart),
            TARGET.mix(0.7).stroke_width(2),
        ))?;
        // White ring under the dot keeps it readable over any point colour.
        chart.draw_series([
            Circle::new(to_chart(&target), 8, WHITE.filled()),
            Circle::new(to_chart(&target), 6, TARGET.filled()),
        ])?;

        // Field-of-view pyramid, 1 m long.
        let apex = to_chart(&sensor.position);
        for (u, vv) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
            let tip = sensor.position + sensor.ray(&pose, u, vv) * 1.0;
            chart.draw_series(LineSeries::new([apex, to_chart(&tip)], INK.mix(0.6)))?;
        }

        // Right: the live 8×8 frame.
        let right = titled(
            &right,
            "Live 8×8 frame",
            "what the MCU receives, distances in cm",
        )?;
        draw_zones(&right, &frame, sensor.max_range)?;
        root.present()?;

        cloud.extend(frame.points.iter().copied());
    }
    Ok(())
}

fn main() -> Res<()> {
    let out = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("renders");
    std::fs::create_dir_all(&out)?;
    let base = room();
    let sensor = TofSensor::vl53l5cx(v(0.0, 0.0, SENSOR_HEIGHT));

    let view = out.join("raycast_view.png");
    render_view(&view, &base, &sensor)?;
    eprintln!("wrote {}", view.display());

    let scan = out.join("tof_scan.gif");
    render_scan(&scan, &base, &sensor)?;
    eprintln!("wrote {}", scan.display());
    Ok(())
}
