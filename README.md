# Sentinel

3-D target tracking with a Kalman filter that runs on a microcontroller, built
on [`frugal_ml`](https://github.com/matiscodjia/frugal_ml), a `no_std` linear
algebra crate with no heap.

**Status: work in progress.** The filter, the simulator and the servo bring-up
work today. The full system (ToF sensor on a pan-tilt arm, target lock, live
3-D view) is planned in [`PLAN.md`](PLAN.md) and is being built step by step.

![Kalman estimate vs ground truth and noisy measurements](trajectoire_kalman.png)

## Workspace

| Crate | Target | What it does |
|---|---|---|
| `sentinel-core` | `no_std` | 9-state / 3-measurement constant-acceleration Kalman filter, written twice: on `nalgebra` (`KalmanFilter`) and on `frugal_ml` (`KalmanFilterFrugal`), with the same API, to compare them |
| `sentinel-sim` | host | Two simulators. The Kalman simulator generates 3-D trajectories (linear, helix, ballistic with wind, chaotic), adds Gaussian sensor noise, runs `KalmanFilterFrugal` and prints CSV; `plot.py` renders it in 3-D. The **digital twin** is a ray-caster (`intersectable.rs`) plus a VL53L5CX sensor model on the pan-tilt arm (`tof.rs`) |
| `sentinel-embedd` | STM32F411 (`thumbv7em-none-eabihf`) | Firmware bring-up: servo PWM on TIM3 / PA6 at 50 Hz, status LED. This is the first step of the pan-tilt arm |

## Results so far

**Speed on the host** (Apple M4 Pro, `cargo bench -p sentinel-core`), one `predict` + `update`:

| Implementation | Time |
|---|---|
| `KalmanFilter` (nalgebra) | 174 ns |
| `KalmanFilterFrugal` (frugal_ml) | 233 ns |

frugal_ml is about 1.34× slower than nalgebra here, without any heap and with
every shape checked at compile time. These numbers come from the host. The
filter has not been timed on the MCU yet.

**Accuracy in simulation** (RMS 3-D position error, σ = 0.7 sensor noise,
`process_noise = 0.05`, 3 runs of 200 steps each):

| Scenario | Raw measurement | Kalman estimate |
|---|---|---|
| Parabolic (constant acceleration) | 1.20 | **0.96** (−20 %) |
| Looping (helix) | 1.19 | 1.42 (+19 %) |
| Chaotic | 1.18 | 1.29 (+10 %) |

The filter helps when the trajectory matches its constant-acceleration model,
and makes things worse when it does not. Lowering the process noise makes the
helix and chaotic cases worse still (2.0 at `Q = 0.01`), so this is model
mismatch, not too much smoothing. Next steps on the filter: tune `Q` per state
instead of `Q = q·I`, use `R = σ²` (the code uses `σ`), and try a model that
fits manoeuvring targets (Singer, or an IMM over CV and CA).

## Digital twin: ray-caster and VL53L5CX model

Before the sensor is wired, the twin produces the frames it will send. That
lets the scan, background-removal and clustering steps of [`PLAN.md`](PLAN.md)
be developed and tested against known ground truth.

- **Ray-caster** (`intersectable.rs`): ray / plane, ray / sphere (quadratic)
  and ray / triangle (Möller-Trumbore, using `frugal_ml`'s `Vector<3>::cross`).
  Boxes are built from 12 triangles. `Scene::cast` returns the closest hit.
- **Sensor model** (`tof.rs`): 8×8 zones over 45°×45°, range up to 4 m,
  ±1.5 cm noise, mounted on the pan-tilt arm. A zone is a ~5.6° cone, not a
  thin ray, so the model casts 4×4 sub-rays per zone and reports their median
  distance. A zone reports no target when fewer than half of its sub-rays come
  back. Edges between near and far objects therefore blur over a whole zone,
  as they do on the real sensor.

![Camera view from the ray-caster next to the 8×8 frame the sensor reports](sentinel-sim/renders/raycast_view.png)

The arm sweeping the room while a target crosses it. The point cloud builds
up on the left (each valid zone is projected back to x, y, z, step 3 of the
plan) and the live 8×8 frame shows on the right:

![Pan-tilt scan animation](sentinel-sim/renders/tof_scan.gif)

```sh
cargo run --release -p sentinel-sim --bin raycast_demo   # writes sentinel-sim/renders/
cargo test -p sentinel-sim                               # 13 tests: intersections, sensor model
```

## Running

frugal_ml is a path dependency, so clone it next to this repo:

```
projects/
├── frugal_ml/
└── sentinel/
```

```sh
cargo run --release -p sentinel-sim > out.csv           # pick the scenario in sentinel-sim/src/main.rs
cd sentinel-sim && uv run python plot.py < ../out.csv   # 3-D plot
cargo bench -p sentinel-core                            # nalgebra vs frugal_ml
cd sentinel-embedd && cargo build --release             # firmware (thumbv7em-none-eabihf)
```

## Roadmap

[`PLAN.md`](PLAN.md) (in French) describes the target system: a VL53L5CX 8×8
ToF sensor on a pan-tilt arm that scans like a radar, removes the static
background, clusters moving points into blobs, locks onto a target and tracks
it with `KalmanFilterFrugal` **on the MCU**. A Mac viewer with a hand-written
software rasterizer then draws the result in 3-D. Each of the 11 steps ends
with a check on real hardware.
