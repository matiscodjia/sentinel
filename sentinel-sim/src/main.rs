use frugal_ml::linalg::Vector;
use nalgebra::SVector;
use rand::prelude::*;
use rand_distr::StandardNormal;
use sentinel_core::KalmanFilterFrugal;

// 1. DÉFINITION DES SCÉNARIOS
#[derive(Debug)]
#[allow(unused)]
enum Scenario {
    Linear,
    Looping,   // Deviens une Spirale en 3D
    Parabolic, // Tir de mortier avec vent latéral
    Chaotic,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- CHOISIS TON SCÉNARIO ICI ---
    let current_scenario = Scenario::Chaotic;
    // --------------------------------

    // 1. Logs pour l'humain (sur stderr pour ne pas polluer le CSV)
    eprintln!("🚀 Lancement du scénario 3D : {:?}", current_scenario);

    // Config
    let dt = 0.05;
    let duration = 10.0;
    let steps = (duration / dt) as i32;

    // Augmente un peu le bruit pour voir le filtre travailler
    let process_noise = 0.05;
    let sensor_noise = 0.7; // Bruit capteur plus fort pour l'effet visuel

    let mut kf = KalmanFilterFrugal::new(dt, process_noise, sensor_noise);
    let mut rng = rand::rng();

    // 2. L'ENTÊTE CSV (CRITIQUE POUR PYTHON)
    // t = temps
    // tx, ty, tz = Vérité Terrain
    // mx, my, mz = Mesure Capteur
    // ex, ey, ez = Estimation Kalman
    println!("t,tx,ty,tz,mx,my,mz,ex,ey,ez");

    // BOUCLE TEMPORELLE
    for i in 0..steps {
        let t = i as f32 * dt;

        // A. VÉRITÉ
        let true_vector = get_true_position(t, &current_scenario);

        // B. MESURE BRUITÉE (Sur les 3 axes !)
        let mx = true_vector.x + rng.sample::<f32, _>(StandardNormal) * sensor_noise;
        let my = true_vector.y + rng.sample::<f32, _>(StandardNormal) * sensor_noise;
        let mz = true_vector.z + rng.sample::<f32, _>(StandardNormal) * sensor_noise;

        let measurement = Vector::<3>::from_data([mx, my, mz]);

        // C. KALMAN
        kf.predict();
        kf.update(measurement);

        // D. SORTIE CSV (STDOUT)
        // ⚠️ HYPOTHÈSE IMPORTANTE :
        // Je suppose que ton vecteur d'état est [PosX, VelX, AccX, PosY, VelY, AccY, PosZ, VelZ, AccZ]
        // Donc les positions sont aux index 0, 3, 6.
        // Si tu utilises un modèle "Vitesse Constante" (6 variables), ce sera [0, 2, 4]. A toi de vérifier !
        println!(
            "{},{},{},{},{},{},{},{},{},{}",
            t, true_vector.x, true_vector.y, true_vector.z, mx, my, mz, kf.x[0], kf.x[3], kf.x[6]
        );

        // Debug humain (optionnel, sur stderr)
        if i % 20 == 0 {
            eprintln!("Step {}/{}", i, steps);
        }
    }

    eprintln!("✅ Simulation terminée.");
    Ok(())
}

// --- LOGIQUE DES MOUVEMENTS 3D ---
fn get_true_position(t: f32, scenario: &Scenario) -> SVector<f32, 3> {
    match scenario {
        Scenario::Linear => {
            // Diagonale qui monte vers le ciel et la droite
            SVector::<f32, 3>::new(10.0 * t, 5.0 * t, 10.0 + 2.0 * t)
        }
        Scenario::Looping => {
            // Spirale (Tire-bouchon)
            let forward_speed = 10.0;
            let radius = 8.0;
            let omega = 1.5;

            let x = forward_speed * t;
            // Y et Z tournent autour de l'axe X
            let y = radius * (omega * t).cos();
            let z = 20.0 + radius * (omega * t).sin();
            SVector::<f32, 3>::new(x, y, z)
        }
        Scenario::Parabolic => {
            // Tir de mortier avec vent latéral constant sur Y
            let vx = 10.0;
            let vy_wind = 3.0;
            let vz_init = 30.0;
            let g = 9.81;

            let x = vx * t;
            let y = vy_wind * t; // Dérive latérale
            let z = vz_init * t - 0.5 * g * t * t;

            let z_clamped = if z > 0.0 { z } else { 0.0 };
            SVector::<f32, 3>::new(x, y, z_clamped)
        }
        Scenario::Chaotic => {
            // Mouche ivre en 3D
            SVector::<f32, 3>::new(
                5.0 * t + 2.0 * (t * 3.0).sin(),        // Avance en oscillant
                5.0 * (t * 0.5).cos() * (t).sin(),      // Oscillation latérale complexe
                10.0 + 2.0 * t + 3.0 * (t * 4.0).cos(), // Monte en vibrant
            )
        }
    }
}

