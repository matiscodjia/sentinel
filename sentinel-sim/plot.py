import sys

import matplotlib.pyplot as plt
import numpy as np  # Indispensable pour les maths du tube
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

# --- CONFIGURATION DES TUBES ---
# Rayon du tube "Mesures" (Représente le bruit du capteur, genre 3*sigma)
# Plus c'est gros, plus le capteur est mauvais.
RADIUS_MEAS = 1.5 

# Rayon du tube "Kalman" (Représente la confiance du filtre)
# Il doit être plus fin que le tube de mesure pour montrer que le filtre améliore la précision.
RADIUS_KALMAN = 0.4
# -------------------------------


def generate_tube_coordinates(x_path, y_path, z_path, radius, segments=12):
    """
    Génère les matrices de coordonnées X, Y, Z nécessaires pour plot_wireframe.
    Crée des cercles horizontaux empilés le long de la trajectoire.
    """
    # Convertir les séries Pandas en tableaux Numpy pour les calculs
    x_center = x_path.to_numpy()
    y_center = y_path.to_numpy()
    z_center = z_path.to_numpy()
    n_steps = len(x_center)

    # Angles autour du cercle (de 0 à 2pi)
    theta = np.linspace(0, 2 * np.pi, segments)
    
    # On crée des grilles 2D (Temps x Angle)
    theta_grid, _ = np.meshgrid(theta, range(n_steps))
    
    # Calcul des coordonnées de la surface du tube
    # On ajoute un décalage circulaire (R*cos, R*sin) au centre (x_center, y_center)
    # '[:, np.newaxis]' permet d'étendre le tableau 1D du centre en 2D pour le calcul
    X_tube = x_center[:, np.newaxis] + radius * np.cos(theta_grid)
    Y_tube = y_center[:, np.newaxis] + radius * np.sin(theta_grid)
    
    # Pour Z, c'est simple, c'est la hauteur du centre répétée pour tous les angles
    Z_tube = z_center[:, np.newaxis] + np.zeros_like(theta_grid)

    return X_tube, Y_tube, Z_tube


def main():
    # 1. LECTURE
    try:
        if sys.stdin.isatty():
            print("⚠️ Usage : cargo run --release --quiet | python3 plot_3d.py")
            sys.exit(1)
        print("📥 Réception des données...")
        df = pd.read_csv(sys.stdin)
    except Exception as e:
        print(f"❌ Erreur : {e}")
        sys.exit(1)

    # 2. SETUP 3D
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # --- TRACÉS ---

    # A. Vraie Trajectoire (Ligne simple verte)
    ax.plot(df['tx'], df['ty'], df['tz'], c='green', lw=2, label='Vraie Trajectoire', zorder=5)

    # B. TUBE DE MESURES (Cage rouge large)
    # On génère le tube autour de la VRAIE trajectoire (car le bruit est centré dessus)
    Xm, Ym, Zm = generate_tube_coordinates(df['tx'], df['ty'], df['tz'], RADIUS_MEAS, segments=10)
    # rstride/cstride contrôlent la densité du maillage (plus c'est grand, moins y'a de fils)
    ax.plot_wireframe(Xm, Ym, Zm, color='black', alpha=0.30, rstride=5, cstride=1, label='Incertitude Capteur')
    # On garde les points pour le style
    ax.scatter(df['mx'], df['my'], df['mz'], c='red', s=2, alpha=0.3)


    # C. TUBE KALMAN (Pipette bleue fine)
    # On génère le tube autour de l'ESTIMATION Kalman
    Xk, Yk, Zk = generate_tube_coordinates(df['ex'], df['ey'], df['ez'], RADIUS_KALMAN, segments=8)
    # Tube plus fin, plus opaque, maillage plus serré le long du temps (rstride=2)
    #ax.plot_wireframe(Xk, Yk, Zk, color='blue', alpha=0.3, rstride=2, cstride=1, label='Estimation Kalman')
    # Ligne centrale bleue
    ax.plot(df['ex'], df['ey'], df['ez'], c='blue', lw=2, zorder=10)


    # 4. ESTHÉTIQUE
    ax.set_title("Tracking 3D avec Tubes d'Incertitude", fontsize=14)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    
    # Hack pour avoir une légende avec wireframe (matplotlib gère mal ça par défaut)
    from matplotlib.lines import Line2D
    custom_lines = [Line2D([0], [0], color='green', lw=2),
                    Line2D([0], [0], color='red', alpha=0.5, linestyle=':'),
                    Line2D([0], [0], color='blue', alpha=0.6, lw=2)]
    ax.legend(custom_lines, ['Vraie Trajectoire', f'Tube Mesure (R={RADIUS_MEAS}m)', f'Tube Kalman (R={RADIUS_KALMAN}m)'])

    # Equal aspect ratio
    mid_x = (df['tx'].max()+df['tx'].min()) * 0.5
    mid_y = (df['ty'].max()+df['ty'].min()) * 0.5
    mid_z = (df['tz'].max()+df['tz'].min()) * 0.5
    max_range = max(df['tx'].max()-df['tx'].min(), df['ty'].max()-df['ty'].min(), df['tz'].max()-df['tz'].min()) / 2.0
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    print("🖱️ Tournez le graphique pour voir les tubes !")
    plt.show()

if __name__ == "__main__":
    main()
