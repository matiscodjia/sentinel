# Radar ToF pan-tilt "défense anti-aérienne" : détection, verrouillage de cible, suivi Kalman, visu 3D, scan fin — le tout from scratch

## Contexte

Objectif : une démo live façon système de défense anti-aérienne — un capteur VL53L5CX monté sur un bras pan-tilt scanne une zone, détecte les objets qui entrent dans le champ (typiquement des objets en mouvement, "volants"), les affiche en 3D sous forme de formes blanches, permet de verrouiller une cible selon des critères, la suit dans le temps avec un filtre de Kalman, et affiche tout ça en live dans une scène 3D sur le Mac.

Contraintes explicites de l'utilisateur :
- Pas de caméra — uniquement le VL53L5CX (matrice ToF 8x8) comme "vision".
- Le matériel est déjà en main (VL53L5CX, bras pan-tilt + 2 servos, carte STM32 déjà utilisée dans `sentinel-embedd`).
- L'utilisateur part de zéro en maths et en électronique : chaque étape doit introduire les notions nécessaires avant de coder, projet étalé dans le temps (pas un rush weekend).
- **Tout à la main** : algorithme de scan, clustering, PWM, driver capteur, filtre de Kalman (déjà fait dans `sentinel-core`), et le rendu 3D **jusqu'au bas niveau** (projection, rasterisation, z-buffer écrits à la main).
- **Le filtre de Kalman tourne directement sur le MCU**, pas sur le Mac — cohérent avec `frugal_ml`, la lib de l'utilisateur, conçue pour du calcul bare-metal no_std ([[kalman_frugal.rs]]).
- Le Mac ne fait que recevoir un flux de données déjà traité (blobs + cible verrouillée) et le rendre en 3D.
- Priorité : **objets volants/mobiles entrant dans le champ**, pas de classification voiture/piéton — l'esprit est "radar de surveillance", pas vision routière.
- Ambition de contour très détaillé (ex: distinguer les 5 doigts d'une main) — voir contrainte physique ci-dessous.

### Contrainte physique clé : résolution vs portée vs vitesse

Le VL53L5CX a une résolution angulaire native fixe d'environ **5,6° par zone** (FOV total 45° / 8 zones dans le sens le plus fin, mode 8x8). Cette résolution ne s'améliore pas en pointant le bras plus finement : chaque mesure intègre le retour sur tout son cône de ~5,6°, quel que soit l'endroit où on le centre.

Conséquence chiffrée : pour distinguer des doigts (écartement ~2-2,5 cm), il faut que ce cône soit plus petit que l'écartement à la distance de l'objet — ce qui n'arrive qu'à **très courte portée (~20-30 cm)**. À l'inverse, suivre un objet volant à plusieurs mètres, en mouvement rapide, demande un scan large et rapide (mode 8x8 à 15-60 Hz), où le contour reste forcément un blob grossier — comme un vrai radar, qui ne résout jamais la forme fine d'une cible.

→ **Décision retenue avec l'utilisateur : deux modes de scan séparés**, pas un compromis unique :
1. **Mode large / poursuite** ("radar") — scan 8x8 rapide sur toute l'amplitude du bras, détection et suivi Kalman d'objets en mouvement, contour en blob (comme un vrai radar).
2. **Mode scan fin** — sur une cible proche et plutôt statique (ex: verrouillée puis immobile, ou déclenché manuellement), raster pan-tilt beaucoup plus fin sur une petite fenêtre angulaire, lecture zone par zone pour suréchantillonner → nuage de points local à bien plus haute densité, au prix d'un scan lent (plusieurs secondes) et courte portée.

## Architecture cible

```
VL53L5CX --I2C--> STM32 (sentinel-embedd)
                     ├─ pilotage pan-tilt (2 canaux PWM)
                     ├─ driver VL53L5CX (I2C, écrit à la main)
                     ├─ MODE LARGE (radar) : scan 8x8 rapide, plage complète
                     │    ├─ projection sphérique→cartésien (pan, tilt, distance → x,y,z)
                     │    ├─ filtrage statique/mobile (équivalent "background subtraction" du ToF :
                     │    │    un point qui ne bouge pas d'un scan à l'autre = décor, pas une cible)
                     │    ├─ clustering des points mobiles en blobs
                     │    ├─ verrouillage de cible selon critères (proximité, vitesse, apparition)
                     │    └─ KalmanFilterFrugal (sentinel-core, déjà existant) sur la cible verrouillée
                     │
                     └─ MODE FIN (scan détaillé) : déclenché sur cible proche/lente ou manuellement
                          └─ raster pan-tilt fin sur une petite fenêtre autour de la cible,
                             lecture point par point → nuage de points local haute densité
                          │
                          └─ trame binaire série (UART/USB) : blobs + cible filtrée (mode large)
                                                              ou nuage fin (mode scan détaillé)
                                     │
                                     v
                          Mac (nouvelle crate sentinel-viz)
                              ├─ décodage trame
                              └─ rendu 3D logiciel from scratch (points blancs, cible mise en évidence,
                                 trajectoire ; nuage dense superposé en mode scan fin)
```

Le MCU fait tout le "cerveau" (scan, vision, décision, tracking) ; le Mac n'est qu'un afficheur.

## Réutilisation de l'existant

- `sentinel-core/src/kalman_frugal.rs` → `KalmanFilterFrugal` réutilisé tel quel pour le suivi de cible (no_std, déjà benchmarké).
- `sentinel-embedd/src/main.rs` → pattern PWM déjà en place (`Channel1`, `TIM3.pwm_hz`, `ch1.set_duty`) à étendre à 2 canaux indépendants (pan + tilt).
- `sentinel-sim/` reste inchangé (simulation synthétique existante) ; la nouvelle démo live vit dans une crate séparée pour ne pas mélanger les deux usages.
- Pattern workspace existant (`sentinel-core` = logique partagée no_std, `sentinel-embedd` = firmware, nouvelle `sentinel-viz` = appli Mac) reproduit la séparation déjà en place.

## Nouveaux modules/crates prévus

Logique partagée, no_std, dans `sentinel-core` (comme `kalman_frugal.rs`) :
- `sentinel-core/src/clustering.rs` — regroupement de points 3D en blobs (grille de voxels + fusion de voisins, mémoire fixe, adapté no_std).
- `sentinel-core/src/motion_filter.rs` — filtrage statique/mobile entre scans successifs (le "background subtraction" du ToF).
- `sentinel-core/src/protocol.rs` — format de trame binaire partagé (mêmes structs encodées côté MCU et décodées côté Mac, pas de dérive de protocole) ; deux types de trame (blobs mode large / nuage dense mode fin).

Spécifique matériel, dans `sentinel-embedd` :
- `sentinel-embedd/src/vl53l5cx.rs` — driver I2C écrit à la main (init, lecture des 64 zones en mode 8x8, lecture zone unique pour le mode fin).
- `sentinel-embedd/src/pan_tilt.rs` — contrôle 2 axes (pan/tilt), calibration angle→duty cycle, mode "raster fin" (petits pas angulaires).
- `sentinel-embedd/src/main.rs` — boucle principale : mode large en continu (scan → lecture capteur → projection → filtrage mobile → clustering → verrouillage → Kalman → envoi trame), avec bascule vers le mode fin sur déclenchement.

Nouvelle crate Mac, ajoutée au workspace racine :
- `sentinel-viz/` — `src/serial.rs` (lecture port série, décodage trame via `sentinel-core::protocol`), `src/render/math.rs` (matrices rotation/translation/projection écrites à la main), `src/render/rasterizer.rs` (framebuffer + z-buffer + dessin des points/formes, à la main), `src/main.rs` (boucle : lire trame → mettre à jour scène → rasteriser → pousser le framebuffer via une crate de fenêtrage minimale).

## Feuille de route par étapes (chaque étape = un checkpoint concret avant de passer à la suite)

### Étape 1 — Piloter le bras pan-tilt
- Concepts : PWM (période, duty cycle, comment un servo lit un angle dans le signal), extension du pattern déjà utilisé dans `sentinel-embedd/src/main.rs` à un 2e canal.
- Tâches : identifier les 2 canaux PWM du STM32 pour les 2 servos, calibrer empiriquement la plage duty cycle ↔ 0°-180°, écrire `pan_tilt::set_angle(pan_deg, tilt_deg)`.
- Point électronique clé : alimentation des servos (souvent besoin d'une alim externe, masse commune avec le STM32 — pas assez de courant sur la broche 3.3V).
- Checkpoint : le bras exécute une séquence de balayage programmée, à l'œil.

### Étape 2 — Lire le VL53L5CX
- Concepts : protocole I2C (adresse, lecture/écriture registre), driver bas niveau écrit à partir du datasheet/registre map ST (pas de crate toute faite).
- Tâches : driver minimal `vl53l5cx.rs` (init + lecture des 64 distances en mode 8x8, + lecture d'une zone unique pour le mode fin plus tard), capteur fixe.
- Checkpoint : les 64 valeurs de distance sont lisibles en debug (UART), cohérentes avec un objet approché à la main.

### Étape 3 — Nuage de points 3D, mode large
- Concepts : trigonométrie — passage repère sphérique (angle pan, angle tilt, distance) → repère cartésien (x, y, z).
- Tâches : pour chaque position du bras, projeter les 64 zones (chacune avec son angle relatif dans le FoV ~45°x45°) en points 3D, accumuler sur un scan complet de l'amplitude du bras.
- Checkpoint : liste de points (x,y,z) cohérente streamée en debug.

### Étape 4 — Filtrage mobile/statique (équivalent "background subtraction")
- Concepts : comparer deux scans successifs de la même zone angulaire — un point qui ne change pas = décor (mur, sol, objets fixes), un point qui apparaît/bouge = candidat cible. C'est l'équivalent ToF du GMM vidéo qui a inspiré le projet.
- Tâches : garder en mémoire une carte de référence (distance "de fond" par direction), comparer chaque nouveau scan à cette référence, ne garder que les écarts significatifs.
- Checkpoint : un objet statique dans le champ disparaît du flux après quelques scans ; un objet qui bouge reste visible.

### Étape 5 — Clustering
- Concepts : distance euclidienne, algorithme de clustering simple et mémoire-fixe adapté à l'embarqué (grille de voxels + fusion des voisins).
- Tâches : regrouper les points mobiles proches en blobs, calculer centroïde + bounding box + vitesse (delta de position entre scans) par blob.
- Checkpoint : deux objets mobiles distincts dans le champ donnent bien deux clusters séparés.

### Étape 6 — Critères de verrouillage de cible
- Concepts : pas de classification sémantique — des critères géométriques/cinématiques (le plus proche, le plus rapide, le premier apparu, le plus gros...).
- Tâches : implémenter la sélection de cible selon un critère configurable, avec hystérésis simple (éviter de changer de cible à chaque frame si plusieurs blobs sont proches en score).
- Checkpoint : le blob verrouillé est identifié correctement en debug selon le critère choisi, reste stable dans le temps.

### Étape 7 — Kalman sur MCU
- Concepts : réutilisation directe de `KalmanFilterFrugal` ([[kalman_frugal.rs]]) — le centroïde du blob verrouillé devient la mesure bruitée injectée à chaque frame.
- Tâches : instancier/reset le filtre au verrouillage, `predict()`/`update()` à chaque frame, gérer la perte temporaire de cible (prédiction seule pendant N frames avant déverrouillage).
- Checkpoint : la position filtrée reste stable/lissée en debug même avec du bruit de mesure simulé (ex: en cachant la cible un instant).

### Étape 8 — Mode scan fin (contour détaillé)
- Concepts : compromis résolution/vitesse/portée expliqué plus haut ; raster à petits pas angulaires (plus fins que la taille native d'une zone) sur une fenêtre réduite, en lecture zone-par-zone plutôt que 8x8 en une passe.
- Tâches : routine de scan fin déclenchée sur une cible proche (~20-30 cm) et peu mobile (ou déclenchement manuel), centrée sur la direction de la cible verrouillée, balaie une petite fenêtre angulaire avec un pas fin, accumule un nuage de points local à densité bien supérieure au mode large.
- Checkpoint : un objet proche et détaillé (ex: une main) donne un nuage de points où les grandes formes (doigts écartés) sont visuellement distinguables en debug — poser honnêtement la limite réelle observée par rapport à l'objectif "5 doigts nets".

### Étape 9 — Protocole série MCU → Mac
- Concepts : framing binaire simple (en-tête, longueur, checksum).
- Tâches : définir les trames dans `sentinel-core/src/protocol.rs` (blobs + cible filtrée pour le mode large ; nuage dense pour le mode fin), encoder côté MCU, décoder côté Mac.
- Checkpoint : le Mac affiche en texte les trames reçues en live, pour les deux modes.

### Étape 10 — Rendu 3D from scratch
- Concepts (le plus gros morceau maths) : matrices de rotation/translation, projection perspective, repère caméra, rasterisation d'un point/triangle projeté en pixels, z-buffer pour la profondeur.
- Tâches : `render/math.rs` (matrices à la main), `render/rasterizer.rs` (framebuffer + z-buffer + dessin des formes), fenêtre + affichage du framebuffer via une crate minimale de fenêtrage (seule brique non "from scratch").
- Checkpoint : les blobs du mode large s'affichent comme des points/formes blanches en 3D, caméra orbitale contrôlable ; le nuage dense du mode fin s'affiche en superposition avec plus de détail visuel.

### Étape 11 — Intégration finale
- Tâches : brancher le flux complet, mise en valeur visuelle de la cible verrouillée (couleur/halo différent) et de sa trajectoire (tracé des états Kalman successifs dans le temps), bascule visuelle claire entre mode large (radar) et mode fin (scan détaillé) déclenché sur la cible.
- Checkpoint final : démo live — le bras scanne en mode large, les formes blanches (objets mobiles) apparaissent et le décor statique disparaît, une cible s'accroche et est suivie avec une trajectoire lissée, et sur demande un scan fin rapproché montre le niveau de détail maximal atteignable sur cette cible.

## Vérification

Chaque étape a son propre checkpoint physique/visuel listé ci-dessus — on ne passe à l'étape suivante que quand le checkpoint précédent fonctionne réellement sur le matériel (pas juste en théorie). Étape 11 = test end-to-end de la démo complète, dans ses deux modes.
