# Sentinel

Bac à sable pour explorer le machine learning et le filtrage de Kalman, avec exécution sur microcontrôleur (STM32).

## Structure

- `sentinel-core` — filtre de Kalman (implémentation nalgebra + implémentation `frugal_ml`, no_std)
- `sentinel-sim` — simulation de trajectoires 3D bruitées, sortie CSV pour visualisation
- `sentinel-embedd` — firmware embarqué (STM32F4, `cortex-m`, PWM servo)

## Build

```sh
cargo build
cargo run -p sentinel-sim > out.csv
```
