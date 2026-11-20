# Recap Implementazione — Minimum Snap Trajectory Generation & Control

Documento di sintesi delle funzionalità implementate nel repository per la pianificazione, verifica fisica, controllo e simulazione di un quadrirotore.

---

## 1. Entry Point e Configurazione
- **Script principali**:
  - `main.m`: pipeline completa (pianificazione iniziale, ottimizzazione tempi, verifica fattibilità, dilatazione temporale, simulazione 3D e metriche).
  - `run_playground.m`: cruscotto modulare per test rapidi su combinazioni di drone, missione, controllore, ottimizzazione tempi e disturbi.
- **Modelli di Drone** (`config/load_drone.m`, `config/drones/`):
  - `heavy_5kg` (5.4 kg) e `hummingbird` (AscTec Hummingbird, 0.5 kg), con massa, inerzia `J`, braccio `L`, coefficienti `kF`, `kM`, `k_drag`, limiti rotori `[w_min, w_max]` e calcolo automatico del `max_tilt_angle_deg` da TWR.
- **Preset di Missione** (`config/load_mission.m`, `config/missions/`):
  - `figure8_3d`, `figure8_smooth`, `figure8_corridors`, `sharp_turn_pca`, `hoop_pass`, `simple_arc` (keyframes `[x, y, z, yaw]`, tempi e parametri `corridor_delta` / `corridor_samples`).

---

## 2. Pianificazione Traiettoria (Planning)
- **Generazione Minimum Snap via QP** (`planning/trajectoryGen.m`, `utils/compute_H_matrix.m`, `utils/poly_basis.m`):
  - Formulazione a Programmazione Quadratica disaccoppiata su `X`, `Y`, `Z` (minimo snap, `k_pos = 4`, polinomi grado `n_pos = 7`) e `Yaw` (minima accelerazione, `k_yaw = 2`, grado `n_yaw = 3`).
  - Vincoli di passaggio ai waypoints, continuità delle derivate fino a `(k-1)`, condizioni al contorno nulle a inizio/fine volo.
  - Adimensionalizzazione temporale e spaziale (`use_scaling`) per stabilità numerica.
  - **Safe Corridors**: vincoli di disuguaglianza per confinare i segmenti entro tubi di tolleranza `corridor_delta`.
- **Ottimizzazione Tempi dei Segmenti** (`planning/optimize_segment_times.m`):
  - Discesa del gradiente numerico sulle durate dei segmenti `T` con backtracking line search opzionale (`use_backtracking`) e salvataggio della configurazione a costo minimo.
- **Verifica di Fattibilità Fisica** (`planning/check_feasibility_advanced.m`, `planning/check_feasibility.m`):
  - Basata su *Differential Flatness*: ricava spinta `u1`, assetto e momenti `tau` dall'inerzia `J` e dalle accelerazioni angolari, invertendo la matrice di mixing per verificare i singoli rotori entro `[w_min, w_max]` con margine `motor_headroom`.
  - Loop automatico di dilatazione temporale (+5% per tentativo) se la manovra supera i limiti fisici.
- **Valutazione Traiettoria** (`planning/eval_traj.m`): estrazione di posizione, velocità, accelerazione, jerk e snap.

---

## 3. Controllo, Allocazione Motori e Dinamica (Simulation)
- **Controllori** (`simulation/control/controller_selector.m`):
  - `geometric_controller.m`: controllo non lineare su $SO(3)$ (inseguimento posizione/velocità nel World Frame e assetto/velocità angolare su $SO(3)$).
  - `pid_controller.m`: controllore PID linearizzato su angoli di Eulero.
  - `lqr_controller.m`: previsto nel selettore (`controller_selector.m`), non ancora implementato su file.
- **Tuning Analitico dei Guadagni** (`simulation/control/tune_gains.m`):
  - Pole placement automatico su specifica temporale (`Ts_pos`, `zeta`, `separation_factor`, `z_rigidity_factor`) scalato su massa `m` (outer loop `Kp`, `Kv`) e matrice d'inerzia `J` (inner loop `KR`, `Komega`).
- **Motor Mixing & Prioritized Control Allocation (PCA)** (`simulation/control/motor_mixing.m`):
  - Conversione `[u1; tau_x; tau_y; tau_z]` $\leftrightarrow$ velocità rotori $\omega_i^2$ (configurazione `+`) con saturazione fisica `[w_min^2, w_max^2]`.
  - Allocazione analitica gerarchica in caso di saturazione: **Roll/Pitch (`tau_x, tau_y`) > Spinta (`u1`) > Yaw (`tau_z`)**, con riduzione progressiva di yaw, clamping della spinta e scalatura proporzionale estrema di roll/pitch.
  - Esporta `motor_info` con velocità ideali/reali, percentuale di carico `load_pct_real` in `[0, 100]%` e flag di saturazione.
- **Modello Fisico e Disturbi** (`simulation/plant/quadrotor_dynamics.m`, `simulation/disturbances/`):
  - Dinamica 6-DoF Newton-Eulero con ri-ortogonalizzazione SVD della matrice di rotazione `Rbw` a ogni step di integrazione (`dt = 0.001 s` di default).
  - Disturbi ambientali (`wind_disturbance.m`, `wind_vel_from_stress.m`): resistenza aerodinamica (`k_drag` su `v_rel = v - v_wind`), vento costante e treno di raffiche ad impulsi (periodo, duty cycle, finestra temporale).

---

## 4. Simulazione 3D, Visualizzazione e Metriche
- **Simulazione 3D Interattiva** (`simulation/simulate_flight.m`):
  - Rendering 3D in tempo reale (dark theme) con traiettoria di riferimento, scia reale, keyframes, Safe Corridors 3D e hitbox del drone.
  - **Check di collisione Hitbox $\leftrightarrow$ Safe Corridors** su tutta la durata del volo con segnalazione dell'eventuale sconfinamento massimo.
  - Barra temporale interattiva (slider) per ispezionare l'assetto e lo stato di collisione istante per istante.
- **Visualizzazione Pianificazione** (`visualization/plot_trajectory.m`, `visualization/plot_trajectory_evolution.m`):
  - Vista 2D (X-Y con corridoi) e 3D della traiettoria iniziale e confronto prima/dopo l'ottimizzazione dei tempi.
- **Metriche Benchmark & Plot Performance** (`benchmarks/compute_metrics.m`, `simulation/simulate_flight.m`):
  - **Report a console**:
    - Durata volo, lunghezza traiettoria e controllore utilizzato.
    - **Errore Posizionale Assoluto**: `MAE 3D` (e assi X, Y, Z), `RMSE 3D` (e assi X, Y, Z), **Errore Assoluto Max 3D** (con istante di picco) e max per asse, **95° percentile** ed **errore finale**.
    - **Velocità**: `RMSE` ed errore massimo.
    - **Saturazione Motori**: carico medio e massimo per singolo motore (`M1..M4` in `%`) e tempo/percentuale di volo in saturazione.
  - **Dashboard grafica `Analisi Performance` (sfondo nero, tema dark 2×2 con legende)**:
    1. Errore di posizione con segno (`e_x, e_y, e_z`).
    2. Errore posizionale assoluto (`||e_p||_2` 3D, `|e_x|, |e_y|, |e_z|`, soglie `MAE` / `RMSE` e picco massimo).
    3. Tracking della velocità (`||v_des||` vs `||v_real||`).
    4. **Curve di saturazione motore per motore** (`M1`..`M4`, `%` carico su Y, soglie `100%` Max, `0%` Idle e limite `motor_headroom` del planner).

