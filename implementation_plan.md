# Piano di Implementazione: Architettura Playground & Benchmark Suite

Questo documento descrive il piano per riorganizzare il repository in una struttura flessibile a **Playground** con un modulo dedicato di **Benchmarking e Calcolo Metriche** (`compute_metrics.m`), consentendo di testare il drone in molteplici scenari, con diversi modelli di velivolo e valutare quantitativamente le prestazioni di tracciamento e consumo.

---

## Architettura Proposta

Il progetto verrà organizzato secondo una struttura modulare pulita:

```
Minimum-snap-trajectory-generation/
├── main.m                      # Entry-point rapido per test veloci
├── run_playground.m            # Entry-point flessibile per eseguire scenari e benchmark
├── scenarios/                  # Libreria degli scenari di volo
│   ├── scenario_hover_to_hover.m
│   ├── scenario_slalom_3d.m
│   └── scenario_narrow_window.m
├── config/                     # Profili fisici e dinamici del drone
│   ├── drone_hummingbird.m     # Drone leggero Mellinger & Kumar (0.5 kg)
│   └── drone_custom_heavy.m    # Drone pesante attuale (5.4 kg)
├── planning/                   # TrajectoryGen, optimize_segment_times, eval_traj, check_feasibility
├── simulation/                 # simulate_flight, quadrotor_dynamics, geometric_controller, motor_mixing
├── benchmarks/                 # Modulo di benchmark e metriche
│   ├── compute_metrics.m       # Calcolo di RMSE, picchi, efficienza, consumo e stabilità
│   └── print_benchmark_report.m # Report formattato a terminale e salvataggio metriche
└── utils/                      # compute_H_matrix, poly_basis
```

---

## Componenti da Sviluppare

### 1. Scenari di Volo (`scenarios/`)
Verranno creati file dedicati per definire rapidamente vari tipi di missione:
* **`scenario_slalom_3d.m`**: Slalom 3D ad alta velocità tra 4+ keyframes con corridoi liberi.
* **`scenario_narrow_window.m`**: Percorso di precisione con Safe Corridor stretto (`corridor_delta = 0.05 m`) per simulare il passaggio in una finestra.
* **`scenario_aggressive_flip.m`**: Cambio repentino di direzione e Yaw a 180° per testare il controllore non lineare su $SO(3)$.

### 2. Profili Drone (`config/`)
Disaccoppiamento dei parametri fisici dal `main.m`:
* **`drone_hummingbird.m`**: $m = 0.5 \text{ kg}$, $J = \text{diag}([2.3, 2.3, 4.0]) \cdot 10^{-3}$, braccio $L = 0.17 \text{ m}$, $w_{max} = 900 \text{ rad/s}$.
* **`drone_custom_heavy.m`**: $m = 5.4 \text{ kg}$ e tensore $J$ attualmente impostato in `main.m`.

### 3. Modulo Metriche & Benchmark (`benchmarks/compute_metrics.m`)
La funzione `compute_metrics` analizzerà i log della simulazione ad anello chiuso e restituirà una struttura `metrics` con i seguenti indicatori:

1. **Errori di Tracciamento (Tracking Accuracy):**
   * **`rmse_pos`**: Root Mean Square Error della posizione (in metri).
   * **`max_pos_error`**: Errore di picco massimo di posizione.
   * **`rmse_vel`**: RMSE della velocità rispetto al profilo teorico.
   * **`rmse_psi`**: Errore medio dell'orientamento su $SO(3)$.

2. **Aggressività & Fluidità (Flight Smoothness):**
   * **`total_snap_cost`**: Integrale effettivo dello Snap sulla traiettoria simulata.
   * **`max_tilt_reached_deg`**: Massimo angolo di inclinazione (roll/pitch) raggiunto durante il volo.

3. **Prestazioni Motori e Saturazione (Motor Stress):**
   * **`max_thrust_ratio`**: Percentuale massima della spinta usata rispetto a $u_{max}$.
   * **`motor_saturation_events`**: Numero di istanti in cui uno o più motori hanno toccato $w_{max}$.

4. **Efficienza dell'Ottimizzazione (Planning Efficiency):**
   * **`cost_reduction_percent`**: Percentuale di abbattimento del costo dello Snap ottenuta da `optimize_segment_times`.
   * **`scaling_retries`**: Numero di tentativi di auto-relax del 5% necessari per rendere ammissibile la traiettoria.

---

## Modifiche ai File Esistenti

### [`MODIFY`] [`simulate_flight.m`](file:///d:/Minimum-snap-trajectory-generation/simulation/simulate_flight.m)
* Estensione della funzione per restituire la struttura `log_data` (contenente le serie temporali di posizione reale/desiderata, velocità reale/desiderata, errori di orientamento $\Psi$, spinta e velocità dei singoli motori).

### [`NEW`] [`run_playground.m`](file:///d:/Minimum-snap-trajectory-generation/run_playground.m)
* Script principale per selezionare scenario e drone, lanciare la pipeline completa e mostrare i grafici e la tabella dei benchmark.

### [`NEW`] [`benchmarks/compute_metrics.m`](file:///d:/Minimum-snap-trajectory-generation/benchmarks/compute_metrics.m)
* Calcolo matematico di tutte le metriche quantitative.

---

## Piano di Verifica

### Verification Steps
1. Eseguire `run_playground` selezionando lo scenario `slalom_3d` e il profilo `hummingbird`.
2. Verificare che la pipeline compia correttamente: Generazione -> Ottimizzazione -> Feasibility -> Simulazione -> Calcolo Metriche.
3. Verificare che il report finale stampi correttamente RMSE, picchi di errore, saturazione motori e riduzione percentuale del costo.
4. Verificare il passaggio tra diversi scenari (es. `narrow_window`) e la corretta applicazione dei vincoli.
